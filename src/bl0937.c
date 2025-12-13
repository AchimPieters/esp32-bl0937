/**
   Copyright 2026 Achim Pieters | StudioPieters®

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NON INFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

   for more information visit https://www.studiopieters.nl
 **/
 
#include "bl0937.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bl0937_filter.h"

static const char *TAG = "bl0937";

struct bl0937_handle {
    bl0937_config_t cfg;

    volatile uint32_t cf_pulses;
    volatile uint32_t cf1_pulses;
    portMUX_TYPE mux;

    bool enabled;
    bool last_cf1_vrms;

    // filtered state
    float filt_v;
    float filt_i;
    float filt_p;

    // overcurrent latch
    bool overcurrent;
    int64_t overcurrent_until_us;

    // event callback
    bl0937_event_cb_t cb;
    void *cb_ctx;
};

static void IRAM_ATTR isr_cf(void *arg) {
    bl0937_handle_t *h = (bl0937_handle_t *)arg;
    portENTER_CRITICAL_ISR(&h->mux);
    h->cf_pulses++;
    portEXIT_CRITICAL_ISR(&h->mux);
}

static void IRAM_ATTR isr_cf1(void *arg) {
    bl0937_handle_t *h = (bl0937_handle_t *)arg;
    portENTER_CRITICAL_ISR(&h->mux);
    h->cf1_pulses++;
    portEXIT_CRITICAL_ISR(&h->mux);
}

static inline void sel_write(bl0937_handle_t *h, int level) {
    if (h->cfg.sel_invert) level = !level;
    gpio_set_level(h->cfg.gpio_sel, level ? 1 : 0);
}

esp_err_t bl0937_set_event_cb(bl0937_handle_t *h, bl0937_event_cb_t cb, void *user_ctx) {
    if (!h) return ESP_ERR_INVALID_ARG;
    h->cb = cb;
    h->cb_ctx = user_ctx;
    return ESP_OK;
}

esp_err_t bl0937_create(const bl0937_config_t *cfg, bl0937_handle_t **out) {
    if (!cfg || !out) return ESP_ERR_INVALID_ARG;

    bl0937_handle_t *h = calloc(1, sizeof(*h));
    if (!h) return ESP_ERR_NO_MEM;

    h->cfg = *cfg;
    h->mux = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

    // Defaults if caller left them 0
    if (h->cfg.vref <= 0.0f)      h->cfg.vref      = 1.218f;
    if (h->cfg.k_f_power <= 0.0f) h->cfg.k_f_power = 1721506.0f;
    if (h->cfg.k_f_vrms <= 0.0f)  h->cfg.k_f_vrms  = 15397.0f;
    if (h->cfg.k_f_irms <= 0.0f)  h->cfg.k_f_irms  = 94638.0f;

    if (h->cfg.cal_vrms == 0.0f)  h->cfg.cal_vrms  = 1.0f;
    if (h->cfg.cal_irms == 0.0f)  h->cfg.cal_irms  = 1.0f;
    if (h->cfg.cal_power == 0.0f) h->cfg.cal_power = 1.0f;

    // Overcurrent defaults (can be disabled by setting threshold=0 explicitly)
    if (h->cfg.overcurrent_hz_threshold == 0.0f) h->cfg.overcurrent_hz_threshold = 6000.0f;
    if (h->cfg.overcurrent_latch_ms == 0)        h->cfg.overcurrent_latch_ms = 2000;

    // EMA defaults (mild smoothing). Disable by setting alpha=0 in cfg.
    if (h->cfg.ema_alpha_v == 0.0f) h->cfg.ema_alpha_v = 0.2f;
    if (h->cfg.ema_alpha_i == 0.0f) h->cfg.ema_alpha_i = 0.2f;
    if (h->cfg.ema_alpha_p == 0.0f) h->cfg.ema_alpha_p = 0.2f;

    h->filt_v = NAN;
    h->filt_i = NAN;
    h->filt_p = NAN;

    h->overcurrent = false;
    h->overcurrent_until_us = 0;

    if (h->cfg.gpio_cf < 0 || h->cfg.gpio_cf1 < 0 || h->cfg.gpio_sel < 0) {
        free(h);
        return ESP_ERR_INVALID_ARG;
    }

    // GPIO setup
    gpio_config_t out_cfg = {
        .pin_bit_mask = 1ULL << h->cfg.gpio_sel,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&out_cfg);
    if (err != ESP_OK) { free(h); return err; }

    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << h->cfg.gpio_cf) | (1ULL << h->cfg.gpio_cf1),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    err = gpio_config(&in_cfg);
    if (err != ESP_OK) { free(h); return err; }

    // ISR service (if already installed -> ESP_ERR_INVALID_STATE)
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        free(h);
        return isr_err;
    }

    err = gpio_isr_handler_add(h->cfg.gpio_cf, isr_cf, h);
    if (err != ESP_OK) { free(h); return err; }
    err = gpio_isr_handler_add(h->cfg.gpio_cf1, isr_cf1, h);
    if (err != ESP_OK) { free(h); return err; }

    h->enabled = true;
    h->last_cf1_vrms = false;

    // Start in SEL=0 mode (commonly IRMS)
    sel_write(h, 0);

    *out = h;
    return ESP_OK;
}

esp_err_t bl0937_destroy(bl0937_handle_t *h) {
    if (!h) return ESP_ERR_INVALID_ARG;

    gpio_isr_handler_remove(h->cfg.gpio_cf);
    gpio_isr_handler_remove(h->cfg.gpio_cf1);

    free(h);
    return ESP_OK;
}

esp_err_t bl0937_enable(bl0937_handle_t *h, bool enable) {
    if (!h) return ESP_ERR_INVALID_ARG;
    h->enabled = enable;
    return ESP_OK;
}

esp_err_t bl0937_set_cf1_mode(bl0937_handle_t *h, bool vrms) {
    if (!h) return ESP_ERR_INVALID_ARG;

    int sel_level;
    if (h->cfg.sel0_is_irms) {
        sel_level = vrms ? 1 : 0;
    } else {
        sel_level = vrms ? 0 : 1;
    }

    sel_write(h, sel_level);
    h->last_cf1_vrms = vrms;

    // small settling delay
    ets_delay_us(20);
    return ESP_OK;
}

esp_err_t bl0937_reset_counters(bl0937_handle_t *h) {
    if (!h) return ESP_ERR_INVALID_ARG;
    portENTER_CRITICAL(&h->mux);
    h->cf_pulses = 0;
    h->cf1_pulses = 0;
    portEXIT_CRITICAL(&h->mux);
    return ESP_OK;
}

static void read_and_clear(bl0937_handle_t *h, uint32_t *cf, uint32_t *cf1) {
    portENTER_CRITICAL(&h->mux);
    *cf = h->cf_pulses;
    *cf1 = h->cf1_pulses;
    h->cf_pulses = 0;
    h->cf1_pulses = 0;
    portEXIT_CRITICAL(&h->mux);
}

esp_err_t bl0937_sample(bl0937_handle_t *h, uint32_t window_ms, bool cf1_vrms, bl0937_reading_t *out) {
    if (!h || !out || window_ms == 0) return ESP_ERR_INVALID_ARG;
    if (!h->enabled) return ESP_ERR_INVALID_STATE;

    memset(out, 0, sizeof(*out));

    esp_err_t err = bl0937_set_cf1_mode(h, cf1_vrms);
    if (err != ESP_OK) return err;
    bl0937_reset_counters(h);

    int64_t t0 = esp_timer_get_time();
    vTaskDelay(pdMS_TO_TICKS(window_ms));
    int64_t t1 = esp_timer_get_time();

    float dt_s = (float)(t1 - t0) / 1000000.0f;
    if (dt_s <= 0.0f) return ESP_FAIL;

    uint32_t cf_count, cf1_count;
    read_and_clear(h, &cf_count, &cf1_count);

    out->f_cf_hz  = cf_count  / dt_s;
    out->f_cf1_hz = cf1_count / dt_s;

    // Power: Fcf = K * V(V)*V(I) / vref^2  => V(V)*V(I) = Fcf * vref^2 / K
    float vv_vi = (out->f_cf_hz * (h->cfg.vref * h->cfg.vref)) / h->cfg.k_f_power;
    out->power_w = vv_vi * h->cfg.cal_power;

    if (cf1_vrms) {
        float v_v = (out->f_cf1_hz * h->cfg.vref) / h->cfg.k_f_vrms;
        out->voltage_v = v_v * h->cfg.cal_vrms;
    } else {
        float v_i = (out->f_cf1_hz * h->cfg.vref) / h->cfg.k_f_irms;
        out->current_a = v_i * h->cfg.cal_irms;
    }

    // Overcurrent detection + latch
    int64_t now_us = esp_timer_get_time();
    if (h->cfg.overcurrent_hz_threshold > 0.0f && out->f_cf_hz >= h->cfg.overcurrent_hz_threshold) {
        h->overcurrent = true;
        h->overcurrent_until_us = now_us + (int64_t)h->cfg.overcurrent_latch_ms * 1000;
        if (h->cb) h->cb(BL0937_EVENT_OVERCURRENT_ON, h->cb_ctx);
    } else {
        if (h->overcurrent && now_us >= h->overcurrent_until_us) {
            h->overcurrent = false;
            if (h->cb) h->cb(BL0937_EVENT_OVERCURRENT_OFF, h->cb_ctx);
        }
    }
    out->overcurrent = h->overcurrent;

    // Optional policy inside driver: clamp power to 0 during overcurrent
    if (out->overcurrent) {
        out->power_w = 0.0f;
    }

    // EMA filtering
    if (h->cfg.ema_alpha_v > 0.0f && out->voltage_v > 0.0f) {
        h->filt_v = bl0937_ema(h->filt_v, out->voltage_v, h->cfg.ema_alpha_v);
        out->voltage_v = h->filt_v;
    }
    if (h->cfg.ema_alpha_i > 0.0f && out->current_a > 0.0f) {
        h->filt_i = bl0937_ema(h->filt_i, out->current_a, h->cfg.ema_alpha_i);
        out->current_a = h->filt_i;
    }
    if (h->cfg.ema_alpha_p > 0.0f && out->power_w >= 0.0f) {
        h->filt_p = bl0937_ema(h->filt_p, out->power_w, h->cfg.ema_alpha_p);
        out->power_w = h->filt_p;
    }

    return ESP_OK;
}

esp_err_t bl0937_sample_va_w(bl0937_handle_t *h, uint32_t window_ms_per_mode, bl0937_reading_t *out) {
    if (!h || !out || window_ms_per_mode == 0) return ESP_ERR_INVALID_ARG;

    bl0937_reading_t i = {0};
    bl0937_reading_t v = {0};

    esp_err_t err = bl0937_sample(h, window_ms_per_mode, false, &i); // IRMS
    if (err != ESP_OK) return err;
    err = bl0937_sample(h, window_ms_per_mode, true, &v); // VRMS
    if (err != ESP_OK) return err;

    // Compose (power average)
    memset(out, 0, sizeof(*out));
    out->f_cf_hz = (i.f_cf_hz + v.f_cf_hz) * 0.5f;
    out->power_w = (i.power_w + v.power_w) * 0.5f;

    out->current_a = i.current_a;
    out->voltage_v = v.voltage_v;
    out->f_cf1_hz = v.f_cf1_hz;

    out->overcurrent = (i.overcurrent || v.overcurrent);
    return ESP_OK;
}
