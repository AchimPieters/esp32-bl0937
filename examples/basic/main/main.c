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

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "bl0937.h"
#include "bl0937_policy.h"

static const char *TAG = "example";

static inline float bl0937_alpha_from_kconfig(int v1000) {
    if (v1000 < 0) v1000 = 0;
    if (v1000 > 1000) v1000 = 1000;
    return ((float)v1000) / 1000.0f;
}

static void relay_ctl(bool relay_on, void *ctx) {
    (void)ctx;
    ESP_LOGI(TAG, "Policy requests relay %s", relay_on ? "ON" : "OFF");
}

void app_main(void) {
    // NVS init (required for bl0937_nvs_*). If your app already does it, you can remove this.
    ESP_ERROR_CHECK(nvs_flash_init());

    bl0937_handle_t *m = NULL;

    bl0937_config_t cfg = {
        .gpio_cf = 4,
        .gpio_cf1 = 5,
        .gpio_sel = 21,
        .sel0_is_irms = true,

        .cal_vrms = 1.0f,
        .cal_irms = 1.0f,
        .cal_power = 1.0f,

        .overcurrent_hz_threshold = 6000.0f,
        .overcurrent_latch_ms = 2000,

        // EMA filters tuned for stability; tweak in sdkconfig.defaults
        .ema_alpha_v = bl0937_alpha_from_kconfig(CONFIG_BL0937_DEFAULT_EMA_ALPHA_V),
        .ema_alpha_i = bl0937_alpha_from_kconfig(CONFIG_BL0937_DEFAULT_EMA_ALPHA_I),
        .ema_alpha_p = bl0937_alpha_from_kconfig(CONFIG_BL0937_DEFAULT_EMA_ALPHA_P),
    };

    char key[32];
    bl0937_make_cal_key_from_mac(key, sizeof(key));

    bl0937_calib_blob_t cal;
    if (bl0937_nvs_load("bl0937", key, &cal) == ESP_OK) {
        bl0937_apply_calib(&cfg, &cal);
        ESP_LOGI(TAG, "Loaded calibration from NVS key=%s", key);
    } else {
        ESP_LOGW(TAG, "No calibration in NVS for key=%s (using defaults)", key);
    }

    ESP_ERROR_CHECK(bl0937_create(&cfg, &m));

    // Optional relay cutoff policy: cut for a hold-off after overcurrent
    bl0937_policy_config_t pcfg = {
        .enable_relay_cutoff = true,
        .cutoff_hold_ms = 5000,
        .auto_clear_after_hold = false,
    };
    bl0937_policy_handle_t *policy = NULL;
    ESP_ERROR_CHECK(bl0937_policy_create(m, &pcfg, relay_ctl, NULL, &policy));

    while (1) {
        bl0937_reading_t r = {0};
        // Dual-window sample: IRMS + VRMS back-to-back for full coverage
        ESP_ERROR_CHECK(bl0937_sample_va_w(m, CONFIG_BL0937_DEFAULT_SAMPLE_MS, &r));

        bl0937_policy_tick(policy);
        bool relay_allowed = bl0937_policy_allows_relay_on(policy);

        ESP_LOGI(TAG, "V=%.2fV I=%.3fA P=%.2fW E=%.3fWh OC=%d RelayOK=%d (CF=%.1fHz)",
                 r.voltage_v, r.current_a, r.power_w, r.energy_wh, (int)r.overcurrent,
                 relay_allowed, r.f_cf_hz);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
