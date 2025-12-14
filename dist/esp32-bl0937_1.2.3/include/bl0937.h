#pragma once

#ifndef BL0937_MAIN_HEADER
#define BL0937_MAIN_HEADER
#endif

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int gpio_cf;     // CF pulse input
    int gpio_cf1;    // CF1 pulse input
    int gpio_sel;    // SEL output

    bool sel_invert; // invert SEL logic if needed

    // Optional internal pulls for CF/CF1 to avoid floating lines on boards
    // without external bias resistors. Defaults are disabled for backward
    // compatibility.
    bool cf_pull_up;
    bool cf_pull_down;
    bool cf1_pull_up;
    bool cf1_pull_down;

    // BL0937 typical reference voltage (datasheet): 1.218V
    float vref;

    // Datasheet frequency constants (typical)
    float k_f_power; // 1721506
    float k_f_vrms;  // 15397
    float k_f_irms;  // 94638

    // Calibration multipliers to map chip-domain RMS to real world units.
    float cal_vrms;
    float cal_irms;
    float cal_power;

    // Mapping of SEL levels to CF1 function:
    // if true: SEL=0 => IRMS, SEL=1 => VRMS (common)
    // if false: SEL=0 => VRMS, SEL=1 => IRMS
    bool sel0_is_irms;

    // Overcurrent detection: CF can jump to a high fixed-ish frequency in some fault modes.
    // Set threshold=0 to disable.
    float overcurrent_hz_threshold; // e.g. 6000.0f
    uint32_t overcurrent_latch_ms;  // e.g. 2000

    // EMA low-pass filtering (0=off, 1=no filter)
    float ema_alpha_v;
    float ema_alpha_i;
    float ema_alpha_p;

} bl0937_config_t;

typedef struct {
    float f_cf_hz;
    float f_cf1_hz;

    float voltage_v; // last VRMS sample (filtered if enabled)
    float current_a; // last IRMS sample (filtered if enabled)
    float power_w;   // derived from CF (filtered if enabled)
    float energy_wh; // accumulated energy since creation/reset

    bool overcurrent;
} bl0937_reading_t;

typedef struct {
    bl0937_reading_t irms; // CF1=IRMS window
    bl0937_reading_t vrms; // CF1=VRMS window
} bl0937_dual_reading_t;

typedef struct bl0937_handle bl0937_handle_t;

typedef enum {
    BL0937_EVENT_OVERCURRENT_ON = 1,
    BL0937_EVENT_OVERCURRENT_OFF = 2,
} bl0937_event_t;

typedef void (*bl0937_event_cb_t)(bl0937_event_t evt, void *user_ctx);

esp_err_t bl0937_create(const bl0937_config_t *cfg, bl0937_handle_t **out);
esp_err_t bl0937_destroy(bl0937_handle_t *h);

esp_err_t bl0937_enable(bl0937_handle_t *h, bool enable);
esp_err_t bl0937_set_cf1_mode(bl0937_handle_t *h, bool vrms);
esp_err_t bl0937_reset_counters(bl0937_handle_t *h);
esp_err_t bl0937_reset_state(bl0937_handle_t *h);
esp_err_t bl0937_set_energy_wh(bl0937_handle_t *h, float energy_wh);
esp_err_t bl0937_get_energy_wh(bl0937_handle_t *h, float *out_energy_wh);

esp_err_t bl0937_add_event_listener(bl0937_handle_t *h, bl0937_event_cb_t cb, void *user_ctx);
esp_err_t bl0937_remove_event_listener(bl0937_handle_t *h, bl0937_event_cb_t cb, void *user_ctx);

// Backward-compatible: replaces all existing listeners with the provided one.
esp_err_t bl0937_set_event_cb(bl0937_handle_t *h, bl0937_event_cb_t cb, void *user_ctx);

esp_err_t bl0937_sample(bl0937_handle_t *h, uint32_t window_ms, bool cf1_vrms, bl0937_reading_t *out);
esp_err_t bl0937_sample_all(bl0937_handle_t *h, uint32_t window_ms_per_mode, bl0937_dual_reading_t *out);

// Convenience: sample IRMS then VRMS (and compute W) using SEL toggling.
// Total time ~= 2*window_ms_per_mode.
esp_err_t bl0937_sample_va_w(bl0937_handle_t *h, uint32_t window_ms_per_mode, bl0937_reading_t *out);

static inline float bl0937_integrate_wh(float energy_wh, float power_w, float dt_seconds) {
    return energy_wh + (power_w * (dt_seconds / 3600.0f));
}

#include "bl0937_nvs.h"
#include "bl0937_nvs_keys.h"

#ifdef __cplusplus
}
#endif
