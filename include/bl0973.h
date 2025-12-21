#pragma once
/**
 * @file bl0973.h
 * @brief BL09x7 (BL0937 / BL0973-style) CF/CF1/SEL pulse energy meter component for ESP-IDF.
 *
 * This component is designed to be "drop-in": include this header and call the getters.
 * It will auto-initialize on first use using Kconfig defaults.
 *
 * NOTE:
 * - This implementation follows the BL0937 datasheet behavior (CF: active power pulses, CF1: IRMS/VRMS via SEL).
 * - If your BL0973 variant differs electrically, adjust Kconfig settings and calibration.
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // GPIOs
    int gpio_cf;
    int gpio_cf1;
    int gpio_sel;
    bool sel_active_high;

    // optional relay cutoff
    bool relay_enable;
    int gpio_relay;
    bool relay_active_high;

    // Measurement timing
    uint32_t update_period_ms;
    bool toggle_sel_every_update;
    uint32_t sel_stable_us;

    // Filter
    uint16_t ema_alpha_permille;

    // Front-end / theoretical scaling parameters
    uint16_t vref_mv;
    uint32_t shunt_uohm;

    // Voltage divider: Vinput = Vmains * divider_num/divider_den
    uint32_t divider_num;
    uint32_t divider_den;

    // Calibration
    uint16_t cal_voltage_permille;
    uint16_t cal_current_permille;
    uint16_t cal_power_permille;
    int32_t  cal_voltage_offset_mv;
    int32_t  cal_current_offset_ma;

    // Protection
    uint32_t overcurrent_threshold_ma;
    uint32_t overcurrent_debounce_ms;
    uint32_t overcurrent_cooldown_s;

    // Optional HW OC detect
    bool hw_oc_detect_enable;
    uint32_t hw_oc_freq_hz;
    uint8_t hw_oc_tol_pct;
} bl0973_config_t;

typedef struct {
    // Latest measurements (filtered)
    float voltage_v;
    float current_a;
    float power_w;
    float energy_wh;
    float power_factor;

    // Raw pulse frequencies
    float freq_cf_hz;     // active power or hardware OC pulses
    float freq_vrms_hz;   // from CF1 when SEL=1
    float freq_irms_hz;   // from CF1 when SEL=0

    // Status
    bool  initialized;
    bool  sel_is_voltage;         // current SEL state (true => VRMS on CF1)
    bool  overcurrent_tripped;    // software protection active
    bool  hw_overcurrent;         // hardware OC pulse detected on CF
    bool  relay_enabled;
    bool  relay_output_on;
    int64_t trip_until_us;        // 0 if not tripped
} bl0973_status_t;


typedef enum {
    BL0973_EVENT_OVERCURRENT_TRIP = 1,     /*!< Software overcurrent trip (threshold/debounce) */
    BL0973_EVENT_OVERCURRENT_RECOVER = 2,  /*!< Overcurrent cooldown ended, relay allowed again */
    BL0973_EVENT_HW_OVERCURRENT = 3,       /*!< Hardware OC pulse detected on CF (e.g. ~6.7kHz) */
} bl0973_event_t;

typedef void (*bl0973_event_cb_t)(bl0973_event_t event, void *user_ctx);

/**
 * @brief Register an optional event callback (called from ESP timer task context).
 * Passing NULL disables callbacks.
 */
void bl0973_set_event_callback(bl0973_event_cb_t cb, void *user_ctx);

/** Get apparent power (VA) estimate = V * I. */
float bl0973_apparent_power_va(void);

/** Get reactive power (var) estimate = sqrt((V*I)^2 - P^2). Always non-negative. */
float bl0973_reactive_power_var(void);

/** Get raw CF pulse frequency (Hz) of the last update window. */
float bl0973_freq_cf_hz(void);

/** Get raw CF1 VRMS pulse frequency (Hz) (filtered). */
float bl0973_freq_vrms_hz(void);

/** Get raw CF1 IRMS pulse frequency (Hz) (filtered). */
float bl0973_freq_irms_hz(void);

/** Convenience: true if voltage/current/power are currently valid (signal present). */
bool bl0973_is_signal_ok(void);

/**
 * @brief Initialize driver with explicit configuration.
 * If you prefer auto-init, you can skip this and call any getter.
 */
esp_err_t bl0973_init(const bl0973_config_t *cfg);

/**
 * @brief Initialize driver using Kconfig defaults.
 */
esp_err_t bl0973_init_default(void);

/**
 * @brief Deinitialize (stop timers/ISRs). Safe to call even if not initialized.
 */
esp_err_t bl0973_deinit(void);


/**
 * @brief Start measuring (enable interrupts + start periodic timer).
 * Safe to call multiple times.
 */
esp_err_t bl0973_start(void);

/**
 * @brief Stop measuring (stop timer + disable interrupts). Energy counter is preserved in RAM.
 * Safe to call multiple times.
 */
esp_err_t bl0973_stop(void);

/** @return true if measuring is currently running. */
bool bl0973_is_running(void);


/** Get latest voltage (V RMS). */
float bl0973_voltage(void);

/** Get latest current (A RMS). */
float bl0973_current(void);

/** Get latest active power (W). */
float bl0973_power(void);

/** Get accumulated energy (Wh). */
float bl0973_energy_wh(void);

/** Get power factor estimate (P / (V*I)). */
float bl0973_power_factor(void);

/** Reset energy accumulator to 0. */
void bl0973_energy_reset(void);

/** Fetch a snapshot status (thread-safe). */
void bl0973_get_status(bl0973_status_t *out_status);

/**
 * @brief Force relay output (if enabled). If overcurrent is tripped, relay stays off.
 * @return true if command applied, false if relay not enabled.
 */
bool bl0973_relay_set(bool on);

/** @return true if relay output is currently on (if enabled). */
bool bl0973_relay_get(void);

/** @return true if currently in overcurrent trip/cooldown. */
bool bl0973_is_tripped(void);

/**
 * @brief Update calibration at runtime (permille factors + offsets).
 * If NVS calibration is enabled, you can also save/restore via the functions below.
 */
esp_err_t bl0973_set_calibration(uint16_t v_permille, uint16_t i_permille, uint16_t p_permille,
                                 int32_t v_offset_mv, int32_t i_offset_ma);

#ifdef CONFIG_BL0973_ENABLE_NVS_ENERGY
/** Force an immediate save of the energy pulse counter to NVS (best-effort). */
esp_err_t bl0973_energy_persist_now(void);
/** Load persisted energy pulse counter from NVS (best-effort). */
esp_err_t bl0973_energy_restore(void);
#endif

#ifdef CONFIG_BL0973_ENABLE_NVS_CALIBRATION
esp_err_t bl0973_calibration_save(void);
esp_err_t bl0973_calibration_load(void);
#endif

#ifdef __cplusplus
}
#endif
