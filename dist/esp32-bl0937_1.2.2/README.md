# esp-bl0937

ESP-IDF component for the **BL0937** energy metering IC (CF/CF1/SEL pulse outputs).

## Features

- Pulse counting on **CF** (active power) and **CF1** (IRMS/VRMS selectable via **SEL**)
- Synchronous sampling over a configurable time window
- **EMA low-pass filtering** (optional) for V/A/W
- Dual-window helper to capture **IRMS + VRMS** together for full BL0937 coverage
- **Energy accumulation** in Wh
- **Overcurrent detection** with latch + optional policy helper to cut relay
- **NVS calibration save/load** (per-device keys supported)
- Basic **auto-calibration helper** (guided with reference values)
- Example project included
- Unity tests for helpers

> Note: converting to real-world V/A/W depends on your module front-end (divider/shunt). This component provides calibration multipliers and guided calibration helpers.

## Install (Espressif Component Registry)

ESP-IDF 5.4 (or newer) is required. Add to your project's `idf_component.yml`:

```yml
dependencies:
  AchimPieters/esp-bl0937: "^1.0.0"
```

## Quick start

```c
#include "bl0937.h"
#include "bl0937_nvs.h"
#include "bl0937_nvs_keys.h"

bl0937_handle_t *m = NULL;

bl0937_config_t cfg = {
  .gpio_cf = 4,
  .gpio_cf1 = 5,
  .gpio_sel = 21,

  // Enable internal pulls when your board lacks external biasing on CF/CF1
  .cf_pull_up = true,
  .cf1_pull_up = true,

  .sel0_is_irms = true,  // SEL=0 => IRMS, SEL=1 => VRMS (common)
  .cal_vrms = 1.0f,
  .cal_irms = 1.0f,
  .cal_power = 1.0f,

  .overcurrent_hz_threshold = 6000.0f,
  .overcurrent_latch_ms = 2000,

  // Tune low-pass filters via menuconfig (defaults set in sdkconfig.defaults)
  .ema_alpha_v = CONFIG_BL0937_DEFAULT_EMA_ALPHA_V / 1000.0f,
  .ema_alpha_i = CONFIG_BL0937_DEFAULT_EMA_ALPHA_I / 1000.0f,
  .ema_alpha_p = CONFIG_BL0937_DEFAULT_EMA_ALPHA_P / 1000.0f,
};

// EMA alphas are configured as fixed-point values (alpha * 1000) in menuconfig, e.g.:
// 0.050 -> 50, 0.200 -> 200, 1.000 -> 1000

char key[32];
bl0937_make_cal_key_from_mac(key, sizeof(key));

bl0937_calib_blob_t cal;
if (bl0937_nvs_load("bl0937", key, &cal) == ESP_OK) {
  bl0937_apply_calib(&cfg, &cal);
}

ESP_ERROR_CHECK(bl0937_create(&cfg, &m));

// Multiple listeners can subscribe; the policy helper will not replace existing callbacks
ESP_ERROR_CHECK(bl0937_add_event_listener(m, my_evt_handler, my_ctx));

bl0937_reading_t r;
ESP_ERROR_CHECK(bl0937_sample_va_w(m, CONFIG_BL0937_DEFAULT_SAMPLE_MS, &r));
ESP_LOGI("meter", "V=%.2fV I=%.3fA P=%.2fW E=%.3fWh", r.voltage_v, r.current_a, r.power_w, r.energy_wh);

// Reset accumulated energy/filters when disabling or reusing a handle
ESP_ERROR_CHECK(bl0937_enable(m, false));

// If you need both CF1 modes separately (raw VRMS+IRMS windows), use:
bl0937_dual_reading_t full;
ESP_ERROR_CHECK(bl0937_sample_all(m, 500, &full));
```

## Relay cutoff policy (optional)

This stays outside the driver. Use `bl0937_policy_*` to cut relay on overcurrent and enforce a cooldown hold.

## License

MIT
