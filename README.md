# esp-bl0937

ESP-IDF component for the **BL0937** energy metering IC (CF/CF1/SEL pulse outputs).

## Features

- Pulse counting on **CF** (active power) and **CF1** (IRMS/VRMS selectable via **SEL**)
- Synchronous sampling over a configurable time window
- **EMA low-pass filtering** (optional) for V/A/W
- **Overcurrent detection** with latch + optional policy helper to cut relay
- **NVS calibration save/load** (per-device keys supported)
- Basic **auto-calibration helper** (guided with reference values)
- Example project included
- Unity tests for helpers

> Note: converting to real-world V/A/W depends on your module front-end (divider/shunt). This component provides calibration multipliers and guided calibration helpers.

## Install (Espressif Component Registry)

Add to your project's `idf_component.yml`:

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

  .sel0_is_irms = true,  // SEL=0 => IRMS, SEL=1 => VRMS (common)
  .cal_vrms = 1.0f,
  .cal_irms = 1.0f,
  .cal_power = 1.0f,

  .overcurrent_hz_threshold = 6000.0f,
  .overcurrent_latch_ms = 2000,

  .ema_alpha_v = 0.2f,
  .ema_alpha_i = 0.2f,
  .ema_alpha_p = 0.2f,
};

char key[32];
bl0937_make_cal_key_from_mac(key, sizeof(key));

bl0937_calib_blob_t cal;
if (bl0937_nvs_load("bl0937", key, &cal) == ESP_OK) {
  bl0937_apply_calib(&cfg, &cal);
}

ESP_ERROR_CHECK(bl0937_create(&cfg, &m));

bl0937_reading_t r;
ESP_ERROR_CHECK(bl0937_sample_va_w(m, 500, &r)); // 0.5s IRMS + 0.5s VRMS
```

## Relay cutoff policy (optional)

This stays outside the driver. Use `bl0937_policy_*` to cut relay on overcurrent and enforce a cooldown hold.

## License

MIT
