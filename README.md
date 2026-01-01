# BL0937 (ESP-IDF Component)

ESP-IDF component for the **BL0937** energy metering IC.

The BL0937 provides pulse outputs:

- **CF**: active power pulse output
- **CF1**: multiplexed voltage/current pulse output (selected by **SEL**)

This driver counts pulses (PCNT if available; optional GPIO ISR fallback) and converts them into:

- Voltage (V)
- Current (A)
- Power (W)
- Accumulated energy (Wh)

---

## Compatibility

- ESP-IDF: **v5.4+**
- Targets: esp32 / esp32c2 / esp32c3 / esp32s2 / esp32s3

---

## Configuration (Kconfig)

Menu: **"BL0937 Energy Monitor"**

Key settings:

- `ESP_BL0937_CF_GPIO`, `ESP_BL0937_CF1_GPIO`, `ESP_BL0937_SEL_GPIO`
- `BL0937_SAMPLE_PERIOD_MS`
- Board parameters used for unit conversions:
  - `BL0937_VREF_V`
  - `BL0937_SHUNT_UOHM`
  - `BL0937_V_DIV_RATIO`
- Optional calibration multipliers:
  - `BL0937_VOLTAGE_CAL`
  - `BL0937_CURRENT_CAL`
  - `BL0937_POWER_CAL`

---

## Public API

Header: `bl0937.h`

Main types:

- `bl0937_config_t`
- `bl0937_measurements_t`

Main functions:

- `bl0937_config_default()`
- `bl0937_init()` / `bl0937_deinit()`
- `bl0937_start()` / `bl0937_stop()`
- `bl0937_get()`
- `bl0937_reset_energy()`

---

## Minimal Example

```c
#include "bl0937.h"

void app_main(void)
{
    bl0937_config_t cfg = bl0937_config_default();

    ESP_ERROR_CHECK(bl0937_init(&cfg));
    ESP_ERROR_CHECK(bl0937_start());

    while (1) {
        bl0937_measurements_t m = bl0937_get();

        // Only use voltage/current when marked valid.
        float v = m.valid_voltage ? m.voltage_v : 0.0f;
        float i = m.valid_current ? m.current_a : 0.0f;

        printf("V=%.1fV I=%.3fA P=%.0fW E=%.2fWh\n", v, i, m.power_w, m.energy_wh);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

---

## Notes

- `BL0937_SAMPLE_PERIOD_MS` defines a full cycle: the driver measures **voltage** on CF1 for half the period and **current** for the other half.
- PCNT glitch filtering can be configured via `BL0937_GLITCH_FILTER_NS`.
- The driver exposes raw frequencies in `bl0937_measurements_t` for debugging or custom conversions.

---

## License

MIT (see `LICENSE`).
