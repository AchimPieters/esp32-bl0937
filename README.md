# ESP-IDF BL0937 Component

ESP-IDF component for the BL0937 energy metering IC. The driver uses pulse counting
from the CF/CF1 outputs to report voltage, current, power, and accumulated energy.

## Features

- Pulse-based measurement task with configurable sample period
- Calibration constants for voltage, current, and power
- Optional power limit check
- Callback-based reporting

## Usage

Add the component to your project and call `bl0937_start()` with your configuration
or `bl0937_start_default()` to use Kconfig defaults.

```c
#include "bl0937.h"

static void bl0937_cb(const bl0937_measurements_t *measurements, void *context) {
    // Use measurements->voltage/current/power/etc.
    (void)context;
}

void app_main(void) {
    bl0937_config_t config = {
        .cf_gpio = GPIO_NUM_25,
        .cf1_gpio = GPIO_NUM_26,
        .sel_gpio = GPIO_NUM_27,
        .sel_inverted = false,
        .voltage_calibration = 1.0f,
        .current_calibration = 1.0f,
        .power_calibration = 1.0f,
        .power_max_watts = 0.0f,
        .frequency_hz = 50.0f,
        .sample_period_ms = 1000,
    };

    bl0937_start(&config, bl0937_cb, NULL);
}
```

## Configuration

Kconfig options are available under **Component config → BL0937 Energy Monitor**:

- CF/CF1/SEL GPIOs
- SEL inversion
- Calibration constants
- Mains frequency
- Sample period
- Optional maximum power limit

## License

MIT.
