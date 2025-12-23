#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <driver/gpio.h>
#include <esp_err.h>

typedef struct {
        gpio_num_t cf_gpio;
        gpio_num_t cf1_gpio;
        gpio_num_t sel_gpio;
        bool sel_inverted;
        float voltage_calibration;
        float current_calibration;
        float power_calibration;
        float power_max_watts;
        float frequency_hz;
        uint32_t sample_period_ms;
} bl0937_config_t;

typedef struct {
        float voltage;
        float current;
        float power;
        float energy;
        float power_factor;
        float frequency;
        float total_consumption;
} bl0937_measurements_t;

typedef void (*bl0937_measurement_cb_t)(const bl0937_measurements_t *measurements,
                                        void *context);

esp_err_t bl0937_start(const bl0937_config_t *config,
                       bl0937_measurement_cb_t callback,
                       void *context);
esp_err_t bl0937_start_default(void);
void bl0937_stop(void);
