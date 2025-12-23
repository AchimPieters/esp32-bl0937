#include "bl0937.h"

#include <stdlib.h>

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "custom_characteristics.h"
#include "sdkconfig.h"

#define BL0937_TAG "BL0937"
#define ENERGY_TAG "ENERGY"

#ifndef CONFIG_ESP_BL0937_SEL_INVERTED
#define CONFIG_ESP_BL0937_SEL_INVERTED 0
#endif

typedef struct {
        bl0937_config_t config;
        bl0937_measurement_cb_t callback;
        void *context;
        TaskHandle_t task_handle;
        volatile uint32_t cf_pulses;
        volatile uint32_t cf1_pulses;
        bool measure_voltage;
        float last_voltage;
        float last_current;
        float total_consumption_kwh;
} bl0937_state_t;

static bl0937_state_t g_state;
static portMUX_TYPE g_bl0937_mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR bl0937_cf_isr(void *arg) {
        bl0937_state_t *state = (bl0937_state_t *)arg;
        state->cf_pulses++;
}

static void IRAM_ATTR bl0937_cf1_isr(void *arg) {
        bl0937_state_t *state = (bl0937_state_t *)arg;
        state->cf1_pulses++;
}

static void bl0937_set_sel(bool measure_voltage) {
        bool sel_level = measure_voltage;
        if (g_state.config.sel_inverted) {
                sel_level = !sel_level;
        }
        gpio_set_level(g_state.config.sel_gpio, sel_level ? 1 : 0);
}

static float bl0937_safe_divide(float numerator, float denominator) {
        if (denominator <= 0.0f) {
                return 0.0f;
        }
        return numerator / denominator;
}

static void bl0937_task(void *arg) {
        bl0937_state_t *state = (bl0937_state_t *)arg;
        const TickType_t period = pdMS_TO_TICKS(state->config.sample_period_ms);
        const float sample_hours = state->config.sample_period_ms / 3600000.0f;

        state->measure_voltage = true;
        bl0937_set_sel(state->measure_voltage);

        while (true) {
                vTaskDelay(period);

                uint32_t cf_count;
                uint32_t cf1_count;

                portENTER_CRITICAL(&g_bl0937_mux);
                cf_count = state->cf_pulses;
                cf1_count = state->cf1_pulses;
                state->cf_pulses = 0;
                state->cf1_pulses = 0;
                portEXIT_CRITICAL(&g_bl0937_mux);

                float power = bl0937_safe_divide((float)cf_count,
                                                 state->config.power_calibration);

                if (state->config.power_max_watts > 0.0f &&
                    power > state->config.power_max_watts) {
                        power = 0.0f;
                }

                if (state->measure_voltage) {
                        state->last_voltage = bl0937_safe_divide(
                                (float)cf1_count,
                                state->config.voltage_calibration);
                } else {
                        state->last_current = bl0937_safe_divide(
                                (float)cf1_count,
                                state->config.current_calibration);
                }

                state->measure_voltage = !state->measure_voltage;
                bl0937_set_sel(state->measure_voltage);

                float energy = power * sample_hours;
                state->total_consumption_kwh += energy / 1000.0f;

                float power_factor = 0.0f;
                if (state->last_voltage > 0.0f && state->last_current > 0.0f) {
                        power_factor = power / (state->last_voltage * state->last_current);
                        if (power_factor > 1.0f) {
                                power_factor = 1.0f;
                        } else if (power_factor < 0.0f) {
                                power_factor = 0.0f;
                        }
                }

                if (state->callback) {
                        bl0937_measurements_t measurements = {
                                .voltage = state->last_voltage,
                                .current = state->last_current,
                                .power = power,
                                .energy = energy,
                                .power_factor = power_factor,
                                .frequency = state->config.frequency_hz,
                                .total_consumption = state->total_consumption_kwh,
                        };
                        state->callback(&measurements, state->context);
                }
        }
}

static void bl0937_measurements_callback(const bl0937_measurements_t *measurements,
                                         void *context) {
        if (!measurements) {
                return;
        }

        custom_characteristics_update(measurements->voltage,
                                      measurements->current,
                                      measurements->power,
                                      measurements->power_factor,
                                      measurements->frequency,
                                      measurements->total_consumption);

        ESP_LOGD(ENERGY_TAG,
                 "V=%.2fV I=%.3fA P=%.2fW E=%.4fWh PF=%.3f F=%.2fHz Tot=%.4fkWh",
                 measurements->voltage,
                 measurements->current,
                 measurements->power,
                 measurements->energy,
                 measurements->power_factor,
                 measurements->frequency,
                 measurements->total_consumption);

        (void)context;
}

esp_err_t bl0937_start(const bl0937_config_t *config,
                       bl0937_measurement_cb_t callback,
                       void *context) {
        if (!config) {
                return ESP_ERR_INVALID_ARG;
        }

        g_state = (bl0937_state_t) {
                .config = *config,
                .callback = callback,
                .context = context,
                .task_handle = NULL,
                .cf_pulses = 0,
                .cf1_pulses = 0,
                .measure_voltage = true,
                .last_voltage = 0.0f,
                .last_current = 0.0f,
                .total_consumption_kwh = 0.0f,
        };

        gpio_config_t sel_cfg = {
                .pin_bit_mask = 1ULL << g_state.config.sel_gpio,
                .mode = GPIO_MODE_OUTPUT,
                .pull_up_en = GPIO_PULLUP_DISABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&sel_cfg));

        gpio_config_t input_cfg = {
                .pin_bit_mask = (1ULL << g_state.config.cf_gpio) |
                                (1ULL << g_state.config.cf1_gpio),
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = GPIO_PULLUP_ENABLE,
                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                .intr_type = GPIO_INTR_POSEDGE,
        };
        ESP_ERROR_CHECK(gpio_config(&input_cfg));

        esp_err_t isr_status = gpio_install_isr_service(0);
        if (isr_status != ESP_OK && isr_status != ESP_ERR_INVALID_STATE) {
                ESP_LOGE(BL0937_TAG, "Failed to install ISR service: %s",
                         esp_err_to_name(isr_status));
                return isr_status;
        }

        ESP_ERROR_CHECK(gpio_isr_handler_add(g_state.config.cf_gpio,
                                             bl0937_cf_isr, &g_state));
        ESP_ERROR_CHECK(gpio_isr_handler_add(g_state.config.cf1_gpio,
                                             bl0937_cf1_isr, &g_state));

        if (g_state.config.sample_period_ms == 0) {
                g_state.config.sample_period_ms = 1000;
        }

        if (xTaskCreate(bl0937_task, "bl0937", 4096, &g_state, 5,
                        &g_state.task_handle) != pdPASS) {
                ESP_LOGE(BL0937_TAG, "Failed to create BL0937 task");
                return ESP_ERR_NO_MEM;
        }

        return ESP_OK;
}

esp_err_t bl0937_start_default(void) {
        bl0937_config_t config = {
                .cf_gpio = CONFIG_ESP_BL0937_CF_GPIO,
                .cf1_gpio = CONFIG_ESP_BL0937_CF1_GPIO,
                .sel_gpio = CONFIG_ESP_BL0937_SEL_GPIO,
                .sel_inverted = CONFIG_ESP_BL0937_SEL_INVERTED,
                .voltage_calibration = strtof(CONFIG_ESP_BL0937_VOLTAGE_CAL, NULL),
                .current_calibration = strtof(CONFIG_ESP_BL0937_CURRENT_CAL, NULL),
                .power_calibration = strtof(CONFIG_ESP_BL0937_POWER_CAL, NULL),
                .power_max_watts = strtof(CONFIG_ESP_BL0937_POWER_MAX, NULL),
                .frequency_hz = strtof(CONFIG_ESP_BL0937_FREQUENCY_HZ, NULL),
                .sample_period_ms = CONFIG_ESP_BL0937_SAMPLE_PERIOD_MS,
        };

        esp_err_t bl0937_err = bl0937_start(&config,
                                            bl0937_measurements_callback,
                                            NULL);
        if (bl0937_err != ESP_OK) {
                ESP_LOGE(ENERGY_TAG, "Failed to start BL0937: %s",
                         esp_err_to_name(bl0937_err));
        }

        return bl0937_err;
}

void bl0937_stop(void) {
        if (g_state.task_handle) {
                vTaskDelete(g_state.task_handle);
                g_state.task_handle = NULL;
        }

        gpio_isr_handler_remove(g_state.config.cf_gpio);
        gpio_isr_handler_remove(g_state.config.cf1_gpio);
}
