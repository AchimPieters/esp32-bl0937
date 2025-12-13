#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "bl0937.h"
#include "bl0937_nvs.h"
#include "bl0937_nvs_keys.h"

static const char *TAG = "example";

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

        .ema_alpha_v = 0.2f,
        .ema_alpha_i = 0.2f,
        .ema_alpha_p = 0.2f,
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

    float energy_wh = 0.0f;

    while (1) {
        bl0937_reading_t r = {0};
        ESP_ERROR_CHECK(bl0937_sample_va_w(m, 500, &r)); // 0.5s IRMS + 0.5s VRMS

        energy_wh = bl0937_integrate_wh(energy_wh, r.power_w, 1.0f);

        ESP_LOGI(TAG, "V=%.2fV I=%.3fA P=%.2fW E=%.3fWh OC=%d (CF=%.1fHz)",
                 r.voltage_v, r.current_a, r.power_w, energy_wh, (int)r.overcurrent, r.f_cf_hz);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
