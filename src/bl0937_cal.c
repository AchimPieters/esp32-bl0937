#include "bl0937_cal.h"

esp_err_t bl0937_calculate_calibration(const bl0937_reading_t *s,
                                       const bl0937_cal_ref_t *r,
                                       bl0937_config_t *cfg) {
    if (!s || !r || !cfg) return ESP_ERR_INVALID_ARG;

    if (r->ref_voltage_v > 0.0f && s->voltage_v > 0.0f) {
        cfg->cal_vrms *= (r->ref_voltage_v / s->voltage_v);
    }
    if (r->ref_current_a > 0.0f && s->current_a > 0.0f) {
        cfg->cal_irms *= (r->ref_current_a / s->current_a);
    }
    if (r->ref_power_w > 0.0f && s->power_w > 0.0f) {
        cfg->cal_power *= (r->ref_power_w / s->power_w);
    }
    return ESP_OK;
}
