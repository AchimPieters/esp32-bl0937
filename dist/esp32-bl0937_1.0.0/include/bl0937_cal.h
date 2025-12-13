#pragma once
#include "esp_err.h"
#include "bl0937.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float ref_voltage_v;  // 0 => skip voltage cal
    float ref_current_a;  // 0 => skip current cal
    float ref_power_w;    // 0 => skip power cal
} bl0937_cal_ref_t;

esp_err_t bl0937_calculate_calibration(const bl0937_reading_t *sample,
                                       const bl0937_cal_ref_t *ref,
                                       bl0937_config_t *cfg_in_out);

#ifdef __cplusplus
}
#endif
