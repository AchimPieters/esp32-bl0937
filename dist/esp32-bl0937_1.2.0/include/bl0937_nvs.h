#pragma once
#include "esp_err.h"
#include "bl0937.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t version;
    uint8_t reserved[3];
    float cal_vrms;
    float cal_irms;
    float cal_power;
    uint32_t crc32;
} bl0937_calib_blob_t;

#define BL0937_CALIB_BLOB_VERSION 1

esp_err_t bl0937_nvs_load(const char *nvs_namespace, const char *key, bl0937_calib_blob_t *out);
esp_err_t bl0937_nvs_save(const char *nvs_namespace, const char *key, const bl0937_calib_blob_t *in);

static inline void bl0937_apply_calib(bl0937_config_t *cfg, const bl0937_calib_blob_t *b) {
    cfg->cal_vrms  = b->cal_vrms;
    cfg->cal_irms  = b->cal_irms;
    cfg->cal_power = b->cal_power;
}

#ifdef __cplusplus
}
#endif
