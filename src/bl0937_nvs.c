#include "bl0937_nvs.h"
#include <string.h>
#include "nvs.h"
#include "esp_crc.h"

static uint32_t blob_crc(const bl0937_calib_blob_t *b) {
    bl0937_calib_blob_t tmp = *b;
    tmp.crc32 = 0;
    return esp_crc32_le(0, (const uint8_t *)&tmp, sizeof(tmp));
}

esp_err_t bl0937_nvs_load(const char *nvs_namespace, const char *key, bl0937_calib_blob_t *out) {
    if (!nvs_namespace || !key || !out) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = nvs_open(nvs_namespace, NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    size_t len = sizeof(*out);
    err = nvs_get_blob(h, key, out, &len);
    nvs_close(h);

    if (err != ESP_OK) return err;
    if (len != sizeof(*out)) return ESP_ERR_INVALID_SIZE;

    uint32_t expected = out->crc32;
    uint32_t actual = blob_crc(out);
    if (expected != actual) return ESP_ERR_INVALID_CRC;

    return ESP_OK;
}

esp_err_t bl0937_nvs_save(const char *nvs_namespace, const char *key, const bl0937_calib_blob_t *in) {
    if (!nvs_namespace || !key || !in) return ESP_ERR_INVALID_ARG;

    bl0937_calib_blob_t blob = *in;
    blob.crc32 = 0;
    blob.crc32 = blob_crc(&blob);

    nvs_handle_t h;
    esp_err_t err = nvs_open(nvs_namespace, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(h, key, &blob, sizeof(blob));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}
