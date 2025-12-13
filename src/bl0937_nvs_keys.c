#include "bl0937_nvs_keys.h"
#include <stdio.h>
#include <string.h>
#include "esp_system.h"

void bl0937_make_cal_key_from_mac(char *out, size_t out_len) {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(out, out_len, "cal_%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void bl0937_make_cal_key_from_serial(const char *serial, char *out, size_t out_len) {
    if (!serial || !serial[0]) {
        snprintf(out, out_len, "cal_default");
        return;
    }
    char tmp[25] = {0};
    strncpy(tmp, serial, 24);
    snprintf(out, out_len, "cal_%s", tmp);
}
