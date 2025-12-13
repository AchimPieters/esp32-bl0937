#pragma once
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void bl0937_make_cal_key_from_mac(char *out, size_t out_len);
void bl0937_make_cal_key_from_serial(const char *serial, char *out, size_t out_len);

#ifdef __cplusplus
}
#endif
