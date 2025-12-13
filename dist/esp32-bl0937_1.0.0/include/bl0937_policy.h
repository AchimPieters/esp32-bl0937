#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "bl0937.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*bl0937_policy_relay_fn_t)(bool relay_on, void *user_ctx);

typedef struct {
    bool enable_relay_cutoff;      // default true
    uint32_t cutoff_hold_ms;       // minimum hold-off after OC (e.g. 5000)
    bool auto_clear_after_hold;    // reserved for future behavior
} bl0937_policy_config_t;

typedef struct bl0937_policy_handle bl0937_policy_handle_t;

esp_err_t bl0937_policy_create(bl0937_handle_t *meter,
                               const bl0937_policy_config_t *cfg,
                               bl0937_policy_relay_fn_t relay_fn,
                               void *user_ctx,
                               bl0937_policy_handle_t **out);

esp_err_t bl0937_policy_destroy(bl0937_policy_handle_t *p);

bool bl0937_policy_allows_relay_on(bl0937_policy_handle_t *p);
void bl0937_policy_tick(bl0937_policy_handle_t *p);

#ifdef __cplusplus
}
#endif
