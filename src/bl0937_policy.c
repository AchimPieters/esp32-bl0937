#include "bl0937_policy.h"
#include "esp_timer.h"
#include <stdlib.h>

struct bl0937_policy_handle {
    bl0937_handle_t *meter;
    bl0937_policy_config_t cfg;
    bl0937_policy_relay_fn_t relay_fn;
    void *user_ctx;

    bool hold_active;
    int64_t hold_until_us;
};

static void meter_evt(bl0937_event_t evt, void *ctx) {
    bl0937_policy_handle_t *p = (bl0937_policy_handle_t *)ctx;
    if (!p || !p->cfg.enable_relay_cutoff) return;

    if (evt == BL0937_EVENT_OVERCURRENT_ON) {
        p->hold_active = true;
        p->hold_until_us = esp_timer_get_time() + (int64_t)p->cfg.cutoff_hold_ms * 1000;

        if (p->relay_fn) {
            p->relay_fn(false, p->user_ctx); // Force relay OFF
        }
    }
}

esp_err_t bl0937_policy_create(bl0937_handle_t *meter,
                               const bl0937_policy_config_t *cfg,
                               bl0937_policy_relay_fn_t relay_fn,
                               void *user_ctx,
                               bl0937_policy_handle_t **out) {
    if (!meter || !cfg || !out) return ESP_ERR_INVALID_ARG;

    bl0937_policy_handle_t *p = calloc(1, sizeof(*p));
    if (!p) return ESP_ERR_NO_MEM;

    p->meter = meter;
    p->cfg = *cfg;
    if (p->cfg.cutoff_hold_ms == 0) p->cfg.cutoff_hold_ms = 5000;

    p->relay_fn = relay_fn;
    p->user_ctx = user_ctx;

    bl0937_set_event_cb(meter, meter_evt, p);

    *out = p;
    return ESP_OK;
}

esp_err_t bl0937_policy_destroy(bl0937_policy_handle_t *p) {
    if (!p) return ESP_ERR_INVALID_ARG;
    bl0937_set_event_cb(p->meter, NULL, NULL);
    free(p);
    return ESP_OK;
}

bool bl0937_policy_allows_relay_on(bl0937_policy_handle_t *p) {
    if (!p) return true;
    if (!p->hold_active) return true;
    if (esp_timer_get_time() >= p->hold_until_us) return true;
    return false;
}

void bl0937_policy_tick(bl0937_policy_handle_t *p) {
    if (!p) return;
    if (p->hold_active && esp_timer_get_time() >= p->hold_until_us) {
        p->hold_active = false;
    }
}
