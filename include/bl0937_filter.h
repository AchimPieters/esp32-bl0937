#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

static inline float bl0937_ema(float prev, float x, float alpha) {
    if (alpha <= 0.0f) return prev;
    if (alpha >= 1.0f) return x;
    if (isnan(prev) || isinf(prev)) return x;
    return prev + alpha * (x - prev);
}

// Notify only if difference exceeds deadband OR max_period elapsed
static inline bool bl0937_should_notify(float prev, float now, float deadband,
                                       uint32_t now_ms, uint32_t *last_notify_ms,
                                       uint32_t max_period_ms) {
    if (!last_notify_ms) return true;

    if (fabsf(now - prev) >= deadband) {
        *last_notify_ms = now_ms;
        return true;
    }
    if (max_period_ms > 0 && (now_ms - *last_notify_ms) >= max_period_ms) {
        *last_notify_ms = now_ms;
        return true;
    }
    return false;
}
