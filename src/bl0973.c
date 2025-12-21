#include "bl0937.h"

#include <math.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "driver/gpio.h"

#if defined(CONFIG_BL0937_ENABLE_NVS_CALIBRATION) || defined(CONFIG_BL0937_ENABLE_NVS_ENERGY)
#include "nvs.h"
#include "nvs_flash.h"
#endif

#ifdef CONFIG_BL0937_ENABLE_NVS_CALIBRATION
#endif

#ifndef CONFIG_BL0937_ENABLE
#warning "BL0937 component compiled but CONFIG_BL0937_ENABLE is not set"
#endif

static const char *TAG = "bl0937";

/* Datasheet typical constants (BL0937):
 * - Active power pulse frequency: F_CF = 1721506 * Vv * Vi / Vref^2
 * - Vrms pulse frequency:         F_V  = 15397   * Vv / Vref
 * - Irms pulse frequency:         F_I  = 94638   * Vi / Vref
 */
#define BL0937_KP  1721506.0f
#define BL0937_KV  15397.0f
#define BL0937_KI  94638.0f

typedef struct {
    bl0937_event_cb_t event_cb;
    void *event_user;

    bl0937_config_t cfg;

    // ISR pulse counters for current window
    volatile uint32_t cf_cnt;
    volatile uint32_t cf1_cnt;

    // Total energy counter in pulses (CF)
    volatile uint64_t cf_total_pulses;

    // current SEL state (true => SEL=1 => VRMS on CF1)
    bool sel_is_voltage;

    // validity tracking
    int64_t last_v_update_us;
    int64_t last_i_update_us;
    int64_t last_p_update_us;
    bool valid_v;
    bool valid_i;
    bool valid_p;

    // energy persistence
#ifdef CONFIG_BL0937_ENABLE_NVS_ENERGY
    int64_t last_energy_save_us;
#endif

    // computed raw frequencies
    float freq_cf_hz;
    float freq_v_hz;
    float freq_i_hz;

    // filtered values
    float voltage_v;
    float current_a;
    float power_w;
    float energy_wh;
    float pf;
    float apparent_va;
    float reactive_var;

    // protection
    bool overcurrent_tripped;
    bool hw_overcurrent;
    int64_t oc_since_us;      // since current exceeded threshold
    int64_t trip_until_us;    // cooldown end time

    // relay state
    bool relay_output_on;

    // infra
    bool initialized;
    bool running;
    esp_timer_handle_t timer;
    portMUX_TYPE mux;
} bl0937_ctx_t;

static bl0937_ctx_t s_ctx = {
    .mux = portMUX_INITIALIZER_UNLOCKED,
};

static inline int level_for_sel(bool sel_is_voltage, bool active_high)
{
    // sel_is_voltage => SEL=1
    int logic = sel_is_voltage ? 1 : 0;
    return active_high ? logic : !logic;
}

static inline int level_for_relay(bool on, bool active_high)
{
    return active_high ? (on ? 1 : 0) : (on ? 0 : 1);
}

static void IRAM_ATTR isr_cf(void *arg)
{
    (void)arg;
    portENTER_CRITICAL_ISR(&s_ctx.mux);
    s_ctx.cf_cnt++;
    s_ctx.cf_total_pulses++;
    portEXIT_CRITICAL_ISR(&s_ctx.mux);
}

static void IRAM_ATTR isr_cf1(void *arg)
{
    (void)arg;
    portENTER_CRITICAL_ISR(&s_ctx.mux);
    s_ctx.cf1_cnt++;
    portEXIT_CRITICAL_ISR(&s_ctx.mux);
}

static float ema_apply(float prev, float x, uint16_t alpha_permille)
{
    // alpha = alpha_permille / 1000
    if (alpha_permille >= 1000) return x;
    if (alpha_permille == 0) return prev;
    float a = (float)alpha_permille / 1000.0f;
    return prev + a * (x - prev);
}

static void set_sel(bool sel_is_voltage)
{
    s_ctx.sel_is_voltage = sel_is_voltage;
    if (s_ctx.cfg.gpio_sel >= 0) {
        gpio_set_level(s_ctx.cfg.gpio_sel, level_for_sel(sel_is_voltage, s_ctx.cfg.sel_active_high));
    }
}

static void relay_apply(bool on)
{
    if (!s_ctx.cfg.relay_enable || s_ctx.cfg.gpio_relay < 0) return;
    gpio_set_level(s_ctx.cfg.gpio_relay, level_for_relay(on, s_ctx.cfg.relay_active_high));
    s_ctx.relay_output_on = on;
}

static bool hw_oc_detected(float freq_cf_hz)
{
    if (!s_ctx.cfg.hw_oc_detect_enable) return false;
    const float f0 = (float)s_ctx.cfg.hw_oc_freq_hz;
    const float tol = (float)s_ctx.cfg.hw_oc_tol_pct / 100.0f;
    return (freq_cf_hz >= f0 * (1.0f - tol)) && (freq_cf_hz <= f0 * (1.0f + tol));
}

static void update_from_counts(uint32_t cf_cnt, uint32_t cf1_cnt, float dt_s, int64_t now_us)
{
    // Convert to frequency
    float f_cf  = (dt_s > 0) ? ((float)cf_cnt / dt_s)  : 0.0f;
    float f_cf1 = (dt_s > 0) ? ((float)cf1_cnt / dt_s) : 0.0f;

    // Route CF1 based on current SEL state (state describes what CF1 was outputting during this window)
    if (s_ctx.sel_is_voltage) {
        s_ctx.freq_v_hz = ema_apply(s_ctx.freq_v_hz, f_cf1, s_ctx.cfg.ema_alpha_permille);
        if (cf1_cnt > 0) s_ctx.last_v_update_us = now_us;
    } else {
        s_ctx.freq_i_hz = ema_apply(s_ctx.freq_i_hz, f_cf1, s_ctx.cfg.ema_alpha_permille);
        if (cf1_cnt > 0) s_ctx.last_i_update_us = now_us;
    }
    s_ctx.freq_cf_hz = ema_apply(s_ctx.freq_cf_hz, f_cf, s_ctx.cfg.ema_alpha_permille);

    // Detect hardware over-current mode on CF
    s_ctx.hw_overcurrent = hw_oc_detected(s_ctx.freq_cf_hz);

    // Scaling constants
    const float vref = (float)s_ctx.cfg.vref_mv / 1000.0f;
    const float shunt = (float)s_ctx.cfg.shunt_uohm / 1e6f;

    float divider_ratio = (float)s_ctx.cfg.divider_num / (float)s_ctx.cfg.divider_den;
    if (divider_ratio <= 0.0f) divider_ratio = 0.001f;
    if (shunt <= 0.0f) divider_ratio = divider_ratio; // keep compiler happy

    // Convert CF1 frequencies back to input RMS voltages
    // Vv = Fv * Vref / KV, Vi = Fi * Vref / KI
    const float Vv_in = (s_ctx.freq_v_hz * vref) / BL0937_KV;   // volts at VP input (RMS)
    const float Vi_in = (s_ctx.freq_i_hz * vref) / BL0937_KI;   // volts across shunt at IP/IN (RMS)

    // Convert to mains/current with front-end parameters
    float voltage = (divider_ratio > 0.0f) ? (Vv_in / divider_ratio) : 0.0f;
    float current = (shunt > 0.0f) ? (Vi_in / shunt) : 0.0f;

    // Active power from CF frequency:
    // P = Fcf * Vref^2 / KP / (divider_ratio * shunt)
    float power = 0.0f;
    if (!s_ctx.hw_overcurrent && divider_ratio > 0.0f && shunt > 0.0f) {
        power = (s_ctx.freq_cf_hz * (vref * vref) / BL0937_KP) / (divider_ratio * shunt);
    }

    if (cf_cnt > 0) s_ctx.last_p_update_us = now_us;

    // Apply calibration and offsets
    voltage *= ((float)s_ctx.cfg.cal_voltage_permille / 1000.0f);
    current *= ((float)s_ctx.cfg.cal_current_permille / 1000.0f);
    power   *= ((float)s_ctx.cfg.cal_power_permille   / 1000.0f);

    voltage += ((float)s_ctx.cfg.cal_voltage_offset_mv / 1000.0f);
    current += ((float)s_ctx.cfg.cal_current_offset_ma / 1000.0f);

    if (!isfinite(voltage) || voltage < 0) voltage = 0;
    if (!isfinite(current) || current < 0) current = 0;
    if (!isfinite(power) || power < 0) power = 0;

    // Energy accumulation: use total CF pulses * energy_per_pulse
    // energy_per_pulse_J = (Vref^2 / KP) / (divider_ratio * shunt)  [J]
    // Wh = J / 3600
    uint64_t total_pulses = 0;
    portENTER_CRITICAL(&s_ctx.mux);
    total_pulses = s_ctx.cf_total_pulses;
    portEXIT_CRITICAL(&s_ctx.mux);

    float e_wh = 0.0f;
    if (!s_ctx.hw_overcurrent && divider_ratio > 0.0f && shunt > 0.0f) {
        const float e_per_pulse_j = (vref * vref / BL0937_KP) / (divider_ratio * shunt);
        e_wh = ((float)total_pulses * e_per_pulse_j) / 3600.0f;
        e_wh *= ((float)s_ctx.cfg.cal_power_permille / 1000.0f); // power calibration also scales energy
    }

    // Apparent & reactive power estimates
    float va = 0.0f;
#if CONFIG_BL0937_ENABLE_APPARENT_POWER
    va = voltage * current;
    if (!isfinite(va) || va < 0) va = 0;
#endif
    float qvar = 0.0f;
#if CONFIG_BL0937_ENABLE_REACTIVE_POWER
    // q = sqrt(S^2 - P^2) (magnitude only; sign requires phase information)
    float s2 = va * va;
    float p2 = power * power;
    float d = s2 - p2;
    if (d > 0.0f) qvar = sqrtf(d);
    if (!isfinite(qvar) || qvar < 0) qvar = 0;
#endif

// PF estimate (avoid div by 0)
    float pf = 0.0f;
    if (voltage > 0.1f && current > 0.01f) {
        pf = power / (voltage * current);
        if (!isfinite(pf)) pf = 0.0f;
        if (pf < 0.0f) pf = 0.0f;
        if (pf > 1.2f) pf = 1.2f; // allow slight calibration overshoot
    }

    // Store outputs (filtered on input freqs already; these are stable)
    s_ctx.voltage_v = voltage;
    s_ctx.current_a = current;
    s_ctx.power_w   = power;
    s_ctx.energy_wh = e_wh;
    s_ctx.pf        = pf;
    s_ctx.apparent_va = va;
    s_ctx.reactive_var = qvar;


    // Validity flags: based on recent updates and minimum frequency threshold
    const int64_t timeout_us = (int64_t)CONFIG_BL0937_SIGNAL_TIMEOUT_MS * 1000LL;
    const float min_hz = (float)CONFIG_BL0937_MIN_VALID_FREQ_HZ / 1000.0f;
    s_ctx.valid_v = (timeout_us <= 0) ? true : ((now_us - s_ctx.last_v_update_us) <= timeout_us);
    s_ctx.valid_i = (timeout_us <= 0) ? true : ((now_us - s_ctx.last_i_update_us) <= timeout_us);
    s_ctx.valid_p = (timeout_us <= 0) ? true : ((now_us - s_ctx.last_p_update_us) <= timeout_us);
    if (min_hz > 0.0f) {
        if (s_ctx.freq_v_hz < min_hz) s_ctx.valid_v = false;
        if (s_ctx.freq_i_hz < min_hz) s_ctx.valid_i = false;
        if (s_ctx.freq_cf_hz < min_hz) s_ctx.valid_p = false;
    }
}

static inline void fire_event(bl0937_event_t ev)
{
    if (s_ctx.event_cb) {
        s_ctx.event_cb(ev, s_ctx.event_user);
    }
}

static void protection_update(int64_t now_us)
{
    // Immediate trip if HW OC detected (optional)
    if (s_ctx.hw_overcurrent) {
        if (!s_ctx.overcurrent_tripped) {
            fire_event(BL0937_EVENT_HW_OVERCURRENT);
            fire_event(BL0937_EVENT_OVERCURRENT_TRIP);
        }
        s_ctx.overcurrent_tripped = true;
        s_ctx.trip_until_us = now_us + (int64_t)s_ctx.cfg.overcurrent_cooldown_s * 1000000LL;
        relay_apply(false);
        s_ctx.oc_since_us = 0;
        return;
    }

    // If already tripped, check cooldown
    if (s_ctx.overcurrent_tripped) {
        if (s_ctx.trip_until_us > 0 && now_us >= s_ctx.trip_until_us) {
            s_ctx.overcurrent_tripped = false;
            s_ctx.trip_until_us = 0;
            s_ctx.oc_since_us = 0;
            fire_event(BL0937_EVENT_OVERCURRENT_RECOVER);
            // relay behavior after recover configurable
#if CONFIG_BL0937_ENABLE_RELAY_CUTOFF
            if (CONFIG_BL0937_RELAY_DEFAULT_ON_AFTER_RECOVER) {
                relay_apply(true);
            } else {
                relay_apply(false);
            }
#else
            (void)0;
#endif
        } else {
            relay_apply(false);
        }
        return;
    }

    // Debounced software OC based on measured current
    const float thr_a = (float)s_ctx.cfg.overcurrent_threshold_ma / 1000.0f;
    if (s_ctx.current_a >= thr_a && thr_a > 0.0f) {
        if (s_ctx.oc_since_us == 0) {
            s_ctx.oc_since_us = now_us;
        } else {
            int64_t dt_ms = (now_us - s_ctx.oc_since_us) / 1000;
            if (dt_ms >= (int64_t)s_ctx.cfg.overcurrent_debounce_ms) {
                s_ctx.overcurrent_tripped = true;
                fire_event(BL0937_EVENT_OVERCURRENT_TRIP);
                s_ctx.trip_until_us = now_us + (int64_t)s_ctx.cfg.overcurrent_cooldown_s * 1000000LL;
                relay_apply(false);
            }
        }
    } else {
        s_ctx.oc_since_us = 0;
    }
}

static void timer_cb(void *arg)
{
    (void)arg;

    const int64_t now_us = esp_timer_get_time();

    // Atomically take counts and reset window counters
    uint32_t cf_cnt = 0, cf1_cnt = 0;
    portENTER_CRITICAL(&s_ctx.mux);
    cf_cnt = s_ctx.cf_cnt;
    cf1_cnt = s_ctx.cf1_cnt;
    s_ctx.cf_cnt = 0;
    s_ctx.cf1_cnt = 0;
    portEXIT_CRITICAL(&s_ctx.mux);

    const float dt_s = (float)s_ctx.cfg.update_period_ms / 1000.0f;

    // Compute
    update_from_counts(cf_cnt, cf1_cnt, dt_s, now_us);

    // Protection / relay
    protection_update(now_us);

    // Toggle SEL if configured
    if (s_ctx.cfg.toggle_sel_every_update) {
        // toggle for next window
        bool next = !s_ctx.sel_is_voltage;
        set_sel(next);
        // settle delay: we just toggled, but counting for next window will start immediately.
        // We'll ignore the first sel_stable_us by temporarily disabling interrupts if desired.
        // To keep it simple and robust, we do not disable interrupts here.
        // If needed, increase update_period_ms and/or reduce sel toggle rate.
        (void)s_ctx.cfg.sel_stable_us;
    }
}

static void load_default_cfg(bl0937_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->gpio_cf  = CONFIG_BL0937_GPIO_CF;
    cfg->gpio_cf1 = CONFIG_BL0937_GPIO_CF1;
    cfg->gpio_sel = CONFIG_BL0937_GPIO_SEL;

    cfg->sel_active_high = CONFIG_BL0937_SEL_ACTIVE_HIGH;

    cfg->relay_enable = CONFIG_BL0937_ENABLE_RELAY_CUTOFF;
#if CONFIG_BL0937_ENABLE_RELAY_CUTOFF
    cfg->gpio_relay = CONFIG_BL0937_GPIO_RELAY;
    cfg->relay_active_high = CONFIG_BL0937_RELAY_ACTIVE_HIGH;
#else
    cfg->gpio_relay = -1;
    cfg->relay_active_high = true;
#endif

    cfg->update_period_ms = CONFIG_BL0937_UPDATE_PERIOD_MS;
    cfg->toggle_sel_every_update = CONFIG_BL0937_TOGGLE_SEL_EVERY_UPDATE;
    cfg->sel_stable_us = CONFIG_BL0937_SEL_STABLE_US;

    cfg->ema_alpha_permille = CONFIG_BL0937_EMA_ALPHA_PERMILLE;

    cfg->vref_mv = CONFIG_BL0937_VREF_MV;
    cfg->shunt_uohm = CONFIG_BL0937_SHUNT_UOHM;

    cfg->divider_num = CONFIG_BL0937_DIVIDER_RATIO_NUM;
    cfg->divider_den = CONFIG_BL0937_DIVIDER_RATIO_DEN;

    cfg->cal_voltage_permille = CONFIG_BL0937_CAL_VOLTAGE_PERMILLE;
    cfg->cal_current_permille = CONFIG_BL0937_CAL_CURRENT_PERMILLE;
    cfg->cal_power_permille   = CONFIG_BL0937_CAL_POWER_PERMILLE;
    cfg->cal_voltage_offset_mv = CONFIG_BL0937_CAL_VOLTAGE_OFFSET_MV;
    cfg->cal_current_offset_ma = CONFIG_BL0937_CAL_CURRENT_OFFSET_MA;

    cfg->overcurrent_threshold_ma = CONFIG_BL0937_OVERCURRENT_THRESHOLD_MA;
    cfg->overcurrent_debounce_ms  = CONFIG_BL0937_OVERCURRENT_DEBOUNCE_MS;
    cfg->overcurrent_cooldown_s   = CONFIG_BL0937_OVERCURRENT_COOLDOWN_S;

    cfg->hw_oc_detect_enable = CONFIG_BL0937_ENABLE_HW_OC_DETECT;
    cfg->hw_oc_freq_hz = CONFIG_BL0937_HW_OC_FREQ_HZ;
    cfg->hw_oc_tol_pct = CONFIG_BL0937_HW_OC_FREQ_TOLERANCE_PCT;
}

static esp_err_t gpio_setup_input(int gpio, gpio_isr_t isr)
{
    if (gpio < 0) return ESP_OK;

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = CONFIG_BL0937_USE_GPIO_PULLUPS ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = CONFIG_BL0937_CF_ACTIVE_HIGH ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config failed");

    // Install ISR service (idempotent)
    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_RETURN_ON_ERROR(err, TAG, "gpio_install_isr_service failed");
    }

    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(gpio, isr, NULL), TAG, "gpio_isr_handler_add failed");
    return ESP_OK;
}


static void intr_enable_all(bool enable)
{
    if (s_ctx.cfg.gpio_cf >= 0) {
        if (enable) gpio_intr_enable(s_ctx.cfg.gpio_cf);
        else gpio_intr_disable(s_ctx.cfg.gpio_cf);
    }
    if (s_ctx.cfg.gpio_cf1 >= 0) {
        if (enable) gpio_intr_enable(s_ctx.cfg.gpio_cf1);
        else gpio_intr_disable(s_ctx.cfg.gpio_cf1);
    }
}

static esp_err_t gpio_setup_output(int gpio, int initial_level)
{
    if (gpio < 0) return ESP_OK;
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config output failed");
    gpio_set_level(gpio, initial_level);
    return ESP_OK;
}

#if defined(CONFIG_BL0937_ENABLE_NVS_CALIBRATION) || defined(CONFIG_BL0937_ENABLE_NVS_ENERGY)
#include "nvs.h"
#include "nvs_flash.h"
#endif

#if defined(CONFIG_BL0937_ENABLE_NVS_CALIBRATION) || defined(CONFIG_BL0937_ENABLE_NVS_ENERGY)
static esp_err_t nvs_open_handle(nvs_handle_t *out)
{
    ESP_RETURN_ON_ERROR(nvs_flash_init(), TAG, "nvs_flash_init failed");
    return nvs_open(CONFIG_BL0937_NVS_NAMESPACE, NVS_READWRITE, out);
}
#endif

esp_err_t bl0937_init_default(void)
{
    bl0937_config_t cfg;
    load_default_cfg(&cfg);
    return bl0937_init(&cfg);
}

esp_err_t bl0937_init(const bl0937_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "cfg null");
    if (s_ctx.initialized) return ESP_OK;

    s_ctx.cfg = *cfg;

    ESP_LOGI(TAG, "Init: CF=%d CF1=%d SEL=%d upd=%ums",
             s_ctx.cfg.gpio_cf, s_ctx.cfg.gpio_cf1, s_ctx.cfg.gpio_sel, (unsigned)s_ctx.cfg.update_period_ms);

    // Setup GPIOs
    ESP_RETURN_ON_ERROR(gpio_setup_input(s_ctx.cfg.gpio_cf, isr_cf), TAG, "CF gpio setup");
    ESP_RETURN_ON_ERROR(gpio_setup_input(s_ctx.cfg.gpio_cf1, isr_cf1), TAG, "CF1 gpio setup");

    if (s_ctx.cfg.gpio_sel >= 0) {
        ESP_RETURN_ON_ERROR(gpio_setup_output(s_ctx.cfg.gpio_sel, level_for_sel(false, s_ctx.cfg.sel_active_high)),
                            TAG, "SEL gpio setup");
    }

    if (s_ctx.cfg.relay_enable && s_ctx.cfg.gpio_relay >= 0) {
        bool init_on = CONFIG_BL0937_RELAY_INITIAL_ON;
        ESP_RETURN_ON_ERROR(gpio_setup_output(s_ctx.cfg.gpio_relay, level_for_relay(init_on, s_ctx.cfg.relay_active_high)),
                            TAG, "RELAY gpio setup");
        s_ctx.relay_output_on = init_on;
    }

    // initial SEL: start with IRMS (SEL=0)
    set_sel(false);

#if defined(CONFIG_BL0937_ENABLE_NVS_CALIBRATION) || defined(CONFIG_BL0937_ENABLE_NVS_ENERGY)
#include "nvs.h"
#include "nvs_flash.h"
#endif

#ifdef CONFIG_BL0937_ENABLE_NVS_ENERGY
    // best-effort restore energy pulse counter
    (void)bl0937_energy_restore();
#endif

#ifdef CONFIG_BL0937_ENABLE_NVS_CALIBRATION
    if (CONFIG_BL0937_ENABLE_NVS_CALIBRATION) {
        // best-effort load
        (void)bl0937_calibration_load();
    }
#endif

    // Start periodic timer
    esp_timer_create_args_t args = {
        .callback = &timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "bl0937",
        .skip_unhandled_events = true,
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&args, &s_ctx.timer), TAG, "esp_timer_create");
    s_ctx.initialized = true;

#if CONFIG_BL0937_AUTOSTART
    ESP_RETURN_ON_ERROR(bl0937_start(), TAG, "bl0937_start");
#endif

    return ESP_OK;
}

esp_err_t bl0937_deinit(void)
{
    if (!s_ctx.initialized) return ESP_OK;

    if (s_ctx.timer) {
        esp_timer_stop(s_ctx.timer);
        esp_timer_delete(s_ctx.timer);
        s_ctx.timer = NULL;
    }
    intr_enable_all(false);
    s_ctx.running = false;

    if (s_ctx.cfg.gpio_cf >= 0) gpio_isr_handler_remove(s_ctx.cfg.gpio_cf);
    if (s_ctx.cfg.gpio_cf1 >= 0) gpio_isr_handler_remove(s_ctx.cfg.gpio_cf1);

    // Note: we don't uninstall ISR service to avoid affecting other components.

    s_ctx.initialized = false;
    return ESP_OK;
}

static void ensure_init(void)
{
    if (s_ctx.initialized) return;
    (void)bl0937_init_default();
}


esp_err_t bl0937_start(void)
{
    ensure_init();
    if (s_ctx.running) return ESP_OK;

    // reset window counters (optional)
    portENTER_CRITICAL(&s_ctx.mux);
    s_ctx.cf_cnt = 0;
    s_ctx.cf1_cnt = 0;
    portEXIT_CRITICAL(&s_ctx.mux);

    intr_enable_all(true);

    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(s_ctx.timer, (uint64_t)s_ctx.cfg.update_period_ms * 1000ULL),
                        TAG, "esp_timer_start_periodic");
    s_ctx.running = true;
    return ESP_OK;
}

esp_err_t bl0937_stop(void)
{
    ensure_init();
    if (!s_ctx.running) return ESP_OK;

    esp_timer_stop(s_ctx.timer);
    intr_enable_all(false);
    s_ctx.running = false;
    return ESP_OK;
}

bool bl0937_is_running(void)
{
    ensure_init();
    return s_ctx.running;
}


void bl0937_set_event_callback(bl0937_event_cb_t cb, void *user_ctx)
{
    ensure_init();
    s_ctx.event_cb = cb;
    s_ctx.event_user = user_ctx;
}


float bl0937_voltage(void) { ensure_init(); return s_ctx.voltage_v; }
float bl0937_current(void) { ensure_init(); return s_ctx.current_a; }
float bl0937_power(void)   { ensure_init(); return s_ctx.power_w; }
float bl0937_energy_wh(void) { ensure_init(); return s_ctx.energy_wh; }
float bl0937_power_factor(void) { ensure_init(); return s_ctx.pf; }
float bl0937_apparent_power_va(void) { ensure_init(); return s_ctx.apparent_va; }
float bl0937_reactive_power_var(void) { ensure_init(); return s_ctx.reactive_var; }
float bl0937_freq_cf_hz(void) { ensure_init(); return s_ctx.freq_cf_hz; }
float bl0937_freq_vrms_hz(void) { ensure_init(); return s_ctx.freq_v_hz; }
float bl0937_freq_irms_hz(void) { ensure_init(); return s_ctx.freq_i_hz; }
bool bl0937_is_signal_ok(void) { ensure_init(); return s_ctx.valid_v && s_ctx.valid_i && s_ctx.valid_p; }

void bl0937_energy_reset(void)
{
    ensure_init();
    portENTER_CRITICAL(&s_ctx.mux);
    s_ctx.cf_total_pulses = 0;
    portEXIT_CRITICAL(&s_ctx.mux);
}

void bl0937_get_status(bl0937_status_t *out_status)
{
    ensure_init();
    if (!out_status) return;
    memset(out_status, 0, sizeof(*out_status));

    out_status->voltage_v = s_ctx.voltage_v;
    out_status->current_a = s_ctx.current_a;
    out_status->power_w = s_ctx.power_w;
    out_status->energy_wh = s_ctx.energy_wh;
    out_status->power_factor = s_ctx.pf;
    out_status->apparent_power_va = s_ctx.apparent_va;
    out_status->reactive_power_var = s_ctx.reactive_var;
    out_status->valid_voltage = s_ctx.valid_v;
    out_status->valid_current = s_ctx.valid_i;
    out_status->valid_power = s_ctx.valid_p;

    out_status->freq_cf_hz = s_ctx.freq_cf_hz;
    out_status->freq_vrms_hz = s_ctx.freq_v_hz;
    out_status->freq_irms_hz = s_ctx.freq_i_hz;

    out_status->initialized = s_ctx.initialized;
    out_status->sel_is_voltage = s_ctx.sel_is_voltage;
    out_status->overcurrent_tripped = s_ctx.overcurrent_tripped;
    out_status->hw_overcurrent = s_ctx.hw_overcurrent;
    out_status->relay_enabled = s_ctx.cfg.relay_enable && (s_ctx.cfg.gpio_relay >= 0);
    out_status->relay_output_on = s_ctx.relay_output_on;
    out_status->trip_until_us = s_ctx.trip_until_us;
}

bool bl0937_relay_set(bool on)
{
    ensure_init();
    if (!s_ctx.cfg.relay_enable || s_ctx.cfg.gpio_relay < 0) return false;
    if (s_ctx.overcurrent_tripped) {
        relay_apply(false);
        return true;
    }
    relay_apply(on);
    return true;
}

bool bl0937_relay_get(void)
{
    ensure_init();
    if (!s_ctx.cfg.relay_enable || s_ctx.cfg.gpio_relay < 0) return false;
    return s_ctx.relay_output_on;
}

bool bl0937_is_tripped(void)
{
    ensure_init();
    return s_ctx.overcurrent_tripped;
}

esp_err_t bl0937_set_calibration(uint16_t v_permille, uint16_t i_permille, uint16_t p_permille,
                                 int32_t v_offset_mv, int32_t i_offset_ma)
{
    ensure_init();
    if (v_permille == 0 || i_permille == 0 || p_permille == 0) return ESP_ERR_INVALID_ARG;

    s_ctx.cfg.cal_voltage_permille = v_permille;
    s_ctx.cfg.cal_current_permille = i_permille;
    s_ctx.cfg.cal_power_permille   = p_permille;
    s_ctx.cfg.cal_voltage_offset_mv = v_offset_mv;
    s_ctx.cfg.cal_current_offset_ma = i_offset_ma;

    return ESP_OK;
}

#if defined(CONFIG_BL0937_ENABLE_NVS_CALIBRATION) || defined(CONFIG_BL0937_ENABLE_NVS_ENERGY)
#include "nvs.h"
#include "nvs_flash.h"
#endif

#ifdef CONFIG_BL0937_ENABLE_NVS_CALIBRATION
typedef struct __attribute__((packed)) {
    uint16_t v_permille, i_permille, p_permille;
    int32_t v_off_mv, i_off_ma;
} calib_blob_t;

esp_err_t bl0937_calibration_save(void)
{
    ensure_init();
    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open_handle(&h), TAG, "nvs_open");

    calib_blob_t b = {
        .v_permille = s_ctx.cfg.cal_voltage_permille,
        .i_permille = s_ctx.cfg.cal_current_permille,
        .p_permille = s_ctx.cfg.cal_power_permille,
        .v_off_mv = s_ctx.cfg.cal_voltage_offset_mv,
        .i_off_ma = s_ctx.cfg.cal_current_offset_ma,
    };

    esp_err_t err = nvs_set_blob(h, "cal", &b, sizeof(b));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

esp_err_t bl0937_calibration_load(void)
{
    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open_handle(&h), TAG, "nvs_open");

    calib_blob_t b;
    size_t len = sizeof(b);
    esp_err_t err = nvs_get_blob(h, "cal", &b, &len);
    nvs_close(h);
    if (err != ESP_OK) return err;
    if (len != sizeof(b)) return ESP_ERR_INVALID_SIZE;

    return bl0937_set_calibration(b.v_permille, b.i_permille, b.p_permille, b.v_off_mv, b.i_off_ma);
}
#endif


#ifdef CONFIG_BL0937_ENABLE_NVS_ENERGY
static esp_err_t energy_save_pulses(uint64_t pulses)
{
    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open_handle(&h), TAG, "nvs_open");
    esp_err_t err = nvs_set_u64(h, CONFIG_BL0937_NVS_ENERGY_KEY, pulses);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

esp_err_t bl0937_energy_persist_now(void)
{
    ensure_init();
    uint64_t pulses;
    portENTER_CRITICAL(&s_ctx.mux);
    pulses = s_ctx.cf_total_pulses;
    portEXIT_CRITICAL(&s_ctx.mux);
    return energy_save_pulses(pulses);
}

esp_err_t bl0937_energy_restore(void)
{
    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open_handle(&h), TAG, "nvs_open");

    uint64_t pulses = 0;
    esp_err_t err = nvs_get_u64(h, CONFIG_BL0937_NVS_ENERGY_KEY, &pulses);
    nvs_close(h);
    if (err != ESP_OK) return err;

    portENTER_CRITICAL(&s_ctx.mux);
    s_ctx.cf_total_pulses = pulses;
    portEXIT_CRITICAL(&s_ctx.mux);
    return ESP_OK;
}
#endif
