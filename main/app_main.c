#include <stdio.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "sdkconfig.h"

#include "soc/soc_caps.h"

#include "button.h"
#include "ch455g.h"
#include "device_config.h"
#include "pwm_led.h"
#include "timekeeper.h"
#include "ble_alarm.h"
#include "battery.h"

static const char *TAG = "APP";

// GPIO mapping per requirement.md
#define GPIO_I2C_SDA      GPIO_NUM_4
#define GPIO_I2C_SCL      GPIO_NUM_5
#define GPIO_PWM_WARM     GPIO_NUM_6
#define GPIO_PWM_COOL     GPIO_NUM_7
#define GPIO_BTN          GPIO_NUM_20
#define GPIO_BAT_ADC_EN   GPIO_NUM_21

// Behavior constants
#define LONG_PRESS_MS            1000
#define TIME_SHOW_MS             (CONFIG_LIGHT_ALARM_TIME_SHOW_SECONDS * 1000)
#define DEFAULT_SUNRISE_MINUTES_FALLBACK (CONFIG_LIGHT_ALARM_GRADIENT_MINUTES)
#define BLE_IDLE_SLEEP_DELAY_MS  3000

typedef enum {
    APP_STATE_DEEP_SLEEP = 0,
    APP_STATE_ACTIVE_IDLE,
    APP_STATE_ALARM_GRADIENT,
    APP_STATE_MANUAL_LIGHT,
} app_state_t;

typedef struct {
    device_config_t cfg;
    app_state_t state;

    TaskHandle_t main_task;
    volatile bool light_update_pending;

    ch455g_t disp;
    bool disp_inited;

    pwm_led_t pwm;
    bool pwm_inited;

    button_t btn;

    bool ble_inited;
    bool ble_adv_running;

    battery_t batt;
    bool batt_inited;
    esp_timer_handle_t batt_notify_timer;

    bool sleep_requested;
    int64_t sleep_at_us;

    time_t next_alarm_ts;
} app_ctx_t;

#define GPIO_BAT_ADC      GPIO_NUM_3

// Forward declarations (used across mode handlers)
static void app_run_manual_light(app_ctx_t *app);

static inline void app_request_light_update(app_ctx_t *app)
{
    if (!app) {
        return;
    }
    app->light_update_pending = true;
    if (app->main_task) {
        xTaskNotifyGive(app->main_task);
    }
}

static inline void app_wait_ms_or_light_update(uint32_t ms)
{
    // Sleep, but allow BLE writes to wake us up early for quicker light updates.
    (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ms));
}

static void batt_notify_timer_cb(void *arg)
{
    app_ctx_t *app = (app_ctx_t *)arg;
    if (!app || !app->batt_inited) {
        return;
    }
    if (!ble_alarm_is_connected()) {
        return;
    }
    uint8_t pct = 0;
    if (battery_read_percent(&app->batt, &pct) == ESP_OK) {
        (void)ble_alarm_notify_battery(pct);
    }
}

static void ble_on_connect(void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    if (!app || !app->batt_inited) {
        return;
    }

    // Best-effort: attempt to send battery once right after connection.
    // Note: client must enable notifications (CCCD) for delivery.
    uint8_t pct = 0;
    if (battery_read_percent(&app->batt, &pct) == ESP_OK) {
        (void)ble_alarm_notify_battery(pct);
    }
}

static void app_recompute_next_alarm(app_ctx_t *app)
{
    if (!app) {
        return;
    }

    if (!app->cfg.alarm_enabled) {
        app->next_alarm_ts = 0;
        ESP_LOGI(TAG, "alarm disabled: next sunrise start cleared");
        return;
    }

    time_t now = time(NULL);
    if (!timekeeper_is_time_sane(now)) {
        app->next_alarm_ts = 0;
        return;
    }
    int64_t seconds = timekeeper_seconds_until_next_alarm(&app->cfg, now);
    app->next_alarm_ts = now + seconds;
    ESP_LOGI(TAG, "next sunrise start in %llds (ts=%lld)", (long long)seconds, (long long)app->next_alarm_ts);
}

static void power_prep_for_sleep(void)
{
    // Ensure battery divider is disabled
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << GPIO_BAT_ADC_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(GPIO_BAT_ADC_EN, 0);

    // Ensure PWM pins low (avoid leakage)
    cfg.pin_bit_mask = (1ULL << GPIO_PWM_WARM) | (1ULL << GPIO_PWM_COOL);
    gpio_config(&cfg);
    gpio_set_level(GPIO_PWM_WARM, 0);
    gpio_set_level(GPIO_PWM_COOL, 0);

    // Ensure I2C pins low (open-drain outputs)
    cfg.pin_bit_mask = (1ULL << GPIO_I2C_SDA) | (1ULL << GPIO_I2C_SCL);
    cfg.mode = GPIO_MODE_OUTPUT_OD;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&cfg);
    gpio_set_level(GPIO_I2C_SDA, 0);
    gpio_set_level(GPIO_I2C_SCL, 0);
}

static void __attribute__((unused)) app_request_sleep_ms(app_ctx_t *app, uint32_t delay_ms)
{
#if CONFIG_LIGHT_ALARM_ALWAYS_ON
    (void)app;
    (void)delay_ms;
    return;
#else
    app->sleep_requested = true;
    app->sleep_at_us = esp_timer_get_time() + (int64_t)delay_ms * 1000;
#endif
}

static void __attribute__((unused)) app_enter_deep_sleep(app_ctx_t *app)
{
#if CONFIG_LIGHT_ALARM_ALWAYS_ON || CONFIG_LIGHT_ALARM_DEBUG_DISABLE_DEEP_SLEEP
    ESP_LOGW(TAG, "deep sleep disabled (ALWAYS_ON=%d DEBUG_DISABLE=%d); staying awake for monitor",
             (int)CONFIG_LIGHT_ALARM_ALWAYS_ON, (int)CONFIG_LIGHT_ALARM_DEBUG_DISABLE_DEEP_SLEEP);
    power_prep_for_sleep();
    while (true) {
        ESP_LOGI(TAG, "alive: BLE connected=%d", (int)ble_alarm_is_connected());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif

    timekeeper_init_if_unset();
    time_t now = time(NULL);

    int64_t seconds = 0;
    if (app->cfg.alarm_enabled && timekeeper_is_time_sane(now)) {
        seconds = timekeeper_seconds_until_next_alarm(&app->cfg, now);
        if (seconds < 0) {
            seconds = 0;
        }
        ESP_LOGI(TAG, "sleeping for %lld seconds until next alarm", (long long)seconds);
    } else {
        ESP_LOGI(TAG, "alarm disabled or time not sane: sleeping without timer wakeup");
    }

    // Stop peripherals
    if (app->disp_inited) {
        (void)ch455g_clear(&app->disp);
        (void)ch455g_set_enabled(&app->disp, false);
        (void)ch455g_set_sleep(&app->disp, true);
    }
    if (app->pwm_inited) {
        (void)pwm_led_off(&app->pwm);
    }

    if (app->ble_inited) {
        (void)ble_alarm_deinit();
        app->ble_inited = false;
        app->ble_adv_running = false;
    }

    if (app->batt_notify_timer) {
        (void)esp_timer_stop(app->batt_notify_timer);
        (void)esp_timer_delete(app->batt_notify_timer);
        app->batt_notify_timer = NULL;
    }
    if (app->batt_inited) {
        battery_deinit(&app->batt);
        app->batt_inited = false;
    }

    power_prep_for_sleep();

    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    if (seconds > 0) {
        ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL));
    }

    // Button wakeup: GPIO low level (preferred on ESP32-C3 since GPIO20 is not an RTC IO).
    gpio_pullup_en(GPIO_BTN);
    gpio_pulldown_dis(GPIO_BTN);
#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
    ESP_ERROR_CHECK(gpio_wakeup_enable(GPIO_BTN, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
#else
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(GPIO_BTN, 0));
#endif

    esp_deep_sleep_start();
}

static void app_display_show_now(app_ctx_t *app)
{
    timekeeper_init_if_unset();
    time_t now = time(NULL);
    struct tm t;
    localtime_r(&now, &t);
    (void)ch455g_show_hhmm(&app->disp, t.tm_hour, t.tm_min);
}

static void app_periph_ensure_display(app_ctx_t *app)
{
    if (!app->disp_inited) {
        ESP_ERROR_CHECK(ch455g_init(&app->disp, GPIO_I2C_SDA, GPIO_I2C_SCL, CONFIG_LIGHT_ALARM_CH455_INTENSITY));
        app->disp_inited = true;
    }
    (void)ch455g_set_sleep(&app->disp, false);
    (void)ch455g_set_enabled(&app->disp, true);
}

static void app_periph_ensure_pwm(app_ctx_t *app)
{
    if (!app->pwm_inited) {
        esp_err_t err = pwm_led_init(&app->pwm, GPIO_PWM_WARM, GPIO_PWM_COOL);
        if (err == ESP_OK) {
            app->pwm_inited = true;
            ESP_LOGI(TAG, "pwm init ok (warm=%d cool=%d)", (int)GPIO_PWM_WARM, (int)GPIO_PWM_COOL);

            // Self-test: briefly light warm-only then cool-only for diagnosis.
            // Warm 100% / Cool 0%
            (void)pwm_led_set_percent(&app->pwm, 100, 0);
            ESP_LOGI(TAG, "pwm selftest: warm=100%% cool=0%% (expect warm LED on)");
            vTaskDelay(pdMS_TO_TICKS(200));
            (void)pwm_led_set_percent(&app->pwm, 0, 0);

            // Cool 100% / Warm 0%
            (void)pwm_led_set_percent(&app->pwm, 0, 100);
            ESP_LOGI(TAG, "pwm selftest: warm=0%% cool=100%% (expect cool LED on)");
            vTaskDelay(pdMS_TO_TICKS(200));
            (void)pwm_led_set_percent(&app->pwm, 0, 0);
        } else {
            ESP_LOGE(TAG, "pwm init failed: %s", esp_err_to_name(err));
        }
    }
}

static void app_apply_light_linear_mix(app_ctx_t *app, uint8_t total_brightness_0_100, uint8_t color_temp_0_100)
{
    if (!app) {
        return;
    }
    if (total_brightness_0_100 > 100) {
        total_brightness_0_100 = 100;
    }
    if (color_temp_0_100 > 100) {
        color_temp_0_100 = 100;
    }

    // Linear fit/mix:
    //  - color_temp=0   => 100% cool
    //  - color_temp=100 => 100% warm
    // Keep warm+cool == total_brightness.
    uint16_t warm = (uint16_t)total_brightness_0_100 * (uint16_t)color_temp_0_100;
    warm = (warm + 50) / 100; // rounded
    if (warm > total_brightness_0_100) {
        warm = total_brightness_0_100;
    }

    // Avoid both channels going to 0 when brightness is very low.
    // If there is a non-zero brightness budget but rounding zeroed one side, force 1% to that side.
    uint16_t cool = (uint16_t)total_brightness_0_100 - warm;
    if (total_brightness_0_100 > 0 && color_temp_0_100 > 0 && warm == 0) {
        warm = 1;
        if (cool > 0) {
            cool -= 1;
        }
    }
    if (total_brightness_0_100 > 0 && color_temp_0_100 < 100 && cool == 0) {
        cool = 1;
        if (warm > 0) {
            warm -= 1;
        }
    }

    uint8_t warm_u8 = (uint8_t)warm;
    uint8_t cool_u8 = (uint8_t)cool;

    app_periph_ensure_pwm(app);
    // Smooth fade to target using hardware fade; keep a moderate fade time to improve visible gradient.
    const uint32_t fade_ms = 300;
    esp_err_t err = pwm_led_fade_percent(&app->pwm, warm_u8, cool_u8, fade_ms);
    static int64_t s_last_mix_log_us;
    int64_t now_us = esp_timer_get_time();
    if (err == ESP_OK) {
        if (s_last_mix_log_us == 0 || (now_us - s_last_mix_log_us) >= 3000000LL) {
            ESP_LOGI(TAG, "light mix: total=%u%% ct=%u%% -> warm=%u%% cool=%u%%",
                     (unsigned)total_brightness_0_100,
                     (unsigned)color_temp_0_100,
                     (unsigned)warm_u8,
                     (unsigned)cool_u8);
            s_last_mix_log_us = now_us;
        }
    } else {
        ESP_LOGE(TAG, "light mix set failed: %s", esp_err_to_name(err));
    }
}

static bool ble_on_write(const uint8_t hhmme5[5], void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    device_config_t new_cfg;
    if (!device_config_parse_hhmme_ascii(hhmme5, 5, &new_cfg)) {
        return false;
    }

    // Merge alarm fields only; keep other persisted settings.
    app->cfg.alarm_hour = new_cfg.alarm_hour;
    app->cfg.alarm_minute = new_cfg.alarm_minute;
    app->cfg.alarm_enabled = new_cfg.alarm_enabled;
    (void)device_config_save(&app->cfg);

    ESP_LOGI(TAG, "alarm updated to %02u%02u (enabled=%u)",
             app->cfg.alarm_hour,
             app->cfg.alarm_minute,
             (unsigned)app->cfg.alarm_enabled);

    // If we are staying awake (ALWAYS_ON), update next-alarm schedule immediately.
    app_recompute_next_alarm(app);

    // New requirement: keep connection active; do not disconnect/sleep after writes.

    return true;
}

static bool ble_on_write_color_temp(uint8_t value_0_100, void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    if (!app || value_0_100 > 100) {
        return false;
    }
    app->cfg.color_temp = value_0_100;
    (void)device_config_save(&app->cfg);
    ESP_LOGI(TAG, "color temp updated to %u (0=cool..100=warm)", (unsigned)app->cfg.color_temp);
    app_request_light_update(app);
    return true;
}

static bool ble_on_write_wake_bright(uint8_t value_0_100, void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    if (!app || value_0_100 > 100) {
        return false;
    }
    app->cfg.wake_bright = value_0_100;
    (void)device_config_save(&app->cfg);
    ESP_LOGI(TAG, "wake bright updated to %u", (unsigned)app->cfg.wake_bright);
    app_request_light_update(app);
    return true;
}

static bool ble_on_write_sunrise_duration(uint8_t minutes_1_60, void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    if (!app || minutes_1_60 < 1 || minutes_1_60 > 60) {
        return false;
    }
    app->cfg.sunrise_duration = minutes_1_60;
    (void)device_config_save(&app->cfg);
    ESP_LOGI(TAG, "sunrise duration updated to %u minutes", (unsigned)app->cfg.sunrise_duration);

    // Schedule changed -> recompute immediately.
    app_recompute_next_alarm(app);
    return true;
}

static bool ble_on_time_sync(const uint8_t hhmmss6[6], void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    if (!app || !hhmmss6) {
        return false;
    }

    for (int i = 0; i < 6; i++) {
        if (hhmmss6[i] < '0' || hhmmss6[i] > '9') {
            return false;
        }
    }

    uint8_t hh = (uint8_t)((hhmmss6[0] - '0') * 10 + (hhmmss6[1] - '0'));
    uint8_t mm = (uint8_t)((hhmmss6[2] - '0') * 10 + (hhmmss6[3] - '0'));
    uint8_t ss = (uint8_t)((hhmmss6[4] - '0') * 10 + (hhmmss6[5] - '0'));

    if (!timekeeper_set_local_hhmmss(hh, mm, ss)) {
        return false;
    }

    ESP_LOGI(TAG, "time synced to %02u:%02u:%02u", (unsigned)hh, (unsigned)mm, (unsigned)ss);

    // Time changed -> recompute schedule immediately.
    app_recompute_next_alarm(app);

    // New requirement: keep connection active; do not disconnect/sleep after time sync.

    return true;
}

static uint8_t ble_on_batt_read(void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    if (!app || !app->batt_inited) {
        ESP_LOGW(TAG, "battery read requested but battery not initialized");
        return 0;
    }
    uint8_t pct = 0;
    uint32_t mv = 0;
    esp_err_t err_mv = battery_read_mv(&app->batt, &mv);
    if (err_mv != ESP_OK) {
        ESP_LOGW(TAG, "battery read failed: mv_err=%s", esp_err_to_name(err_mv));
        return 0;
    }
    pct = battery_mv_to_percent(mv);
    ESP_LOGI(TAG, "battery read: %u mV -> %u%%", (unsigned)mv, (unsigned)pct);
    return pct;
}

static void ble_on_read(uint8_t out_hhmme5[5], void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    device_config_format_hhmme_ascii(&app->cfg, out_hhmme5);
}

static void ble_on_disconnect(void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;

    // New requirement: any time we are not connected, keep advertising (GAP packets).
    (void)ble_alarm_start_advertising();
    app->ble_adv_running = true;
}

static void app_ble_ensure_adv(app_ctx_t *app)
{
    if (!app->ble_inited) {
        ESP_ERROR_CHECK(ble_alarm_init(ble_on_write,
                                       ble_on_read,
                                       ble_on_time_sync,
                                       ble_on_batt_read,
                                       ble_on_write_color_temp,
                                       ble_on_write_wake_bright,
                                       ble_on_write_sunrise_duration,
                                       ble_on_connect,
                                       ble_on_disconnect,
                                       app));
        app->ble_inited = true;

        // Start a periodic battery notify while awake; ble_alarm will only send when CCCD enabled.
        if (!app->batt_notify_timer) {
            const esp_timer_create_args_t args = {
                .callback = &batt_notify_timer_cb,
                .arg = app,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "batt_notify",
                .skip_unhandled_events = true,
            };
            (void)esp_timer_create(&args, &app->batt_notify_timer);
            if (app->batt_notify_timer) {
                (void)esp_timer_start_periodic(app->batt_notify_timer, 60ULL * 1000000ULL);
            }
        }
    }
    (void)ble_alarm_start_advertising();
    app->ble_adv_running = true;
}
static void app_run_show_time(app_ctx_t *app, uint32_t show_ms)
{
    // Do not change state; allow long-press to be detected and break out to manual light.
    // Keep advertising and display enabled during the window.
    app_periph_ensure_display(app);
    app_ble_ensure_adv(app);

    int64_t end_us = esp_timer_get_time() + (int64_t)show_ms * 1000;

    while (esp_timer_get_time() < end_us) {
        app_display_show_now(app);

        // Let long-press break out to manual light immediately.
        button_event_t ev = button_poll(&app->btn);
        if (ev == BUTTON_EVENT_LONG) {
            ESP_LOGI(TAG, "show_time: long press -> manual light");
            app_run_manual_light(app);
            return;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    (void)ch455g_clear(&app->disp);
    (void)ch455g_set_enabled(&app->disp, false);
}

static void app_run_alarm_gradient(app_ctx_t *app)
{
    app->state = APP_STATE_ALARM_GRADIENT;

    app_periph_ensure_display(app);
    app_periph_ensure_pwm(app);

    // Prevent a stale release from being interpreted as an immediate SHORT cancel.
    button_sync_state(&app->btn);

    uint8_t sunrise_min = app->cfg.sunrise_duration;
    if (sunrise_min < 1 || sunrise_min > 60) {
        sunrise_min = (uint8_t)DEFAULT_SUNRISE_MINUTES_FALLBACK;
        if (sunrise_min < 1) sunrise_min = 1;
        if (sunrise_min > 60) sunrise_min = 60;
    }

    int64_t total_ms = (int64_t)sunrise_min * 60 * 1000;

    // Keep total_ms strictly based on sunrise_duration.
    // Shortening based on "time until alarm" can collapse the ramp into a few seconds if the clock/alarm time
    // is not aligned (user feedback: jumps to max brightness too fast).

    ESP_LOGI(TAG, "gradient: sunrise_min=%u total_ms=%lld target_bright=%u ct=%u",
             (unsigned)sunrise_min,
             (long long)total_ms,
             (unsigned)((app->cfg.wake_bright > 100) ? 100 : app->cfg.wake_bright),
             (unsigned)((app->cfg.color_temp > 100) ? 100 : app->cfg.color_temp));

    int64_t start_us = esp_timer_get_time();
    int64_t end_us = start_us + total_ms * 1000;

    bool canceled = false;

    while (esp_timer_get_time() < end_us) {
        int64_t now_us = esp_timer_get_time();
        int64_t elapsed_ms = (now_us - start_us) / 1000;

        // Scale gradient by configured wake max brightness.
        uint8_t target = app->cfg.wake_bright;
        if (target > 100) {
            target = 100;
        }

        // Exponential-ish ramp (cubic): brightness = target * progress^3.
        // This makes the beginning even smoother than quadratic.
        uint8_t brightness = 0;
        if (target == 0 || elapsed_ms <= 0) {
            brightness = 0;
        } else if (elapsed_ms >= total_ms) {
            brightness = target;
        } else {
            // progress in Q15 (0..32768)
            uint32_t p = (uint32_t)(((uint64_t)elapsed_ms << 15) / (uint64_t)total_ms);
            if (p > (1u << 15)) {
                p = (1u << 15);
            }
            // p2 and p3 in Q15
            uint32_t p2 = (uint32_t)(((uint64_t)p * (uint64_t)p) >> 15);
            uint32_t p3 = (uint32_t)(((uint64_t)p2 * (uint64_t)p) >> 15);
            uint32_t b = (uint32_t)(((uint64_t)p3 * (uint64_t)target + (1u << 14)) >> 15);
            if (b > target) {
                b = target;
            }
            brightness = (uint8_t)b;
            // Make sure we don't stay totally dark for too long when the ramp just started.
            if (brightness == 0) {
                brightness = 1;
            }
        }

        // Rate-limited progress log (helps diagnose "few seconds to full bright" cases).
        static int64_t s_last_grad_log_us;
        if (s_last_grad_log_us == 0 || (now_us - s_last_grad_log_us) >= 3000000LL) {
            int64_t remain_ms = (end_us - now_us) / 1000;
            ESP_LOGI(TAG, "gradient: elapsed=%lldms remain=%lldms bright=%u/%u", (long long)elapsed_ms, (long long)remain_ms, (unsigned)brightness, (unsigned)target);
            s_last_grad_log_us = now_us;
        }
        app_apply_light_linear_mix(app, brightness, app->cfg.color_temp);
        app_display_show_now(app);

        // short press cancels alarm immediately (both during ramp and after alarm time)
        button_event_t ev = button_poll(&app->btn);
        if (ev == BUTTON_EVENT_SHORT) {
            ESP_LOGI(TAG, "alarm canceled by short press");
            canceled = true;
            break;
        }

        app_wait_ms_or_light_update(500);
    }

    // After the sunrise finishes, keep light ON until user cancels with a short press.
    // If it was canceled during ramp, turn off immediately.
    if (canceled) {
        (void)pwm_led_off(&app->pwm);
    } else {
        uint8_t target = app->cfg.wake_bright;
        if (target > 100) {
            target = 100;
        }
        app_apply_light_linear_mix(app, target, app->cfg.color_temp);

        // Wait here until user short-presses to close the alarm. Allow BLE updates to change
        // brightness/color temperature while the alarm is active.
        button_sync_state(&app->btn);
        for (;;) {
            button_event_t ev = button_poll(&app->btn);
            if (ev == BUTTON_EVENT_SHORT) {
                ESP_LOGI(TAG, "alarm closed by short press");
                (void)pwm_led_off(&app->pwm);
                break;
            }
            if (app->light_update_pending) {
                app->light_update_pending = false;
                target = app->cfg.wake_bright;
                if (target > 100) {
                    target = 100;
                }
                app_apply_light_linear_mix(app, target, app->cfg.color_temp);
            }
            app_wait_ms_or_light_update(100);
        }
    }
    (void)ch455g_clear(&app->disp);
    (void)ch455g_set_enabled(&app->disp, false);

    // New requirement: cancel deep sleep mode.
    return;
}

static void app_run_manual_light(app_ctx_t *app)
{
    app->state = APP_STATE_MANUAL_LIGHT;

    app_periph_ensure_display(app);
    app_periph_ensure_pwm(app);
    app_ble_ensure_adv(app);

    // Manual light uses configured color temperature; brightness uses user-configured max (wake_bright).
    uint8_t manual_bright = app->cfg.wake_bright;
    if (manual_bright > 100) {
        manual_bright = 100;
    }
    app_apply_light_linear_mix(app, manual_bright, app->cfg.color_temp);

    uint8_t last_bright = manual_bright;
    uint8_t last_ct = (app->cfg.color_temp > 100) ? 100 : app->cfg.color_temp;
    app->light_update_pending = false;

    for (;;) {
        app_display_show_now(app);

        // Apply only when changed (reduces constant fade restarts -> less noise, more responsiveness).
        uint8_t cur_bright = app->cfg.wake_bright;
        if (cur_bright > 100) {
            cur_bright = 100;
        }
        uint8_t cur_ct = app->cfg.color_temp;
        if (cur_ct > 100) {
            cur_ct = 100;
        }
        if (app->light_update_pending || cur_bright != last_bright || cur_ct != last_ct) {
            app->light_update_pending = false;
            app_apply_light_linear_mix(app, cur_bright, cur_ct);
            last_bright = cur_bright;
            last_ct = cur_ct;
        }

        button_event_t ev = button_poll(&app->btn);
        if (ev == BUTTON_EVENT_LONG) {
            ESP_LOGI(TAG, "manual light OFF");
            break;
        }
        if (ev == BUTTON_EVENT_SHORT) {
            // Short press shows time briefly; must not interfere with manual light.
            ESP_LOGI(TAG, "manual light: short press -> show time");
            app_run_show_time(app, TIME_SHOW_MS);
            app->state = APP_STATE_MANUAL_LIGHT;
            app_periph_ensure_display(app);
            app_periph_ensure_pwm(app);
            app_ble_ensure_adv(app);
        }

        app_wait_ms_or_light_update(200);
    }

    (void)pwm_led_off(&app->pwm);
    (void)ch455g_clear(&app->disp);
    (void)ch455g_set_enabled(&app->disp, false);

    // New requirement: cancel deep sleep mode.
    return;
}

static void app_run_always_on(app_ctx_t *app)
{
    app->state = APP_STATE_ACTIVE_IDLE;

    // Keep peripherals off by default (only BLE advertising for GAP visibility).
    power_prep_for_sleep();

    app_ble_ensure_adv(app);
    ESP_LOGI(TAG, "ALWAYS_ON: BLE advertising requested");

    // Give GAP a moment to configure payloads and start advertising.
    int waited_ms = 0;
    while (waited_ms < 1500 && !ble_alarm_is_advertising() && !ble_alarm_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(50));
        waited_ms += 50;
    }
    ESP_LOGI(TAG, "ALWAYS_ON: adv=%d (after %d ms)", (int)ble_alarm_is_advertising(), waited_ms);

    // Initialize next-alarm schedule while awake.
    timekeeper_init_if_unset();
    app_recompute_next_alarm(app);

    for (;;) {
        // Alarm trigger while staying awake (ALWAYS_ON). This keeps the PWM wake-up behavior testable
        // without deep sleep.
        time_t now = time(NULL);
        if (app->cfg.alarm_enabled && app->next_alarm_ts == 0 && timekeeper_is_time_sane(now)) {
            app_recompute_next_alarm(app);
        }
        if (app->next_alarm_ts != 0 && timekeeper_is_time_sane(now) && now >= app->next_alarm_ts) {
            ESP_LOGI(TAG, "ALWAYS_ON: alarm due -> start gradient");
            app_run_alarm_gradient(app);
            app_recompute_next_alarm(app);
        }

        // In ALWAYS_ON debug, also handle BTN so display/LED can be verified without deep sleep.
        button_event_t ev = button_poll(&app->btn);
        if (ev == BUTTON_EVENT_SHORT) {
            ESP_LOGI(TAG, "ALWAYS_ON: short press -> show time");
            app_run_show_time(app, TIME_SHOW_MS);
        } else if (ev == BUTTON_EVENT_LONG) {
            ESP_LOGI(TAG, "ALWAYS_ON: long press -> manual light toggle");
            // Debounce long vs short: once we enter manual, we stay until long press inside manual exits.
            app_run_manual_light(app);
        }

        // Periodic log so monitor has continuous output.
        static int tick = 0;
        tick++;
        if ((tick % 10) == 0) {
            ESP_LOGI(TAG, "ALWAYS_ON tick: connected=%d adv=%d", (int)ble_alarm_is_connected(), (int)ble_alarm_is_advertising());
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "boot (deep sleep disabled by requirement)");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    app_ctx_t app = {0};
    app.main_task = xTaskGetCurrentTaskHandle();
    ESP_ERROR_CHECK(device_config_load(&app.cfg));

    ESP_ERROR_CHECK(button_init(&app.btn, GPIO_BTN, true, LONG_PRESS_MS));

    // Always keep BAT_ADC_EN off unless sampling.
    gpio_config_t en = {
        .pin_bit_mask = (1ULL << GPIO_BAT_ADC_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&en);
#if CONFIG_LIGHT_ALARM_DEBUG_BAT_ADC_EN_ALWAYS_HIGH
    gpio_set_level(GPIO_BAT_ADC_EN, 1);
    ESP_LOGW(TAG, "DEBUG: BAT_ADC_EN forced HIGH permanently");
#else
    gpio_set_level(GPIO_BAT_ADC_EN, 0);
#endif

    // Battery ADC (best-effort): used by BLE battery characteristic.
    if (battery_init(&app.batt, GPIO_BAT_ADC, GPIO_BAT_ADC_EN) == ESP_OK) {
        app.batt_inited = true;
    } else {
        ESP_LOGW(TAG, "battery init failed; battery characteristic will report 0%%");
        app.batt_inited = false;
    }

    // New requirement: cancel deep sleep mode entirely; stay awake and keep BLE advertising active.
    app_run_always_on(&app);
}
