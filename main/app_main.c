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
#define LONG_PRESS_MS            2000
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

static void app_enter_deep_sleep(app_ctx_t *app)
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
        ESP_ERROR_CHECK(pwm_led_init(&app->pwm, GPIO_PWM_WARM, GPIO_PWM_COOL));
        app->pwm_inited = true;
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
    uint8_t warm_u8 = (uint8_t)warm;
    uint8_t cool_u8 = (uint8_t)(total_brightness_0_100 - warm_u8);

    app_periph_ensure_pwm(app);
    (void)pwm_led_set_percent(&app->pwm, warm_u8, cool_u8);
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

#if !CONFIG_LIGHT_ALARM_ALWAYS_ON
    // After successful write: disconnect then sleep a few seconds later.
    app_request_sleep_ms(app, BLE_IDLE_SLEEP_DELAY_MS);
    (void)ble_alarm_disconnect();
#endif

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
    return true;
}

static bool ble_on_write_sunrise_duration(uint8_t minutes_5_60, void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    if (!app || minutes_5_60 < 5 || minutes_5_60 > 60) {
        return false;
    }
    app->cfg.sunrise_duration = minutes_5_60;
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

#if !CONFIG_LIGHT_ALARM_ALWAYS_ON
    // After successful time sync: disconnect then sleep a few seconds later.
    app_request_sleep_ms(app, BLE_IDLE_SLEEP_DELAY_MS);
    (void)ble_alarm_disconnect();
#endif

    return true;
}

static uint8_t ble_on_batt_read(void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    if (!app || !app->batt_inited) {
        return 0;
    }
    uint8_t pct = 0;
    if (battery_read_percent(&app->batt, &pct) != ESP_OK) {
        return 0;
    }
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

#if CONFIG_LIGHT_ALARM_ALWAYS_ON
    (void)ble_alarm_start_advertising();
    app->ble_adv_running = true;
#else
    if (app->sleep_requested) {
        // ensure we actually wait 3-5 seconds after disconnect
        app_request_sleep_ms(app, BLE_IDLE_SLEEP_DELAY_MS);
    }
#endif
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
                                       NULL,
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
    app->state = APP_STATE_ACTIVE_IDLE;

    app_periph_ensure_display(app);
    app_ble_ensure_adv(app);

    int64_t end_us = esp_timer_get_time() + (int64_t)show_ms * 1000;

    while (esp_timer_get_time() < end_us) {
        app_display_show_now(app);
        vTaskDelay(pdMS_TO_TICKS(200));

        if (app->sleep_requested && esp_timer_get_time() >= app->sleep_at_us && !ble_alarm_is_connected()) {
            break;
        }
    }

    (void)ch455g_clear(&app->disp);
    (void)ch455g_set_enabled(&app->disp, false);

    // If config was written, go sleep now (after delay gate).
    if (app->sleep_requested && esp_timer_get_time() >= app->sleep_at_us && !ble_alarm_is_connected()) {
        app_enter_deep_sleep(app);
    }
}

static void app_run_alarm_gradient(app_ctx_t *app)
{
    app->state = APP_STATE_ALARM_GRADIENT;

    app_periph_ensure_display(app);
    app_periph_ensure_pwm(app);

    uint8_t sunrise_min = app->cfg.sunrise_duration;
    if (sunrise_min < 5 || sunrise_min > 60) {
        sunrise_min = (uint8_t)DEFAULT_SUNRISE_MINUTES_FALLBACK;
        if (sunrise_min < 5) sunrise_min = 5;
        if (sunrise_min > 60) sunrise_min = 60;
    }

    int64_t total_ms = (int64_t)sunrise_min * 60 * 1000;

    // Best-effort: if we woke late (after scheduled sunrise start), shorten to remaining time until alarm.
    time_t now_s = time(NULL);
    if (timekeeper_is_time_sane(now_s)) {
        struct tm local;
        localtime_r(&now_s, &local);
        struct tm target = local;
        target.tm_hour = app->cfg.alarm_hour;
        target.tm_min = app->cfg.alarm_minute;
        target.tm_sec = 0;
        time_t alarm_t = mktime(&target);
        if (alarm_t >= 0) {
            if (alarm_t <= now_s) {
                alarm_t += 24 * 60 * 60;
            }
            int64_t remain_ms = ((int64_t)(alarm_t - now_s)) * 1000;
            if (remain_ms > 0 && remain_ms < total_ms) {
                total_ms = remain_ms;
            }
        }
    }

    int64_t start_us = esp_timer_get_time();
    int64_t end_us = start_us + total_ms * 1000;

    while (esp_timer_get_time() < end_us) {
        int64_t now_us = esp_timer_get_time();
        int64_t elapsed_ms = (now_us - start_us) / 1000;
        // Use ceil division so PWM doesn't stay at 0% for a long time at the beginning.
        uint8_t pct = 0;
        if (elapsed_ms <= 0) {
            pct = 0;
        } else if (elapsed_ms >= total_ms) {
            pct = 100;
        } else {
            pct = (uint8_t)((elapsed_ms * 100 + (total_ms - 1)) / total_ms);
            if (pct > 100) {
                pct = 100;
            }
        }

        // Scale gradient by configured wake max brightness.
        uint8_t target = app->cfg.wake_bright;
        if (target > 100) {
            target = 100;
        }
        uint8_t brightness = 0;
        if (pct == 0 || target == 0) {
            brightness = 0;
        } else {
            brightness = (uint8_t)((pct * target + 99) / 100); // ceil so it's visible early
            if (brightness > target) {
                brightness = target;
            }
        }
        app_apply_light_linear_mix(app, brightness, app->cfg.color_temp);
        app_display_show_now(app);

        // short press cancels alarm -> deep sleep
        button_event_t ev = button_poll(&app->btn);
        if (ev == BUTTON_EVENT_SHORT) {
            ESP_LOGI(TAG, "alarm canceled by short press");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    (void)pwm_led_off(&app->pwm);
    (void)ch455g_clear(&app->disp);
    (void)ch455g_set_enabled(&app->disp, false);

#if CONFIG_LIGHT_ALARM_ALWAYS_ON
    // Debug ALWAYS_ON: do not enter deep sleep; return to main loop.
    return;
#else
    app_enter_deep_sleep(app);
#endif
}

static void app_run_manual_light(app_ctx_t *app)
{
    app->state = APP_STATE_MANUAL_LIGHT;

    app_periph_ensure_display(app);
    app_periph_ensure_pwm(app);

    // Manual light uses configured color temperature; brightness is fixed to 100%.
    app_apply_light_linear_mix(app, 100, app->cfg.color_temp);

    int64_t adv_until_us = 0;

    for (;;) {
        app_display_show_now(app);

        // Apply settings continuously so BLE writes take effect while staying in manual mode.
        app_apply_light_linear_mix(app, 100, app->cfg.color_temp);

        button_event_t ev = button_poll(&app->btn);
        if (ev == BUTTON_EVENT_LONG) {
            ESP_LOGI(TAG, "manual light OFF -> sleep");
            break;
        }
        if (ev == BUTTON_EVENT_SHORT) {
            // Keep manual light, but make device discoverable for a window.
            app_ble_ensure_adv(app);
            adv_until_us = esp_timer_get_time() + (int64_t)TIME_SHOW_MS * 1000;
        }

        if (adv_until_us > 0 && esp_timer_get_time() >= adv_until_us && !ble_alarm_is_connected()) {
    #if !CONFIG_LIGHT_ALARM_ALWAYS_ON
            (void)ble_alarm_stop_advertising();
            adv_until_us = 0;
    #else
            // ALWAYS_ON keeps advertising running.
            adv_until_us = 0;
    #endif
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }

    (void)pwm_led_off(&app->pwm);
    (void)ch455g_clear(&app->disp);
    (void)ch455g_set_enabled(&app->disp, false);

#if CONFIG_LIGHT_ALARM_ALWAYS_ON
    // Debug ALWAYS_ON: do not enter deep sleep; return to caller loop.
    return;
#else
    app_enter_deep_sleep(app);
#endif
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
    ESP_LOGI(TAG, "boot: ALWAYS_ON=%d", (int)CONFIG_LIGHT_ALARM_ALWAYS_ON);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    } else {
        ESP_ERROR_CHECK(err);
    }

    app_ctx_t app = {0};
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
    gpio_set_level(GPIO_BAT_ADC_EN, 0);

    // Battery ADC (best-effort): used by BLE battery characteristic.
    if (battery_init(&app.batt, GPIO_BAT_ADC, GPIO_BAT_ADC_EN) == ESP_OK) {
        app.batt_inited = true;
    } else {
        ESP_LOGW(TAG, "battery init failed; battery characteristic will report 0%%");
        app.batt_inited = false;
    }

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    ESP_LOGI(TAG, "wakeup cause=%d", cause);

#if CONFIG_LIGHT_ALARM_ALWAYS_ON
    app_run_always_on(&app);
    return;
#endif

    if (cause == ESP_SLEEP_WAKEUP_TIMER) {
        app_run_alarm_gradient(&app);
        return;
    }

    if (cause == ESP_SLEEP_WAKEUP_EXT0 || cause == ESP_SLEEP_WAKEUP_EXT1) {
        // Measure press duration right after wake to classify short/long from deep sleep.
        uint32_t dur_ms = button_measure_press_ms(&app.btn, 6000);
        if (dur_ms >= LONG_PRESS_MS) {
            ESP_LOGI(TAG, "long press from sleep -> manual light");
            app_run_manual_light(&app);
        } else {
            ESP_LOGI(TAG, "short press from sleep -> show time");
            app_run_show_time(&app, TIME_SHOW_MS);
            app_enter_deep_sleep(&app);
        }
        return;
    }

    // Power-on / reset path: init done, then compute next wake and sleep.
    app_enter_deep_sleep(&app);
}
