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
#define ALARM_GRADIENT_MS        (CONFIG_LIGHT_ALARM_GRADIENT_MINUTES * 60 * 1000)
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

    bool sleep_requested;
    int64_t sleep_at_us;
} app_ctx_t;

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

    int64_t seconds = timekeeper_seconds_until_next_alarm(&app->cfg, now);
    ESP_LOGI(TAG, "sleeping for %lld seconds until next alarm", (long long)seconds);

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

    power_prep_for_sleep();

    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL));

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

static bool ble_on_write(const uint8_t hhmm4[4], void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    device_config_t new_cfg;
    if (!device_config_parse_hhmm_ascii(hhmm4, 4, &new_cfg)) {
        return false;
    }

    app->cfg = new_cfg;
    (void)device_config_save(&app->cfg);

    ESP_LOGI(TAG, "alarm updated to %02u%02u", app->cfg.alarm_hour, app->cfg.alarm_minute);

#if !CONFIG_LIGHT_ALARM_ALWAYS_ON
    // After successful write: disconnect then sleep a few seconds later.
    app_request_sleep_ms(app, BLE_IDLE_SLEEP_DELAY_MS);
    (void)ble_alarm_disconnect();
#endif

    return true;
}

static void ble_on_read(uint8_t out_hhmm4[4], void *ctx)
{
    app_ctx_t *app = (app_ctx_t *)ctx;
    device_config_format_hhmm_ascii(&app->cfg, out_hhmm4);
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
        ESP_ERROR_CHECK(ble_alarm_init(ble_on_write, ble_on_read, NULL, ble_on_disconnect, app));
        app->ble_inited = true;
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

    int64_t start_us = esp_timer_get_time();
    int64_t end_us = start_us + (int64_t)ALARM_GRADIENT_MS * 1000;

    while (esp_timer_get_time() < end_us) {
        int64_t now_us = esp_timer_get_time();
        int64_t elapsed_ms = (now_us - start_us) / 1000;
        uint8_t pct = (uint8_t)((elapsed_ms >= ALARM_GRADIENT_MS) ? 100 : (elapsed_ms * 100 / ALARM_GRADIENT_MS));

        (void)pwm_led_set_percent(&app->pwm, pct, pct);
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

    app_enter_deep_sleep(app);
}

static void app_run_manual_light(app_ctx_t *app)
{
    app->state = APP_STATE_MANUAL_LIGHT;

    app_periph_ensure_display(app);
    app_periph_ensure_pwm(app);

    (void)pwm_led_set_percent(&app->pwm, 100, 100);

    int64_t adv_until_us = 0;

    for (;;) {
        app_display_show_now(app);

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

    app_enter_deep_sleep(app);
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

    for (;;) {
        // Periodic log so monitor has continuous output.
        ESP_LOGI(TAG, "ALWAYS_ON tick: connected=%d adv=%d", (int)ble_alarm_is_connected(), (int)ble_alarm_is_advertising());
        vTaskDelay(pdMS_TO_TICKS(2000));
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
