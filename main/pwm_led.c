#include "pwm_led.h"

#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "PWM";

static uint32_t percent_to_duty(uint8_t percent, uint32_t duty_max)
{
    if (percent >= 100) {
        return duty_max;
    }
    return (uint32_t)((uint64_t)percent * duty_max / 100ULL);
}

esp_err_t pwm_led_init(pwm_led_t *led, gpio_num_t warm_gpio, gpio_num_t cool_gpio)
{
    if (!led) {
        return ESP_ERR_INVALID_ARG;
    }

    led->warm_gpio = warm_gpio;
    led->cool_gpio = cool_gpio;
    // Requirement: >20kHz. Use 24kHz.
    led->freq_hz = 24000;

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_9_BIT, // 0..511 (<=500-ish requirement)
        .timer_num = LEDC_TIMER_0,
        .freq_hz = led->freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "timer config failed: %s", esp_err_to_name(err));
        return err;
    }

    led->duty_max = (1u << 9) - 1;

    ledc_channel_config_t ch0 = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = warm_gpio,
        .duty = 0,
        .hpoint = 0,
    };
    err = ledc_channel_config(&ch0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "warm channel config failed: %s", esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t ch1 = ch0;
    ch1.channel = LEDC_CHANNEL_1;
    ch1.gpio_num = cool_gpio;
    ch1.duty = 0;
    err = ledc_channel_config(&ch1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cool channel config failed: %s", esp_err_to_name(err));
        return err;
    }

    led->inited = true;
    ESP_LOGI(TAG, "init ok: warm_gpio=%d cool_gpio=%d freq=%luHz duty_max=%lu", (int)warm_gpio, (int)cool_gpio, (unsigned long)led->freq_hz, (unsigned long)led->duty_max);
    return ESP_OK;
}

esp_err_t pwm_led_set_percent(pwm_led_t *led, uint8_t warm_percent, uint8_t cool_percent)
{
    if (!led || !led->inited) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t warm_duty = percent_to_duty(warm_percent, led->duty_max);
    uint32_t cool_duty = percent_to_duty(cool_percent, led->duty_max);

    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, warm_duty);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set duty warm failed: %s", esp_err_to_name(err));
        return err;
    }
    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "update duty warm failed: %s", esp_err_to_name(err));
        return err;
    }

    err = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, cool_duty);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set duty cool failed: %s", esp_err_to_name(err));
        return err;
    }
    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "update duty cool failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "set percent warm=%u%% cool=%u%% duty=(%lu,%lu)", (unsigned)warm_percent, (unsigned)cool_percent, (unsigned long)warm_duty, (unsigned long)cool_duty);
    return ESP_OK;
}

esp_err_t pwm_led_off(pwm_led_t *led)
{
    if (!led || !led->inited) {
        return ESP_OK;
    }
    return pwm_led_set_percent(led, 0, 0);
}
