#include "pwm_led.h"

#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "PWM";

// Many EN/PWM dimming drivers won't respond to extremely short PWM pulses.
// Enforce a minimum high-time (pulse width) so low percentages (e.g. <=3%) still light.
// Unit: nanoseconds.
#define PWM_MIN_PULSE_NS 600ULL

// Map a user-facing 0..100% value to hardware duty.
// Use a quadratic curve to make low percentages much dimmer (perceptual dimming),
// while keeping 0% off and snapping near 100% to fully-on to help reduce high-brightness whine.
static uint32_t percent_to_duty_curve(uint8_t percent, uint32_t duty_max)
{
    if (percent == 0) {
        return 0;
    }
    // Snap near max to constant-on (reduces modulation-related noise on some EN/PWM drivers).
    if (percent >= 99) {
        return duty_max;
    }

    // Quadratic curve: duty ~= duty_max * (p/100)^2
    // Use integer math: duty = duty_max * p*p / 10000
    uint64_t p = percent;
    uint64_t duty = (uint64_t)duty_max * p * p;
    duty = (duty + 5000ULL) / 10000ULL; // rounded
    if (duty == 0) {
        duty = 1; // ensure >0% is still visible, but much dimmer than linear
    }
    if (duty > duty_max) {
        duty = duty_max;
    }
    return (uint32_t)duty;
}

static uint32_t calc_min_duty(uint32_t freq_hz, uint32_t duty_max)
{
    // duty_counts >= min_ns * (duty_max+1) * freq / 1e9
    // ceil division to guarantee >= min pulse.
    uint64_t denom = 1000000000ULL;
    uint64_t num = PWM_MIN_PULSE_NS * (uint64_t)(duty_max + 1) * (uint64_t)freq_hz;
    uint32_t duty = (uint32_t)((num + (denom - 1)) / denom);
    if (duty < 1) {
        duty = 1;
    }
    if (duty > duty_max) {
        duty = duty_max;
    }
    return duty;
}

static void percents_to_duties(uint8_t warm_percent, uint8_t cool_percent,
                              uint32_t duty_max, uint32_t duty_min,
                              uint32_t *out_warm_duty, uint32_t *out_cool_duty)
{
    uint32_t warm_duty = 0;
    uint32_t cool_duty = 0;

    uint16_t sum = (uint16_t)warm_percent + (uint16_t)cool_percent;
    if (sum == 0) {
        warm_duty = 0;
        cool_duty = 0;
    } else if (sum <= 100) {
        // Treat warm/cool as components of a single brightness budget.
        uint32_t total_duty = percent_to_duty_curve((uint8_t)sum, duty_max);

        // Guarantee a minimum pulse width for any non-zero channel.
        uint32_t nonzero = 0;
        if (warm_percent > 0) nonzero++;
        if (cool_percent > 0) nonzero++;
        uint32_t min_total = duty_min * nonzero;
        if (total_duty < min_total) {
            total_duty = min_total;
        }

        // Split total duty by ratio (rounded) to preserve color temperature.
        warm_duty = (uint32_t)(((uint64_t)total_duty * (uint64_t)warm_percent + (sum / 2)) / (uint64_t)sum);
        if (warm_duty > total_duty) {
            warm_duty = total_duty;
        }
        cool_duty = total_duty - warm_duty;

        // Enforce per-channel minimums while keeping total_duty constant.
        if (warm_percent > 0 && warm_duty < duty_min) {
            warm_duty = duty_min;
            cool_duty = (total_duty > duty_min) ? (total_duty - duty_min) : 0;
        }
        if (cool_percent > 0 && cool_duty < duty_min) {
            cool_duty = duty_min;
            warm_duty = (total_duty > duty_min) ? (total_duty - duty_min) : 0;
        }
    } else {
        // If caller provides independent channel percents (sum>100), map each channel separately.
        warm_duty = percent_to_duty_curve(warm_percent, duty_max);
        cool_duty = percent_to_duty_curve(cool_percent, duty_max);

        if (warm_percent > 0 && warm_duty < duty_min) {
            warm_duty = duty_min;
        }
        if (cool_percent > 0 && cool_duty < duty_min) {
            cool_duty = duty_min;
        }
    }

    if (out_warm_duty) {
        *out_warm_duty = warm_duty;
    }
    if (out_cool_duty) {
        *out_cool_duty = cool_duty;
    }
}

// Internal fade helper
static esp_err_t set_duty_and_fade(gpio_num_t warm_gpio, gpio_num_t cool_gpio,
                                   uint32_t warm_duty, uint32_t cool_duty,
                                   uint32_t time_ms)
{
    // Warm channel
    esp_err_t err = ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, warm_duty, time_ms);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "fade set warm failed: %s", esp_err_to_name(err));
        return err;
    }
    err = ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, cool_duty, time_ms);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "fade set cool failed: %s", esp_err_to_name(err));
        return err;
    }
    err = ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "fade start warm failed: %s", esp_err_to_name(err));
        return err;
    }
    err = ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, LEDC_FADE_NO_WAIT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "fade start cool failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

esp_err_t pwm_led_init(pwm_led_t *led, gpio_num_t warm_gpio, gpio_num_t cool_gpio)
{
    if (!led) {
        return ESP_ERR_INVALID_ARG;
    }

    led->warm_gpio = warm_gpio;
    led->cool_gpio = cool_gpio;
    // Requirement: set to 40kHz to avoid audible noise.
    led->freq_hz = 40000;

    // NOTE: LEDC_AUTO_CLK may select REF_TICK (1MHz) on some targets, which would force
    // ~1.95kHz at 9-bit resolution (audible whine with EN/PWM dimming drivers).
    // Force APB clock so requested >20kHz PWM is actually achievable.
    // Prefer higher resolution to make very-low brightness (e.g. 1%) actually dim.
    // We'll step down resolution if we can't keep ultrasonic PWM frequency.
    uint32_t duty_bits = 10;
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT, // prefer 10-bit
        .timer_num = LEDC_TIMER_0,
        .freq_hz = led->freq_hz,
        .clk_cfg = LEDC_USE_APB_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "timer config failed: %s", esp_err_to_name(err));
        return err;
    }

    uint32_t actual_hz = ledc_get_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
    if (actual_hz < 38000) {
        ESP_LOGW(TAG, "PWM freq too low (%uHz) at 10-bit; reconfig to 9-bit", (unsigned)actual_hz);
        duty_bits = 9;
        timer.duty_resolution = LEDC_TIMER_9_BIT;
        err = ledc_timer_config(&timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "timer reconfig (9-bit) failed: %s", esp_err_to_name(err));
            return err;
        }
        actual_hz = ledc_get_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
    }
    if (actual_hz < 38000) {
        ESP_LOGW(TAG, "PWM freq too low (%uHz) at 9-bit; reconfig to 8-bit", (unsigned)actual_hz);
        duty_bits = 8;
        timer.duty_resolution = LEDC_TIMER_8_BIT;
        err = ledc_timer_config(&timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "timer reconfig (8-bit) failed: %s", esp_err_to_name(err));
            return err;
        }
        actual_hz = ledc_get_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0);
    }

    led->duty_max = (1u << duty_bits) - 1;
    led->duty_min = calc_min_duty(led->freq_hz, led->duty_max);

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

    // Install fade service once for all fade operations.
    err = ledc_fade_func_install(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { // INVALID_STATE if already installed; treat as OK
        ESP_LOGE(TAG, "fade func install failed: %s", esp_err_to_name(err));
        return err;
    }

    led->inited = true;
    ESP_LOGI(TAG, "init ok: warm_gpio=%d cool_gpio=%d req=%luHz actual=%uHz duty_max=%lu duty_min=%lu (min_pulse=%lluns)",
             (int)warm_gpio,
             (int)cool_gpio,
             (unsigned long)led->freq_hz,
             (unsigned)actual_hz,
             (unsigned long)led->duty_max,
             (unsigned long)led->duty_min,
             (unsigned long long)PWM_MIN_PULSE_NS);
    return ESP_OK;
}

esp_err_t pwm_led_set_percent(pwm_led_t *led, uint8_t warm_percent, uint8_t cool_percent)
{
    if (!led || !led->inited) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t warm_duty = 0;
    uint32_t cool_duty = 0;
    percents_to_duties(warm_percent, cool_percent, led->duty_max, led->duty_min, &warm_duty, &cool_duty);

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

    static int64_t s_last_log_us;
    int64_t now_us = esp_timer_get_time();
    if (s_last_log_us == 0 || (now_us - s_last_log_us) >= 3000000LL) {
        ESP_LOGI(TAG, "set percent warm=%u%% cool=%u%% duty=(%lu,%lu)", (unsigned)warm_percent, (unsigned)cool_percent, (unsigned long)warm_duty, (unsigned long)cool_duty);
        s_last_log_us = now_us;
    }
    return ESP_OK;
}

esp_err_t pwm_led_fade_percent(pwm_led_t *led, uint8_t warm_percent, uint8_t cool_percent, uint32_t time_ms)
{
    if (!led || !led->inited) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t warm_duty = 0;
    uint32_t cool_duty = 0;
    percents_to_duties(warm_percent, cool_percent, led->duty_max, led->duty_min, &warm_duty, &cool_duty);

    esp_err_t err = set_duty_and_fade(led->warm_gpio, led->cool_gpio, warm_duty, cool_duty, time_ms);
    if (err != ESP_OK) {
        return err;
    }

    static int64_t s_last_log_us;
    int64_t now_us = esp_timer_get_time();
    if (s_last_log_us == 0 || (now_us - s_last_log_us) >= 3000000LL) {
        ESP_LOGI(TAG, "fade percent warm=%u%% cool=%u%% duty=(%lu,%lu) time=%lums",
                 (unsigned)warm_percent, (unsigned)cool_percent,
                 (unsigned long)warm_duty, (unsigned long)cool_duty,
                 (unsigned long)time_ms);
        s_last_log_us = now_us;
    }

    return ESP_OK;
}

esp_err_t pwm_led_off(pwm_led_t *led)
{
    if (!led || !led->inited) {
        return ESP_OK;
    }
    return pwm_led_set_percent(led, 0, 0);
}
