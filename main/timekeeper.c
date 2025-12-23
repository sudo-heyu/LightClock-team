#include "timekeeper.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "sys/time.h"

static const char *TAG = "TIME";

static bool parse_build_time(struct tm *out)
{
    if (!out) {
        return false;
    }

    // __DATE__ format: "Mmm dd yyyy"; __TIME__ format: "hh:mm:ss"
    const char *date = __DATE__;
    const char *time_str = __TIME__;

    char mon_str[4] = {0};
    int day = 0, year = 0;
    int hour = 0, minute = 0, second = 0;

    if (sscanf(date, "%3s %d %d", mon_str, &day, &year) != 3) {
        return false;
    }
    if (sscanf(time_str, "%d:%d:%d", &hour, &minute, &second) != 3) {
        return false;
    }

    static const char *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    const char *p = strstr(months, mon_str);
    if (!p) {
        return false;
    }
    int mon = (int)((p - months) / 3);

    memset(out, 0, sizeof(*out));
    out->tm_year = year - 1900;
    out->tm_mon = mon;
    out->tm_mday = day;
    out->tm_hour = hour;
    out->tm_min = minute;
    out->tm_sec = second;
    out->tm_isdst = -1;
    return true;
}

bool timekeeper_is_time_sane(time_t now)
{
    // 2023-01-01 00:00:00 UTC
    return now >= 1672531200;
}

void timekeeper_init_if_unset(void)
{
    time_t now = time(NULL);
    if (timekeeper_is_time_sane(now)) {
        return;
    }

    struct tm tm_build;
    if (!parse_build_time(&tm_build)) {
        ESP_LOGW(TAG, "Time not set and build time parse failed");
        return;
    }

    time_t t = mktime(&tm_build);
    if (t < 0) {
        ESP_LOGW(TAG, "Time not set and build time mktime failed");
        return;
    }

    struct timeval tv = {.tv_sec = t, .tv_usec = 0};
    settimeofday(&tv, NULL);
    ESP_LOGW(TAG, "RTC time was unset; set to build time");
}

int64_t timekeeper_seconds_until_next_alarm(const device_config_t *cfg, time_t now)
{
    if (!cfg || !timekeeper_is_time_sane(now)) {
        return 60;
    }

    uint8_t sunrise_min = cfg->sunrise_duration;
    if (sunrise_min < 5 || sunrise_min > 60) {
        sunrise_min = DEVICE_CONFIG_DEFAULT_SUNRISE_DURATION_MINUTES;
        if (sunrise_min < 5) sunrise_min = 5;
        if (sunrise_min > 60) sunrise_min = 60;
    }

    struct tm local;
    localtime_r(&now, &local);

    struct tm target = local;
    target.tm_hour = cfg->alarm_hour;
    target.tm_min = cfg->alarm_minute;
    target.tm_sec = 0;

    time_t target_t = mktime(&target);
    if (target_t < 0) {
        return 60;
    }

    if (target_t <= now) {
        target_t += 24 * 60 * 60;
    }

    // Sunrise starts before the alarm.
    time_t sunrise_t = target_t - (time_t)sunrise_min * 60;
    if (sunrise_t <= now) {
        // If we are already past today's sunrise start, schedule the next day.
        // This avoids immediate re-trigger loops after the user cancels the sunrise simulation.
        sunrise_t += 24 * 60 * 60;
    }

    int64_t delta = (int64_t)(sunrise_t - now);
    if (delta < 1) {
        delta = 1;
    }
    return delta;
}

bool timekeeper_set_local_hhmmss(uint8_t hour, uint8_t minute, uint8_t second)
{
    if (hour >= 24 || minute >= 60 || second >= 60) {
        return false;
    }

    // Ensure we have at least a sane date.
    timekeeper_init_if_unset();

    time_t now = time(NULL);
    if (!timekeeper_is_time_sane(now)) {
        ESP_LOGW(TAG, "time still not sane after init; refusing to set HHMMSS");
        return false;
    }

    struct tm local;
    localtime_r(&now, &local);

    local.tm_hour = hour;
    local.tm_min = minute;
    local.tm_sec = second;
    local.tm_isdst = -1;

    time_t t = mktime(&local);
    if (t < 0) {
        ESP_LOGW(TAG, "mktime failed while setting HHMMSS");
        return false;
    }

    struct timeval tv = {.tv_sec = t, .tv_usec = 0};
    settimeofday(&tv, NULL);
    ESP_LOGI(TAG, "RTC time set to %02u:%02u:%02u (local)", (unsigned)hour, (unsigned)minute, (unsigned)second);
    return true;
}
