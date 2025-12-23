#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "device_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// If RTC time looks unset, set it to build time (best-effort).
void timekeeper_init_if_unset(void);

bool timekeeper_is_time_sane(time_t now);

// Returns seconds until next sunrise-start time (>=1), i.e. alarm time minus cfg->sunrise_duration minutes.
// Falls back to 60s if time is not sane.
int64_t timekeeper_seconds_until_next_alarm(const device_config_t *cfg, time_t now);

// Sets current local time-of-day (HH:MM:SS) while keeping the current date.
// If RTC time looks unset, this will first set it to build time and then apply HHMMSS.
bool timekeeper_set_local_hhmmss(uint8_t hour, uint8_t minute, uint8_t second);

#ifdef __cplusplus
}
#endif
