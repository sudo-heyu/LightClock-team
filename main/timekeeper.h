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

// Returns seconds until next alarm time (>=1). Falls back to 60s if time is not sane.
int64_t timekeeper_seconds_until_next_alarm(const device_config_t *cfg, time_t now);

#ifdef __cplusplus
}
#endif
