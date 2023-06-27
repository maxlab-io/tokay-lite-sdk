#pragma once

#include <stdbool.h>
#include "cJSON.h"

typedef enum {
    AUTO_MODE_CONFIG_ENABLED,
    AUTO_MODE_CONFIG_LOW_POWER_ENABLED,
    AUTO_MODE_CONFIG_PIR_ENABLED,
    AUTO_MODE_CONFIG_TFLITE_TRIGGER_ENABLED,
    AUTO_MODE_CONFIG_RTC_WAKEUP_ENABLED,
    AUTO_MODE_CONFIG_WAKEUP_PERIOD_SECONDS,
    AUTO_MODE_CONFIG_INTEGRATION_ID,
    AUTO_MODE_CONFIG_MAX,
} auto_mode_config_t;

void auto_mode_init(void);

bool auto_mode_enabled(void);

bool auto_mode_low_power_enabled(void);
bool auto_mode_pir_wakeup_enabled(void);
int auto_mode_get_wakeup_period_seconds(void);

/**
 * Run the automation and return number of seconds to sleep
 */
bool auto_mode_run(void);

void auto_mode_settings_set_json(const cJSON *p_settings);
const cJSON *auto_mode_settings_get_json(void);
void auto_mode_settings_apply(void);
