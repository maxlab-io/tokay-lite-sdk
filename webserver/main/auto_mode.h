#pragma once

#include <stdbool.h>
#include "cJSON.h"

typedef enum {
    AUTO_MODE_CONFIG_ENABLED,
    AUTO_MODE_CONFIG_PIR_ENABLED,
    AUTO_MODE_CONFIG_TFLITE_TRIGGER_ENABLED,
    AUTO_MODE_CONFIG_RTC_WAKEUP_ENABLED,
    AUTO_MODE_CONFIG_WAKEUP_PERIOD_SECONDS,
    AUTO_MODE_CONFIG_INTEGRATION_ID,
    AUTO_MODE_CONFIG_MAX,
} auto_mode_config_t;

bool auto_mode_enabled(void);

/**
 * Run the automation and return number of seconds to sleep
 */
int auto_mode_run(bool *p_enable_pir_wakeup);

void auto_mode_settings_set_json(const cJSON *p_settings);
const cJSON *auto_mode_settings_get_json(void);
void auto_mode_settings_apply(void);
