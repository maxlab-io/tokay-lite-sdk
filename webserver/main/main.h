#pragma once

#include "config.h"
#include "cJSON.h"

typedef enum {
    SYSTEM_CONFIG_PIR_ENABLED,
    SYSTEM_CONFIG_WIFI_SSID,
    SYSTEM_CONFIG_WIFI_PASSWORD,
    SYSTEM_CONFIG_MAX,
} system_config_t;

typedef enum {
    APP_EVENT_WIFI_CONNECTED,
    APP_EVENT_PIR_MOTION_DETECTED,
    APP_EVENT_BUTTON_PRESSED,
    APP_EVENT_NEW_WIFI_SETTINGS,

    APP_EVENT_MAX,
} app_event_t;

const char *system_config_get_name(system_config_t config);
const void *system_config_get_value(system_config_t config);
void system_config_set_value(system_config_t config, const void *p_value);
system_config_t system_config_get_by_name(const char *name);
config_type_t system_config_get_type(system_config_t config);
void system_config_apply(void);
void system_config_process_json(const cJSON *p_settings);

void app_send_event(app_event_t ev);
