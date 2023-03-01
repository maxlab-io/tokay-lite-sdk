#pragma once

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

void system_settings_set_json(const cJSON *p_settings);
const cJSON *system_settings_get_json(void);
void system_settings_apply(void);

void app_send_event(app_event_t ev);
