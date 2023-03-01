#pragma once

#include <stddef.h>
#include <stdbool.h>
#include "cJSON.h"

typedef enum {
    INTEGRATION_HOME_ASSISTANT,
    INTEGRATION_S3,
    INTEGRATION_THINGSBOARD,
    INTEGRATION_GENERIC_HTTP,
    INTEGRATION_GENERIC_MQTT,

    INTEGRATION_MAX,
} integration_t;

void integrations_init(void);

const cJSON *integrations_get(integration_t integration);

const cJSON *integrations_settings_get_json(void);
void integrations_settings_set_json(const cJSON *p_cfg);

bool integration_execute(integration_t integration, const void *p_buf, size_t len);
