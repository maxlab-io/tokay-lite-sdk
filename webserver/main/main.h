#pragma once

#include "config.h"

typedef enum {
    SYSTEM_CONFIG_PIR_ENABLED,
    SYSTEM_CONFIG_MAX,
} system_config_t;

const char *system_config_get_name(system_config_t config);
const void *system_config_get_value(system_config_t config);
void system_config_set_value(system_config_t config, const void *p_value);
system_config_t system_config_get_by_name(const char *name);
config_type_t system_config_get_type(system_config_t config);
void system_config_apply(void);
