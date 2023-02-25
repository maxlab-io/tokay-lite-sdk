#pragma once

#include <stdbool.h>
#include "cJSON.h"

void json_settings_helpers_init(void);

static inline const char *json_settings_get_string_or(const cJSON *p_obj, const char *p_key, const char *p_default)
{
    const cJSON *p_ret = cJSON_GetObjectItem(p_obj, p_key);
    if (NULL == p_ret) {
        return p_default;
    }
    return p_ret->valuestring;
}

static inline int json_settings_get_int_or(const cJSON *p_obj, const char *p_key, int def)
{
    const cJSON *p_ret = cJSON_GetObjectItem(p_obj, p_key);
    if (NULL == p_ret) {
        return def;
    }
    return p_ret->valueint;
}

static inline void json_settings_set_string(cJSON *p_obj, const char *p_key, const char *p_value)
{
    cJSON *p_ret = cJSON_GetObjectItem(p_obj, p_key);
    if (NULL == p_ret) {
        cJSON_AddStringToObject(p_obj, p_key, p_value);
    } else {
        cJSON_SetValuestring(p_ret, p_value);
    }
}

static inline void json_settings_set_int(cJSON *p_obj, const char *p_key, int value)
{
    cJSON *p_ret = cJSON_GetObjectItem(p_obj, p_key);
    if (NULL == p_ret) {
        cJSON_AddNumberToObject(p_obj, p_key, value);
    } else {
        p_ret->valueint = value;
    }
}

cJSON *json_settings_load_from_nvs(const char *p_category);
bool json_settings_save_to_nvs(const char *p_category, const cJSON *p_settings);
