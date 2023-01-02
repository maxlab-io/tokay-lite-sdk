#pragma once

typedef enum {
    CONFIG_TYPE_STRING,
    CONFIG_TYPE_INT,

    CONFIG_TYPE_MAX
} config_type_t;

typedef struct {
    const char *name;
    config_type_t type;
} config_desc_t;

typedef struct {
    union {
        char *value_string;
        int   value_int;
    };
} config_value_t;

typedef struct {
    int num_vars;
    config_desc_t *p_desc;
    config_value_t *p_values;
} config_ctx_t;

const char *config_get_name(config_ctx_t *p_ctx, int config_var);
config_type_t config_get_type(config_ctx_t *p_ctx, int config_var);
const void *config_get_value(config_ctx_t *p_ctx, int config_var);
void config_set_value(config_ctx_t *p_ctx, int config_var, const void *p_value);
int config_get_by_name(config_ctx_t *p_ctx, const char *name);
