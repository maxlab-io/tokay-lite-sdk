#include "config.h"

#include "esp_log.h"

#include <string.h>

const char *config_get_name(config_ctx_t *p_ctx, int config_var)
{
    assert(NULL != p_ctx);
    assert(config_var < p_ctx->num_vars);
    return p_ctx->p_desc[config_var].name;
}

config_type_t config_get_type(config_ctx_t *p_ctx, int config_var)
{
    assert(NULL != p_ctx);
    assert(config_var < p_ctx->num_vars);
    return p_ctx->p_desc[config_var].type;
}

const void *config_get_value(config_ctx_t *p_ctx, int config_var)
{
    assert(NULL != p_ctx);
    assert(config_var < p_ctx->num_vars);
    return p_ctx->p_values[config_var].value_pv;
}

void config_set_value(config_ctx_t *p_ctx, int config_var, const void *p_value)
{
    assert(NULL != p_ctx);
    assert(config_var < p_ctx->num_vars);
    switch (p_ctx->p_desc[config_var].type) {
    case CONFIG_TYPE_STRING:
        free(p_ctx->p_values[config_var].value_string);
        p_ctx->p_values[config_var].value_string = strdup(p_value);
        ESP_LOGI("asd", "Set %d to %s", config_var, p_ctx->p_values[config_var].value_string);
        break;
    case CONFIG_TYPE_INT:
        p_ctx->p_values[config_var].value_int = (int)p_value;
        break;
    default:
        assert(0);
        break;
    }
}

int config_get_by_name(config_ctx_t *p_ctx, const char *name)
{
    assert(NULL != p_ctx);
    for (int i = 0; i < p_ctx->num_vars; i++) {
        if (strcmp(name, p_ctx->p_desc[i].name) == 0) {
            return i;
        }
    }
    return p_ctx->num_vars;
}
