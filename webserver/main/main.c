#include "main.h"

#include "esp_camera.h"
#include "sensor.h"

#include "esp_log.h"
#include "cJSON.h"
#include "driver/gpio.h"

#include "nvs_flash.h"

#include "bsp.h"
#include "pir.h"
#include "config.h"
#include "ai_camera.h"
#include "app_httpd.h"
#include "network.h"

#define TAG "main"

static config_desc_t system_config_desc[SYSTEM_CONFIG_MAX] = {
    [SYSTEM_CONFIG_PIR_ENABLED] = { "pir_enabled", CONFIG_TYPE_INT },
};

static config_value_t system_config_values[SYSTEM_CONFIG_MAX];
static config_ctx_t system_config_ctx = {
    .num_vars = SYSTEM_CONFIG_MAX,
    .p_desc = system_config_desc,
    .p_values = system_config_values,
};

static void pir_motion_callback(void *ctx);
static void wifi_connection_callback(void);

void app_main(void)
{
    gpio_install_isr_service(0);
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    network_init("ai-camera", wifi_connection_callback);

    bsp_init();
    pir_init(pir_motion_callback, NULL);

    ai_camera_init(BSP_I2C_BUS_ID);
    ai_camera_start(AI_CAMERA_PIPELINE_DISCARD, NULL, NULL);

    app_httpd_start();
}

const char *system_config_get_name(system_config_t config)
{
    return config_get_name(&system_config_ctx, (int)config);
}

const void *system_config_get_value(system_config_t config)
{
    return config_get_value(&system_config_ctx, (int)config);
}

void system_config_set_value(system_config_t config, const void *p_value)
{
    config_set_value(&system_config_ctx, (int)config, p_value);
}

system_config_t system_config_get_by_name(const char *name)
{
    return (int)config_get_by_name(&system_config_ctx, name);
}

config_type_t system_config_get_type(system_config_t config)
{
    return config_get_type(&system_config_ctx, (int)config);
}

void system_config_apply(void)
{
    // TODO
}

static void pir_motion_callback(void *ctx)
{
    // TODO: store events in a list to be reported to the front-end
}

static void wifi_connection_callback(void)
{
}
