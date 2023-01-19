#include "main.h"

#include <string.h>

#include "esp_camera.h"
#include "sensor.h"

#include "cJSON.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "nvs_flash.h"
#include "esp_log.h"

#include "bsp.h"
#include "pir.h"
#include "config.h"
#include "ai_camera.h"
#include "app_httpd.h"
#include "network.h"

#define TAG "main"

#define EVENT_QUEUE_SIZE 5
#define APP_TASK_STACK_SIZE 4096
#define APP_TASK_PRIORITY   5

static config_desc_t system_config_desc[SYSTEM_CONFIG_MAX] = {
    [SYSTEM_CONFIG_PIR_ENABLED] = { "pir_enabled", CONFIG_TYPE_INT },
    [SYSTEM_CONFIG_WIFI_SSID] = { "wifi_ssid", CONFIG_TYPE_STRING },
    [SYSTEM_CONFIG_WIFI_PASSWORD] = { "wifi_password", CONFIG_TYPE_STRING },
};

static config_value_t system_config_values[SYSTEM_CONFIG_MAX];
static config_ctx_t system_config_ctx = {
    .num_vars = SYSTEM_CONFIG_MAX,
    .p_desc = system_config_desc,
    .p_values = system_config_values,
};

static QueueHandle_t event_queue;

static void pir_motion_callback(void *ctx);
static void wifi_connection_callback(void);
static void button_callback(void);
static void app_task(void *pvArg);

void app_main(void)
{
    BaseType_t xRet = xTaskCreatePinnedToCore(app_task, "APP_TSK", APP_TASK_STACK_SIZE,
            NULL, APP_TASK_PRIORITY, NULL, 0);
    configASSERT(pdTRUE == xRet);
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
    char curr_ssid[33] = { 0 };
    char curr_password[64] = { 0 };
    const char *new_ssid = system_config_get_value(SYSTEM_CONFIG_WIFI_SSID);
    const char *new_password = system_config_get_value(SYSTEM_CONFIG_WIFI_PASSWORD);
    network_get_credentials(curr_ssid, curr_password);
    if (0 != strcmp(curr_ssid, new_ssid) || 0 != strcmp(curr_password, new_password)) {
        ESP_LOGI(TAG, "Updating WiFi settings from %s to %s", curr_ssid, new_ssid);
        app_send_event(APP_EVENT_NEW_WIFI_SETTINGS);
    }
}

void app_send_event(app_event_t ev)
{
    BaseType_t xRet = xQueueSend(event_queue, &ev, 0);
    assert(pdTRUE == xRet);
}

void system_config_process_json(const cJSON *p_settings)
{
    cJSON *p_cfg_val = NULL;
    cJSON_ArrayForEach(p_cfg_val, p_settings)
    {
        system_config_t config = system_config_get_by_name(p_cfg_val->string);
        if (SYSTEM_CONFIG_MAX == config) {
            ESP_LOGE(TAG, "Unknown config variable %s", p_cfg_val->string);
            continue;
        }
        switch (system_config_get_type(config)) {
        case CONFIG_TYPE_STRING:
            if (!cJSON_IsString(p_cfg_val)) {
                ESP_LOGE(TAG, "Wrong config variable type %s, expected string", p_cfg_val->string);
                continue;
            }
            ESP_LOGI(TAG, "Set %d to %s", config, p_cfg_val->valuestring);
            system_config_set_value(config, (const void *)p_cfg_val->valuestring);
            break;
        case CONFIG_TYPE_INT: {
            if (!cJSON_IsNumber(p_cfg_val)) {
                ESP_LOGE(TAG, "Wrong config variable type %s, expected number", p_cfg_val->string);
                continue;
            }
            const int valueint = (int)p_cfg_val->valuedouble;
            system_config_set_value(config, (const void *)valueint);
            break;
        }
        default:
            assert(0);
            break;
        }
    }
}

static void pir_motion_callback(void *ctx)
{
    app_send_event(APP_EVENT_PIR_MOTION_DETECTED);
}

static void button_callback(void)
{
    app_send_event(APP_EVENT_BUTTON_PRESSED);
}

static void wifi_connection_callback(void)
{
    app_send_event(APP_EVENT_WIFI_CONNECTED);
}

static bool load_wifi_creds(void)
{
    char wifi_ssid[33] = { 0 };
    char wifi_password[64] = { 0 } ;

    network_get_credentials(wifi_ssid, wifi_password);

    system_config_set_value(SYSTEM_CONFIG_WIFI_SSID, wifi_ssid);
    system_config_set_value(SYSTEM_CONFIG_WIFI_PASSWORD, wifi_password);

    return wifi_ssid[0] != 0;
}

static void app_task(void *pvArg)
{
    bool ap_mode = false;

    gpio_install_isr_service(0);
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    event_queue = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(app_event_t));

    network_init("ai-camera", wifi_connection_callback);

    if (load_wifi_creds()) {
        const char *p_ssid = system_config_get_value(SYSTEM_CONFIG_WIFI_SSID);
        const char *p_password = system_config_get_value(SYSTEM_CONFIG_WIFI_PASSWORD);
        ESP_LOGI(TAG, "Starting in STA mode, connecting to %s", p_ssid);
        network_sta_mode(p_ssid, p_password);
        ap_mode = false;
    } else {
        ESP_LOGI(TAG, "No STA config saved, starting in AP mode");
        network_ap_mode();
        app_httpd_start(true);
        ap_mode = true;
    }

    bsp_init(button_callback);
    pir_init(pir_motion_callback, NULL);

    ai_camera_init(BSP_I2C_BUS_ID);
    ai_camera_start(AI_CAMERA_PIPELINE_DISCARD, NULL, NULL);

    while (1) {
        app_event_t event = APP_EVENT_MAX;
        xQueueReceive(event_queue, &event, portMAX_DELAY);
        switch (event) {
        case APP_EVENT_WIFI_CONNECTED:
            if (!app_httpd_is_running()) {
                app_httpd_start(false);
            }
            break;
        case APP_EVENT_PIR_MOTION_DETECTED:
            ESP_LOGI(TAG, "Motion detected");
            break;
        case APP_EVENT_BUTTON_PRESSED:
            if (ap_mode) {
                const char *p_ssid = system_config_get_value(SYSTEM_CONFIG_WIFI_SSID);
                const char *p_password = system_config_get_value(SYSTEM_CONFIG_WIFI_PASSWORD);
                if (p_ssid != NULL && p_ssid[0] != 0) {
                    ESP_LOGI(TAG, "Switching to STA mode");
                    app_httpd_stop();
                    network_sta_mode(p_ssid, p_password);
                    ap_mode = false;
                } else {
                    ESP_LOGW(TAG, "Wifi creds unavailable, ignoring button press");
                }
            } else {
                ESP_LOGI(TAG, "Switching to AP mode");
                app_httpd_stop();
                network_ap_mode();
                app_httpd_start(true);
                ap_mode = true;
            }
            break;
        case APP_EVENT_NEW_WIFI_SETTINGS:
            ESP_LOGI(TAG, "Reconnecting to the new AP");
            app_httpd_stop();
            network_sta_mode(system_config_get_value(SYSTEM_CONFIG_WIFI_SSID), system_config_get_value(SYSTEM_CONFIG_WIFI_PASSWORD));
            ap_mode = false;
        default:
            break;
        }
    }
}
