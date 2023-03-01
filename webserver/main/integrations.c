#include "integrations.h"

#include "esp_http_client.h"
#include "esp_log.h"

#include "json_settings_helpers.h"

#define HA_HTTP_TIMEOUT_MS 3000
#define TAG "integrations"

static cJSON *p_settings;

const char *integration_names[INTEGRATION_MAX] = {
    [INTEGRATION_HOME_ASSISTANT] = "home_assistant",
    [INTEGRATION_S3] = "s3",
    [INTEGRATION_THINGSBOARD] = "thingsboard",
    [INTEGRATION_GENERIC_HTTP] = "http",
    [INTEGRATION_GENERIC_MQTT] = "mqtt",
};

static bool home_assistant_upload_picture(const cJSON *p_cfg, const void *p_buf, size_t len);

void integrations_init(void)
{
    p_settings = json_settings_load_from_nvs("integrations");
    if (NULL == p_settings) {
        ESP_LOGE(TAG, "Failed to load integrations settings from NVS");
        p_settings = cJSON_CreateObject();
        json_settings_save_to_nvs("integrations", p_settings);
    }
}

const cJSON *integrations_get(integration_t integration)
{
    return cJSON_GetObjectItem(p_settings, integration_names[integration]);
}

const cJSON *integrations_settings_get_json(void)
{
    return p_settings;
}

void integrations_settings_set_json(const cJSON *p_cfg)
{
    if (!cJSON_Compare(p_settings, p_cfg, false)) {
        cJSON_Delete(p_settings);
        p_settings = cJSON_Duplicate(p_cfg, true);
        // TODO: make sure all integration configs are merged into one JSON
        json_settings_save_to_nvs("integrations", p_settings);
    }
}

bool integrations_run(integration_t integration, const void *p_buf, size_t len)
{
    const cJSON *p_cfg = cJSON_GetObjectItem(p_settings, integration_names[integration]);
    if (NULL == p_cfg) {
        ESP_LOGE(TAG, "Failed to run integration %d", integration);
        return false;
    }
    switch (integration) {
    case INTEGRATION_HOME_ASSISTANT:
        return home_assistant_upload_picture(p_cfg, p_buf, len);
    default:
        return false;
    }
}

static bool home_assistant_upload_picture(const cJSON *p_cfg, const void *p_buf, size_t len)
{
    ESP_LOGI(TAG, "Running Home Assistant integration");
    const char *host = cJSON_GetObjectItem(p_cfg, "host")->valuestring;
    const int port = cJSON_GetObjectItem(p_cfg, "port")->valueint;
    const char *webhook_id = cJSON_GetObjectItem(p_cfg, "webhook_id")->valuestring;
    ESP_LOGI(TAG, "Host %s port %d webhook_id %s", host, port, webhook_id);
    esp_http_client_config_t config = {
        .host = host,
        .port = port,
        .path = "/",
        .disable_auto_redirect = true,
        .timeout_ms = HA_HTTP_TIMEOUT_MS,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (NULL == client) {
        return false;
    }
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    return true;
}
