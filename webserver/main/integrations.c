#include "integrations.h"

#include "esp_http_client.h"
#include "esp_log.h"

#include "json_settings_helpers.h"

#define HA_HTTP_TIMEOUT_MS 3000
#define TAG "integrations"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define HA_HTTP_BOUNDARY "edge-ai-camera"
#define HA_WEBHOOK_BASE_PATH "/api/webhook/"

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
    assert(integration < INTEGRATION_MAX);
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
    if (NULL == p_buf || 0 == len) {
        return false;
    }

    ESP_LOGI(TAG, "Running Home Assistant integration");

    const char *host = cJSON_GetObjectItem(p_cfg, "host")->valuestring;
    const int port = cJSON_GetObjectItem(p_cfg, "port")->valueint;
    const char *webhook_id = cJSON_GetObjectItem(p_cfg, "webhook_id")->valuestring;
    const size_t path_len = strlen(webhook_id) + strlen(HA_WEBHOOK_BASE_PATH) + 1;
    char *path = malloc(path_len);
    snprintf(path, path_len, "%s%s", HA_WEBHOOK_BASE_PATH, webhook_id);

    ESP_LOGI(TAG, "Host %s port %d webhook_id %s", host, port, webhook_id);

    esp_http_client_config_t config = {
        .host = host,
        .port = port,
        .path = path,
        .disable_auto_redirect = true,
        .timeout_ms = HA_HTTP_TIMEOUT_MS,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (NULL == client) {
        free(path);
        return false;
    }

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "multipart/form-data; boundary="HA_HTTP_BOUNDARY);

    const char *multipart_headers =
        "--"HA_HTTP_BOUNDARY"\r\nContent-Disposition: form-data; name=\"image\"; filename=\"image.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";

    const int content_len = strlen(multipart_headers) + 2 + len + strlen(HA_HTTP_BOUNDARY) + 6;

    esp_http_client_open(client, content_len);

    esp_http_client_write(client, multipart_headers, strlen(multipart_headers));
    esp_http_client_write(client, p_buf, len);
    esp_http_client_write(client, "\r\n", 2);

    esp_http_client_write(client, "--", 2);
    esp_http_client_write(client, HA_HTTP_BOUNDARY, strlen(HA_HTTP_BOUNDARY));
    esp_http_client_write(client, "--\r\n", 4);

    ESP_LOGI(TAG, "Uploaded img of size %d", len);

    const int resp_len = esp_http_client_fetch_headers(client);
    const int status = esp_http_client_get_status_code(client);
    ESP_LOGI(TAG, "HTTP Status = %d, len %d", status, resp_len);
    if (200 != status || -1 == content_len) {
        const int resp_size = MIN(resp_len, 512);
        char *buf = malloc(resp_size + 1);
        const int ret = esp_http_client_read_response(client, buf, resp_size);
        if (ret != resp_size) {
            ESP_LOGE(TAG, "Failed to read response %d, %d", ret, resp_size);
            free(buf);
        } else {
            buf[resp_size] = 0;
            ESP_LOGI(TAG, "%s", buf);
            free(buf);
        }
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    free(path);
    return true;
}
