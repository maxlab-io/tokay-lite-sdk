#include "thingsboard.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "esp_tls_crypto.h"

static const char *TAG = "thingsboard";
static esp_mqtt_client_handle_t client_handle;

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // Handle MQTT events here
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            // Once connected, subscribe to the attributes topic
            msg_id = esp_mqtt_client_subscribe(client, "v1/devices/me/attributes", 0);
            ESP_LOGI(TAG, "Subscribed to topic v1/devices/me/attributes, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            // Handle incoming MQTT data here
            break;
        default:
            break;
    }
    return ESP_OK;
}

static bool tb_connect(const char *host, const char *auth)
{
    if (NULL == host || NULL == auth) {
        return false;
    }
    esp_mqtt_client_config_t mqtt_cfg = {
        .hostname = host,
        .event_handle = mqtt_event_handler_cb,
        .username = auth,
    };
    client_handle = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client_handle);
    return true;
}

static void tb_disconnect(void)
{
    esp_mqtt_client_stop(client_handle);
    esp_mqtt_client_destroy(client_handle);
}

static char *get_telemetry_message(const coid *p_buf, size_t len)
{
    const size_t base64_len = ((len * 4 / 3) + 3) & 3;

    const char *json_start = "{\"image\":\"";
    const char *json_end = "\"}";
    const size_t json_len = strlen(json_start) + strlen(json_end);

    char *p_msg = (uint8_t *) heap_caps_malloc(base64_len + json_len + 1, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    strcpy(p_msg, json_start);
    size_t olen = 0;
    esp_crypto_base64_encode(p_msg + strlen(json_start), base64_len, &olen, p_buf, len);
    strcpy(p_msg + strlen(json_start) + olen, json_end);
    return p_msg;
}

static bool tb_upload_telemetry(const void *p_buf, size_t len)
{
    char *p_msg = get_telemetry_message(p_buf, len);
    const int ret = esp_mqtt_client_publish(client_handle, "v1/devices/me/telemetry",
                                            p_msg, strlen(p_msg), 0, 0);
    free(p_msg);
    return ret >= 0;
}


bool thingsboard_init(const cJSON *p_cfg)
{
    const bool persistent_connection = json_settings_get_int_or(p_cfg, "persistent_connection", 0);
    if (!persistent_connection) {
        return true;
    }
    const char *host = json_settings_get_string_or(p_cfg, "host", NULL);
    const char *auth = json_settings_get_string_or(p_cfg, "access_token", NULL);
    return tb_connect(host, auth);
}

bool thingsboard_upload_picture(const cJSON *p_cfg, const void *p_buf, size_t len)
{
    const bool persistent_connection = json_settings_get_int_or(p_cfg, "persistent_connection", 0);
    if (!persistent_connection) {
        tb_connect();
    }

    tb_upload_telemetry(p_buf, len);

    if (!persistent_connection) {
        tb_disconnect();
    }
}
