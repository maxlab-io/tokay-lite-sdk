#include "thingsboard.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "esp_tls_crypto.h"

#include "json_settings_helpers.h"
#include "app_httpd.h"
#include "ai_camera.h"

static const char *TAG = "thingsboard";
static esp_mqtt_client_handle_t client_handle;

static bool tb_connect(const char *host, const char *auth);
static void tb_disconnect(void);
static char *get_telemetry_message(const void *p_buf, size_t len);
static bool tb_upload_telemetry(const void *p_buf, size_t len);
static void tb_take_image(void *p_ctx);

static void tb_take_image(void *p_ctx)
{
    camera_fb_t *p_fb = ai_camera_get_frame(PIXFORMAT_JPEG, pdMS_TO_TICKS(1000));
    if (NULL != p_fb) {
        tb_upload_telemetry(p_fb->buf, p_fb->len);
        ai_camera_fb_return(p_fb);
    } else {
        ESP_LOGE(TAG, "Image capture timeout");
    }
}

static void mqtt_event_handler_cb(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    int msg_id;
    // Handle MQTT events here
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            // Once connected, subscribe to the attributes topic
            msg_id = esp_mqtt_client_subscribe(client_handle, "v1/devices/me/attributes", 0);
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
            app_httpd_post_task(tb_take_image, NULL);
            break;
        default:
            break;
    }
}

static bool tb_connect(const char *host, const char *auth)
{
    if (NULL == host || NULL == auth) {
        return false;
    }
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.hostname = host,
        .broker.address.transport = MQTT_TRANSPORT_OVER_TCP,
        .broker.address.port = 1883,
        .credentials.username = auth,
    };
    client_handle = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client_handle);
    esp_mqtt_client_register_event(client_handle, ESP_EVENT_ANY_ID, mqtt_event_handler_cb, client_handle);
    return true;
}

static void tb_disconnect(void)
{
    esp_mqtt_client_stop(client_handle);
    esp_mqtt_client_destroy(client_handle);
}

static char *get_telemetry_message(const void *p_buf, size_t len)
{
    const size_t base64_len = ((len * 4 / 3) + 3) & 3;

    const char *json_start = "{\"image\":\"";
    const char *json_end = "\"}";
    const size_t json_len = strlen(json_start) + strlen(json_end);

    char *p_msg = heap_caps_malloc(base64_len + json_len + 1, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    strcpy(p_msg, json_start);
    size_t olen = 0;
    esp_crypto_base64_encode((uint8_t *)p_msg + strlen(json_start), base64_len, &olen, p_buf, len);
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
        const char *host = json_settings_get_string_or(p_cfg, "host", NULL);
        const char *auth = json_settings_get_string_or(p_cfg, "access_token", NULL);
        if (!tb_connect(host, auth)) {
            return false;
        }
    }

    tb_upload_telemetry(p_buf, len);

    if (!persistent_connection) {
        tb_disconnect();
    }

    return true;
}
