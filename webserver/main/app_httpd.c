#include "app_httpd.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "esp_heap_caps.h"

#include "main.h"
#include "bsp.h"
#include "pir.h"
#include "config.h"
#include "ai_camera.h"
#include "auto_mode.h"
#include "integrations.h"

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#define PART_BOUNDARY        "123456789000000000000987654321"
#define _STREAM_CONTENT_TYPE "multipart/x-mixed-replace;boundary=" PART_BOUNDARY
#define _STREAM_BOUNDARY     "\r\n--" PART_BOUNDARY "\r\n"
#define _STREAM_PART         "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n"

#define STREAMING_HTTP_SERVER_PORT 1111
#define SETTINGS_JSON_MAX_SIZE     1024

#define TELEMETRY_UPDATE_PERIOD_MS 1000

#define CONTROL_WS_MAX_FRAME_LEN 512

#define TAG "app_httpd"

typedef enum {
    HTTP_IMAGE_FORMAT_JPEG,
    HTTP_IMAGE_FORMAT_YUV422,
    HTTP_IMAGE_FORMAT_RGB565,
} http_image_format_t;

static esp_err_t http_handler_get_static_page(httpd_req_t *req);
static esp_err_t http_handler_get_picture(httpd_req_t *req);
static esp_err_t http_handler_get_settings(httpd_req_t *req);
static esp_err_t http_handler_set_settings(httpd_req_t *req);
static esp_err_t http_handler_ws_control(httpd_req_t *req);

static esp_err_t http_handler_get_stream(httpd_req_t *req);
static void camera_metadata_cb(cJSON *p_meta_root, void *p_ctx);
static void camera_frame_cb(pixformat_t format, const uint8_t *p_data, uint32_t size,
        bool start_of_frame, void *p_ctx);

static void telemetry_timer_cb(TimerHandle_t handle);
static void send_telemetry(void *arg);

extern const char index_html_start[] asm("_binary_index_themed_html_start");
extern const char wifi_setup_html_start[] asm("_binary_wifi_setup_html_start");

static const httpd_uri_t handler_index = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = http_handler_get_static_page,
    .user_ctx  = (void *)index_html_start,
};

static const httpd_uri_t handler_wifi_setup = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = http_handler_get_static_page,
    .user_ctx  = (void *)wifi_setup_html_start,
};

static const httpd_uri_t handler_jpeg = {
    .uri       = "/image.jpg",
    .method    = HTTP_GET,
    .handler   = http_handler_get_picture,
    .user_ctx  = (void *)HTTP_IMAGE_FORMAT_JPEG,
};

static const httpd_uri_t handler_yuv422 = {
    .uri       = "/image.yuv422",
    .method    = HTTP_GET,
    .handler   = http_handler_get_picture,
    .user_ctx  = (void *)HTTP_IMAGE_FORMAT_YUV422,
};

static const httpd_uri_t handler_rgb565 = {
    .uri       = "/image.rgb565",
    .method    = HTTP_GET,
    .handler   = http_handler_get_picture,
    .user_ctx  = (void *)HTTP_IMAGE_FORMAT_RGB565,
};

static const httpd_uri_t handler_get_settings = {
    .uri       = "/settings/*",
    .method    = HTTP_GET,
    .handler   = http_handler_get_settings,
};

static const httpd_uri_t handler_set_settings = {
    .uri       = "/settings/*",
    .method    = HTTP_POST,
    .handler   = http_handler_set_settings,
};

static const httpd_uri_t handler_ws_control = {
    .uri       = "/control",
    .method    = HTTP_GET,
    .handler   = http_handler_ws_control,
    .user_ctx  = NULL,
    .is_websocket = true,
};

static const httpd_uri_t handler_stream = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = http_handler_get_stream,
    .user_ctx  = NULL,
    .is_websocket = true,
};

static struct {
    bool is_running;
    httpd_handle_t http_server_handle;
    TimerHandle_t telemetry_timer;
} app_httpd_ctx;

void app_httpd_start(bool wifi_setup)
{
    if (NULL == app_httpd_ctx.telemetry_timer) {
        app_httpd_ctx.telemetry_timer = xTimerCreate("TELEMETRY", pdMS_TO_TICKS(TELEMETRY_UPDATE_PERIOD_MS),
                             pdFALSE, NULL, telemetry_timer_cb);
    }
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.server_port = 80;
    config.lru_purge_enable = true;
    config.core_id = 0;
    config.max_uri_handlers = 10;
    ESP_ERROR_CHECK(httpd_start(&app_httpd_ctx.http_server_handle, &config));

    if (wifi_setup) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(app_httpd_ctx.http_server_handle, &handler_wifi_setup));
        ESP_ERROR_CHECK(httpd_register_uri_handler(app_httpd_ctx.http_server_handle, &handler_get_settings));
        ESP_ERROR_CHECK(httpd_register_uri_handler(app_httpd_ctx.http_server_handle, &handler_set_settings));
    } else {
        ESP_ERROR_CHECK(httpd_register_uri_handler(app_httpd_ctx.http_server_handle, &handler_index));
        ESP_ERROR_CHECK(httpd_register_uri_handler(app_httpd_ctx.http_server_handle, &handler_jpeg));
        ESP_ERROR_CHECK(httpd_register_uri_handler(app_httpd_ctx.http_server_handle, &handler_yuv422));
        ESP_ERROR_CHECK(httpd_register_uri_handler(app_httpd_ctx.http_server_handle, &handler_rgb565));
        ESP_ERROR_CHECK(httpd_register_uri_handler(app_httpd_ctx.http_server_handle, &handler_get_settings));
        ESP_ERROR_CHECK(httpd_register_uri_handler(app_httpd_ctx.http_server_handle, &handler_set_settings));
        ESP_ERROR_CHECK(httpd_register_uri_handler(app_httpd_ctx.http_server_handle, &handler_ws_control));
        ESP_ERROR_CHECK(httpd_register_uri_handler(app_httpd_ctx.http_server_handle, &handler_stream));
    }

    app_httpd_ctx.is_running = true;
}

void app_httpd_stop(void)
{
    httpd_stop(app_httpd_ctx.http_server_handle);
    app_httpd_ctx.is_running = false;
}

bool app_httpd_is_running(void)
{
    return app_httpd_ctx.is_running;
}

static esp_err_t http_handler_get_static_page(httpd_req_t *req)
{
    const char *p_page = req->user_ctx;
    if (NULL == p_page) {
        httpd_resp_set_status(req, "404 Not Found");
        return httpd_resp_send(req, NULL, 0);
    }

    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, p_page, strlen(p_page));
}

static esp_err_t http_handler_get_picture(httpd_req_t *req)
{
    pixformat_t target_pixformat;
    const char *http_content_type = NULL;
    http_image_format_t http_format = (http_image_format_t)req->user_ctx;
    switch (http_format) {
    case HTTP_IMAGE_FORMAT_JPEG:
        target_pixformat = PIXFORMAT_JPEG;
        http_content_type = "image/jpeg";
        break;
    case HTTP_IMAGE_FORMAT_YUV422:
        target_pixformat = PIXFORMAT_YUV422;
        http_content_type = "application/octet-stream";
        break;
    case HTTP_IMAGE_FORMAT_RGB565:
        target_pixformat = PIXFORMAT_RGB565;
        http_content_type = "application/octet-stream";
        break;
    default:
        assert(0);
    }
    camera_fb_t *p_fb = ai_camera_get_frame(target_pixformat, pdMS_TO_TICKS(1000));
    if (NULL == p_fb) {
        httpd_resp_set_status(req, "500 Capture timeout");
        httpd_resp_send(req, NULL, 0);
        return ESP_OK;
    }

    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, http_content_type);
    httpd_resp_send(req, (const char *)p_fb->buf, p_fb->len);
    ai_camera_fb_return(p_fb);
    return ESP_OK;
}

static esp_err_t http_handler_get_settings(httpd_req_t *req)
{
    const cJSON *p_root = NULL;

    const char *p_uri_end = strrchr(req->uri, '/') + 1;

    if (0 == strcmp(p_uri_end, "camera")) {
        p_root = ai_camera_settings_get_json();
    } else if (0 == strcmp(p_uri_end, "system")) {
        p_root = system_settings_get_json();
    } else if (0 == strcmp(p_uri_end, "auto_mode")) {
        p_root = auto_mode_settings_get_json();
    } else if (0 == strcmp(p_uri_end, "integrations")) {
        p_root = integrations_settings_get_json();
    }

    char *p_response = cJSON_Print(p_root);
    if (NULL == p_response) {
        goto fail;
    }
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, p_response, strlen(p_response));
    free(p_response);
    return ret;
fail:
    httpd_resp_set_status(req, "500 Internal server error");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t http_handler_set_settings(httpd_req_t *req)
{
    if (req->content_len > SETTINGS_JSON_MAX_SIZE) {
        ESP_LOGE(TAG, "Settings JSON too big %d", req->content_len);
        return ESP_FAIL;
    }

    char *request_buf = malloc(req->content_len);
    if (NULL == request_buf) {
        ESP_LOGE(TAG, "OOM");
        return ESP_FAIL;
    }

    int req_len = httpd_req_recv(req, request_buf, req->content_len);
    if (req_len != req->content_len) {
        ESP_LOGE(TAG, "Failed to read request body");
        free(request_buf);
        return ESP_FAIL;
    }

    cJSON *p_settings = cJSON_ParseWithLength(request_buf, req_len);
    free(request_buf);
    if (p_settings == NULL) {
        ESP_LOGE(TAG, "Failed to parse settings JSON");
        return ESP_FAIL;
    }

    const char *p_uri_end = strrchr(req->uri, '/') + 1;

    if (0 == strcmp(p_uri_end, "camera")) {
        ai_camera_settings_set_json(p_settings);
        ai_camera_settings_apply();
    } else if (0 == strcmp(p_uri_end, "system")) {
        system_settings_set_json(p_settings);
        system_settings_apply();
    } else if (0 == strcmp(p_uri_end, "auto_mode")) {
        auto_mode_settings_set_json(p_settings);
    } else if (0 == strcmp(p_uri_end, "integrations")) {
        integrations_settings_set_json(p_settings);
    }

    cJSON_Delete(p_settings);

    httpd_resp_set_status(req, "200 OK");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t http_handler_ws_control(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake done, a new WS connection was opened");
        send_telemetry((void *)httpd_req_to_sockfd(req));
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    /* Set max_len = 0 to get the frame len */
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);
    if (ws_pkt.len && ws_pkt.len < CONTROL_WS_MAX_FRAME_LEN) {
        /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
        buf = calloc(1, ws_pkt.len + 1);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        /* Set max_len = ws_pkt.len to get the frame payload */
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);
    }
    ESP_LOGI(TAG, "Packet type: %d", ws_pkt.type);
    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
        cJSON *p_root = cJSON_ParseWithLength((const char *)ws_pkt.payload, ws_pkt.len);
        if (NULL == p_root) {
            ESP_LOGE(TAG, "Failed to parse JSON");
            free(buf);
            return ESP_FAIL;
        }
        cJSON *p_camera_imm_config = cJSON_GetObjectItem(p_root, "cameraSettings");
        cJSON *p_cfg = NULL;
        cJSON_ArrayForEach(p_cfg, p_camera_imm_config) {
            ai_camera_settings_apply_single_var(p_cfg);
        }
        cJSON_Delete(p_root);
    }
    free(buf);

    return ESP_OK;
}

static void telemetry_timer_cb(TimerHandle_t handle)
{
    httpd_queue_work(app_httpd_ctx.http_server_handle, send_telemetry, pvTimerGetTimerID(handle));
}

static void send_telemetry(void *arg)
{
    const int fd = (int)arg;
    vTimerSetTimerID(app_httpd_ctx.telemetry_timer, (void *)arg);
    xTimerReset(app_httpd_ctx.telemetry_timer, portMAX_DELAY);

    cJSON *p_root = cJSON_CreateObject();
    if (NULL == p_root) {
        goto fail;
    }
    float tsens_value = 0;
    bsp_temp_sensor_read(&tsens_value);

    cJSON_AddNumberToObject(p_root, "light", ai_camera_get_light_level());
    cJSON_AddNumberToObject(p_root, "motion", 0); // TODO: figure out how to report motion
    cJSON_AddNumberToObject(p_root, "ir", ai_camera_get_ir_state());
    cJSON_AddNumberToObject(p_root, "temp", (int)tsens_value);
    cJSON_AddNumberToObject(p_root, "vbat", bsp_read_vbat());
    cJSON_AddNumberToObject(p_root, "dram", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    cJSON_AddNumberToObject(p_root, "psram", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    char *p_response = cJSON_Print(p_root);
    if (NULL == p_response) {
        goto fail;
    }
    cJSON_Delete(p_root);

    httpd_ws_frame_t ws_pkt = { 0 };
    ws_pkt.payload = (uint8_t*)p_response;
    ws_pkt.len = strlen(p_response);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    const esp_err_t ret = httpd_ws_send_frame_async(app_httpd_ctx.http_server_handle, fd, &ws_pkt);
    if (ESP_OK != ret) {
        ESP_LOGE(TAG, "Failed to send WS frame: %d", ret);
        xTimerStop(app_httpd_ctx.telemetry_timer, portMAX_DELAY);
        httpd_sess_trigger_close(app_httpd_ctx.http_server_handle, fd);
    }
    free(p_response);
    return;
fail:
    cJSON_Delete(p_root);
    ESP_LOGE(TAG, "Failed to generate telemetry JSON");
    httpd_sess_trigger_close(app_httpd_ctx.http_server_handle, fd);
}

static esp_err_t http_handler_get_stream(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Stream handshake done, a new WS connection was opened");
        ai_camera_stop();
        ai_camera_start(AI_CAMERA_PIPELINE_CNN, camera_frame_cb, camera_metadata_cb,
                (void *)httpd_req_to_sockfd(req));
    }

    return ESP_OK;
}

static void camera_metadata_cb(cJSON *p_meta_root, void *p_ctx)
{
    const int fd = (int)p_ctx;

    char *p_response = cJSON_Print(p_meta_root);
    if (NULL == p_response) {
        ESP_LOGE(TAG, "Failed to generate frame meta JSON");
        ai_camera_stop();
        httpd_sess_trigger_close(app_httpd_ctx.http_server_handle, fd);
        return;
    }

    httpd_ws_frame_t ws_pkt = { 0 };
    ws_pkt.payload = (uint8_t*)p_response;
    ws_pkt.len = strlen(p_response);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t ret = httpd_ws_send_frame_async(app_httpd_ctx.http_server_handle, fd, &ws_pkt);
    free(p_response);
    if (ESP_OK != ret) {
        ESP_LOGE(TAG, "Failed to send WS frame: %d", ret);
        ai_camera_stop();
        httpd_sess_trigger_close(app_httpd_ctx.http_server_handle, fd);
        return;
    }
}

static void camera_frame_cb(pixformat_t format, const uint8_t *p_data, uint32_t size,
                                     bool start_of_frame, void *p_ctx)
{
    const int fd = (int)p_ctx;

    httpd_ws_frame_t ws_pkt = { 0 };
    ws_pkt.payload = (uint8_t *)p_data;
    ws_pkt.len = size;
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;
    esp_err_t ret = httpd_ws_send_frame_async(app_httpd_ctx.http_server_handle, fd, &ws_pkt);
    if (ESP_OK != ret) {
        ESP_LOGE(TAG, "Failed to send WS frame: %d", ret);
        ai_camera_stop();
        httpd_sess_trigger_close(app_httpd_ctx.http_server_handle, fd);
    }
}
