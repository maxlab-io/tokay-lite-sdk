#include "esp_camera.h"
#include "sensor.h"

#include "esp_log.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/temperature_sensor.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "ai_camera.h"
#include "config.h"
#include "pir.h"

#define PART_BOUNDARY        "123456789000000000000987654321"
#define _STREAM_CONTENT_TYPE "multipart/x-mixed-replace;boundary=" PART_BOUNDARY
#define _STREAM_BOUNDARY     "\r\n--" PART_BOUNDARY "\r\n"
#define _STREAM_PART         "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n"

#define STREAMING_HTTP_SERVER_PORT 1111
#define SETTINGS_JSON_MAX_SIZE     1024

#define TAG "webserver"

#define I2C_BUS_ID    0
#define I2C_FREQUENCY 100000

#define I2C_SDA_PIN 13
#define I2C_SCL_PIN 3

#define BATT_DETECT_ADC_CHANNEL  ADC_CHANNEL_0 // GPIO1
#define BATT_DETECT_MV_TO_BATT_MV(x) (x * 2)

typedef enum {
    HTTP_IMAGE_FORMAT_JPEG,
    HTTP_IMAGE_FORMAT_YUV422,
    HTTP_IMAGE_FORMAT_RGB565,
} http_image_format_t;

typedef enum {
    SYSTEM_CONFIG_PIR_ENABLED,
    SYSTEM_CONFIG_MAX,
} system_config_t;

static esp_err_t http_handler_get_static_page(httpd_req_t *req);
static esp_err_t http_handler_get_picture(httpd_req_t *req);
static esp_err_t http_handler_get_stream(httpd_req_t *req);
static esp_err_t http_handler_get_settings(httpd_req_t *req);
static esp_err_t http_handler_set_settings(httpd_req_t *req);
static esp_err_t http_handler_get_sensors(httpd_req_t *req);

const char *system_config_get_name(system_config_t config);
const void *system_config_get_value(system_config_t config);
void system_config_set_value(system_config_t config, const void *p_value);
system_config_t system_config_get_by_name(const char *name);
config_type_t system_config_get_type(system_config_t config);
void system_config_apply(void);

extern const char index_html_start[] asm("_binary_index_html_start");

static const httpd_uri_t handler_index = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = http_handler_get_static_page,
    .user_ctx  = (void *)index_html_start,
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

static const httpd_uri_t handler_stream = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = http_handler_get_stream,
    .user_ctx  = NULL,
};

static const httpd_uri_t handler_get_settings = {
    .uri       = "/settings",
    .method    = HTTP_GET,
    .handler   = http_handler_get_settings,
    .user_ctx  = NULL,
};

static const httpd_uri_t handler_set_settings = {
    .uri       = "/settings",
    .method    = HTTP_POST,
    .handler   = http_handler_set_settings,
    .user_ctx  = NULL,
};

static const httpd_uri_t handler_get_sensors = {
    .uri       = "/sensors",
    .method    = HTTP_GET,
    .handler   = http_handler_get_sensors,
    .user_ctx  = NULL,
};

static httpd_handle_t http_server_handle;
static httpd_handle_t streaming_server_handle;

static temperature_sensor_handle_t temp_sensor;

static config_desc_t system_config_desc[SYSTEM_CONFIG_MAX] = {
    [SYSTEM_CONFIG_PIR_ENABLED] = { "pir_en", CONFIG_TYPE_INT },
};
static config_value_t system_config_values[SYSTEM_CONFIG_MAX];
static config_ctx_t system_config_ctx = {
    .num_vars = SYSTEM_CONFIG_MAX,
    .p_desc = system_config_desc,
    .p_values = system_config_values,
};

static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t vbat_adc_cali_handle;

static bool init_adc_calibration(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void init_vbat_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, BATT_DETECT_ADC_CHANNEL, &config));

    init_adc_calibration(ADC_UNIT_1, ADC_ATTEN_DB_11, &vbat_adc_cali_handle);
}

static uint32_t measure_vbat(void)
{
    int ret = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, BATT_DETECT_ADC_CHANNEL, &ret));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(vbat_adc_cali_handle, ret, &ret));
    return BATT_DETECT_MV_TO_BATT_MV(ret);
}

static void start_httpd(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = NULL;
    config.server_port = 80;
    config.lru_purge_enable = true;
    config.core_id = 0;
    ESP_ERROR_CHECK(httpd_start(&http_server_handle, &config));

    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &handler_index));
    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &handler_jpeg));
    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &handler_yuv422));
    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &handler_rgb565));
    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &handler_get_settings));
    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &handler_set_settings));
    ESP_ERROR_CHECK(httpd_register_uri_handler(http_server_handle, &handler_get_sensors));

    // Run a separate httpd instance for JPEG streaming
    httpd_config_t streamer_config = HTTPD_DEFAULT_CONFIG();
    streamer_config.uri_match_fn = NULL;
    streamer_config.server_port = STREAMING_HTTP_SERVER_PORT;
    streamer_config.lru_purge_enable = true;
    streamer_config.core_id = 0;
    ESP_ERROR_CHECK(httpd_start(&streaming_server_handle, &streamer_config));
    ESP_ERROR_CHECK(httpd_register_uri_handler(streaming_server_handle, &handler_stream));
}

static void pir_motion_callback(void *ctx)
{
    // TODO: store events in a list to be reported to the front-end
}

void app_main(void)
{
    gpio_install_isr_service(0);
    pir_init(pir_motion_callback, NULL);

    ESP_ERROR_CHECK(i2c_driver_install(I2C_BUS_ID, I2C_MODE_MASTER, 0, 0, 0));

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQUENCY,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_BUS_ID, &conf));

    init_vbat_adc();

    ai_camera_init(I2C_BUS_ID);
    ai_camera_start(PIXFORMAT_JPEG); // TODO: manage FPS?

    temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 50);
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));

    start_httpd();
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
    ai_camera_set_pixformat(target_pixformat);
    camera_fb_t *p_fb = ai_camera_get_frame(pdMS_TO_TICKS(500));
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

static esp_err_t http_handler_get_stream(httpd_req_t *req)
{
    ai_camera_set_pixformat(PIXFORMAT_JPEG);
    esp_err_t ret = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (ret != ESP_OK) {
        return ret;
    }

    char part_buf[64];

    while (true) {
        camera_fb_t *p_fb = ai_camera_get_frame(pdMS_TO_TICKS(500));
        if (NULL == p_fb) {
            httpd_resp_set_status(req, "500 Capture timeout");
            httpd_resp_send(req, NULL, 0);
            return ESP_OK;
        }

        ret = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        if (ESP_OK == ret) {
            const int len = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, p_fb->len);
            if (len >= 0 && len <= sizeof(part_buf)) {
                ret = httpd_resp_send_chunk(req, (const char *)part_buf, len);
            } else {
                ret = ESP_FAIL;
            }
        }
        if (ESP_OK == ret) {
            ret = httpd_resp_send_chunk(req, (const char *)p_fb->buf, p_fb->len);
        }
        ai_camera_fb_return(p_fb);
        if (ESP_OK != ret) {
            return ret;
        }
    }
    return ESP_OK;
}

static esp_err_t http_handler_get_settings(httpd_req_t *req)
{
    cJSON *p_root = cJSON_CreateObject();
    if (NULL == p_root) {
        goto fail;
    }

    cJSON *p_sensor_config = cJSON_AddObjectToObject(p_root, "sensor_config");
    if (NULL == p_sensor_config) {
        goto fail;
    }
    for (int i = 0; i < AI_CAMERA_CONFIG_MAX; i++) {
        const char *p_name = ai_camera_config_get_name(i);
        switch (ai_camera_config_get_type(i)) {
        case CONFIG_TYPE_STRING:
            cJSON_AddStringToObject(p_sensor_config, p_name,
                                                *(const char **)ai_camera_config_get_value(i));
            break;
        case CONFIG_TYPE_INT:
            cJSON_AddNumberToObject(p_sensor_config, p_name,
                                                *(int *)ai_camera_config_get_value(i));
            break;
        default:
            assert(0);
            break;
        }
    }

    cJSON *p_system_config = cJSON_AddObjectToObject(p_root, "system_config");
    if (NULL == p_system_config) {
        goto fail;
    }
    for (int i = 0; i < SYSTEM_CONFIG_MAX; i++) {
        const char *p_name = config_get_name(&system_config_ctx, i);
        switch (config_get_type(&system_config_ctx, i)) {
        case CONFIG_TYPE_STRING:
            cJSON_AddStringToObject(p_sensor_config, p_name,
                                    *(const char **)config_get_value(&system_config_ctx, i));
            break;
        case CONFIG_TYPE_INT:
            cJSON_AddNumberToObject(p_sensor_config, p_name,
                                    *(int *)config_get_value(&system_config_ctx, i));
            break;
        default:
            assert(0);
            break;
        }
    }

    char *p_response = cJSON_Print(p_root);
    if (NULL == p_response) {
        goto fail;
    }
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, p_response, strlen(p_response));
    free(p_response);
    cJSON_Delete(p_root);
    return ret;
fail:
    cJSON_Delete(p_root);
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

    cJSON *p_sensor_config = cJSON_GetObjectItemCaseSensitive(p_settings, "sensor_config");
    ai_camera_process_settings_json(p_sensor_config);

    cJSON *p_system_config = cJSON_GetObjectItemCaseSensitive(p_settings, "system_config");
    cJSON *p_cfg_val = NULL;
    cJSON_ArrayForEach(p_cfg_val, p_system_config)
    {
        system_config_t config = config_get_by_name(&system_config_ctx, p_cfg_val->string);
        if (SYSTEM_CONFIG_MAX == config) {
            ESP_LOGE(TAG, "Unknown config variable %s", p_cfg_val->string);
            continue;
        }
        switch (config_get_type(&system_config_ctx, config)) {
        case CONFIG_TYPE_STRING:
            if (!cJSON_IsString(p_cfg_val)) {
                ESP_LOGE(TAG, "Wrong config variable type %s, expected string", p_cfg_val->string);
                continue;
            }
            config_set_value(&system_config_ctx, config, &p_cfg_val->valuestring);
            break;
        case CONFIG_TYPE_INT: {
            if (!cJSON_IsNumber(p_cfg_val)) {
                ESP_LOGE(TAG, "Wrong config variable type %s, expected number", p_cfg_val->string);
                continue;
            }
            const int valueint = (int)p_cfg_val->valuedouble;
            config_set_value(&system_config_ctx, config, &valueint);
            break;
        }
        default:
            assert(0);
            break;
        }
    }

    cJSON_Delete(p_settings);

    ai_camera_config_apply();
    system_config_apply();

    httpd_resp_set_status(req, "200 OK");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t http_handler_get_sensors(httpd_req_t *req)
{
    cJSON *p_root = cJSON_CreateObject();
    if (NULL == p_root) {
        goto fail;
    }
    float tsens_value = 0;
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &tsens_value));

    cJSON_AddNumberToObject(p_root, "light", ai_camera_get_light_level());
    cJSON_AddNumberToObject(p_root, "motion", 0); // TODO: figure out how to report motion
    cJSON_AddNumberToObject(p_root, "ir", ai_camera_get_ir_state());
    cJSON_AddNumberToObject(p_root, "temp", tsens_value);
    cJSON_AddNumberToObject(p_root, "vbat", measure_vbat());

    char *p_response = cJSON_Print(p_root);
    if (NULL == p_response) {
        goto fail;
    }
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, p_response, strlen(p_response));
    free(p_response);
    cJSON_Delete(p_root);
    return ret;
fail:
    cJSON_Delete(p_root);
    httpd_resp_set_status(req, "500 Internal server error");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

void system_config_apply(void)
{
    // TODO
}
