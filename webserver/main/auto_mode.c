#include "auto_mode.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "json_settings_helpers.h"
#include "ai_camera.h"
#include "integrations.h"

#define TAG "auto_mode"

static cJSON *p_settings;

static cJSON *auto_mode_settings_get_default(void);

static const char *config_names[AUTO_MODE_CONFIG_MAX] = {
    [AUTO_MODE_CONFIG_ENABLED] = "auto_mode_enabled",
    [AUTO_MODE_CONFIG_PIR_ENABLED] = "pir_wakeup_enabled",
    [AUTO_MODE_CONFIG_TFLITE_TRIGGER_ENABLED] = "tflite_trigger_enabled",
    [AUTO_MODE_CONFIG_RTC_WAKEUP_ENABLED] = "rtc_wakeup_enabled",
    [AUTO_MODE_CONFIG_WAKEUP_PERIOD_SECONDS] = "wakeup_period_seconds",
    [AUTO_MODE_CONFIG_INTEGRATION_ID] = "integration_id",
};

static int default_config[AUTO_MODE_CONFIG_MAX] = {
    [AUTO_MODE_CONFIG_ENABLED] = 0,
    [AUTO_MODE_CONFIG_PIR_ENABLED] = 0,
    [AUTO_MODE_CONFIG_TFLITE_TRIGGER_ENABLED] = 0,
    [AUTO_MODE_CONFIG_RTC_WAKEUP_ENABLED] = 0,
    [AUTO_MODE_CONFIG_WAKEUP_PERIOD_SECONDS] = 0,
    [AUTO_MODE_CONFIG_INTEGRATION_ID] = INTEGRATION_MAX,
};

void auto_mode_init(void)
{
    p_settings = json_settings_load_from_nvs("auto_mode");
    if (NULL == p_settings) {
        p_settings = auto_mode_settings_get_default();
        json_settings_save_to_nvs("auto_mode", p_settings);
    }
}

bool auto_mode_enabled(void)
{
    return cJSON_GetObjectItem(p_settings, config_names[AUTO_MODE_CONFIG_ENABLED])->valueint;
}

int auto_mode_run(bool *p_enable_pir_wakeup)
{
    ai_camera_start(AI_CAMERA_PIPELINE_PASSTHROUGH, NULL, NULL, NULL);
    vTaskDelay(pdMS_TO_TICKS(5000));
    camera_fb_t *p_fb = ai_camera_get_frame(PIXFORMAT_JPEG, pdMS_TO_TICKS(3000));
    if (NULL != p_fb) {
        const cJSON *p_integration_id = cJSON_GetObjectItem(p_settings, config_names[AUTO_MODE_CONFIG_INTEGRATION_ID]);
        if (NULL != p_integration_id) {
            integrations_run(p_integration_id->valueint, p_fb->buf, p_fb->len);
        } else {
            ESP_LOGE(TAG, "No integration assigned");
        }
        ai_camera_fb_return(p_fb);
    } else {
        ESP_LOGI(TAG, "Failed to get a frame");
    }
    *p_enable_pir_wakeup = cJSON_GetObjectItem(p_settings, config_names[AUTO_MODE_CONFIG_PIR_ENABLED])->valueint;
    if (cJSON_GetObjectItem(p_settings, config_names[AUTO_MODE_CONFIG_RTC_WAKEUP_ENABLED])->valueint) {
        return cJSON_GetObjectItem(p_settings, config_names[AUTO_MODE_CONFIG_WAKEUP_PERIOD_SECONDS])->valueint;
    } else {
        return 0;
    }
}

void auto_mode_settings_set_json(const cJSON *p_cfg)
{
    if (!cJSON_Compare(p_settings, p_cfg, false)) {
        cJSON_Delete(p_settings);
        p_settings = cJSON_Duplicate(p_cfg, true);
        json_settings_save_to_nvs("auto_mode", p_settings);
    }
}

const cJSON *auto_mode_settings_get_json(void)
{
    return p_settings;
}

static cJSON *auto_mode_settings_get_default(void)
{
    cJSON *p_root = cJSON_CreateObject();
    if (NULL == p_root) {
        return NULL;
    }
    for (int i = 0; i < AUTO_MODE_CONFIG_MAX; i++) {
        cJSON_AddNumberToObject(p_root, config_names[i], default_config[i]);
    }
    return p_root;
}
