#include "auto_mode.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "json_settings_helpers.h"
#include "ai_camera.h"
#include "integrations.h"

#define TAG "auto_mode"

#define NUM_FRAMES_TO_UPLOAD 1

static cJSON *p_settings;

static cJSON *auto_mode_settings_get_default(void);

static const char *config_names[AUTO_MODE_CONFIG_MAX] = {
    [AUTO_MODE_CONFIG_ENABLED] = "auto_mode_enabled",
    [AUTO_MODE_CONFIG_LOW_POWER_ENABLED] = "low_power_enabled",
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
    return json_settings_get_int_or(p_settings, config_names[AUTO_MODE_CONFIG_ENABLED], 0);
}

bool auto_mode_low_power_enabled(void)
{
    return json_settings_get_int_or(p_settings, config_names[AUTO_MODE_CONFIG_LOW_POWER_ENABLED], 0);
}

bool auto_mode_pir_wakeup_enabled(void)
{
    return json_settings_get_int_or(p_settings, config_names[AUTO_MODE_CONFIG_PIR_ENABLED], 0);
}

int auto_mode_get_wakeup_period_seconds(void)
{
    if (json_settings_get_int_or(p_settings, config_names[AUTO_MODE_CONFIG_RTC_WAKEUP_ENABLED], 0)) {
        return json_settings_get_int_or(p_settings, config_names[AUTO_MODE_CONFIG_WAKEUP_PERIOD_SECONDS], 0);
    } else {
        return 0;
    }
}

typedef struct {
    int frame_counter;
    int max_frames;
    void **p_bufs;
    size_t *p_lens;
    void *p_psram_iter;
    SemaphoreHandle_t done_sem;
} frame_ctx_t;

static void frame_cb(pixformat_t format, const uint8_t *p_buf, uint32_t len, void *ctx)
{
    frame_ctx_t *p_frame_ctx = ctx;
    if (p_frame_ctx->frame_counter == p_frame_ctx->max_frames) {
        xSemaphoreGive(p_frame_ctx->done_sem);
    } else {
        memcpy(p_frame_ctx->p_psram_iter, p_buf, len);
        p_frame_ctx->p_bufs[p_frame_ctx->frame_counter] = p_frame_ctx->p_psram_iter;
        p_frame_ctx->p_lens[p_frame_ctx->frame_counter] = len;
        p_frame_ctx->p_psram_iter += len;
        p_frame_ctx->frame_counter++;
    }
}

bool auto_mode_run(void)
{
    void *psram_buf = heap_caps_malloc(1024 * 1024 * 4, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    void *bufs[NUM_FRAMES_TO_UPLOAD] = { 0 };
    size_t lens[NUM_FRAMES_TO_UPLOAD] = { 0 };
    frame_ctx_t frame_ctx = {
        .frame_counter = 0,
        .max_frames = NUM_FRAMES_TO_UPLOAD,
        .p_bufs = bufs,
        .p_lens = lens,
        .p_psram_iter = psram_buf,
        .done_sem = xSemaphoreCreateBinary(),
    };
    ai_camera_start(AI_CAMERA_PIPELINE_PASSTHROUGH, NULL, NULL, NULL);
    vTaskDelay(pdMS_TO_TICKS(500));
    camera_fb_t *p_frame = ai_camera_get_frame(PIXFORMAT_JPEG, pdMS_TO_TICKS(1000));
    if (NULL == p_frame) {
        ESP_LOGE(TAG, "Failed to get camera frame");
    } else {
        frame_cb(p_frame->format, p_frame->buf, p_frame->len, &frame_ctx);
    }
    ai_camera_fb_return(p_frame);
    ai_camera_stop();

    const cJSON *p_integration_id = cJSON_GetObjectItem(p_settings, config_names[AUTO_MODE_CONFIG_INTEGRATION_ID]);
    bool ret = true;
    if (NULL != p_integration_id) {
        if (!integrations_run(p_integration_id->valueint, bufs[0], lens[0])) {
            ret = false;
        }
    } else {
        ESP_LOGE(TAG, "No integration assigned");
    }

    heap_caps_free(psram_buf);
    vSemaphoreDelete(frame_ctx.done_sem);
    return ret;
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
