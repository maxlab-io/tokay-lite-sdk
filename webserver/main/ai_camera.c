#include "ai_camera.h"

#include <string.h>
#include <math.h>

#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#include "config.h"
#include "light_sensor.h"
#include "ai_pipeline.h"
#include "json_settings_helpers.h"

#define CAM_PIN_PWDN   9
#define CAM_PIN_RESET  11
#define CAM_PIN_XCLK   8
#define CAM_PIN_D7     12
#define CAM_PIN_D6     18
#define CAM_PIN_D5     17
#define CAM_PIN_D4     15
#define CAM_PIN_D3     6
#define CAM_PIN_D2     4
#define CAM_PIN_D1     5
#define CAM_PIN_D0     7
#define CAM_PIN_VSYNC  10
#define CAM_PIN_HREF   40
#define CAM_PIN_PCLK   16

#define IRCUT_CTRL_PIN 48 // TODO: update from the latest schematic
#define IR_LED_PWM_PIN 46

#define IR_LED_PWM_FREQ_HZ 1000

#define IRCUT_ENERGIZE_DURATION_US (60 * 1000)

#define XCLK_DEFAULT_FREQ_HZ 15000000

#define IR_MODE_TIMER_PERIOD_MS 1000

#define CAMERA_THREAD_STACK_SIZE (4096)
#define CAMERA_THREAD_PRIORITY   5

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define TAG "ai_camera"

static const char *config_names[AI_CAMERA_CONFIG_MAX] = {
    [AI_CAMERA_CONFIG_RESOLUTION] = "resolution",
    [AI_CAMERA_CONFIG_CONTRAST ] = "contrast",
    [AI_CAMERA_CONFIG_BRIGHTNESS] = "brightness",
    [AI_CAMERA_CONFIG_SATURATION] = "saturation",
    [AI_CAMERA_CONFIG_SHARPNESS] = "sharpness",
    [AI_CAMERA_CONFIG_DENOISE] = "denoise",
    [AI_CAMERA_CONFIG_GAINCEILING] = "gainceiling",
    [AI_CAMERA_CONFIG_JPEG_QUALITY] = "jpeg_quality",
    [AI_CAMERA_CONFIG_COLORBAR] = "colorbar",
    [AI_CAMERA_CONFIG_WHITEBAL] = "whitebal",
    [AI_CAMERA_CONFIG_GAIN_CTRL] = "gain_ctrl",
    [AI_CAMERA_CONFIG_EXPOSURE_CTRL] = "exposure_ctrl",
    [AI_CAMERA_CONFIG_HMIRROR] = "hmirror",
    [AI_CAMERA_CONFIG_VFLIP] = "vflip",
    [AI_CAMERA_CONFIG_AEC2] = "aec2",
    [AI_CAMERA_CONFIG_AWB_GAIN] = "awb_gain",
    [AI_CAMERA_CONFIG_AGC_GAIN] = "agc_gain",
    [AI_CAMERA_CONFIG_AEC_VALUE] = "aec_value",
    [AI_CAMERA_CONFIG_SPECIAL_EFFECT] = "special_effect",
    [AI_CAMERA_CONFIG_WB_MODE] = "wb_mode",
    [AI_CAMERA_CONFIG_AE_LEVEL] = "ae_level",
    [AI_CAMERA_CONFIG_DCW] = "dcw",
    [AI_CAMERA_CONFIG_BPC] = "bpc",
    [AI_CAMERA_CONFIG_WPC] = "wpc",
    [AI_CAMERA_CONFIG_RAW_GMA] = "raw_gma",
    [AI_CAMERA_CONFIG_LENC] = "lenc",
    [AI_CAMERA_CONFIG_XCLK_FREQ] = "xclk_freq",
    [AI_CAMERA_CONFIG_IR_MODE] = "ir_mode",
    [AI_CAMERA_CONFIG_IR_LIGHT_THRESH_HIGH] = "ir_light_thresh_high",
    [AI_CAMERA_CONFIG_IR_LIGHT_THRESH_LOW] = "ir_light_thresh_low",
    [AI_CAMERA_CONFIG_IR_BRIGHTNESS] = "ir_brightness",
};

static int default_config[AI_CAMERA_CONFIG_MAX] = {
    [AI_CAMERA_CONFIG_RESOLUTION] = FRAMESIZE_UXGA,
    [AI_CAMERA_CONFIG_CONTRAST] = 0,
    [AI_CAMERA_CONFIG_BRIGHTNESS] = 0,
    [AI_CAMERA_CONFIG_SATURATION] = 0,
    [AI_CAMERA_CONFIG_SHARPNESS] = 0,
    [AI_CAMERA_CONFIG_DENOISE] = 1,
    [AI_CAMERA_CONFIG_GAINCEILING] = 0,
    [AI_CAMERA_CONFIG_JPEG_QUALITY] = 40,
    [AI_CAMERA_CONFIG_COLORBAR] = 0,
    [AI_CAMERA_CONFIG_WHITEBAL] = 0,
    [AI_CAMERA_CONFIG_GAIN_CTRL] = 0,
    [AI_CAMERA_CONFIG_EXPOSURE_CTRL] = 0,
    [AI_CAMERA_CONFIG_HMIRROR] = 0,
    [AI_CAMERA_CONFIG_VFLIP] = 0,
    [AI_CAMERA_CONFIG_AEC2] = 0,
    [AI_CAMERA_CONFIG_AWB_GAIN] = 0,
    [AI_CAMERA_CONFIG_AGC_GAIN] = 0,
    [AI_CAMERA_CONFIG_AEC_VALUE] = 0,
    [AI_CAMERA_CONFIG_SPECIAL_EFFECT] = 0,
    [AI_CAMERA_CONFIG_WB_MODE] = 0,
    [AI_CAMERA_CONFIG_AE_LEVEL] = 0,
    [AI_CAMERA_CONFIG_DCW] = 1,
    [AI_CAMERA_CONFIG_BPC] = 1,
    [AI_CAMERA_CONFIG_WPC] = 1,
    [AI_CAMERA_CONFIG_RAW_GMA] = 0,
    [AI_CAMERA_CONFIG_LENC] = 1,
    [AI_CAMERA_CONFIG_XCLK_FREQ] = XCLK_DEFAULT_FREQ_HZ,
    [AI_CAMERA_CONFIG_IR_MODE] = AI_CAMERA_IR_MODE_AUTO,
    [AI_CAMERA_CONFIG_IR_LIGHT_THRESH_HIGH] = AI_CAMERA_IR_THRESH_HIGH_DEFAULT,
    [AI_CAMERA_CONFIG_IR_LIGHT_THRESH_LOW] = AI_CAMERA_IR_THRESH_LOW_DEFAULT,
    [AI_CAMERA_CONFIG_IR_BRIGHTNESS] = AI_CAMERA_IR_BRIGHTNESS_DEFAULT,
};

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = -1,
    .pin_sscb_scl = -1,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = XCLK_DEFAULT_FREQ_HZ,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_768X768,

    .jpeg_quality = 10,
    .fb_count = 2,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .fb_location = CAMERA_FB_IN_PSRAM,
    //.fb_size = 1024 * 120, // TODO: port to esp32-camera upstream
};

typedef enum {
    CAMERA_CMD_START,
    CAMERA_CMD_STOP,
    CAMERA_CMD_RESP_STOP_DONE,
    CAMERA_CMD_READ_LIGHT_SENSOR,

    CAMERA_CMD_MAX = 32
} camera_cmd_t;

static struct {
    bool running;
    float light_sensor_value;
    ai_camera_ir_state_t ir_state;
    TimerHandle_t ir_mode_timer;
    ai_camera_pipeline_t pipeline;
    ai_camera_frame_cb_t p_frame_cb;
    ai_camera_meta_cb_t p_meta_cb;
    void *p_cb_ctx;
    TaskHandle_t camera_thread;
    EventGroupHandle_t camera_thread_commands;
    ai_camera_stats_t stats;
    cJSON *p_settings;
    bool settings_require_restart;
} camera_ctx;

static void ir_cut_on(void);
static void ir_cut_off(void);
static void ir_leds_on(uint8_t duty);
static void ir_leds_off(void);
static void ir_mode_day(void);
static void ir_mode_night(void);
static void update_ir_mode(void);
static void ir_mode_timer_handler(TimerHandle_t timer);
static void camera_thread_entry(void *pvParam);
static void camera_thread_sleep(void);
static cJSON *ai_camera_settings_get_default(void);
static int ai_camera_settings_get_value(ai_camera_config_t config);
static bool stop_camera_thread(TickType_t timeout_ticks);
static void resume_camera_thread(void);
static void set_sensor_settings(void);

void ai_camera_init(int i2c_bus_id)
{
    camera_ctx.p_settings = json_settings_load_from_nvs("camera");
    if (NULL == camera_ctx.p_settings) {
        camera_ctx.p_settings = ai_camera_settings_get_default();
        json_settings_save_to_nvs("camera", camera_ctx.p_settings);
    }

    gpio_set_level(IRCUT_CTRL_PIN, 0);
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1LLU << IRCUT_CTRL_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    camera_config.sccb_i2c_port = i2c_bus_id;
    camera_ctx.camera_thread_commands = xEventGroupCreate();

    camera_ctx.ir_mode_timer = xTimerCreate("ir_tmr", pdMS_TO_TICKS(IR_MODE_TIMER_PERIOD_MS),
                             pdFALSE, NULL, ir_mode_timer_handler);

    xTaskCreatePinnedToCore(camera_thread_entry, "AICAM", CAMERA_THREAD_STACK_SIZE,
            NULL, CAMERA_THREAD_PRIORITY, &camera_ctx.camera_thread, 0);
    configASSERT(&camera_ctx.camera_thread);
}

void ai_camera_start(ai_camera_pipeline_t pipeline, ai_camera_frame_cb_t p_frame_cb,
        ai_camera_meta_cb_t p_meta_cb, void *p_ctx)
{
    camera_ctx.running = true;
    camera_ctx.pipeline = pipeline;
    camera_ctx.p_frame_cb = p_frame_cb;
    camera_ctx.p_meta_cb = p_meta_cb;
    camera_ctx.p_cb_ctx = p_ctx;
    update_ir_mode();
    xEventGroupSetBits(camera_ctx.camera_thread_commands, (1 << CAMERA_CMD_START));
}

void ai_camera_stop(void)
{
    xEventGroupSetBits(camera_ctx.camera_thread_commands, (1 << CAMERA_CMD_STOP));
    xEventGroupWaitBits(camera_ctx.camera_thread_commands,
            (1 << CAMERA_CMD_RESP_STOP_DONE), pdTRUE, pdTRUE, portMAX_DELAY);
    xTimerStop(camera_ctx.ir_mode_timer, portMAX_DELAY); // not reliable, but good for now
    camera_ctx.running = false;
}

camera_fb_t *ai_camera_get_frame(pixformat_t format, TickType_t timeout_ms)
{
    if (camera_ctx.running) {
        return NULL;
    }
    if (!stop_camera_thread(pdMS_TO_TICKS(5000))) {
        return NULL;
    }
    if (format != PIXFORMAT_JPEG) {
        // This is far from perfect:
        // the reinit sequence is required to change the DMA buffer size
        // and this also resets sensor AE and AWB states, producing bad quality images
        esp_camera_deinit();
        camera_config.pixel_format = format;
        esp_camera_init(&camera_config);
    }
    // TODO: incorporate timeout
    return esp_camera_fb_get();
}

void ai_camera_fb_return(camera_fb_t *p_fb)
{
    esp_camera_fb_return(p_fb);
    if (camera_config.pixel_format != PIXFORMAT_JPEG) {
        esp_camera_deinit();
        camera_config.pixel_format = PIXFORMAT_JPEG;
        esp_camera_init(&camera_config);
    }
    resume_camera_thread();
    xEventGroupSetBits(camera_ctx.camera_thread_commands, (1 << CAMERA_CMD_START));
}

void ai_camera_get_stats(ai_camera_stats_t *p_stats_out)
{

}

ai_camera_ir_state_t ai_camera_get_ir_state(void)
{
    return camera_ctx.ir_state;
}

float ai_camera_get_light_level(void)
{
    return camera_ctx.light_sensor_value;
}

void ai_camera_settings_set_json(const cJSON *p_settings)
{
    if (cJSON_Compare(camera_ctx.p_settings, p_settings, false)) {
        return;
    }
    camera_ctx.settings_require_restart = false;
    const pixformat_t current_res = ai_camera_settings_get_value(AI_CAMERA_CONFIG_RESOLUTION);
    const pixformat_t new_res = json_settings_get_int_or(camera_ctx.p_settings,
            config_names[AI_CAMERA_CONFIG_RESOLUTION], default_config[AI_CAMERA_CONFIG_RESOLUTION]);
    if (current_res != new_res) {
        camera_ctx.settings_require_restart = true;
    }
    const int current_xclk = ai_camera_settings_get_value(AI_CAMERA_CONFIG_XCLK_FREQ);
    const pixformat_t new_xclk = json_settings_get_int_or(camera_ctx.p_settings,
            config_names[AI_CAMERA_CONFIG_XCLK_FREQ], default_config[AI_CAMERA_CONFIG_XCLK_FREQ]);
    if (current_xclk != new_xclk) {
        camera_ctx.settings_require_restart = true;
    }

    cJSON_Delete(camera_ctx.p_settings);
    camera_ctx.p_settings = cJSON_Duplicate(p_settings, true);
    json_settings_save_to_nvs("camera", camera_ctx.p_settings);
}

const cJSON *ai_camera_settings_get_json(void)
{
    return camera_ctx.p_settings;
}

void ai_camera_settings_apply(void)
{
    if (!camera_ctx.running) {
        camera_config.frame_size = ai_camera_settings_get_value(AI_CAMERA_CONFIG_RESOLUTION);
        camera_config.xclk_freq_hz = ai_camera_settings_get_value(AI_CAMERA_CONFIG_XCLK_FREQ);
        return;
    }
    if (camera_ctx.settings_require_restart) {
        if (!stop_camera_thread(pdMS_TO_TICKS(5000))) {
            ESP_LOGE(TAG, "Failed to stop camera thread");
            return;
        }
        camera_config.frame_size = ai_camera_settings_get_value(AI_CAMERA_CONFIG_RESOLUTION);
        camera_config.xclk_freq_hz = ai_camera_settings_get_value(AI_CAMERA_CONFIG_XCLK_FREQ);
        esp_camera_deinit();
        esp_camera_init(&camera_config);
    }

    set_sensor_settings();
    resume_camera_thread();
}

static void ir_cut_on(void)
{
    gpio_set_level(IRCUT_CTRL_PIN, 0);
}

static void ir_cut_off(void)
{
    gpio_set_level(IRCUT_CTRL_PIN, 1);
}

static void ir_leds_on(uint8_t duty)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_1,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .freq_hz          = IR_LED_PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    ESP_ERROR_CHECK(ret);

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER_1,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = IR_LED_PWM_PIN,
        .duty           = duty,
        .hpoint         = 0
    };
    ret = ledc_channel_config(&ledc_channel);
    ESP_ERROR_CHECK(ret);
}

static void ir_leds_off(void)
{
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
}

static void ir_mode_day(void)
{
    ir_cut_on();
    ir_leds_off();
}

static void ir_mode_night(void)
{
    ir_cut_off();
    ir_leds_on(ai_camera_settings_get_value(AI_CAMERA_CONFIG_IR_BRIGHTNESS));
}

static void update_ir_mode(void)
{
    const ai_camera_ir_mode_t ir_mode = ai_camera_settings_get_value(AI_CAMERA_CONFIG_IR_MODE);
    const ai_camera_ir_mode_t ir_light_thresh_high =
        ai_camera_settings_get_value(AI_CAMERA_CONFIG_IR_LIGHT_THRESH_HIGH);
    const ai_camera_ir_mode_t ir_light_thresh_low =
        ai_camera_settings_get_value(AI_CAMERA_CONFIG_IR_LIGHT_THRESH_LOW);
    if (light_sensor_read(&camera_ctx.light_sensor_value)) {
        if (AI_CAMERA_IR_MODE_AUTO == ir_mode) {
            if (camera_ctx.light_sensor_value >
                ir_light_thresh_high) {
                if (camera_ctx.ir_state != AI_CAMERA_IR_STATE_DAY) {
                    ESP_LOGI(TAG, "Switching to day");
                    ir_mode_day();
                    camera_ctx.ir_state = AI_CAMERA_IR_STATE_DAY;
                }
            } else if (camera_ctx.light_sensor_value <
                       ir_light_thresh_low) {
                if (camera_ctx.ir_state != AI_CAMERA_IR_STATE_NIGHT) {
                    ESP_LOGI(TAG, "Switching to night");
                    ir_mode_night();
                    camera_ctx.ir_state = AI_CAMERA_IR_STATE_NIGHT;
                }
            }
        }
    }

    if (ir_mode == AI_CAMERA_IR_MODE_DAY) {
        if (AI_CAMERA_IR_STATE_DAY != camera_ctx.ir_state) {
            ir_mode_day();
            camera_ctx.ir_state = AI_CAMERA_IR_STATE_DAY;
        }
    } else if (ir_mode == AI_CAMERA_IR_MODE_NIGHT) {
        if (AI_CAMERA_IR_STATE_NIGHT != camera_ctx.ir_state) {
            ir_mode_night();
            camera_ctx.ir_state = AI_CAMERA_IR_STATE_NIGHT;
        }
    }

    xTimerStart(camera_ctx.ir_mode_timer, portMAX_DELAY);
}

static void ir_mode_timer_handler(TimerHandle_t timer)
{
    xEventGroupSetBits(camera_ctx.camera_thread_commands, (1 << CAMERA_CMD_READ_LIGHT_SENSOR));
}

static void camera_thread_entry(void *pvParam)
{
    esp_err_t err = esp_camera_init(&camera_config);
    if (ESP_OK != err) {
        ESP_LOGE(TAG, "Camera initialization failed: %s", esp_err_to_name(err));
    }
    set_sensor_settings();
    ai_pipeline_init();
    cJSON *p_meta_root = cJSON_CreateObject();
    cJSON_AddItemToObject(p_meta_root, "cnn_output", ai_pipeline_get_results());
    cJSON *p_stats_fps = cJSON_AddNumberToObject(p_meta_root, "fps", 0);
    int64_t total_frame_time_us = 0;
    uint32_t total_frames = 0;
    while (1) {
        const uint32_t commands = xEventGroupWaitBits(camera_ctx.camera_thread_commands,
                (1 << CAMERA_CMD_STOP), pdTRUE, pdTRUE, 0);
        if (commands & (1 << CAMERA_CMD_STOP)) {
            ESP_LOGI(TAG, "Stopping camera thread");
            camera_thread_sleep();
            ESP_LOGI(TAG, "Resuming camera thread");
        }
        if (commands & (1 << CAMERA_CMD_READ_LIGHT_SENSOR)) {
            update_ir_mode();
        }
        const int64_t start_frame_us = esp_timer_get_time();
        camera_fb_t *p_frame = esp_camera_fb_get();
        if (NULL == p_frame) {
            ESP_LOGE(TAG, "Failed to get frame");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        const ai_camera_pipeline_t current_pipeline = camera_ctx.pipeline;
        if (AI_CAMERA_PIPELINE_DISCARD != current_pipeline) {
            if (AI_CAMERA_PIPELINE_CNN == current_pipeline) {
                ai_pipeline_start(p_frame->buf, p_frame->len);
            }
            if (NULL != camera_ctx.p_frame_cb) {
                camera_ctx.p_frame_cb(p_frame->format, p_frame->buf, p_frame->len, true, camera_ctx.p_cb_ctx);
            }
            if (AI_CAMERA_PIPELINE_CNN == current_pipeline) {
                ai_pipeline_wait_decode_finish(pdMS_TO_TICKS(600));
            }
            esp_camera_fb_return(p_frame);
            if (AI_CAMERA_PIPELINE_CNN == current_pipeline) {
                ai_pipeline_wait(portMAX_DELAY);
            }
        } else {
            esp_camera_fb_return(p_frame);
        }
        const int64_t end_frame_us = esp_timer_get_time();
        total_frame_time_us += end_frame_us - start_frame_us;
        total_frames++;
        if (NULL != camera_ctx.p_meta_cb) {
            p_stats_fps->valuedouble = (total_frames * 1000000.f) / total_frame_time_us;
            camera_ctx.p_meta_cb(p_meta_root, camera_ctx.p_cb_ctx);
        }
        if (total_frames % 5 == 0) {
            total_frames = 0;
            total_frame_time_us = 0;
        }
    }
}

static void camera_thread_sleep(void)
{
    xEventGroupSetBits(camera_ctx.camera_thread_commands, (1 << CAMERA_CMD_RESP_STOP_DONE));
    xEventGroupWaitBits(camera_ctx.camera_thread_commands, (1 << CAMERA_CMD_START),
            pdTRUE, pdTRUE, portMAX_DELAY);
}

static cJSON *ai_camera_settings_get_default(void)
{
    cJSON *p_root = cJSON_CreateObject();
    if (NULL == p_root) {
        return NULL;
    }
    for (int i = 0; i < AI_CAMERA_CONFIG_MAX; i++) {
        cJSON_AddNumberToObject(p_root, config_names[i], default_config[i]);
    }
    return p_root;
}

static int ai_camera_settings_get_value(ai_camera_config_t config)
{
    return json_settings_get_int_or(camera_ctx.p_settings, config_names[config], default_config[config]);
}

static bool stop_camera_thread(TickType_t timeout_ticks)
{
    xEventGroupSetBits(camera_ctx.camera_thread_commands, (1 << CAMERA_CMD_STOP));
    const EventBits_t bits = xEventGroupWaitBits(camera_ctx.camera_thread_commands,
            (1 << CAMERA_CMD_RESP_STOP_DONE), pdTRUE, pdTRUE, timeout_ticks);
    if (0 == (bits & (1 << CAMERA_CMD_RESP_STOP_DONE))) {
        xEventGroupSetBits(camera_ctx.camera_thread_commands, (1 << CAMERA_CMD_START));
        return false;
    }
    return true;
}

static void resume_camera_thread(void)
{
    xEventGroupSetBits(camera_ctx.camera_thread_commands, (1 << CAMERA_CMD_START));
}

static void set_sensor_settings(void)
{
    sensor_t *p_sensor = esp_camera_sensor_get();
    if (NULL == p_sensor) {
        ESP_LOGE(TAG, "Failed to get sensor object");
        return;
    }

    if (0 != p_sensor->set_contrast(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_CONTRAST))) {
        goto error;
    }
    if (0 != p_sensor->set_brightness(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_BRIGHTNESS))) {
        goto error;
    }
    if (0 != p_sensor->set_saturation(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_SATURATION))) {
        goto error;
    }
    if (0 != p_sensor->set_sharpness(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_SHARPNESS))) {
        goto error;
    }
    if (0 != p_sensor->set_denoise(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_DENOISE))) {
        goto error;
    }
    if (0 != p_sensor->set_gainceiling(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_GAINCEILING))) {
        goto error;
    }
    if (0 != p_sensor->set_quality(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_JPEG_QUALITY))) {
        goto error;
    }
    if (0 != p_sensor->set_colorbar(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_COLORBAR))) {
        goto error;
    }
    if (0 != p_sensor->set_whitebal(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_WHITEBAL))) {
        goto error;
    }
    if (0 != p_sensor->set_gain_ctrl(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_GAIN_CTRL))) {
        goto error;
    }
    if (0 != p_sensor->set_exposure_ctrl(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_EXPOSURE_CTRL))) {
        goto error;
    }
    if (0 != p_sensor->set_hmirror(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_HMIRROR))) {
        goto error;
    }
    if (0 != p_sensor->set_vflip(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_VFLIP))) {
        goto error;
    }
    if (0 != p_sensor->set_aec2(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_AEC2))) {
        goto error;
    }
    if (0 != p_sensor->set_awb_gain(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_AWB_GAIN))) {
        goto error;
    }
    if (0 != p_sensor->set_agc_gain(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_AGC_GAIN))) {
        goto error;
    }
    if (0 != p_sensor->set_aec_value(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_AEC_VALUE))) {
        goto error;
    }
    if (0 != p_sensor->set_special_effect(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_SPECIAL_EFFECT))) {
        goto error;
    }
    if (0 != p_sensor->set_wb_mode(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_WB_MODE))) {
        goto error;
    }
    if (0 != p_sensor->set_ae_level(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_AE_LEVEL))) {
        goto error;
    }
    if (0 != p_sensor->set_dcw(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_DCW))) {
        goto error;
    }
    if (0 != p_sensor->set_bpc(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_BPC))) {
        goto error;
    }
    if (0 != p_sensor->set_wpc(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_WPC))) {
        goto error;
    }
    if (0 != p_sensor->set_raw_gma(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_RAW_GMA))) {
        goto error;
    }
    if (0 != p_sensor->set_lenc(p_sensor, ai_camera_settings_get_value(AI_CAMERA_CONFIG_LENC))) {
        goto error;
    }
    ESP_LOGI(TAG, "Sensor settings applied successfully");
    return;
 error:
    ESP_LOGE(TAG, "Failed to configure sensor");
}
