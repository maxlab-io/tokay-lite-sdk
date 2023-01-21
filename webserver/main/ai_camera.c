#include "ai_camera.h"

#include <string.h>
#include <math.h>

#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#include "config.h"
#include "light_sensor.h"
#include "ai_pipeline.h"

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

static config_desc_t config_desc[AI_CAMERA_CONFIG_MAX] = {
    [AI_CAMERA_CONFIG_RESOLUTION] = { "resolution", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_CONTRAST ] = { "contrast", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_BRIGHTNESS] = { "brightness", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_SATURATION] = { "saturation", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_SHARPNESS] = { "sharpness", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_DENOISE] = { "denoise", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_GAINCEILING] = { "gainceiling", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_JPEG_QUALITY] = { "jpeg_quality", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_COLORBAR] = { "colorbar", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_WHITEBAL] = { "whitebal", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_GAIN_CTRL] = { "gain_ctrl", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_EXPOSURE_CTRL] = { "exposure_ctrl", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_HMIRROR] = { "hmirror", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_VFLIP] = { "vflip", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_AEC2] = { "aec2", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_AWB_GAIN] = { "awb_gain", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_AGC_GAIN] = { "agc_gain", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_AEC_VALUE] = { "aec_value", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_SPECIAL_EFFECT] = { "special_effect", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_WB_MODE] = { "wb_mode", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_AE_LEVEL] = { "ae_level", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_DCW] = { "dcw", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_BPC] = { "bpc", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_WPC] = { "wpc", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_RAW_GMA] = { "raw_gma", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_LENC] = { "lenc", CONFIG_TYPE_INT},
    [AI_CAMERA_CONFIG_XCLK_FREQ] = { "xclk_freq", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_IR_MODE] = { "ir_mode", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_IR_LIGHT_THRESH_HIGH] = { "ir_light_thresh_high", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_IR_LIGHT_THRESH_LOW] = { "ir_light_thresh_low", CONFIG_TYPE_INT },
    [AI_CAMERA_CONFIG_IR_BRIGHTNESS] = { "ir_brightness", CONFIG_TYPE_INT },
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
    [AI_CAMERA_CONFIG_DCW] = 0,
    [AI_CAMERA_CONFIG_BPC] = 0,
    [AI_CAMERA_CONFIG_WPC] = 0,
    [AI_CAMERA_CONFIG_RAW_GMA] = 0,
    [AI_CAMERA_CONFIG_LENC] = 0,
    [AI_CAMERA_CONFIG_XCLK_FREQ] = XCLK_DEFAULT_FREQ_HZ,
    [AI_CAMERA_CONFIG_IR_MODE] = AI_CAMERA_IR_MODE_AUTO,
    [AI_CAMERA_CONFIG_IR_LIGHT_THRESH_HIGH] = 100,
    [AI_CAMERA_CONFIG_IR_LIGHT_THRESH_LOW] = 50,
    [AI_CAMERA_CONFIG_IR_BRIGHTNESS] = 255,
};

static config_value_t config_values[AI_CAMERA_CONFIG_MAX];
static config_ctx_t config_ctx = {
    .num_vars = AI_CAMERA_CONFIG_MAX,
    .p_desc = config_desc,
    .p_values = config_values,
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
    .frame_size = FRAMESIZE_UXGA,

    .jpeg_quality = 30,
    .fb_count = 1,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
    .fb_location = CAMERA_FB_IN_PSRAM,
    //.fb_size = 1024 * 120, // TODO: port to esp32-camera upstream
};

typedef enum {
    CAMERA_CMD_APPLY_SETTINGS,
    CAMERA_CMD_GET_JPEG,
    CAMERA_CMD_GET_YUV422,
    CAMERA_CMD_GET_RGB565,
    CAMERA_CMD_START,
    CAMERA_CMD_STOP,

    CAMERA_CMD_MAX = 32
} camera_cmd_t;

static struct {
    bool running;
    uint32_t light_sensor_value;
    ai_camera_ir_state_t ir_state;
    TimerHandle_t ir_mode_timer;
    ai_camera_pipeline_t pipeline;
    ai_camera_frame_cb_t p_frame_cb;
    ai_camera_meta_cb_t p_meta_cb;
    void *p_cb_ctx;
    TaskHandle_t camera_thread;
    EventGroupHandle_t camera_thread_commands;
    ai_camera_stats_t stats;
} camera_ctx;

static void load_config(void);
static void set_default_config(void);
static void ir_cut_on(void);
static void ir_cut_off(void);
static void ir_leds_on(uint8_t duty);
static void ir_leds_off(void);
static void ir_mode_day(void);
static void ir_mode_night(void);
static void update_ir_mode(void);
static void ir_mode_timer_handler(TimerHandle_t timer);
static void camera_thread_entry(void *pvParam);
static camera_fb_t *camera_get_frame(void);
static void camera_thread_sleep(void);

void ai_camera_init(int i2c_bus_id)
{
    set_default_config();
    load_config();

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
    xEventGroupClearBits(camera_ctx.camera_thread_commands, (1 << CAMERA_CMD_STOP));
    xEventGroupSetBits(camera_ctx.camera_thread_commands, (1 << CAMERA_CMD_START));
}

void ai_camera_stop(void)
{
    xEventGroupClearBits(camera_ctx.camera_thread_commands, (1 << CAMERA_CMD_START));
    xEventGroupSetBits(camera_ctx.camera_thread_commands, (1 << CAMERA_CMD_STOP));
    xTimerStop(camera_ctx.ir_mode_timer, portMAX_DELAY); // not reliable, but good for now
    camera_ctx.running = false;
}

camera_fb_t *ai_camera_get_frame(pixformat_t format, TickType_t timeout_ms)
{
    return NULL;
}

void ai_camera_fb_return(camera_fb_t *p_fb)
{
    esp_camera_fb_return(p_fb);
}

void ai_camera_get_stats(ai_camera_stats_t *p_stats_out)
{

}

ai_camera_ir_state_t ai_camera_get_ir_state(void)
{
    return camera_ctx.ir_state;
}

uint32_t ai_camera_get_light_level(void)
{
    return camera_ctx.light_sensor_value;
}

const char *ai_camera_config_get_name(ai_camera_config_t config)
{
    return config_get_name(&config_ctx, (int)config);
}

const void *ai_camera_config_get_value(ai_camera_config_t config)
{
    return config_get_value(&config_ctx, (int)config);
}

int ai_camera_config_get_value_int(ai_camera_config_t config)
{
    return (int)config_get_value(&config_ctx, (int)config);
}

void ai_camera_config_set_value(ai_camera_config_t config, const void *p_value)
{
    config_set_value(&config_ctx, (int)config, p_value);
}

void ai_camera_config_set_value_int(ai_camera_config_t config, int value)
{
    config_set_value(&config_ctx, (int)config, (void *)value);
}

ai_camera_config_t ai_camera_config_get_by_name(const char *name)
{
    return (int)config_get_by_name(&config_ctx, name);
}

config_type_t ai_camera_config_get_type(ai_camera_config_t config)
{
    return config_get_type(&config_ctx, (int)config);
}

void ai_camera_config_apply(void)
{
}

void ai_camera_process_settings_json(const cJSON *p_settings)
{
    cJSON *p_cfg_val = NULL;
    cJSON_ArrayForEach(p_cfg_val, p_settings)
    {
        ai_camera_config_t config = ai_camera_config_get_by_name(p_cfg_val->string);
        if (AI_CAMERA_CONFIG_MAX == config) {
            ESP_LOGE(TAG, "Unknown config variable %s", p_cfg_val->string);
            continue;
        }
        switch (ai_camera_config_get_type(config)) {
        case CONFIG_TYPE_STRING:
            if (!cJSON_IsString(p_cfg_val)) {
                ESP_LOGE(TAG, "Wrong config variable type %s, expected string", p_cfg_val->string);
                continue;
            }
            ai_camera_config_set_value(config, &p_cfg_val->valuestring);
            break;
        case CONFIG_TYPE_INT: {
            if (!cJSON_IsNumber(p_cfg_val)) {
                ESP_LOGE(TAG, "Wrong config variable type %s, expected number", p_cfg_val->string);
                continue;
            }
            ai_camera_config_set_value_int(config, (int)p_cfg_val->valuedouble);
            break;
        }
        default:
            assert(0);
            break;
        }
    }
}

static void load_config(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("config", NVS_READWRITE, &nvs_handle);
    if (ESP_OK != err) {
        ESP_LOGE(TAG, "Failed to open camera settings NVS namespace: %s", esp_err_to_name(err));
    }
    size_t settings_size;
    err = nvs_get_str(nvs_handle, "ai_camera", NULL, &settings_size);
    char* settings_buf = NULL;
    if (ESP_ERR_NVS_INVALID_LENGTH == err) {
        settings_buf = malloc(settings_size);
        err = nvs_get_str(nvs_handle, "ai_camera", settings_buf, &settings_size);
        if (ESP_OK != err) {
            ESP_LOGE(TAG, "Failed to load camera settings JSON: %s", esp_err_to_name(err));
        } else {
            cJSON *p_settings = cJSON_ParseWithLength(settings_buf, settings_size);
            if (p_settings == NULL) {
                ESP_LOGE(TAG, "Failed to parse settings JSON");
            } else {
                ai_camera_process_settings_json(p_settings);
                cJSON_Delete(p_settings);
            }
        }
        free(settings_buf);
    }

    nvs_close(nvs_handle);
}

static void set_default_config(void)
{
    for (int i = 0; i < AI_CAMERA_CONFIG_MAX; i++) {
        ai_camera_config_set_value_int(i, default_config[i]);
    }
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
    ir_leds_on(ai_camera_config_get_value_int(AI_CAMERA_CONFIG_IR_BRIGHTNESS));
}

static void update_ir_mode(void)
{
    if (light_sensor_read(&camera_ctx.light_sensor_value)) {
        if (ai_camera_config_get_value_int(AI_CAMERA_CONFIG_IR_MODE) == AI_CAMERA_IR_MODE_AUTO) {
            if (camera_ctx.light_sensor_value >
                ai_camera_config_get_value_int(AI_CAMERA_CONFIG_IR_LIGHT_THRESH_HIGH)) {
                ir_mode_day();
                camera_ctx.ir_state = AI_CAMERA_IR_STATE_DAY;
            } else if (camera_ctx.light_sensor_value <
                       ai_camera_config_get_value_int(AI_CAMERA_CONFIG_IR_LIGHT_THRESH_LOW)) {
                ir_mode_night();
                camera_ctx.ir_state = AI_CAMERA_IR_STATE_NIGHT;
            }
        }
    }

    if (ai_camera_config_get_value_int(AI_CAMERA_CONFIG_IR_MODE) == AI_CAMERA_IR_MODE_DAY) {
        if (AI_CAMERA_IR_STATE_DAY != camera_ctx.ir_state) {
            ir_mode_day();
            camera_ctx.ir_state = AI_CAMERA_IR_STATE_DAY;
        }
    }
    if (ai_camera_config_get_value_int(AI_CAMERA_CONFIG_IR_MODE) == AI_CAMERA_IR_MODE_NIGHT) {
        if (AI_CAMERA_IR_STATE_NIGHT != camera_ctx.ir_state) {
            ir_mode_night();
            camera_ctx.ir_state = AI_CAMERA_IR_STATE_NIGHT;
        }
    }

    xTimerStart(camera_ctx.ir_mode_timer, portMAX_DELAY);
}

static void ir_mode_timer_handler(TimerHandle_t timer)
{
    update_ir_mode();
}

static void camera_thread_entry(void *pvParam)
{
    esp_err_t err = esp_camera_init(&camera_config);
    if (ESP_OK != err) {
        ESP_LOGE(TAG, "Camera initialization failed: %s", esp_err_to_name(err));
    }
    ai_pipeline_init();
    camera_thread_sleep(); // Wait for ai_camera_start();
    while (1) {
        const uint32_t commands = xEventGroupGetBits(camera_ctx.camera_thread_commands);
        if (commands & (1 << CAMERA_CMD_STOP)) {
            ESP_LOGI(TAG, "Stopping camera thread");
            camera_thread_sleep();
        }
        if (commands & (1 << CAMERA_CMD_APPLY_SETTINGS)) {
            // TODO
        }
        if (commands & (1 << CAMERA_CMD_GET_JPEG)) {
            // TODO
        }
        if (commands & (1 << CAMERA_CMD_GET_YUV422)) {
            // TODO
        }
        if (commands & (1 << CAMERA_CMD_GET_RGB565)) {
            // TODO
        }
        camera_fb_t *p_frame = camera_get_frame();
        if (NULL == p_frame) {
            ESP_LOGE(TAG, "Failed to get frame");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        const ai_camera_pipeline_t current_pipeline = camera_ctx.pipeline;
        if (AI_CAMERA_PIPELINE_DISCARD != current_pipeline) {
            if (AI_CAMERA_PIPELINE_CNN == current_pipeline) {
                ai_pipeline_start(p_frame->buf, p_frame->len);
                ai_pipeline_wait(pdMS_TO_TICKS(600));
                ai_pipeline_get_results();
            }
            if (NULL != camera_ctx.p_frame_cb) {
                camera_ctx.p_frame_cb(p_frame->format, p_frame->buf, p_frame->len, true, camera_ctx.p_cb_ctx);
            }
        }
        ai_camera_fb_return(p_frame);
    }
}

static camera_fb_t *camera_get_frame(void)
{
    if (!camera_ctx.running) {
        return NULL;
    }
    return esp_camera_fb_get();
}

static void camera_thread_sleep(void)
{
    xEventGroupWaitBits(camera_ctx.camera_thread_commands, (1 << CAMERA_CMD_START),
            pdTRUE, pdTRUE, portMAX_DELAY);
}
