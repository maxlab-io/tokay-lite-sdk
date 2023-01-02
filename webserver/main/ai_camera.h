#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "cJSON.h"

#include "config.h"

typedef enum {
    AI_CAMERA_IR_MODE_AUTO,
    AI_CAMERA_IR_MODE_DAY,
    AI_CAMERA_IR_MODE_NIGHT,
} ai_camera_ir_mode_t;

typedef enum {
    AI_CAMERA_IR_STATE_DAY,
    AI_CAMERA_IR_STATE_NIGHT,
} ai_camera_ir_state_t;

// All values are integers, some of them refer to the respective enumerations from the esp_camera
// or ai_camera.h, the rest are literal values.
typedef enum {
    AI_CAMERA_CONFIG_RESOLUTION,
    AI_CAMERA_CONFIG_CONTRAST,
    AI_CAMERA_CONFIG_BRIGHTNESS,
    AI_CAMERA_CONFIG_SATURATION,
    AI_CAMERA_CONFIG_SHARPNESS,
    AI_CAMERA_CONFIG_DENOISE,
    AI_CAMERA_CONFIG_GAINCEILING,
    AI_CAMERA_CONFIG_JPEG_QUALITY,
    AI_CAMERA_CONFIG_COLORBAR,
    AI_CAMERA_CONFIG_WHITEBAL,
    AI_CAMERA_CONFIG_GAIN_CTRL,
    AI_CAMERA_CONFIG_EXPOSURE_CTRL,
    AI_CAMERA_CONFIG_HMIRROR,
    AI_CAMERA_CONFIG_VFLIP,
    AI_CAMERA_CONFIG_AEC2,
    AI_CAMERA_CONFIG_AWB_GAIN,
    AI_CAMERA_CONFIG_AGC_GAIN,
    AI_CAMERA_CONFIG_AEC_VALUE,
    AI_CAMERA_CONFIG_SPECIAL_EFFECT,
    AI_CAMERA_CONFIG_WB_MODE,
    AI_CAMERA_CONFIG_AE_LEVEL,
    AI_CAMERA_CONFIG_DCW,
    AI_CAMERA_CONFIG_BPC,
    AI_CAMERA_CONFIG_WPC,
    AI_CAMERA_CONFIG_RAW_GMA,
    AI_CAMERA_CONFIG_LENC,
    AI_CAMERA_CONFIG_XCLK_FREQ,
    AI_CAMERA_CONFIG_IR_MODE, // See ai_camera_ir_mode_t
    AI_CAMERA_CONFIG_IR_LIGHT_THRESH_HIGH,
    AI_CAMERA_CONFIG_IR_LIGHT_THRESH_LOW,
    AI_CAMERA_CONFIG_IR_BRIGHTNESS,

    AI_CAMERA_CONFIG_MAX
} ai_camera_config_t;

void ai_camera_init(int i2c_bus_id);
void ai_camera_start(pixformat_t pixformat);
void ai_camera_stop(void);

camera_fb_t *ai_camera_get_frame(TickType_t timeout_ms);
void ai_camera_fb_return(camera_fb_t *p_fb);

void ai_camera_set_pixformat(pixformat_t pixformat);

ai_camera_ir_state_t ai_camera_get_ir_state(void);
uint32_t ai_camera_get_light_level(void);

const char *ai_camera_config_get_name(ai_camera_config_t config);
const void *ai_camera_config_get_value(ai_camera_config_t config);
int ai_camera_config_get_value_int(ai_camera_config_t config);
void ai_camera_config_set_value(ai_camera_config_t config, const void *p_value);
void ai_camera_config_set_value_int(ai_camera_config_t config, int value);
ai_camera_config_t ai_camera_config_get_by_name(const char *name);
config_type_t ai_camera_config_get_type(ai_camera_config_t config);
void ai_camera_config_apply(void);

void ai_camera_process_settings_json(const cJSON *p_settings);
