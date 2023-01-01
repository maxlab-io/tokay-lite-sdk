#include "ai_camera.h"

void ai_camera_init(void)
{

}

void ai_camera_start(pixformat_t pixformat)
{

}

void ai_camera_stop(void)
{

}

camera_fb_t *ai_camera_get_frame(TickType_t timeout_ms)
{
    return NULL;
}

void ai_camera_fb_return(camera_fb_t *p_fb)
{

}

void ai_camera_set_pixformat(pixformat_t pixformat)
{

}

bool ai_camera_get_ir_state(void)
{
    return false;
}

uint32_t ai_camera_read_light_sensor(void)
{
    return 0;
}

const char *ai_camera_config_get_name(ai_camera_config_t config)
{
    return "";
}

const void *ai_camera_config_get_value(ai_camera_config_t config)
{
    return NULL;
}

void ai_camera_config_set_value(ai_camera_config_t config, const void *p_value)
{

}

ai_camera_config_t ai_camera_config_get_by_name(const char *name)
{
    return AI_CAMERA_CONFIG_MAX;
}

config_type_t ai_camera_config_get_type(ai_camera_config_t config)
{
    return CONFIG_TYPE_STRING;
}

void ai_camera_config_apply(void)
{

}
