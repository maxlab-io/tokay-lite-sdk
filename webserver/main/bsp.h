#pragma once

#include <stdint.h>
#include <stdbool.h>

#define BSP_I2C_BUS_ID    0

void bsp_init(void (*usr_button_cb)(void));
uint32_t bsp_read_vbat(void);
void bsp_temp_sensor_read(float *p_out);
bool bsp_deep_sleep(uint32_t sleep_duration_seconds);
bool bsp_get_timer_alarm(void);
