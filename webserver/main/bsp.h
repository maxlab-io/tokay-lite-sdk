#pragma once

#include <stdint.h>

#define BSP_I2C_BUS_ID    0

void bsp_init(void (*usr_button_cb)(void));
uint32_t bsp_read_vbat(void);
void bsp_temp_sensor_read(float *p_out);
