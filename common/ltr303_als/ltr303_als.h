#pragma once

#include <stdbool.h>

#define LTR303_MEASUREMENT_TIME_MS 120

typedef enum {
    LTR303_GAIN_1X,
    LTR303_GAIN_2X,
    LTR303_GAIN_4X,
    LTR303_GAIN_8X,
    LTR303_GAIN_48X,
    LTR303_GAIN_96X,
} ltr303_gain_t;

bool ltr_303_als_init(int i2c_bus_id, ltr303_gain_t gain);
bool ltr_303_als_start_measurement(int *p_measurement_time_ms);
bool ltr_303_als_read_measurement(float *p_out);
