#pragma once

#include <stdint.h>
#include <stdbool.h>

void light_sensor_init(int i2c_bus_id);
bool light_sensor_start_measurement(int *p_measurement_time_ms);
bool light_sensor_read_measurement(float *p_out);
