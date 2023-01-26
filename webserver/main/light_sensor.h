#pragma once

#include <stdint.h>
#include <stdbool.h>

void light_sensor_init(int i2c_bus_id);
bool light_sensor_read(float *p_out);
