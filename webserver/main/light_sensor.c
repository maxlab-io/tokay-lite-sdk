#include "light_sensor.h"

#include "esp_log.h"
#include "ltr303_als.h"

#define TAG "LIGHT_SENS"

#define ALS_GAIN LTR303_GAIN_4X

bool light_sensor_init(int i2c_bus_id)
{
    return ltr_303_als_init(i2c_bus_id, ALS_GAIN);
}

bool light_sensor_start_measurement(int *p_measurement_time_ms)
{
    return ltr_303_als_start_measurement(p_measurement_time_ms);
}

bool light_sensor_read_measurement(float *p_out)
{
    return ltr_303_als_read_measurement(p_out);
}
