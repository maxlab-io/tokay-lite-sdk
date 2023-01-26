#include "light_sensor.h"

#include "esp_log.h"
#include "apds9306.h"
#include "apds9306_errors.h"

#define TAG "LIGHT_SENS"

#define ALS_GAIN APDS9306_ALS_GAIN_3

void light_sensor_init(int i2c_bus_id)
{
    (void)i2c_bus_id;
    if (APDS9306_OK != apds9306_reset()) {
        ESP_LOGE(TAG, "Failed to reset APDS9306");
        return;
    }

    if (APDS9306_OK != apds9306_probe()) {
        ESP_LOGE(TAG, "Failed to probe APDS9306");
        return;
    }

    if (APDS9306_OK != apds9306_set_als_gain(ALS_GAIN)) {
        ESP_LOGE(TAG, "Failed to set APDS9306 gain");
        return;
    }

    if (APDS9306_OK != apds9306_als_enable()) {
        ESP_LOGE(TAG, "Failed to set APDS9306 gain");
        return;
    }
}

bool light_sensor_read(float *p_out)
{
    return APDS9306_OK == apds9306_read_wait(p_out);
}
