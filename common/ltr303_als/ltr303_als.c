#include "ltr303_als.h"

#include <stdint.h>

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LTR_303_ALS_ADDR 0x52

#define PART_ID_ADDR 0x86
#define LTR303_PART_ID 0xB1

#define ALS_MAIN_CTRL_ADDR 0x80
#define ALS_MAIN_CTRL_DATA 0x01
#define ALS_MAIN_CTRL_SW_RESET 0x02

#define ALS_MEAS_RATE_ADDR 0x85

#define ALS_STATUS_ADDR 0x8C
#define ALS_STATUS_NEW_DATA_BIT 0x4

#define ALS_DATA_ADDR 0x88

#define ALS_IT_TIME_100MS_REG_VAL 0
#define ALS_MEAS_RATE_100MS_REG_VAL 1
#define ALS_IT_TIME_100MS_CONV_VAL 1

#define ALS_MEASUREMENT_DURATION_MS 25
#define ALS_MAX_POLL_TIME_MS 100

#define ALS_RESET_DELAY_MS 20

static const uint8_t gain_reg_map[] = {
    [LTR303_GAIN_1X] = 0x00, // 000 Gain 1X 1 lux to 64k lux (default)
    [LTR303_GAIN_2X] = 0x01, // 001 Gain 2X 0.5 lux to 32k lux
    [LTR303_GAIN_4X] = 0x02, // 010 Gain 4X 0.25 lux to 16k lux
    [LTR303_GAIN_8X] = 0x03, // 011 Gain 8X 0.125 lux to 8k lux
    [LTR303_GAIN_48X] = 0x06, // 110 Gain 48X 0.02 lux to 1.3k lux
    [LTR303_GAIN_96X] = 0x07, // 111 Gain 96X 0.01 lux to 600 lux
};

static const uint8_t gain_value_map[] = {
    [LTR303_GAIN_1X] = 1,
    [LTR303_GAIN_2X] = 2,
    [LTR303_GAIN_4X] = 4,
    [LTR303_GAIN_8X] = 8,
    [LTR303_GAIN_48X] = 48,
    [LTR303_GAIN_96X] = 96,
};

static bool ltr_303_als_write(uint8_t addr, uint8_t val);
static bool ltr_303_als_read(uint8_t addr, uint8_t *data, uint8_t len);
static float get_visible_lux(uint16_t ch0, uint16_t ch1, int als_gain, int als_int);

static int ltr303_als_i2c_bus_id;
static ltr303_gain_t ltr303_gain;

bool ltr_303_als_init(int i2c_bus_id, ltr303_gain_t gain)
{
    ltr303_als_i2c_bus_id = i2c_bus_id;

    uint8_t part_id = 0;
    if (!ltr_303_als_read(PART_ID_ADDR, &part_id, 1)) {
        return false;
    }

    if (part_id != LTR303_PART_ID) {
        return false;
    }

    uint8_t main_ctrl = 0;
    if (!ltr_303_als_read(ALS_MAIN_CTRL_ADDR, &main_ctrl, 1)) {
        return false;
    }

    if (!ltr_303_als_write(ALS_MAIN_CTRL_ADDR, ALS_MAIN_CTRL_SW_RESET)) {
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(ALS_RESET_DELAY_MS));

    ltr303_gain = gain;

    const uint8_t meas_rate_value = (ALS_IT_TIME_100MS_REG_VAL << 3) | ALS_MEAS_RATE_100MS_REG_VAL;
    if (!ltr_303_als_write(ALS_MEAS_RATE_ADDR, meas_rate_value)) {
        return false;
    }
    return true;
}

bool ltr_303_als_start_measurement(int *p_measurement_time_ms)
{
    *p_measurement_time_ms = ALS_MEASUREMENT_DURATION_MS;
    return ltr_303_als_write(ALS_MAIN_CTRL_ADDR, ALS_MAIN_CTRL_DATA | gain_reg_map[ltr303_gain]);
}

bool ltr_303_als_read_measurement(float *p_out)
{
    uint8_t als_status = 0;
    TickType_t start = xTaskGetTickCount();
    do {
        if (xTaskGetTickCount() - start > pdMS_TO_TICKS(ALS_MAX_POLL_TIME_MS)) {
            return false;
        }

        if (ltr_303_als_read(ALS_STATUS_ADDR, &als_status, 1)) {
            return false;
        }
    } while ((als_status & ALS_STATUS_NEW_DATA_BIT) == 0);

    uint8_t als_data[4] = { 0 };
    if (!ltr_303_als_read(ALS_DATA_ADDR, als_data, sizeof(als_data))) {
        return false;
    }
    const uint8_t ch0 = als_data[1] | (als_data[2] << 8);
    const uint8_t ch1 = als_data[0] | (als_data[1] << 8);
    *p_out = get_visible_lux(ch0, ch1, gain_value_map[ltr303_gain], ALS_IT_TIME_100MS_CONV_VAL);
    return true;
}

static bool ltr_303_als_write(uint8_t addr, uint8_t val)
{
    uint8_t buf[3];
    buf[0] = (LTR_303_ALS_ADDR << 1) | I2C_MASTER_WRITE;
    buf[1] = addr;
    buf[2] = val;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write(cmd, buf, 3, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(ltr303_als_i2c_bus_id, cmd, pdMS_TO_TICKS(50));
    if (ESP_OK != ret) {
        return false;
    }
    i2c_cmd_link_delete(cmd);
    return true;
}

static bool ltr_303_als_read(uint8_t addr, uint8_t *data, uint8_t len)
{
    uint8_t buf[2]={0};
    buf[0]=(LTR_303_ALS_ADDR << 1) | I2C_MASTER_WRITE;
    buf[1]= addr;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write(cmd, buf, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ltr303_als_i2c_bus_id, cmd, pdMS_TO_TICKS(50));
    if (ESP_OK != ret) {
        return false;
    }
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LTR_303_ALS_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(ltr303_als_i2c_bus_id, cmd, pdMS_TO_TICKS(50));
    if (ESP_OK != ret) {
        return false;
    }
    i2c_cmd_link_delete(cmd);
    return 1;
}


static float get_visible_lux(uint16_t ch0, uint16_t ch1, int als_gain, int als_int)
{
    float ret; // variable to hold the computed ambient light value in Lux

    // Compute the ratio of ch1 to the sum of ch0 and ch1
    float ratio = (float)ch1 / (ch0 + ch1);

    // Compute the ret value based on the ratio and the LTR303-ALS01 sensor's gain and integration time values
    if (ratio < 0.45) {
        ret = (1.7743 * ch0 + 1.1059 * ch1) / (als_gain * als_int);
    }
    else if (ratio >= 0.45 && ratio < 0.64) {
        ret = (4.2785 * ch0 - 1.9548 * ch1) / (als_gain * als_int);
    }
    else if (ratio >= 0.64 && ratio < 0.85) {
        ret = (0.5926 * ch0 + 0.1185 * ch1) / (als_gain * als_int);
    }
    else {
        ret = 0;
    }

    return ret;
}
