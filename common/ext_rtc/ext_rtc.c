#include "ext_rtc.h"

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"

#define REG_CONTROL1    0x00
#define REG_CONTROL2    0x01
#define REG_RAM_BYTE    0x03
#define REG_SECONDS     0x04
#define REG_MINUTES     0x05
#define REG_HOURS       0x06
#define REG_ALM_SECONDS 0x0B
#define REG_ALM_MINUTES 0x0C
#define REG_ALM_HOURS   0x0D

#define CONTROL1_STOP   (1 << 5)
#define CONTROL2_AIE    (1 << 7)
#define CONTROL2_AF     (1 << 6)

#define PCF_ADDR         0x51

static int pcf_i2c_bus_id;

static bool write_reg(uint8_t dev_addr, uint8_t addr, uint8_t data);
static bool read_reg(uint8_t dev_addr, uint8_t addr, uint8_t *p_data);
static bool probe(uint8_t addr);

bool ext_rtc_init(int i2c_bus_id)
{
    pcf_i2c_bus_id = i2c_bus_id;
    /*
    for (int i = 0; i < 128; i++) {
        if (probe(i)) {
            printf("found @ %02x\r\n", i);
        }
    }
    */
    return ext_rtc_check();
}

bool ext_rtc_check(void)
{
    uint8_t control1;
    return read_reg(PCF_ADDR, REG_CONTROL1, &control1);
}

bool ext_rtc_set_alarm(int sec)
{
    const uint8_t control1 = CONTROL1_STOP;
    const uint8_t control2 = CONTROL2_AIE;
    const uint8_t seconds = 0;
    const uint8_t minutes = 0;
    const uint8_t hours   = 0;

    const uint8_t hours_duration   = sec / 3600;
    const uint8_t minutes_duration = (sec % 3600) / 60;
    const uint8_t seconds_duration = (sec % 3600) % 60;

    const uint8_t alarm_hours   = (hours_duration % 10)   | ((hours_duration / 10) << 4);
    const uint8_t alarm_minutes = (minutes_duration % 10) | ((minutes_duration / 10) << 4);
    const uint8_t alarm_seconds = (seconds_duration % 10) | ((seconds_duration / 10) << 4);

    if (!write_reg(PCF_ADDR, REG_CONTROL1, control1)) {
        return false;
    }

    if (!write_reg(PCF_ADDR, REG_SECONDS, seconds)) {
        return false;
    }

    if (!write_reg(PCF_ADDR, REG_MINUTES, minutes)) {
        return false;
    }

    if (!write_reg(PCF_ADDR, REG_HOURS, hours)) {
        return false;
    }

    if (!write_reg(PCF_ADDR, REG_ALM_SECONDS, alarm_seconds)) {
        return false;
    }

    if (!write_reg(PCF_ADDR, REG_ALM_MINUTES, alarm_minutes)) {
        return false;
    }

    if (!write_reg(PCF_ADDR, REG_ALM_HOURS, alarm_hours)) {
        return false;
    }

    if (!write_reg(PCF_ADDR, REG_CONTROL2, control2)) {
        return false;
    }

    // Enable RTC
    if (!write_reg(PCF_ADDR, REG_CONTROL1, 0)) {
        return false;
    }

    return true;
}

bool ext_rtc_alarm_active(bool *alarm_active)
{
    uint8_t control2 = 0;
    if (!read_reg(PCF_ADDR, REG_CONTROL2, &control2)) {
        return false;
    }

    *alarm_active = control2 & CONTROL2_AF;
    return true;
}

bool ext_rtc_clear_alarm_flag(void)
{
    uint8_t control2 = 0;
    if (!read_reg(PCF_ADDR, REG_CONTROL2, &control2)) {
        return false;
    }

    control2 &= ~CONTROL2_AF;

    return write_reg(PCF_ADDR, REG_CONTROL2, control2);
}

bool ext_rtc_set_ram_byte(uint8_t data)
{
    return write_reg(PCF_ADDR, REG_RAM_BYTE, data);
}

bool ext_rtc_get_ram_byte(uint8_t *data)
{
    return read_reg(PCF_ADDR, REG_RAM_BYTE, data);
}

static bool write_reg(uint8_t dev_addr, uint8_t addr, uint8_t val)
{
    uint8_t buf[3];
    buf[0] = (dev_addr << 1) | I2C_MASTER_WRITE;
    buf[1] = addr;
    buf[2] = val;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write(cmd, buf, 3, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(pcf_i2c_bus_id, cmd, pdMS_TO_TICKS(50));
    if (ESP_OK != ret) {
        return false;
    }
    i2c_cmd_link_delete(cmd);
    return true;
}

static bool read_reg(uint8_t dev_addr, uint8_t addr, uint8_t *data)
{
    uint8_t buf[2]={0};
    buf[0]=(dev_addr << 1) | I2C_MASTER_WRITE;
    buf[1]= addr;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write(cmd, buf, 2, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(pcf_i2c_bus_id, cmd, pdMS_TO_TICKS(50));
    if (ESP_OK != ret) {
        return false;
    }
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCF_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(pcf_i2c_bus_id, cmd, pdMS_TO_TICKS(50));
    if (ESP_OK != ret) {
        return false;
    }
    i2c_cmd_link_delete(cmd);
    return 1;
}

static bool probe(uint8_t addr)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(pcf_i2c_bus_id, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return ret == ESP_OK;
}
