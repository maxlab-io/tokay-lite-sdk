#pragma once

#include <stdbool.h>
#include <stdint.h>

bool ext_rtc_init(int i2c_bus_id);
bool ext_rtc_check(void);
bool ext_rtc_set_alarm(int alarm_seconds);
bool ext_rtc_alarm_active(bool *alarm_active);
bool ext_rtc_clear_alarm_flag(void);
bool ext_rtc_set_ram_byte(uint8_t data);
bool ext_rtc_get_ram_byte(uint8_t *data);
