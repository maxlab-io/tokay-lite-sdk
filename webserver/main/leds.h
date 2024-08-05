#pragma once

#include "bsp.h"

enum leds_status
{
    LEDS_DEFAULT,
    /* Camera just started, waiting for WiFi credentials */
    LEDS_STATUS_INITIAL,
    /* Normal operation - connected to WiFi, camera silent */
    LEDS_STATUS_STANDBY,
    /* Normal operation - connected to WiFi and streaming */
    LEDS_STATUS_STREAMING,
    /* Something went wrong */
    LEDS_FAULT,
};

void leds_init(void);
void leds_status_set(enum leds_status s);
