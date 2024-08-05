#include "leds.h"
#include "bsp.h"

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/gpio.h>

/* TODO: mutex/queue */
static volatile enum leds_status current_status;

static void led_ctrl(void *pvParameters)
{
    unsigned step_ms = 250;
    unsigned steps = 0;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(step_ms));

        enum leds_status st = current_status;

        switch (st) {
        case LEDS_STATUS_INITIAL: {
            /* f=1Hz, T=1000ms, 50% duty */

            step_ms = 1000;

            int state = steps & 0x1;
            gpio_set_level(STATUS_LED_PIN, state);
        };
        break;
        case LEDS_STATUS_STANDBY: {
            /* f=0.25Hz, T=4000ms, 50% duty */

            /* Do not keep step too small to react for status changes */
            step_ms = 500;

            int state = steps & 0x4;
            gpio_set_level(STATUS_LED_PIN, state);
        };
        break;
        case LEDS_STATUS_STREAMING: {
            /* f=2Hz, T=500ms, 50% duty */

            /* Do not keep step too small to react for status changes */
            step_ms = 250;

            int state = steps & 0x1;
            gpio_set_level(STATUS_LED_PIN, state);
        };
        break;
        case LEDS_FAULT: {
            /*
             * 3 short pulses - f=2Hz, T=500ms, 50% duty
             * 1 long pause - 1000ms
             */

            step_ms = 250;

            /*
             * Extract relevant portion of the counter
             * 3 pulses with f=2Hz means 6 steps of 250ms
             * 1 long pause of 1000ms means another 4 steps of 250ms
             * total of 10 steps
             */
            unsigned step_portion = steps % 10;

            if (step_portion < 6) {
                /* We're in the block of "3 pulses" - toggle LEDs */
                int state = steps & 0x1;
                gpio_set_level(STATUS_LED_PIN, state);
            } else {
                /* Stay silent for other 4 steps */
                gpio_set_level(STATUS_LED_PIN, 0);
            }
        }
        break;
        case LEDS_DEFAULT:
            /* Who knows why we need this state */
            gpio_set_level(STATUS_LED_PIN, 1);
            step_ms = 500;
        break;
        default:
            /* Programming error */
            assert(0);
            break;
        }

        steps++;
    }
}

void leds_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1LLU << STATUS_LED_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    gpio_config(&io_conf);
    xTaskCreate(&led_ctrl, "led_ctrl", 1024, NULL, 5, NULL);
    current_status = LEDS_DEFAULT;
}

void leds_status_set(enum leds_status s)
{
    current_status = s;
}
