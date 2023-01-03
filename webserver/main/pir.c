#include "pir.h"

#include "driver/gpio.h"

#define PIR_OUT_PIN     47
#define PIR_CTRL_D_PIN  21
#define PIR_CTRL_LE_PIN 45

void pir_init(pir_callback_t pir_cb, void *p_ctx)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1LLU << PIR_OUT_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_isr_handler_add(PIR_OUT_PIN, pir_cb, p_ctx);

    gpio_set_level(PIR_CTRL_LE_PIN, 0);
    gpio_set_level(PIR_CTRL_D_PIN, 0);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1LLU << PIR_CTRL_D_PIN) | (1LLU << PIR_CTRL_LE_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    pir_enable();
}

bool pir_is_motion_detected(void)
{
    return gpio_get_level(PIR_OUT_PIN);
}

void pir_enable(void)
{
    gpio_set_level(PIR_CTRL_D_PIN, 1);
    gpio_set_level(PIR_CTRL_LE_PIN, 1);

    gpio_set_level(PIR_CTRL_LE_PIN, 0);
    gpio_set_level(PIR_CTRL_D_PIN, 0);
}

void pir_disable(void)
{
    gpio_set_level(PIR_CTRL_D_PIN, 0);
    gpio_set_level(PIR_CTRL_LE_PIN, 1);

    gpio_set_level(PIR_CTRL_LE_PIN, 0);
    gpio_set_level(PIR_CTRL_D_PIN, 0);
}
