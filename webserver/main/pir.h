#pragma once

#include <stdbool.h>

typedef void (*pir_callback_t)(void *ctx);

void pir_init(pir_callback_t pir_cb, void *p_ctx);
bool pir_is_motion_detected(void);
void pir_enable(void);
void pir_disable(void);
