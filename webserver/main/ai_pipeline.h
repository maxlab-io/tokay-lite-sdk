#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"

typedef struct TfLiteTensor TfLiteTensor;

#ifdef __cplusplus
extern "C" {
#endif

void ai_pipeline_init(void);
void ai_pipeline_start(const uint8_t *p_frame_data, uint32_t size);
void ai_pipeline_wait(TickType_t timeous_ticks);
void ai_pipeline_wait_decode_finish(TickType_t timeous_ticks);
TfLiteTensor *ai_pipeline_get_results(void);
void ai_pipeline_finish(void);

#ifdef __cplusplus
}
#endif
