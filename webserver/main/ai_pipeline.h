#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "cJSON.h"

typedef struct TfLiteTensor TfLiteTensor;

#ifdef __cplusplus
extern "C" {
#endif

void ai_pipeline_init(void);
void ai_pipeline_submit(const uint8_t *p_frame_data, uint32_t size);
cJSON *ai_pipeline_create_results_object(void);
void ai_pipeline_fill_results_object(cJSON *p_obj);
void ai_pipeline_finish(void);

#ifdef __cplusplus
}
#endif
