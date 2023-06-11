#pragma once

#include <stddef.h>
#include <stdbool.h>
#include "cJSON.h"

bool thingsboard_init(const cJSON *p_cfg);
bool thingsboard_upload_picture(const cJSON *p_cfg, const void *p_buf, size_t len);
