#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "cJSON.h"

bool upload_io_upload_picture(const cJSON *p_cfg, const void *p_buf, size_t len);
