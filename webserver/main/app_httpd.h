#pragma once

#include <stdbool.h>

void app_httpd_start(bool wifi_setup);
void app_httpd_stop(void);
bool app_httpd_is_running(void);
void app_httpd_trigger_telemetry_update(void);
void app_httpd_post_task(void (*p_task)(void *p_ctx), void *p_ctx);
