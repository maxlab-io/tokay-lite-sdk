#pragma once

#include <stdbool.h>

void app_httpd_start(bool wifi_setup);
void app_httpd_stop(void);
bool app_httpd_is_running(void);