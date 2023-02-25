#include "json_settings_helpers.h"

#include <string.h>

#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#define TAG "json_settings"

static SemaphoreHandle_t nvs_mutex;

void json_settings_helpers_init(void)
{
    nvs_mutex = xSemaphoreCreateMutex();
}

cJSON *json_settings_load_from_nvs(const char *p_category)
{
    xSemaphoreTake(nvs_mutex, portMAX_DELAY);
    cJSON *p_ret = NULL;
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("config", NVS_READWRITE, &nvs_handle);
    if (ESP_OK != err) {
        ESP_LOGE(TAG, "Failed to open settings NVS namespace: %s", esp_err_to_name(err));
        xSemaphoreGive(nvs_mutex);
        return NULL;
    }
    size_t settings_size;
    err = nvs_get_blob(nvs_handle, p_category, NULL, &settings_size);
    char* settings_buf = NULL;
    if (ESP_ERR_NVS_INVALID_LENGTH == err) {
        settings_buf = malloc(settings_size);
        err = nvs_get_blob(nvs_handle, p_category, settings_buf, &settings_size);
        if (ESP_OK != err) {
            ESP_LOGE(TAG, "Failed to load settings JSON for \"%s\": %s", p_category, esp_err_to_name(err));
        } else {
            p_ret = cJSON_ParseWithLength(settings_buf, settings_size);
            if (p_ret == NULL) {
                ESP_LOGE(TAG, "Failed to parse settings JSON for \"%s\"", p_category);
            }
        }
        free(settings_buf);
    }

    nvs_close(nvs_handle);
    xSemaphoreGive(nvs_mutex);
    return p_ret;
}

bool json_settings_save_to_nvs(const char *p_category, const cJSON *p_settings)
{
    xSemaphoreTake(nvs_mutex, portMAX_DELAY);
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("config", NVS_READWRITE, &nvs_handle);
    if (ESP_OK != err) {
        ESP_LOGE(TAG, "Failed to open \"%s\" settings NVS namespace: %s", p_category, esp_err_to_name(err));
        xSemaphoreGive(nvs_mutex);
        return false;
    }
    char* settings_buf = cJSON_PrintUnformatted(p_settings);
    err = nvs_set_blob(nvs_handle, p_category, settings_buf, strlen(settings_buf));
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    free(settings_buf);
    xSemaphoreGive(nvs_mutex);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write \"%s\" settings to NVS: %s", p_category, esp_err_to_name(err));
        return false;
    }
    return true;
}
