#include "main.h"

#include <string.h>

#include "esp_camera.h"
#include "sensor.h"

#include "cJSON.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include "nvs_flash.h"
#include "esp_log.h"

#include "bsp.h"
#include "pir.h"
#include "ai_camera.h"
#include "app_httpd.h"
#include "network.h"
#include "json_settings_helpers.h"
#include "auto_mode.h"
#include "integrations.h"

#define TAG "main"

#define EVENT_QUEUE_SIZE 5
#define APP_TASK_STACK_SIZE 4096
#define APP_TASK_PRIORITY   5

#define MOTION_COOLDOWN_TIME_MS 5000

static QueueHandle_t event_queue;
static cJSON *p_system_settings;
static TimerHandle_t motion_cooldown_timer;
static bool motion_active;

static void pir_motion_callback(void *ctx);
static void wifi_connection_callback(void);
static void button_callback(void);
static void app_task(void *pvArg);
static cJSON *system_settings_make_default(void);
static void motion_cooldown_timer_cb(TimerHandle_t handle);

void app_main(void)
{
    gpio_install_isr_service(0);
    bsp_init(button_callback);
    pir_init(pir_motion_callback, NULL);
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    json_settings_helpers_init();
    p_system_settings = json_settings_load_from_nvs("system");
    if (NULL == p_system_settings) {
        ESP_LOGE(TAG, "Failed to load system settings from NVS");
        p_system_settings = system_settings_make_default();
        json_settings_save_to_nvs("system", p_system_settings);
    }
    BaseType_t xRet = xTaskCreatePinnedToCore(app_task, "APP_TSK", APP_TASK_STACK_SIZE,
            NULL, APP_TASK_PRIORITY, NULL, 0);
    configASSERT(pdTRUE == xRet);
}

void system_settings_set_json(const cJSON *p_settings)
{
    if (!cJSON_Compare(p_system_settings, p_settings, false)) {
        cJSON_Delete(p_system_settings);
        p_system_settings = cJSON_Duplicate(p_settings, true);
    }
}

const cJSON *system_settings_get_json(void)
{
    return p_system_settings;
}

void system_settings_apply(void)
{
    char curr_ssid[33] = { 0 };
    char curr_password[64] = { 0 };
    const char *new_ssid = json_settings_get_string_or(p_system_settings, "wifi_ssid", "");
    const char *new_password = json_settings_get_string_or(p_system_settings, "wifi_password", "");
    network_get_credentials(curr_ssid, curr_password);
    if (0 != strcmp(curr_ssid, new_ssid) || 0 != strcmp(curr_password, new_password)) {
        ESP_LOGI(TAG, "Updating WiFi settings from %s to %s", curr_ssid, new_ssid);
        app_send_event(APP_EVENT_NEW_WIFI_SETTINGS);
    }
    json_settings_save_to_nvs("system", p_system_settings);
}

void app_send_event(app_event_t ev)
{
    BaseType_t xRet = xQueueSend(event_queue, &ev, 0);
    configASSERT(pdTRUE == xRet);
}

void app_send_event_from_isr(app_event_t ev)
{
    BaseType_t wakeup_needed = pdFALSE;
    BaseType_t xRet = xQueueSendFromISR(event_queue, &ev, &wakeup_needed);
    configASSERT(pdTRUE == xRet);
    portYIELD_FROM_ISR(wakeup_needed);
}

bool app_is_motion_active(void)
{
    return motion_active;
}

static void pir_motion_callback(void *ctx)
{
    app_send_event(APP_EVENT_PIR_MOTION_DETECTED);
}

static void button_callback(void)
{
    app_send_event(APP_EVENT_BUTTON_PRESSED);
}

static void wifi_connection_callback(void)
{
    app_send_event(APP_EVENT_WIFI_CONNECTED);
}

static void app_task(void *pvArg)
{
    bool ap_mode = false;

    event_queue = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(app_event_t));

    network_init("tokay-lite", wifi_connection_callback);
    auto_mode_init();
    integrations_init();

    {
        const char *p_ssid = json_settings_get_string_or(p_system_settings, "wifi_ssid", "");
        const char *p_password = json_settings_get_string_or(p_system_settings, "wifi_password", "");

        if (p_ssid[0] != 0) {
            ESP_LOGI(TAG, "Starting in STA mode, connecting to %s", p_ssid);
            network_sta_mode(p_ssid, p_password);
            ap_mode = false;
        } else {
            ESP_LOGI(TAG, "No STA config saved, starting in AP mode");
            network_ap_mode();
            app_httpd_start(true);
            ap_mode = true;
        }
    }

    const bool is_pir_triggered = pir_is_motion_detected();
    const bool is_timer_triggered = bsp_get_timer_alarm();

    const bool enter_auto_mode = (is_pir_triggered || is_timer_triggered) && auto_mode_enabled();
    const bool low_power_mode = auto_mode_low_power_enabled();
    const bool pir_wakeup_enabled = auto_mode_pir_wakeup_enabled();
    const int wakeup_period_seconds = auto_mode_get_wakeup_period_seconds();

    ai_camera_init(BSP_I2C_BUS_ID);

    if (!ap_mode && enter_auto_mode) {
        ESP_LOGI(TAG, "Running in auto mode");
        app_event_t event = APP_EVENT_MAX;
        TickType_t start = xTaskGetTickCount();
        ESP_LOGI(TAG, "Waiting for WiFi connection...");
        do {
            xQueueReceive(event_queue, &event, portMAX_DELAY);
        } while (APP_EVENT_WIFI_CONNECTED != event && xTaskGetTickCount() - start < pdMS_TO_TICKS(10000));
        bool enable_pir_wakeup = true;
        const int sleep_duration_seconds = auto_mode_run(&enable_pir_wakeup);
        if (enable_pir_wakeup) {
            pir_enable();
        } else {
            pir_disable();
        }
        if (low_power_mode) {
            if (0 == sleep_duration_seconds) {
                bsp_deep_sleep(UINT32_MAX);
            } else {
                bsp_deep_sleep(sleep_duration_seconds);
            }

            while (1) {
                vTaskDelay(100);
            }
        }
    }

    motion_cooldown_timer = xTimerCreate("MOTION_CD", pdMS_TO_TICKS(MOTION_COOLDOWN_TIME_MS),
                             pdFALSE, NULL, motion_cooldown_timer_cb);

    while (1) {
        app_event_t event = APP_EVENT_MAX;
        xQueueReceive(event_queue, &event, portMAX_DELAY);
        switch (event) {
        case APP_EVENT_WIFI_CONNECTED:
            if (!app_httpd_is_running()) {
                app_httpd_start(false);
            }
            break;
        case APP_EVENT_PIR_MOTION_DETECTED:
            xTimerReset(motion_cooldown_timer, portMAX_DELAY);
            if (!motion_active) {
                motion_active = true;
                if (app_httpd_is_running()) {
                    app_httpd_trigger_telemetry_update();
                }
            }
            ESP_LOGI(TAG, "Motion detected");
            break;
        case APP_EVENT_BUTTON_PRESSED:
            if (ap_mode) {
                const char *p_ssid = json_settings_get_string_or(p_system_settings, "wifi_ssid", "");
                const char *p_password = json_settings_get_string_or(p_system_settings, "wifi_password", "");
                if (p_ssid != NULL && p_ssid[0] != 0) {
                    ESP_LOGI(TAG, "Switching to STA mode");
                    app_httpd_stop();
                    network_sta_mode(p_ssid, p_password);
                    ap_mode = false;
                } else {
                    ESP_LOGW(TAG, "Wifi creds unavailable, ignoring button press");
                }
            } else {
                ESP_LOGI(TAG, "Switching to AP mode");
                app_httpd_stop();
                network_ap_mode();
                app_httpd_start(true);
                ap_mode = true;
            }
            break;
        case APP_EVENT_NEW_WIFI_SETTINGS:
            ESP_LOGI(TAG, "Reconnecting to the new AP");
            app_httpd_stop();
            network_sta_mode(json_settings_get_string_or(p_system_settings, "wifi_ssid", ""),
                    json_settings_get_string_or(p_system_settings, "wifi_password", ""));
            ap_mode = false;
        default:
            break;
        }
    }
}

static cJSON *system_settings_make_default(void)
{
    const char *p_default_json = "{\"wifi_ssid\":\"\",\"wifi_password\":\"\"}";
    return cJSON_ParseWithLength(p_default_json, strlen(p_default_json));
}

static void motion_cooldown_timer_cb(TimerHandle_t handle)
{
    motion_active = false;
}
