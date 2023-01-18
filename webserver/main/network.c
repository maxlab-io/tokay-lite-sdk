#include "network.h"

#include <string.h>

#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"

#include "mdns.h"

#define TAG "network"

static SemaphoreHandle_t wifi_start_semaphore;
static esp_netif_t *sta_netif;
static esp_netif_t *ap_netif;
static network_sta_connected_cb_t network_connected_cb;

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void start_mdns(const char *name);

void network_init(const char *name, network_sta_connected_cb_t connected_cb)
{
    network_connected_cb = connected_cb;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    sta_netif = esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_netif_set_hostname(sta_netif, name));
    ap_netif = esp_netif_create_default_wifi_ap();
    ESP_ERROR_CHECK(esp_netif_set_hostname(sta_netif, name));
    esp_netif_ip_info_t info_t;
    start_mdns(name);
    memset(&info_t, 0, sizeof(esp_netif_ip_info_t));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = 1;
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    wifi_config_t ap_config = { 0 };
    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_AP, &ap_config));
    strncpy((char *)ap_config.ap.ssid, name, SSID_LEN);
    ap_config.ap.ssid_len = strlen(name);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_handler, NULL));

    wifi_start_semaphore = xSemaphoreCreateBinary();

    ESP_ERROR_CHECK(esp_wifi_start());
    xSemaphoreTake(wifi_start_semaphore, portMAX_DELAY);
}

void network_get_credentials(char *ssid, char *password)
{
    wifi_config_t config = { 0 };
    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &config));
    strncpy(ssid, (const char *)config.sta.ssid, SSID_LEN);
    strncpy(password, (const char *)config.sta.password, PASSWORD_LEN);
}

void network_ap_mode(void)
{
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_start());
    xSemaphoreTake(wifi_start_semaphore, portMAX_DELAY);
}

void network_sta_mode(const char *ssid, const char *password)
{
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t config = { 0 };
    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &config));
    strncpy((char *)config.sta.ssid, ssid, sizeof(config.sta.ssid));
    strncpy((char *)config.sta.password, password, sizeof(config.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config));

    ESP_ERROR_CHECK(esp_wifi_start());
    xSemaphoreTake(wifi_start_semaphore, portMAX_DELAY);
    ESP_ERROR_CHECK(esp_wifi_connect());
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    switch (event_id) {
    case WIFI_EVENT_STA_START:
    case WIFI_EVENT_AP_START:
        ESP_LOGI(TAG, "Start");
        xSemaphoreGive(wifi_start_semaphore);
        break;
    case WIFI_EVENT_STA_STOP:
    case WIFI_EVENT_AP_STOP:
        ESP_LOGI(TAG, "Stop");
        break;
    case WIFI_EVENT_AP_STACONNECTED:
    {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);
        break;
    }
    case WIFI_EVENT_AP_STADISCONNECTED:
    {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d", MAC2STR(event->mac), event->aid);
        break;
    }
    case WIFI_EVENT_STA_CONNECTED:
    {
        wifi_event_sta_connected_t *connected = (wifi_event_sta_connected_t *)event_data;
        ESP_LOGI(TAG, "Connected to %.*s, ("MACSTR")", connected->ssid_len,
                connected->ssid, MAC2STR(connected->bssid));
        break;
    }
    case WIFI_EVENT_STA_DISCONNECTED:
    {
        wifi_event_sta_disconnected_t *disconnected = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGI(TAG, "Disconnected from %.*s, BSSID "MACSTR", reason %d",
                disconnected->ssid_len, disconnected->ssid,
                MAC2STR(disconnected->bssid), disconnected->reason);
        esp_wifi_connect(); // Initiate re-connect
        break;
    }
    default:
        ESP_LOGW(TAG, "Unhandled event %ld", event_id);
        break;
    }
}

static void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    switch (event_id) {
    case IP_EVENT_STA_GOT_IP:
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "Got IP " IPSTR ", gateway " IPSTR, IP2STR(&event->ip_info.ip),
                IP2STR(&event->ip_info.gw));
        if (NULL != network_connected_cb) {
            network_connected_cb();
        }
        break;
    }
    case IP_EVENT_STA_LOST_IP:
        ESP_LOGI(TAG, "IP lost");
        break;
    default:
        ESP_LOGW(TAG, "Unhandled event %s %ld", event_base, event_id);
        break;
    }
}

static void start_mdns(const char *name)
{
    esp_err_t err = mdns_init();
    if (err) {
        ESP_LOGE(TAG, "MDNS Init failed: %d\n", err);
        return;
    }

    mdns_hostname_set(name);
}
