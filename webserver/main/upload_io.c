#include "upload_io.h"

#include "esp_mac.h"
#include "esp_log.h"
#include "esp_http_client.h"

#include "json_settings_helpers.h"

#define UPLOAD_IO_TIMEOUT_MS (20 * 1000)
#define TAG "upload_io"

static const char *upload_io_server_certificate =
"-----BEGIN CERTIFICATE-----\n"
"MIIEdTCCA12gAwIBAgIJAKcOSkw0grd/MA0GCSqGSIb3DQEBCwUAMGgxCzAJBgNV\n"
"BAYTAlVTMSUwIwYDVQQKExxTdGFyZmllbGQgVGVjaG5vbG9naWVzLCBJbmMuMTIw\n"
"MAYDVQQLEylTdGFyZmllbGQgQ2xhc3MgMiBDZXJ0aWZpY2F0aW9uIEF1dGhvcml0\n"
"eTAeFw0wOTA5MDIwMDAwMDBaFw0zNDA2MjgxNzM5MTZaMIGYMQswCQYDVQQGEwJV\n"
"UzEQMA4GA1UECBMHQXJpem9uYTETMBEGA1UEBxMKU2NvdHRzZGFsZTElMCMGA1UE\n"
"ChMcU3RhcmZpZWxkIFRlY2hub2xvZ2llcywgSW5jLjE7MDkGA1UEAxMyU3RhcmZp\n"
"ZWxkIFNlcnZpY2VzIFJvb3QgQ2VydGlmaWNhdGUgQXV0aG9yaXR5IC0gRzIwggEi\n"
"MA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDVDDrEKvlO4vW+GZdfjohTsR8/\n"
"y8+fIBNtKTrID30892t2OGPZNmCom15cAICyL1l/9of5JUOG52kbUpqQ4XHj2C0N\n"
"Tm/2yEnZtvMaVq4rtnQU68/7JuMauh2WLmo7WJSJR1b/JaCTcFOD2oR0FMNnngRo\n"
"Ot+OQFodSk7PQ5E751bWAHDLUu57fa4657wx+UX2wmDPE1kCK4DMNEffud6QZW0C\n"
"zyyRpqbn3oUYSXxmTqM6bam17jQuug0DuDPfR+uxa40l2ZvOgdFFRjKWcIfeAg5J\n"
"Q4W2bHO7ZOphQazJ1FTfhy/HIrImzJ9ZVGif/L4qL8RVHHVAYBeFAlU5i38FAgMB\n"
"AAGjgfAwge0wDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMCAYYwHQYDVR0O\n"
"BBYEFJxfAN+qAdcwKziIorhtSpzyEZGDMB8GA1UdIwQYMBaAFL9ft9HO3R+G9FtV\n"
"rNzXEMIOqYjnME8GCCsGAQUFBwEBBEMwQTAcBggrBgEFBQcwAYYQaHR0cDovL28u\n"
"c3MyLnVzLzAhBggrBgEFBQcwAoYVaHR0cDovL3guc3MyLnVzL3guY2VyMCYGA1Ud\n"
"HwQfMB0wG6AZoBeGFWh0dHA6Ly9zLnNzMi51cy9yLmNybDARBgNVHSAECjAIMAYG\n"
"BFUdIAAwDQYJKoZIhvcNAQELBQADggEBACMd44pXyn3pF3lM8R5V/cxTbj5HD9/G\n"
"VfKyBDbtgB9TxF00KGu+x1X8Z+rLP3+QsjPNG1gQggL4+C/1E2DUBc7xgQjB3ad1\n"
"l08YuW3e95ORCLp+QCztweq7dp4zBncdDQh/U90bZKuCJ/Fp1U1ervShw3WnWEQt\n"
"8jxwmKy6abaVd38PMV4s/KCHOkdp8Hlf9BRUpJVeEXgSYCfOn8J3/yNTd126/+pZ\n"
"59vPr5KW7ySaNRB6nJHGDn2Z9j8Z3/VyVOEVqQdZe4O/Ui5GjLIAZHYcSNPYeehu\n"
"VsyuLAOQ1xk4meTKCRlb/weWsKh/NEnfVqn3sF/tM+2MR7cwA130A4w=\n"
"-----END CERTIFICATE-----";

bool upload_io_upload_picture(const cJSON *p_cfg, const void *p_buf, size_t len)
{
    const char *p_api_key = json_settings_get_string_or(p_cfg, "api_key", NULL);
    if (NULL == p_api_key) {
        ESP_LOGE(TAG, "API key for upload.io required");
        return false;
    }
    esp_http_client_config_t config = {
        .url = "https://api.upload.io/v1/files/basic",
        .disable_auto_redirect = true,
        .cert_pem = upload_io_server_certificate,
        .timeout_ms = UPLOAD_IO_TIMEOUT_MS,
    };
    esp_http_client_handle_t http_client = esp_http_client_init(&config);
    if (NULL == http_client) {
        ESP_LOGE(TAG, "Failed to create HTTP client");
        return false;
    }
    uint8_t mac[6] = { 0 };
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char upload_tags[128];
    char date_time_str[64];
    time_t t = time(NULL);
    struct tm *tm = localtime(&t);
    strftime(date_time_str, sizeof(date_time_str), "%Y-%m-%d-%H-%M-%S", tm);
    const char *p_base_name = json_settings_get_string_or(p_cfg, "base_name", "");
    snprintf(upload_tags, sizeof(upload_tags), "[{\"name\":\"%s_%02x%02x%02x%02x%02x%02x_%s\",\"searchable\":true}]",
             p_base_name, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], date_time_str);
    char auth[128];
    snprintf(auth, sizeof(auth), "Bearer %s", p_api_key);
    esp_http_client_set_method(http_client, HTTP_METHOD_POST);
    esp_http_client_set_header(http_client, "Content-Type", "image/jpeg");
    esp_http_client_set_header(http_client, "X-Upload-Tags", upload_tags);
    esp_http_client_set_header(http_client, "Authorization", auth);
    int ret = esp_http_client_open(http_client, len);
    if (0 != ret) {
        ESP_LOGE(TAG, "Failed to connect to the server");
        esp_http_client_cleanup(http_client);
        return false;
    }

    ret = esp_http_client_write(http_client, p_buf, len);
    if (ret != len) {
        ESP_LOGE(TAG, "HTTP write failed %d", ret);
        esp_http_client_cleanup(http_client);
        return false;
    }
    const int content_len = esp_http_client_fetch_headers(http_client);
    const int status = esp_http_client_get_status_code(http_client);
    ESP_LOGI(TAG, "Got %d", status);
    if (-1 != content_len) {
        char *buf = malloc(content_len + 1);
        const int ret = esp_http_client_read_response(http_client, buf, content_len);
        if (ret != content_len) {
            ESP_LOGE(TAG, "Failed to read HTTP response %d", ret);
        } else {
            buf[content_len] = 0;
            ESP_LOGI(TAG, "%s", buf);
        }
        free(buf);
        esp_http_client_cleanup(http_client);
        return false;
    }
    esp_http_client_cleanup(http_client);
    return true;
}
