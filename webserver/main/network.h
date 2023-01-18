#pragma once

#define SSID_LEN     32
#define PASSWORD_LEN 64

typedef void (*network_sta_connected_cb_t)(void);

void network_init(const char *name, network_sta_connected_cb_t connected_cb);
void network_get_credentials(char *ssid, char *password);
void network_ap_mode(void);
void network_sta_mode(const char *ssid, const char *password);
