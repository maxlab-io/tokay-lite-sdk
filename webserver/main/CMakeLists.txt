idf_component_register(SRCS main.c ai_camera.c config.c light_sensor.c pir.c bsp.c network.c app_httpd.c
  PRIV_REQUIRES esp32-camera esp_http_server json nvs_flash esp_adc tflite-lib
  esp_event esp_wifi mdns esp_timer
  EMBED_TXTFILES
  ${CMAKE_CURRENT_SOURCE_DIR}/static/index.html
  ${CMAKE_CURRENT_SOURCE_DIR}/static/wifi_setup.html)
