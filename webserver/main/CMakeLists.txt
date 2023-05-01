idf_component_register(
  SRCS
  main.c ai_camera.c light_sensor.c pir.c bsp.c network.c
  app_httpd.c ai_pipeline.cpp person_detect_model_data.cc
  json_settings_helpers.c auto_mode.c integrations.c

  PRIV_REQUIRES
  esp32-camera esp_http_server json nvs_flash esp_adc tflite-lib
  esp_event esp_wifi mdns esp_timer apds9306 ext_rtc esp_http_client
  ltr303_als

  EMBED_TXTFILES
  ${CMAKE_CURRENT_SOURCE_DIR}/static/index.html
  ${CMAKE_CURRENT_SOURCE_DIR}/static/index-themed.html
  ${CMAKE_CURRENT_SOURCE_DIR}/static/wifi_setup.html
)
