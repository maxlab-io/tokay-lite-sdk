idf_component_register(SRCS main.c ai_camera.c config.c light_sensor.c pir.c
                       PRIV_REQUIRES esp32-camera esp_http_server json nvs_flash esp_adc tflite-lib
                       EMBED_TXTFILES ${CMAKE_CURRENT_SOURCE_DIR}/static/index.html)
