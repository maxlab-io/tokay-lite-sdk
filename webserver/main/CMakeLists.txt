idf_component_register(SRCS main.c ai_camera.c
                       PRIV_REQUIRES esp32-camera esp_http_server json
                       EMBED_TXTFILES ${CMAKE_CURRENT_SOURCE_DIR}/index.html)
