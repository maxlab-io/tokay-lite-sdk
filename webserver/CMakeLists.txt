cmake_minimum_required(VERSION 3.5)
set(EXTRA_COMPONENT_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/../common/ext_rtc
  ${CMAKE_CURRENT_SOURCE_DIR}/../common/ltr303_als
  ${CMAKE_CURRENT_SOURCE_DIR}/../external/esp32-camera
  ${CMAKE_CURRENT_SOURCE_DIR}/../external/tflite-micro-esp-examples/components/esp-nn
  ${CMAKE_CURRENT_SOURCE_DIR}/../external/tflite-micro-esp-examples/components/tflite-lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../external/apds9306
  ${CMAKE_CURRENT_SOURCE_DIR}/../external/esp-protocols/components/mdns)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tokay-lite-webserver)
