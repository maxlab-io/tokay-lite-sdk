cmake_minimum_required(VERSION 3.5)
set(EXTRA_COMPONENT_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/../external
  ${CMAKE_CURRENT_SOURCE_DIR}/../external/tflite-micro-esp-examples/components/esp-nn
  ${CMAKE_CURRENT_SOURCE_DIR}/../external/tflite-micro-esp-examples/components/tflite-lib
  ${CMAKE_CURRENT_SOURCE_DIR}/../external/esp-protocols/components/mdns)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ai-camera-demo)
