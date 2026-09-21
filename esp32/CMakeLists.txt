# ESP-IDF v5.x project for the stereo-link drive node.
#   idf.py set-target esp32
#   idf.py build flash monitor
# Jitter experiment build:
#   idf.py -DENABLE_LOAD_GENERATOR=1 build flash monitor
cmake_minimum_required(VERSION 3.16)

if(DEFINED ENABLE_LOAD_GENERATOR)
    add_compile_definitions(ENABLE_LOAD_GENERATOR=${ENABLE_LOAD_GENERATOR})
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(stereolink_drive)
