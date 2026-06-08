cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32-doorbell)

if(DEFINED DOORBELL_ID)
    add_compile_definitions(DOORBELL_ID=${DOORBELL_ID})
endif()
