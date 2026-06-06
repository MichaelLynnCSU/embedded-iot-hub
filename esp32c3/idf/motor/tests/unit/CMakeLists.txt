cmake_minimum_required(VERSION 3.16)
project(motor_unit_tests C)

set(CMAKE_C_STANDARD 11)

include(FetchContent)
FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.5.2)
FetchContent_MakeAvailable(unity)

# cJSON from managed_components
set(CJSON_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../managed_components/espressif__cjson/cJSON/cJSON.c)

# Stub dir shadows ESP-IDF and FreeRTOS headers
set(STUB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/stubs)

add_executable(test_motor
    src/test_motor_main.c
    ../../main/motor_control.c
    ${CJSON_SRC}
    ${unity_SOURCE_DIR}/src/unity.c)

target_compile_options(test_motor PRIVATE -iquote ${STUB_DIR})

target_include_directories(test_motor PRIVATE
    # Stubs first -- shadow real ESP-IDF / FreeRTOS headers
    ${STUB_DIR}
    ${STUB_DIR}
    ${STUB_DIR}/freertos
    # cJSON header
    ${CMAKE_CURRENT_SOURCE_DIR}/../../managed_components/espressif__cjson/cJSON
    # Production headers
    ../../main
    ${unity_SOURCE_DIR}/src)

target_sources(test_motor PRIVATE
    src/test_motor_main.c
    ../../main/motor_control.c
    stubs/stubs.c          # <-- add this line
    ${CJSON_SRC}
    ${unity_SOURCE_DIR}/src/unity.c)

# wifi.h is included by motor_control.c -- stub it via STUB_DIR/wifi.h
# tcp_client.h is included but tcp_client_exchange() is not under test --
# declare a stub inline via target_compile_definitions if needed.

enable_testing()
add_test(NAME motor_tests COMMAND test_motor)
