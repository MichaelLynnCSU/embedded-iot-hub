cmake_minimum_required(VERSION 3.16)
project(hub_unit_tests C)
set(CMAKE_C_STANDARD 11)

include(FetchContent)
FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.5.2)
FetchContent_MakeAvailable(unity)

set(STUB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/stubs)

# ── test_hub ─────────────────────────────────────────────────────────────────
# Tests: config constants, motor_sm, pi_controller, BLE modules, pir_window
# Uses stub getters for aws/uart so pi_controller tests remain deterministic
add_executable(test_hub
    src/test_hub_main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../main/pir_window.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../main/motor_sm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../main/pi_controller.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../main/ble_pir.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../main/ble_reed.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../main/ble_temp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../main/ble_light.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../main/ble_lock.c
    stubs/stubs.c
    ${unity_SOURCE_DIR}/src/unity.c)

target_include_directories(test_hub PRIVATE
    ${STUB_DIR}
    ${STUB_DIR}/lwip
    ${STUB_DIR}/freertos
    ${STUB_DIR}/esp_ble
    ${STUB_DIR}/driver
    ${CMAKE_CURRENT_SOURCE_DIR}/../../main
    ${unity_SOURCE_DIR}/src)

# ── test_aws ─────────────────────────────────────────────────────────────────
# Tests: aws_manager parse behavior, uart_manager parse behavior
# Compiles real aws_manager.c and uart_manager.c; no pi_controller conflict
add_executable(test_aws
    src/test_aws_main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../main/aws_manager.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../main/uart_manager.c
    stubs/stubs_aws.c
    ${unity_SOURCE_DIR}/src/unity.c)

target_include_directories(test_aws PRIVATE
    ${STUB_DIR}
    ${STUB_DIR}/lwip
    ${STUB_DIR}/freertos
    ${STUB_DIR}/esp_ble
    ${STUB_DIR}/driver
    ${CMAKE_CURRENT_SOURCE_DIR}/../../main
    ${unity_SOURCE_DIR}/src)

enable_testing()
add_test(NAME hub_tests COMMAND test_hub)
add_test(NAME aws_tests COMMAND test_aws)
