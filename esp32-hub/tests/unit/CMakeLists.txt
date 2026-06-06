cmake_minimum_required(VERSION 3.16)
project(hub_unit_tests C)
set(CMAKE_C_STANDARD 11)

include(FetchContent)
FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.5.2)
FetchContent_MakeAvailable(unity)

set(STUB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/stubs)

add_executable(test_hub
    src/test_hub_main.c
    ../../main/pir_window.c
    ../../main/motor_sm.c
    ../../main/pi_controller.c
    stubs/stubs.c
    ${unity_SOURCE_DIR}/src/unity.c)

target_include_directories(test_hub PRIVATE
    ${STUB_DIR}
    ${STUB_DIR}/lwip
    ${STUB_DIR}/freertos
    ../../main
    ${unity_SOURCE_DIR}/src)

enable_testing()
add_test(NAME hub_tests COMMAND test_hub)
