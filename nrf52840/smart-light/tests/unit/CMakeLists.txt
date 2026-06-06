cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(smart_light_unit_tests)

target_sources(app PRIVATE
    src/test_smart_light_main.c
    stubs/light_hw_stub.c
    ../../src/relay.c
    ../../src/trinity_boot.c
    ../../src/trinity_canary.c
    ../../src/ble_gatt.c)

target_include_directories(app BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs/zephyr
    ../../src)
