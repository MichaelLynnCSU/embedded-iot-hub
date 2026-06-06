cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(smart_lock_unit_tests)

target_sources(app PRIVATE
    src/test_smart_lock_main.c
    stubs/stub_adc.c
    stubs/lock_hw_stub.c
    stubs/motor_stub.c
    ../../src/trinity_boot.c
    ../../src/trinity_canary.c
    ../../src/battery.c
    ../../src/ble_gatt.c)

target_include_directories(app BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs/zephyr
    ../../src)
