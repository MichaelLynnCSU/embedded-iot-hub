cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(temp_unit_tests)
target_sources(app PRIVATE
    src/test_temp_main.c
    stubs/stub_device.c
    stubs/temp_hw_stub.c
    ../../src/trinity_boot.c
    ../../src/trinity_canary.c
    ../../src/battery.c
    ../../src/temp.c
    ../../src/ble_adv.c)
target_include_directories(app BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs/zephyr
    ../../src)
