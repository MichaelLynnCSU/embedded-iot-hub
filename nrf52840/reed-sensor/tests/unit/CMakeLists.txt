cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(reed_unit_tests)

target_sources(app PRIVATE
    src/test_reed_main.c
    ../../src/trinity_boot.c
    ../../src/trinity_canary.c
    ../../src/battery.c
    ../../src/ble_adv.c
    stubs/stub_device.c
)

target_include_directories(app BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs/zephyr
    ../../src
)

target_compile_options(app PRIVATE
    -include ${CMAKE_CURRENT_SOURCE_DIR}/stubs/stub_overrides.h
)
