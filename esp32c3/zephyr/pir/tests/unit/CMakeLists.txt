cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pir_unit_tests)

target_sources(app PRIVATE
    src/test_pir_main.c
    ../../src/trinity_canary_esp.c)

target_include_directories(app PRIVATE
    ../../src)
