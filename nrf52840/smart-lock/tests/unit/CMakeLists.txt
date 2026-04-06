cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(smart_lock_unit_tests)

target_sources(app PRIVATE
    src/test_smart_lock_main.c
    ../../src/trinity_boot.c
    ../../src/trinity_canary.c)

target_include_directories(app PRIVATE
    ../../src)
