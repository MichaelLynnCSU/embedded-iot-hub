cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(smartlight_service)
target_sources(app PRIVATE
    src/main.c
    src/build_info.c
    src/trinity_wdt.c
    src/trinity_canary.c
    src/trinity_flash.c
    src/trinity_stats.c
    src/trinity_fault.c
    src/trinity_boot.c
)
