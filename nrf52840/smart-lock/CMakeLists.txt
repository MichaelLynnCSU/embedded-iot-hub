cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(smartlock_service)

# Trinity split -- each file can be swapped for a stub independently
# to isolate which subsystem is broken during debugging.
#
# Dependency order (strict -- never invert):
#   wdt    -> standalone
#   canary -> standalone
#   flash  -> wdt, canary
#   stats  -> flash (via trinity_log_event), wdt
#   fault  -> flash (via write_panic), canary (via g_init_stage)
#   boot   -> flash (via trinity_log_event)
#
# To stub out a subsystem, replace e.g.:
#   src/trinity_flash.c  ->  src/stubs/trinity_flash_stub.c

target_sources(app PRIVATE
    src/main.c
    src/battery.c
    src/build_info.c
    src/trinity_wdt.c
    src/trinity_canary.c
    src/trinity_flash.c
    src/trinity_stats.c
    src/trinity_fault.c
    src/trinity_boot.c
)
