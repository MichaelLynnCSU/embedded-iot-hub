cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pir_ble)
# Trinity split -- each file can be swapped for a stub independently.
#
# Dependency order (strict -- never invert):
#   wdt_esp    -> standalone
#   canary_esp -> standalone
#   flash_esp  -> canary (noinit vars)
#   stats_esp  -> flash (via trinity_log_event), wdt
#   fault_esp  -> flash (via write_panic), canary (via g_init_stage)
target_sources(app PRIVATE
    src/main.c
    src/ble_adv.c
    src/max17048.c
    src/build_info.c
    src/trinity_wdt_esp.c
    src/trinity_canary_esp.c
    src/trinity_flash_esp.c
    src/trinity_stats_esp.c
    src/trinity_fault_esp.c
)
