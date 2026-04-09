idf_component_register(
    SRCS
        "motor_control.c"
        "battery.c"
        "wifi_tx_sweep.c"
        "build_info.c"
        "trinity_wdt_idf.c"
        "trinity_canary_idf.c"
        "trinity_nvs_idf.c"
        "trinity_stats_idf.c"
        "trinity_panic_idf.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        esp_driver_gpio
        driver
        esp_adc
        lwip
        cjson
        esp_timer
        esp_system
        esp_driver_ledc
)

# Trinity split -- each file can be swapped for a stub independently.
#
# Dependency order (strict -- never invert):
#   wdt_idf    -> standalone
#   canary_idf -> standalone
#   nvs_idf    -> canary (RTC vars)
#   stats_idf  -> nvs (via trinity_log_event)
#   panic_idf  -> nvs (via trinity_log_event)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    TRINITY_CHIP_ESP32C3_IDF=1
    "TRINITY_NVS_NAMESPACE=\"motor_log\""
)

if(DEFINED TRINITY_MODE AND "${TRINITY_MODE}" STREQUAL "BENCH")
    target_compile_definitions(${COMPONENT_LIB} PUBLIC
        CONFIG_TRINITY_MODE_BENCH=1
        CONFIG_TRINITY_MODE_FIELD=0
        CONFIG_TRINITY_MODE_USB=0)
    message(STATUS "Trinity mode: BENCH")
elseif(DEFINED TRINITY_MODE AND "${TRINITY_MODE}" STREQUAL "USB")
    target_compile_definitions(${COMPONENT_LIB} PUBLIC
        CONFIG_TRINITY_MODE_BENCH=0
        CONFIG_TRINITY_MODE_FIELD=0
        CONFIG_TRINITY_MODE_USB=1)
    message(STATUS "Trinity mode: USB")
else()
    target_compile_definitions(${COMPONENT_LIB} PUBLIC
        CONFIG_TRINITY_MODE_BENCH=0
        CONFIG_TRINITY_MODE_FIELD=1
        CONFIG_TRINITY_MODE_USB=0)
    message(STATUS "Trinity mode: FIELD")
endif()
