idf_component_register(
    SRCS
        "main.c"
        "uart_manager.c"
        "wifi_manager.c"
        "ble_manager.c"
        "ble_scan.c"
        "ble_pir.c"
        "ble_reed.c"
        "ble_temp.c"
        "ble_gattc.c"
        "ble_light.c"
        "ble_lock.c"
        "tcp_manager.c"
        "motor_sm.c"
        "pir_window.c"
        "pi_controller.c"
        "device_gateway.c"
        "build_info.c"
        "wroom_bus.c"
        "trinity_wdt_idf.c"
        "trinity_canary_idf.c"
        "trinity_nvs_idf.c"
        "trinity_stats_idf.c"
        "trinity_panic_idf.c"
        "motor_server.c"
        "trinity_log_stack.c"
    INCLUDE_DIRS "." "include" "../../include"
    REQUIRES
        bt
        nvs_flash
        esp_driver_gpio
        esp_driver_uart
        esp_http_client
        esp_wifi
        esp_netif
        esp_event
        lwip
        mbedtls
        esp_timer
        esp_system
)

# Hub namespace -- different from motor
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    TRINITY_CHIP_ESP32_HUB_IDF=1
    "TRINITY_NVS_NAMESPACE=\"hub_log\""
)

# Note: No USB mode on ESP32 hub -- classic ESP32 has no native USB CDC.
if(DEFINED TRINITY_MODE AND "${TRINITY_MODE}" STREQUAL "BENCH")
    target_compile_definitions(${COMPONENT_LIB} PUBLIC
        CONFIG_TRINITY_MODE_BENCH=1
        CONFIG_TRINITY_MODE_FIELD=0
        CONFIG_TRINITY_MODE_USB=0)
    message(STATUS "Trinity mode: BENCH (JTAG halt)")
else()
    target_compile_definitions(${COMPONENT_LIB} PUBLIC
        CONFIG_TRINITY_MODE_BENCH=0
        CONFIG_TRINITY_MODE_FIELD=1
        CONFIG_TRINITY_MODE_USB=0)
    message(STATUS "Trinity mode: FIELD (log + restart)")
endif()
