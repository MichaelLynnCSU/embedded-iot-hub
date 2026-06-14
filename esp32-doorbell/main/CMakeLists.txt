idf_component_register(SRCS "main.c"
        "trinity_wdt_idf.c"
        "trinity_canary_idf.c"
        "trinity_nvs_idf.c"
        "trinity_stats_idf.c"
        "trinity_panic_idf.c"
        "build_info.c"
        "trinity_log_stack.c"
        "buzzer.c"
    INCLUDE_DIRS "." "include")

if(DEFINED DOORBELL_ID)
    target_compile_definitions(__idf_main PUBLIC DOORBELL_ID=${DOORBELL_ID})
endif()

target_compile_definitions(__idf_main PUBLIC
    TRINITY_CHIP_ESP32_DOORBELL_IDF=1
    "TRINITY_NVS_NAMESPACE=\"doorbell_log\""
)

if(DEFINED TRINITY_MODE AND "${TRINITY_MODE}" STREQUAL "BENCH")
    target_compile_definitions(__idf_main PUBLIC
        CONFIG_TRINITY_MODE_BENCH=1
        CONFIG_TRINITY_MODE_FIELD=0
        CONFIG_TRINITY_MODE_USB=0)
    message(STATUS "Trinity mode: BENCH (JTAG halt)")
else()
    target_compile_definitions(__idf_main PUBLIC
        CONFIG_TRINITY_MODE_BENCH=0
        CONFIG_TRINITY_MODE_FIELD=1
        CONFIG_TRINITY_MODE_USB=0)
    message(STATUS "Trinity mode: FIELD (log + restart)")
endif()
