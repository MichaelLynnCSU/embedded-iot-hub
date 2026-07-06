idf_component_register(SRCS "main.c"
        "trinity_wdt_idf.c"
        "trinity_canary_idf.c"
        "trinity_nvs_idf.c"
        "trinity_stats_idf.c"
        "trinity_panic_idf.c"
        "build_info.c"
        "trinity_log_stack.c"
        "buzzer.c"
    INCLUDE_DIRS "." "include" "../../include")

if(DEFINED DOORBELL_ID)
    target_compile_definitions(__idf_main PUBLIC DOORBELL_ID=${DOORBELL_ID})
endif()

# DOORBELL_MODE build flag (2026-06-14):
#   idf.py -DDOORBELL_MODE=SNAPSHOT build   (default if unset)
#   idf.py -DDOORBELL_MODE=STREAM   build
#
# Translated to the integer macros DOORBELL_MODE_SNAPSHOT (0) /
# DOORBELL_MODE_STREAM (1) defined in cam_logic.h, since
# target_compile_definitions passes raw text to the preprocessor and
# "STREAM"/"SNAPSHOT" are not themselves valid macro values.
if(DEFINED DOORBELL_MODE)
    if("${DOORBELL_MODE}" STREQUAL "STREAM")
        target_compile_definitions(__idf_main PUBLIC DOORBELL_MODE=DOORBELL_MODE_STREAM)
        message(STATUS "Doorbell mode: STREAM (continuous MJPEG to BBB:9093)")
    elseif("${DOORBELL_MODE}" STREQUAL "SNAPSHOT")
        target_compile_definitions(__idf_main PUBLIC DOORBELL_MODE=DOORBELL_MODE_SNAPSHOT)
        message(STATUS "Doorbell mode: SNAPSHOT (single JPEG to BBB:9091)")
    else()
        message(FATAL_ERROR "Invalid DOORBELL_MODE='${DOORBELL_MODE}' — must be SNAPSHOT or STREAM")
    endif()
else()
    message(STATUS "Doorbell mode: SNAPSHOT (default)")
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
