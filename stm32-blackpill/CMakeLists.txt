cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME stm32-blackpill)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C ASM)

# Add symbols globally BEFORE subdirectories so LVGL sees them
add_compile_definitions(LV_CONF_INCLUDE_SIMPLE)

# Create the executable
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add User Sources (HAL drivers + refactored dashboard modules)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # HAL / board support
    "User/Core/crash_log.c"
    "User/Core/ili9341.c"
    "User/Core/xpt2046.c"

    # Refactored dashboard modules
    "User/Core/log.c"
    "User/Core/parser.c"
    "User/Core/ui.c"

    # Trinity logging — panic, stack canary, heap hooks, HardFault handler
    "User/Core/trinity_log_stm32_f4.c"
    "User/Core/fram_driver.c"
)

# Add Include Paths
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
    "User/Core"
    "Core/Inc"
    "USB_DEVICE/App"
    "USB_DEVICE/Target"
)

# Add the fresh LVGL pull
add_subdirectory(Middlewares/lvgl)

# Silence LVGL pragma warnings by forcing definition visibility
target_compile_definitions(lvgl PUBLIC LV_CONF_INCLUDE_SIMPLE)
target_compile_definitions(lvgl_demos PUBLIC LV_CONF_INCLUDE_SIMPLE)
target_include_directories(lvgl PUBLIC "Core/Inc")
target_include_directories(lvgl_demos PUBLIC "Core/Inc")

# ── Trinity build mode ───────────────────────────────────────────────────────
# Select exactly ONE mode by passing -DTRINITY_MODE=BENCH|FIELD|USB to cmake.
# Default is FIELD (auto-reset + RTC black box) if not specified.
#
#   Bench  — UART + BKPT halt  (ST-Link inspection, dev only)
#   Field  — UART + auto-reset + RTC black box (battery / production)
#   USB    — USB CDC + auto-reset + RTC black box (USB tethered field test)
#
if(NOT DEFINED TRINITY_MODE)
    set(TRINITY_MODE "FIELD")
endif()
message("Trinity mode: " ${TRINITY_MODE})
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    TRINITY_MODE_${TRINITY_MODE}
    TRINITY_CHIP_STM32_F4=1       # Specify the Blackpill SoC
    TRINITY_USE_FRAM=1              # Essential: Blackpill uses SPI FRAM for logs
)

# Link libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    lvgl
    lvgl_demos
)

# Post-build: generate .bin from .elf
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            ${CMAKE_PROJECT_NAME}.elf
            ${CMAKE_PROJECT_NAME}.bin
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating binary file from ELF"
)
