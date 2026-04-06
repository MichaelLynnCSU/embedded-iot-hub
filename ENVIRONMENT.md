# Environment & Build Specifications
This document defines the verified toolchains, SDKs, and kernel versions required to build and deploy the **Embedded IoT Hub** ecosystem.

## Quick Reference: Verified Host Versions
| Target         | SDK / Framework            | Toolchain / Compiler                 |
|:---------------|:---------------------------|:-------------------------------------|
| **BeagleBone** | Debian 13 (6.17.8-rt7)     | arm-linux-gnueabihf-gcc (13.3.0)     |
| **ESP32-Hub**  | ESP-IDF v6.1-dev           | xtensa-esp-elf-gcc (15.2.0)          |
| **ESP32-C3**   | ESP-IDF v6.1-dev           | riscv32-esp-elf-gcc (15.2.0)         |
| **nRF52840**   | Zephyr v3.4.0              | Zephyr SDK 0.17.4 (GCC 12.2.0)       |
| **STM32F103**  | STM32CubeMX + CMake        | arm-none-eabi-gcc (13.2.1)           |
| **STM32F411**  | STM32CubeMX + CMake        | arm-none-eabi-gcc (13.2.1)           |

---

## 1. BeagleBone Black (Central Gateway)
- **Kernel Version:** `6.17.8+`
- **Kernel Configuration:** `PREEMPT_RT (6.17.8-rt7)` enabled for deterministic processing.
- **Compiler:** `arm-linux-gnueabihf-gcc (Ubuntu 13.3.0-6ubuntu2~24.04)`
- **Bootloader:** U-Boot `v2022.04` (Binary: `u-boot-dtb.img`)
- **Device Tree:** `am335x-boneblack.dtb` (Compiled from 6.17 source)
- **Primary Communications:** UART4 (Pins 28, 29) and I2C2 (Pins 94, 95)

---

## 2. ESP32-Hub (Communication Bridge)
- **Architecture:** Xtensa Dual-Core
- **SDK/Framework:** ESP-IDF `v6.1-dev-1532-gb6e2de03f0`
- **Toolchain:** `xtensa-esp-elf-gcc (15.2.0)`
- **Build System:** CMake / Ninja
- **Pinned Dependencies:**
  - `espressif/cjson`: `v1.7.19`
- **Storage Layout:** Custom `partitions.csv` for NVS and OTA support.
- **Build/Flash Commands:**
```bash
idf.py build
idf.py flash -p /dev/ttyUSB0
```

---

## 3. ESP32-C3-Motor (Actuator Node)
- **Architecture:** RISC-V Single-Core
- **SDK/Framework:** ESP-IDF `v6.1-dev`
- **Toolchain:** `riscv32-esp-elf-gcc (15.2.0)`
- **Power Management:** RTC Retain Memory "Black Box" logging for brownout recovery.
- **Profiles:**
  - `sdkconfig.usb`: USB-Serial-JTAG Console
  - `sdkconfig.uart`: Low-power deployment mode
- **Zephyr HAL Note:** For Zephyr builds on ESP32-C3 boards, ensure `hal_espressif` module is fetched and overlays exist:
```bash
west update
west zephyr-export
```

---

## 4. STM32-BluePill (Sensor Hub)
- **Hardware:** STM32F103C8T6
- **Toolchain:** `arm-none-eabi-gcc (13.2.1)`
- **Build System:** STM32CubeMX + CMake
- **Firmware Structure:** Modular `UserCore` architecture (drivers in `UserCore/Src/`, headers in `UserCore/Inc/`)
- **Linker Script:** `STM32F103XX_FLASH.ld` (64KB flash, 20KB RAM)
- **Startup File:** `startup_stm32f103xb.s`
- **RTOS:** FreeRTOS via CMSIS-RTOS v1
- **Key Dependencies:**
  - STM32F1xx HAL
  - FreeRTOS (CMSIS-RTOS)
  - Custom drivers: `dht11_driver`, `fram_driver`, `crash_log`
- **CubeMX Regeneration Safety:** `.ioc` configured with `DeletePrevious=false`, `BackupPrevious=true`, `KeepUserCode=true`
- **Build Commands:**
```bash
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake --build . -j$(nproc)
```
- **Flash Command:**
```bash
st-flash write build/stm32-bluepill.elf 0x08000000
```

---

## 5. STM32-BlackPill (UI Dashboard)
- **Hardware:** STM32F411CEU6
- **Toolchain:** `arm-none-eabi-gcc (13.2.1)`
- **Build System:** STM32CubeMX + CMake
- **Firmware Structure:** Modular `UserCore` architecture, LVGL display stack
- **Linker Script:** `STM32F411CEUX_FLASH.ld` (512KB flash, 128KB RAM)
- **Startup File:** `startup_stm32f411xe.s`
- **Key Dependencies:**
  - STM32F4xx HAL
  - USB Device Library (CDC)
  - LVGL v8.3 (`Middlewares/lvgl`)
  - Custom drivers: `ili9341`, `xpt2046`, `crash_log`
- **CubeMX Regeneration Safety:** `.ioc` configured with `DeletePrevious=false`, `BackupPrevious=true`, `KeepUserCode=true`
- **Build Commands:**
```bash
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake --build . -j$(nproc)
```
- **Flash Command:**
```bash
st-flash write build/stm32-blackpill.bin 0x08000000
```

---

## 6. nRF52840 (Smart Lock/Sensor)
- **RTOS:** Zephyr `v3.4.0`
- **SDK:** Zephyr SDK `0.17.4`
- **Toolchain:** `arm-zephyr-eabi-gcc (12.2.0)`
- **Management Tool:** `west`
- **Security:** Hardware-bound identity persistence and custom BLE advertising data
- **Zephyr Modules:** Fetch all modules before building:
```bash
west update
west zephyr-export
```

---

## Build Host Requirements
- **CMake:** 3.24+ (3.28.3 validated)
- **Python:** 3.12.3
- **Ninja Build:** 1.11.1
- **Build Host:** x86_64 Linux

### Setup Commands
```bash
# For ESP32 targets
source ~/esp/esp-idf/export.sh

# For Zephyr targets
export ZEPHYR_SDK_INSTALL_DIR=$HOME/zephyr-sdk-0.17.4
west update
west zephyr-export
```
