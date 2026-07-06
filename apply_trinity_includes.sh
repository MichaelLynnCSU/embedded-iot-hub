#!/usr/bin/env bash
# Run this from the repo ROOT: cd ~/embedded-iot-hub && bash apply_trinity_includes.sh
set -euo pipefail

# --- nRF52840 Zephyr boards ---
sed -i '/^    drivers\/fuel_gauge\/max17048$/a\    ../../include' \
    nrf52840/reed-sensor/CMakeLists.txt \
    nrf52840/temp-sensor/CMakeLists.txt

cat >> nrf52840/smart-light/CMakeLists.txt << 'EOF'

target_include_directories(app PRIVATE
    ../../include
)
EOF

cat >> nrf52840/smart-lock/CMakeLists.txt << 'EOF'

target_include_directories(app PRIVATE
    ../../include
)
EOF

# --- ESP32-C3 Zephyr (pir) ---
cat >> esp32c3/zephyr/pir/CMakeLists.txt << 'EOF'

target_include_directories(app PRIVATE
    ../../../include
)
EOF

# --- ESP32 IDF boards ---
sed -i 's/INCLUDE_DIRS "\." "include"/INCLUDE_DIRS "." "include" "..\/..\/include"/' \
    esp32-cam/main/CMakeLists.txt \
    esp32-doorbell/main/CMakeLists.txt \
    esp32-hub/main/CMakeLists.txt

# motor is one directory deeper than cam/doorbell/hub -- different relative depth
sed -i 's/INCLUDE_DIRS "\."$/INCLUDE_DIRS "." "..\/..\/..\/..\/include"/' \
    esp32c3/idf/motor/main/CMakeLists.txt

# --- STM32 (both boards) ---
sed -i 's|"USB_DEVICE/Target"|"USB_DEVICE/Target"\n    "../include"|' \
    stm32-blackpill/CMakeLists.txt

sed -i 's|UserCore/Inc$|UserCore/Inc\n    ../include|' \
    stm32-bluepill/CMakeLists.txt

echo "Done. Review with: git diff --stat"
