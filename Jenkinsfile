pipeline {
    agent any

    options {
        timestamps()
        timeout(time: 30, unit: 'MINUTES')
    }

    stages {

        // ─────────────────────────────────────────────
        // BeagleBone — CUnit + sqlite3 (system deps)
        // ─────────────────────────────────────────────

        stage('BeagleBone Controller Tests') {
            steps {
                sh '''
                    cd beaglebone/controller/tests/unit
                    rm -rf build && mkdir build && cd build
                    cmake .. -DCMAKE_BUILD_TYPE=Debug
                    make -j$(nproc)
                    ctest --output-on-failure
                '''
            }
        }

        stage('BeagleBone Server Tests') {
            steps {
                sh '''
                    cd beaglebone/server/tests/unit
                    rm -rf build && mkdir build && cd build
                    cmake .. -DCMAKE_BUILD_TYPE=Debug
                    make -j$(nproc)
                    ctest --output-on-failure
                '''
            }
        }

        // ─────────────────────────────────────────────
        // ESP32 Hub — Unity via FetchContent
        // ─────────────────────────────────────────────

        stage('ESP32 Hub Tests') {
            steps {
                sh '''
                    cd esp32-hub/tests/unit
                    rm -rf build && mkdir build && cd build
                    cmake .. -DCMAKE_BUILD_TYPE=Debug
                    make -j$(nproc)
                    ctest --output-on-failure
                '''
            }
        }

        // ─────────────────────────────────────────────
        // ESP32-C3 Motor — Unity via FetchContent
        // ─────────────────────────────────────────────

        stage('ESP32-C3 Motor Tests') {
            steps {
                sh '''
                    cd esp32c3/idf/motor/tests/unit
                    rm -rf build && mkdir build && cd build
                    cmake .. -DCMAKE_BUILD_TYPE=Debug
                    make -j$(nproc)
                    ctest --output-on-failure
                '''
            }
        }

        // ─────────────────────────────────────────────
        // STM32 Blackpill — Unity via FetchContent
        // ─────────────────────────────────────────────

        stage('STM32 Blackpill Tests') {
            steps {
                sh '''
                    cd stm32-blackpill/tests/unit
                    rm -rf build && mkdir build && cd build
                    cmake .. -DCMAKE_BUILD_TYPE=Debug
                    make -j$(nproc)
                    ctest --output-on-failure
                '''
            }
        }

        // ─────────────────────────────────────────────
        // STM32 Bluepill — Unity via FetchContent
        // ─────────────────────────────────────────────

        stage('STM32 Bluepill Tests') {
            steps {
                sh '''
                    cd stm32-bluepill/tests/unit
                    rm -rf build && mkdir build && cd build
                    cmake .. -DCMAKE_BUILD_TYPE=Debug
                    make -j$(nproc)
                    ctest --output-on-failure
                '''
            }
        }

        // ─────────────────────────────────────────────
        // nRF52840 + PIR — Zephyr ztest (native_sim)
        // Requires: Zephyr SDK + west installed on agent
        // ─────────────────────────────────────────────

        stage('Zephyr Tests (nRF52840 + PIR)') {
            when {
                expression {
                    sh(script: 'which west', returnStatus: true) == 0
                }
            }
            steps {
                sh '''
                    cd nrf52840/reed-sensor
                    west build -b native_sim -d build_test_ci tests/unit -- -DBOARD_FLASH_RUNNER=none
                    ./build_test_ci/zephyr/zephyr.exe

                    cd ../../nrf52840/smart-light
                    west build -b native_sim -d build_test_ci tests/unit -- -DBOARD_FLASH_RUNNER=none
                    ./build_test_ci/zephyr/zephyr.exe

                    cd ../../nrf52840/smart-lock
                    west build -b native_sim -d build_test_ci tests/unit -- -DBOARD_FLASH_RUNNER=none
                    ./build_test_ci/zephyr/zephyr.exe

                    cd ../../esp32c3/zephyr/pir
                    west build -b native_sim -d build_test_ci tests/unit -- -DBOARD_FLASH_RUNNER=none
                    ./build_test_ci/zephyr/zephyr.exe
                '''
            }
        }
    }

    post {
        success {
            echo '✅ All tests passed.'
        }
        failure {
            echo '❌ One or more tests failed — check the stage logs above.'
        }
        always {
            cleanWs()
        }
    }
}
