pipeline {
    agent any

    options {
        timestamps()
        timeout(time: 30, unit: 'MINUTES')
    }

    environment {
        // Point Jenkins to your Zephyr virtual environment
        PATH = "/home/hailviral-server/zephyrproject/.venv/bin:${env.PATH}"
        ZEPHYR_BASE = "/home/hailviral-server/zephyrproject/zephyr"
    }

    stages {
        // ---------------------------------------------
        // BeagleBone - CUnit + sqlite3 (system deps)
        // ---------------------------------------------

        stage('BeagleBone Controller Tests') {
            steps {
                sh '''
                    cd beaglebone/controller/tests/unit
                    rm -rf build && mkdir build && cd build
                    cmake .. -DCMAKE_BUILD_TYPE=Debug \
                             -DCMAKE_C_FLAGS="--coverage" \
                             -DCMAKE_EXE_LINKER_FLAGS="--coverage"
                    make -j$(nproc)
                    ctest --output-on-failure
                    gcovr --xml -o cobertura.xml \
                        --root ${WORKSPACE} \
                        --filter "${WORKSPACE}/beaglebone/controller/" \
                        --exclude ".*/tests/.*" \
                        --exclude ".*/CMakeFiles/.*" \
                        .
                '''
            }
            post {
                always {
                    junit 'beaglebone/controller/tests/unit/build/junit_controller.xml'
                    junit 'beaglebone/controller/tests/unit/build/junit_db_tx.xml'
                    junit 'beaglebone/controller/tests/unit/build/junit_db_writes.xml'
                    recordCoverage(
                        id: 'bb-controller',
                        name: 'BeagleBone Controller',
                        tools: [[parser: 'COBERTURA', pattern: 'beaglebone/controller/tests/unit/build/cobertura.xml']],
                        sourceDirectories: [[path: 'beaglebone/controller']]
                    )
                }
            }
        }

        stage('BeagleBone Server Tests') {
            steps {
                sh '''
                    cd beaglebone/server/tests/unit
                    rm -rf build && mkdir build && cd build
                    cmake .. -DCMAKE_BUILD_TYPE=Debug \
                             -DCMAKE_C_FLAGS="--coverage" \
                             -DCMAKE_EXE_LINKER_FLAGS="--coverage"
                    make -j$(nproc)
                    ctest --output-on-failure
                    gcovr --xml -o cobertura.xml \
                        --root ${WORKSPACE} \
                        --filter "${WORKSPACE}/beaglebone/server/" \
                        --exclude ".*/tests/.*" \
                        --exclude ".*/CMakeFiles/.*" \
                        .
                '''
            }
            post {
                always {
                    recordCoverage(
                        id: 'bb-server',
                        name: 'BeagleBone Server',
                        tools: [[parser: 'COBERTURA', pattern: 'beaglebone/server/tests/unit/build/cobertura.xml']],
                        sourceDirectories: [[path: 'beaglebone/server']]
                    )
                }
            }
        }

        // ---------------------------------------------
        // ESP32 Hub - Unity via FetchContent
        // ---------------------------------------------

        stage('ESP32 Hub Tests') {
            steps {
                sh '''
                    cd esp32-hub/tests/unit
                    rm -rf build && mkdir build && cd build
                    cmake .. -DCMAKE_BUILD_TYPE=Debug \
                             -DCMAKE_C_FLAGS="--coverage" \
                             -DCMAKE_EXE_LINKER_FLAGS="--coverage"
                    make -j$(nproc)
                    ctest --output-on-failure
                    gcovr --xml -o cobertura.xml \
                        --root ${WORKSPACE} \
                        --filter "${WORKSPACE}/esp32-hub/" \
                        --exclude ".*/tests/.*" \
                        --exclude ".*/CMakeFiles/.*" \
                        --exclude ".*/unity/.*" \
                        .
                '''
            }
            post {
                always {
                    recordCoverage(
                        id: 'esp32-hub',
                        name: 'ESP32 Hub',
                        tools: [[parser: 'COBERTURA', pattern: 'esp32-hub/tests/unit/build/cobertura.xml']],
                        sourceDirectories: [[path: 'esp32-hub']]
                    )
                }
            }
        }

        // ---------------------------------------------
        // ESP32-C3 Motor - Unity via FetchContent
        // ---------------------------------------------

        stage('ESP32-C3 Motor Tests') {
            steps {
                sh '''
                    cd esp32c3/idf/motor/tests/unit
                    rm -rf build && mkdir build && cd build
                    cmake .. -DCMAKE_BUILD_TYPE=Debug \
                             -DCMAKE_C_FLAGS="--coverage" \
                             -DCMAKE_EXE_LINKER_FLAGS="--coverage"
                    make -j$(nproc)
                    ctest --output-on-failure
                    gcovr --xml -o cobertura.xml \
                        --root ${WORKSPACE} \
                        --filter "${WORKSPACE}/esp32c3/" \
                        --exclude ".*/tests/.*" \
                        --exclude ".*/CMakeFiles/.*" \
                        --exclude ".*/unity/.*" \
                        .
                '''
            }
            post {
                always {
                    recordCoverage(
                        id: 'esp32c3-motor',
                        name: 'ESP32-C3 Motor',
                        tools: [[parser: 'COBERTURA', pattern: 'esp32c3/idf/motor/tests/unit/build/cobertura.xml']],
                        sourceDirectories: [[path: 'esp32c3']]
                    )
                }
            }
        }

        // ---------------------------------------------
        // STM32 Blackpill - Unity via FetchContent
        // ---------------------------------------------

        stage('STM32 Blackpill Tests') {
            steps {
                sh '''
                    cd stm32-blackpill/tests/unit
                    rm -rf build && mkdir build && cd build
                    cmake .. -DCMAKE_BUILD_TYPE=Debug \
                             -DCMAKE_C_FLAGS="--coverage" \
                             -DCMAKE_EXE_LINKER_FLAGS="--coverage"
                    make -j$(nproc)
                    ctest --output-on-failure
                    gcovr --xml -o cobertura.xml \
                        --root ${WORKSPACE} \
                        --filter "${WORKSPACE}/stm32-blackpill/" \
                        --exclude ".*/tests/.*" \
                        --exclude ".*/CMakeFiles/.*" \
                        --exclude ".*/unity/.*" \
                        .
                '''
            }
            post {
                always {
                    recordCoverage(
                        id: 'stm32-blackpill',
                        name: 'STM32 Blackpill',
                        tools: [[parser: 'COBERTURA', pattern: 'stm32-blackpill/tests/unit/build/cobertura.xml']],
                        sourceDirectories: [[path: 'stm32-blackpill']]
                    )
                }
            }
        }

        // ---------------------------------------------
        // STM32 Bluepill - Unity via FetchContent
        // ---------------------------------------------

        stage('STM32 Bluepill Tests') {
            steps {
                sh '''
                    cd stm32-bluepill/tests/unit
                    rm -rf build && mkdir build && cd build
                    cmake .. -DCMAKE_BUILD_TYPE=Debug \
                             -DCMAKE_C_FLAGS="--coverage" \
                             -DCMAKE_EXE_LINKER_FLAGS="--coverage"
                    make -j$(nproc)
                    ctest --output-on-failure
                    gcovr --xml -o cobertura.xml \
                        --root ${WORKSPACE} \
                        --filter "${WORKSPACE}/stm32-bluepill/" \
                        --exclude ".*/tests/.*" \
                        --exclude ".*/CMakeFiles/.*" \
                        --exclude ".*/unity/.*" \
                        .
                '''
            }
            post {
                always {
                    recordCoverage(
                        id: 'stm32-bluepill',
                        name: 'STM32 Bluepill',
                        tools: [[parser: 'COBERTURA', pattern: 'stm32-bluepill/tests/unit/build/cobertura.xml']],
                        sourceDirectories: [[path: 'stm32-bluepill']]
                    )
                }
            }
        }

        // ---------------------------------------------
        // nRF52840 + PIR - Zephyr ztest (native_sim)
        // ---------------------------------------------

        stage('Zephyr Tests (nRF52840 + PIR)') {
            when {
                expression {
                    sh(script: 'which west', returnStatus: true) == 0
                }
            }
            steps {
                // Install both the 32-bit compiler toolchain AND the 32-bit
                // kernel headers that native_sim needs for <asm/errno.h>.
                // Without linux-libc-dev:i386 the -m32 build fails even when
                // gcc-multilib is present.
                sh '''
                    PKGS_NEEDED=""
                    dpkg -l gcc-multilib   > /dev/null 2>&1 || PKGS_NEEDED="$PKGS_NEEDED gcc-multilib g++-multilib"
                    dpkg -l linux-libc-dev:i386 > /dev/null 2>&1 || PKGS_NEEDED="$PKGS_NEEDED linux-libc-dev:i386"
                    if [ -n "$PKGS_NEEDED" ]; then
                        sudo dpkg --add-architecture i386
                        sudo apt-get update -q
                        sudo apt-get install -y $PKGS_NEEDED
                    fi
                '''

                catchError(buildResult: 'UNSTABLE', stageResult: 'UNSTABLE') {
                    sh '''
                        cd nrf52840/reed-sensor
                        west build -b native_sim -d build_test_ci tests/unit -- \
                            -DBOARD_FLASH_RUNNER=none \
                            -DCONFIG_COVERAGE=y
                        ./build_test_ci/zephyr/zephyr.exe
                        gcovr --xml -o build_test_ci/cobertura.xml \
                            --root ${WORKSPACE} \
                            --filter "${WORKSPACE}/nrf52840/reed-sensor/" \
                            --exclude ".*/tests/.*" \
                            --exclude ".*/build_test_ci/.*" \
                            build_test_ci
                    '''
                }

                catchError(buildResult: 'UNSTABLE', stageResult: 'UNSTABLE') {
                    sh '''
                        cd nrf52840/smart-light
                        west build -b native_sim -d build_test_ci tests/unit -- \
                            -DBOARD_FLASH_RUNNER=none \
                            -DCONFIG_COVERAGE=y
                        ./build_test_ci/zephyr/zephyr.exe
                        gcovr --xml -o build_test_ci/cobertura.xml \
                            --root ${WORKSPACE} \
                            --filter "${WORKSPACE}/nrf52840/smart-light/" \
                            --exclude ".*/tests/.*" \
                            --exclude ".*/build_test_ci/.*" \
                            build_test_ci
                    '''
                }

                catchError(buildResult: 'UNSTABLE', stageResult: 'UNSTABLE') {
                    sh '''
                        cd nrf52840/smart-lock
                        west build -b native_sim -d build_test_ci tests/unit -- \
                            -DBOARD_FLASH_RUNNER=none \
                            -DCONFIG_COVERAGE=y
                        ./build_test_ci/zephyr/zephyr.exe
                        gcovr --xml -o build_test_ci/cobertura.xml \
                            --root ${WORKSPACE} \
                            --filter "${WORKSPACE}/nrf52840/smart-lock/" \
                            --exclude ".*/tests/.*" \
                            --exclude ".*/build_test_ci/.*" \
                            build_test_ci
                    '''
                }

                catchError(buildResult: 'UNSTABLE', stageResult: 'UNSTABLE') {
                    sh '''
                        cd esp32c3/zephyr/pir
                        west build -b native_sim -d build_test_ci tests/unit -- \
                            -DBOARD_FLASH_RUNNER=none \
                            -DCONFIG_COVERAGE=y
                        ./build_test_ci/zephyr/zephyr.exe
                        gcovr --xml -o build_test_ci/cobertura.xml \
                            --root ${WORKSPACE} \
                            --filter "${WORKSPACE}/esp32c3/zephyr/pir/" \
                            --exclude ".*/tests/.*" \
                            --exclude ".*/build_test_ci/.*" \
                            build_test_ci
                    '''
                }
            }
            post {
                always {
                    recordCoverage(
                        id: 'zephyr-reed-sensor',
                        name: 'Zephyr nRF52840 Reed Sensor',
                        tools: [[parser: 'COBERTURA', pattern: 'nrf52840/reed-sensor/build_test_ci/cobertura.xml']]
                    )
                    recordCoverage(
                        id: 'zephyr-smart-light',
                        name: 'Zephyr nRF52840 Smart Light',
                        tools: [[parser: 'COBERTURA', pattern: 'nrf52840/smart-light/build_test_ci/cobertura.xml']]
                    )
                    recordCoverage(
                        id: 'zephyr-smart-lock',
                        name: 'Zephyr nRF52840 Smart Lock',
                        tools: [[parser: 'COBERTURA', pattern: 'nrf52840/smart-lock/build_test_ci/cobertura.xml']]
                    )
                    recordCoverage(
                        id: 'zephyr-pir',
                        name: 'Zephyr ESP32-C3 PIR',
                        tools: [[parser: 'COBERTURA', pattern: 'esp32c3/zephyr/pir/build_test_ci/cobertura.xml']]
                    )
                }
            }
        }
    }

    post {
        success {
            echo 'All tests passed - coverage reports processed.'
        }
        unstable {
            echo 'Some Zephyr tests could not run - check the stage logs above.'
        }
        failure {
            echo 'One or more tests failed - check the stage logs above.'
        }
        always {
            echo 'Cleaning up workspace...'
            cleanWs()
        }
    }
}
