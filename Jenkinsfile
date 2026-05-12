pipeline {
    agent any

    options {
        timestamps()
        timeout(time: 30, unit: 'MINUTES')
    }

    stages {
        // ─────────────────────────────────────────────
        // BeagleBone — CUnit + sqlite3
        // ─────────────────────────────────────────────
        stage('BeagleBone Controller Tests') {
            steps {
                sh '''
                    cd beaglebone/controller/tests/unit
                    rm -rf build && mkdir build && cd build
                    cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_FLAGS="--coverage" -DCMAKE_EXE_LINKER_FLAGS="--coverage"
                    make -j$(nproc)
                    ctest --output-on-failure
                    gcovr --xml -o cobertura.xml --root ${WORKSPACE} --filter "${WORKSPACE}/beaglebone/controller/" --exclude ".*/tests/.*" --exclude ".*/CMakeFiles/.*" .
                '''
            }
            post {
                always {
                    recordCoverage(
                        id: 'bb-controller',
                        name: 'BeagleBone Controller',
                        tools: [[parser: 'COBERTURA', pattern: 'beaglebone/controller/tests/unit/build/cobertura.xml']]
                    )
                }
            }
        }

        stage('BeagleBone Server Tests') {
            steps {
                sh '''
                    cd beaglebone/server/tests/unit
                    rm -rf build && mkdir build && cd build
                    cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_FLAGS="--coverage" -DCMAKE_EXE_LINKER_FLAGS="--coverage"
                    make -j$(nproc)
                    ctest --output-on-failure
                    gcovr --xml -o cobertura.xml --root ${WORKSPACE} --filter "${WORKSPACE}/beaglebone/server/" --exclude ".*/tests/.*" --exclude ".*/CMakeFiles/.*" .
                '''
            }
            post {
                always {
                    recordCoverage(
                        id: 'bb-server',
                        name: 'BeagleBone Server',
                        tools: [[parser: 'COBERTURA', pattern: 'beaglebone/server/tests/unit/build/cobertura.xml']]
                    )
                }
            }
        }

        // ─────────────────────────────────────────────
        // ESP32 Hub
        // ─────────────────────────────────────────────
        stage('ESP32 Hub Tests') {
            steps {
                sh '''
                    cd esp32-hub/tests/unit
                    rm -rf build && mkdir build && cd build
                    cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_FLAGS="--coverage" -DCMAKE_EXE_LINKER_FLAGS="--coverage"
                    make -j$(nproc)
                    ctest --output-on-failure
                    gcovr --xml -o cobertura.xml --root ${WORKSPACE} --filter "${WORKSPACE}/esp32-hub/" --exclude ".*/tests/.*" --exclude ".*/CMakeFiles/.*" --exclude ".*/unity/.*" .
                '''
            }
            post {
                always {
                    recordCoverage(
                        id: 'esp32-hub',
                        name: 'ESP32 Hub',
                        tools: [[parser: 'COBERTURA', pattern: 'esp32-hub/tests/unit/build/cobertura.xml']]
                    )
                }
            }
        }

        // ─────────────────────────────────────────────
        // ESP32-C3 Motor
        // ─────────────────────────────────────────────
        stage('ESP32-C3 Motor Tests') {
            steps {
                sh '''
                    cd esp32c3/idf/motor/tests/unit
                    rm -rf build && mkdir build && cd build
                    cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_FLAGS="--coverage" -DCMAKE_EXE_LINKER_FLAGS="--coverage"
                    make -j$(nproc)
                    ctest --output-on-failure
                    gcovr --xml -o cobertura.xml --root ${WORKSPACE} --filter "${WORKSPACE}/esp32c3/" --exclude ".*/tests/.*" --exclude ".*/CMakeFiles/.*" --exclude ".*/unity/.*" .
                '''
            }
            post {
                always {
                    recordCoverage(
                        id: 'esp32c3-motor',
                        name: 'ESP32-C3 Motor',
                        tools: [[parser: 'COBERTURA', pattern: 'esp32c3/idf/motor/tests/unit/build/cobertura.xml']]
                    )
                }
            }
        }

        // ─────────────────────────────────────────────
        // STM32 Blackpill
        // ─────────────────────────────────────────────
        stage('STM32 Blackpill Tests') {
            steps {
                sh '''
                    cd stm32-blackpill/tests/unit
                    rm -rf build && mkdir build && cd build
                    cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_FLAGS="--coverage" -DCMAKE_EXE_LINKER_FLAGS="--coverage"
                    make -j$(nproc)
                    ctest --output-on-failure
                    gcovr --xml -o cobertura.xml --root ${WORKSPACE} --filter "${WORKSPACE}/stm32-blackpill/" --exclude ".*/tests/.*" --exclude ".*/CMakeFiles/.*" --exclude ".*/unity/.*" .
                '''
            }
            post {
                always {
                    recordCoverage(
                        id: 'stm32-blackpill',
                        name: 'STM32 Blackpill',
                        tools: [[parser: 'COBERTURA', pattern: 'stm32-blackpill/tests/unit/build/cobertura.xml']]
                    )
                }
            }
        }

        // ─────────────────────────────────────────────
        // STM32 Bluepill
        // ─────────────────────────────────────────────
        stage('STM32 Bluepill Tests') {
            steps {
                sh '''
                    cd stm32-bluepill/tests/unit
                    rm -rf build && mkdir build && cd build
                    cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_FLAGS="--coverage" -DCMAKE_EXE_LINKER_FLAGS="--coverage"
                    make -j$(nproc)
                    ctest --output-on-failure
                    gcovr --xml -o cobertura.xml --root ${WORKSPACE} --filter "${WORKSPACE}/stm32-bluepill/" --exclude ".*/tests/.*" --exclude ".*/CMakeFiles/.*" --exclude ".*/unity/.*" .
                '''
            }
            post {
                always {
                    recordCoverage(
                        id: 'stm32-bluepill',
                        name: 'STM32 Bluepill',
                        tools: [[parser: 'COBERTURA', pattern: 'stm32-bluepill/tests/unit/build/cobertura.xml']]
                    )
                }
            }
        }
    }

    post {
        success { echo '✅ All tests passed.' }
        failure { echo '❌ One or more tests failed — check the stage logs above.' }
        always { cleanWs() }
    }
}
