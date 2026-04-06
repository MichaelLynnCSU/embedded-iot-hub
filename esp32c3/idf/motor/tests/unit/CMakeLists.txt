cmake_minimum_required(VERSION 3.16)
project(motor_unit_tests C)

set(CMAKE_C_STANDARD 11)

include(FetchContent)
FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.5.2)
FetchContent_MakeAvailable(unity)

add_executable(test_motor
    src/test_motor_main.c)

target_include_directories(test_motor PRIVATE
    ../../main
    ${unity_SOURCE_DIR}/src)

target_sources(test_motor PRIVATE
    ${unity_SOURCE_DIR}/src/unity.c)

enable_testing()
add_test(NAME motor_tests COMMAND test_motor)
