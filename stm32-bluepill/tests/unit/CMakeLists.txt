cmake_minimum_required(VERSION 3.16)
project(bluepill_unit_tests C)

set(CMAKE_C_STANDARD 11)

include(FetchContent)
FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.5.2)
FetchContent_MakeAvailable(unity)

add_executable(test_bluepill
    src/test_bluepill_main.c)

target_include_directories(test_bluepill PRIVATE
    ../../UserCore/Inc
    ${unity_SOURCE_DIR}/src)

target_sources(test_bluepill PRIVATE
    ${unity_SOURCE_DIR}/src/unity.c)

enable_testing()
add_test(NAME bluepill_tests COMMAND test_bluepill)
