cmake_minimum_required(VERSION 3.16)
project(doorbell_unit_tests C)
set(CMAKE_C_STANDARD 11)

include(FetchContent)
FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.5.2)
FetchContent_MakeAvailable(unity)

add_executable(test_doorbell
    src/test_doorbell_main.c
    ${unity_SOURCE_DIR}/src/unity.c)

target_include_directories(test_doorbell PRIVATE
    ../../main
    ${unity_SOURCE_DIR}/src)

enable_testing()
add_test(NAME doorbell_tests COMMAND test_doorbell)
