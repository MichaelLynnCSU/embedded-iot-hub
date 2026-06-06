cmake_minimum_required(VERSION 3.16)
project(cam_unit_tests C)
set(CMAKE_C_STANDARD 11)

include(FetchContent)
FetchContent_Declare(unity
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
    GIT_TAG        v2.5.2)
FetchContent_MakeAvailable(unity)

set(STUB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/stubs)

add_executable(test_cam
    src/test_cam_main.c
    ${unity_SOURCE_DIR}/src/unity.c)

target_include_directories(test_cam PRIVATE
    ${STUB_DIR}
    ../../main
    ${unity_SOURCE_DIR}/src)

enable_testing()
add_test(NAME cam_tests COMMAND test_cam)
