cmake_minimum_required(VERSION 3.16)
project(controller_unit_tests C)

set(CMAKE_C_STANDARD 11)

find_library(CUNIT_LIB cunit REQUIRED)
find_path(CUNIT_INC CUnit/CUnit.h REQUIRED)

add_executable(test_controller
    src/test_controller_main.c)

target_include_directories(test_controller PRIVATE
    ../../
    ${CUNIT_INC})

target_link_libraries(test_controller ${CUNIT_LIB})

enable_testing()
add_test(NAME controller_tests COMMAND test_controller)
