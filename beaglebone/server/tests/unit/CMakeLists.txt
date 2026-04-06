cmake_minimum_required(VERSION 3.16)
project(beaglebone_server_tests C)

set(CMAKE_C_STANDARD 11)

find_library(CUNIT_LIB cunit REQUIRED)
find_path(CUNIT_INC CUnit/CUnit.h REQUIRED)

add_executable(test_server
    src/test_server_main.c)

target_include_directories(test_server PRIVATE
    ../../
    ${CUNIT_INC})

target_link_libraries(test_server ${CUNIT_LIB})

enable_testing()
add_test(NAME server_tests COMMAND test_server)
