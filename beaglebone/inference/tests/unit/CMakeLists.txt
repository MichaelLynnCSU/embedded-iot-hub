cmake_minimum_required(VERSION 3.16)
project(inference_unit_tests C)
set(CMAKE_C_STANDARD 11)

find_library(CUNIT_LIB cunit REQUIRED)
find_path   (CUNIT_INC CUnit/CUnit.h REQUIRED)

add_executable(test_inference
    src/test_inference_main.c)

target_include_directories(test_inference PRIVATE
    ../../app
    ${CUNIT_INC})

target_compile_definitions(test_inference PRIVATE
    "JUNIT_OUTPUT_PATH=\"${CMAKE_CURRENT_BINARY_DIR}/junit_inference.xml\"")

target_link_libraries(test_inference ${CUNIT_LIB})

enable_testing()
add_test(NAME inference_tests COMMAND test_inference)
