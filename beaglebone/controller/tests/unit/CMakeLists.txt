cmake_minimum_required(VERSION 3.16)
project(controller_unit_tests C)
set(CMAKE_C_STANDARD 11)

find_library(CUNIT_LIB  cunit   REQUIRED)
find_path   (CUNIT_INC  CUnit/CUnit.h REQUIRED)
find_library(SQLITE_LIB sqlite3 REQUIRED)

# --- test_controller ---
add_executable(test_controller
    src/test_controller_main.c
    src/controller_shim.c)
target_include_directories(test_controller PRIVATE
    ../../
    ${CUNIT_INC})
target_link_libraries(test_controller ${CUNIT_LIB} ${SQLITE_LIB} pthread)

# --- test_db_tx ---
#
# DB_PATH is defined in controller_internal.h as the BeagleBone on-disk path.
# We override it here at compile time for db_manager.c — the only TU that
# actually calls sqlite3_open(DB_PATH).  The shim redefines it too (for
# documentation/warning-suppression), but that only affects the shim TU.
add_executable(test_db_tx
    src/test_db_transaction.c
    src/db_manager_shim.c
    ../../db_manager.c)
target_include_directories(test_db_tx PRIVATE
    ../../
    ${CUNIT_INC})
target_compile_definitions(test_db_tx PRIVATE
    "DB_PATH=\":memory:\"")
target_link_libraries(test_db_tx ${CUNIT_LIB} ${SQLITE_LIB} pthread)

enable_testing()
add_test(NAME controller_tests     COMMAND test_controller)
add_test(NAME db_transaction_tests COMMAND test_db_tx)
