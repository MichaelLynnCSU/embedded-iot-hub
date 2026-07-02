/******************************************************************************
 * \file test_db_writes.c
 *
 * \brief Unit tests for db_manager.c individual write and query functions.
 *
 * \details Tests each write/query function in isolation:
 *          - db_save_uart()        — value + batt, null batt, null db
 *          - db_save_event()       — inserts row, null db noop
 *          - db_save_motor()       — online+batt, offline+null batt, null db
 *          - db_query_rooms()      — returns seeded row, returns latest state,
 *                                    respects max_rooms cap, null db returns -1
 *
 *          Companion to test_db_transaction.c which tests BEGIN/COMMIT/ROLLBACK
 *          atomicity. This file tests that each function stores the right data.
 *
 *          DB_PATH is overridden to :memory: via CMake compile-definition
 *          on the test_db_writes target (see CMakeLists.txt).
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date   2026-05-12
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sqlite3.h>
#include <pthread.h>
#include <time.h>
#include "ipc_proto.h"
#include "sensor_types.h"
#include "db_manager.h"

/*---------------------------------------------------------------------------*/
/* Test infrastructure                                                        */
/*---------------------------------------------------------------------------*/
static int g_pass = 0;
static int g_fail = 0;

#define CHECK(desc, expr) \
   do { \
      if (expr) { \
         printf("  PASS  %s\n", (desc)); \
         g_pass++; \
      } else { \
         printf("  FAIL  %s\n", (desc)); \
         g_fail++; \
      } \
   } while (0)

/*---------------------------------------------------------------------------*/
/* JUnit XML output                                                           */
/*---------------------------------------------------------------------------*/
typedef struct { const char *name; int passed; } JCase;
static JCase g_cases[256];
static int   g_case_count = 0;

#undef CHECK
#define CHECK(desc, expr) \
   do { \
      int _ok = !!(expr); \
      if (_ok) { printf("  PASS  %s\n", (desc)); g_pass++; } \
      else     { printf("  FAIL  %s\n", (desc)); g_fail++; } \
      if (g_case_count < 256) { \
         g_cases[g_case_count].name   = (desc); \
         g_cases[g_case_count].passed = _ok;    \
         g_case_count++;                         \
      } \
   } while (0)

static void junit_write(const char *suite, const char *path)
{
   int failures = 0;
   for (int i = 0; i < g_case_count; i++)
      if (!g_cases[i].passed) failures++;

   FILE *f = fopen(path, "w");
   if (!f) return;
   fprintf(f, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
   fprintf(f, "<testsuite name=\"%s\" tests=\"%d\" failures=\"%d\">\n",
           suite, g_case_count, failures);
   for (int i = 0; i < g_case_count; i++) {
      fprintf(f, "  <testcase name=\"%s\">\n", g_cases[i].name);
      if (!g_cases[i].passed)
         fprintf(f, "    <failure>%s</failure>\n", g_cases[i].name);
      fprintf(f, "  </testcase>\n");
   }
   fprintf(f, "</testsuite>\n");
   fclose(f);
}

/*---------------------------------------------------------------------------*/
/* Helpers                                                                    */
/*---------------------------------------------------------------------------*/
static int count_rows(const char *table)
{
   sqlite3_stmt *stmt = NULL;
   char          sql[128];
   int           n    = 0;
   (void)snprintf(sql, sizeof(sql), "SELECT COUNT(*) FROM %s;", table);
   if (SQLITE_OK != sqlite3_prepare_v2(db, sql, -1, &stmt, NULL))
      return -1;
   if (SQLITE_ROW == sqlite3_step(stmt))
      n = sqlite3_column_int(stmt, 0);
   sqlite3_finalize(stmt);
   return n;
}

static void clear_tables(void)
{
   (void)sqlite3_exec(db, "DELETE FROM readings;",       NULL, NULL, NULL);
   (void)sqlite3_exec(db, "DELETE FROM motor_readings;", NULL, NULL, NULL);
   (void)sqlite3_exec(db, "DELETE FROM reed_readings;",  NULL, NULL, NULL);
   (void)sqlite3_exec(db, "DELETE FROM device_events;",  NULL, NULL, NULL);
   (void)sqlite3_exec(db, "DELETE FROM room_sensors;",   NULL, NULL, NULL);
   (void)sqlite3_exec(db, "DELETE FROM uart_readings;",  NULL, NULL, NULL);
}

static void seed_room(const char *room, int sensor_id,
                      const char *state, const char *loc)
{
   sqlite3_stmt *stmt = NULL;
   sqlite3_prepare_v2(db,
      "INSERT INTO room_sensors (ts,room_name,sensor_id,state,location,batt)"
      " VALUES (?,?,?,?,?,?);",
      -1, &stmt, NULL);
   sqlite3_bind_int64(stmt, 1, (sqlite3_int64)time(NULL));
   sqlite3_bind_text (stmt, 2, room,      -1, SQLITE_STATIC);
   sqlite3_bind_int  (stmt, 3, sensor_id);
   sqlite3_bind_text (stmt, 4, state,     -1, SQLITE_STATIC);
   sqlite3_bind_text (stmt, 5, loc,       -1, SQLITE_STATIC);
   sqlite3_bind_int  (stmt, 6, 90);
   sqlite3_step(stmt);
   sqlite3_finalize(stmt);
}

/*---------------------------------------------------------------------------*/
/* db_save_motor                                                              */
/*---------------------------------------------------------------------------*/
static void test_save_motor_online_with_batt(void)
{
   clear_tables();
   db_begin();
   db_save_motor(1, 85);
   db_commit();

   sqlite3_stmt *stmt = NULL;
   sqlite3_prepare_v2(db,
      "SELECT online, batt FROM motor_readings LIMIT 1;",
      -1, &stmt, NULL);
   sqlite3_step(stmt);
   CHECK("motor: online=1 stored correctly",
         sqlite3_column_int(stmt, 0) == 1);
   CHECK("motor: batt=85 stored correctly",
         sqlite3_column_int(stmt, 1) == 85);
   sqlite3_finalize(stmt);
}

static void test_save_motor_offline_null_batt(void)
{
   clear_tables();
   db_begin();
   db_save_motor(0, -1);   /* batt=-1 must bind NULL, not -1 */
   db_commit();

   sqlite3_stmt *stmt = NULL;
   sqlite3_prepare_v2(db,
      "SELECT online, batt FROM motor_readings LIMIT 1;",
      -1, &stmt, NULL);
   sqlite3_step(stmt);
   CHECK("motor: offline stores online=0",
         sqlite3_column_int(stmt, 0) == 0);
   CHECK("motor: batt=-1 stores NULL",
         sqlite3_column_type(stmt, 1) == SQLITE_NULL);
   sqlite3_finalize(stmt);
}

static void test_save_motor_null_db_is_noop(void)
{
   clear_tables();
   sqlite3 *saved = db;
   db = NULL;
   db_save_motor(1, 50);   /* must not crash */
   db = saved;
   CHECK("motor: null db is noop — no rows inserted",
         count_rows("motor_readings") == 0);
}

/*---------------------------------------------------------------------------*/
/* db_save_uart                                                               */
/*---------------------------------------------------------------------------*/
static void test_save_uart_with_batt(void)
{
   clear_tables();
   db_begin();
   db_save_uart("PIR", 1, 72);
   db_commit();

   sqlite3_stmt *stmt = NULL;
   sqlite3_prepare_v2(db,
      "SELECT device, value, batt FROM uart_readings LIMIT 1;",
      -1, &stmt, NULL);
   sqlite3_step(stmt);
   CHECK("uart: device stored correctly",
         strcmp((const char *)sqlite3_column_text(stmt, 0), "PIR") == 0);
   CHECK("uart: value stored correctly",
         sqlite3_column_int(stmt, 1) == 1);
   CHECK("uart: batt stored correctly",
         sqlite3_column_int(stmt, 2) == 72);
   sqlite3_finalize(stmt);
}

static void test_save_uart_null_batt(void)
{
   clear_tables();
   db_begin();
   db_save_uart("LGT", 0, -1);   /* batt=-1 must bind NULL */
   db_commit();

   sqlite3_stmt *stmt = NULL;
   sqlite3_prepare_v2(db,
      "SELECT batt FROM uart_readings LIMIT 1;",
      -1, &stmt, NULL);
   sqlite3_step(stmt);
   CHECK("uart: batt=-1 stores NULL",
         sqlite3_column_type(stmt, 0) == SQLITE_NULL);
   sqlite3_finalize(stmt);
}

static void test_save_uart_null_db_is_noop(void)
{
   clear_tables();
   sqlite3 *saved = db;
   db = NULL;
   db_save_uart("PIR", 1, 50);
   db = saved;
   CHECK("uart: null db is noop — no rows inserted",
         count_rows("uart_readings") == 0);
}

/*---------------------------------------------------------------------------*/
/* db_save_event                                                              */
/*---------------------------------------------------------------------------*/
static void test_save_event_inserts_row(void)
{
   clear_tables();
   db_begin();
   db_save_event("esp32-hub", "online");
   db_commit();

   sqlite3_stmt *stmt = NULL;
   sqlite3_prepare_v2(db,
      "SELECT device, event FROM device_events LIMIT 1;",
      -1, &stmt, NULL);
   sqlite3_step(stmt);
   CHECK("event: device stored correctly",
         strcmp((const char *)sqlite3_column_text(stmt, 0), "esp32-hub") == 0);
   CHECK("event: type stored correctly",
         strcmp((const char *)sqlite3_column_text(stmt, 1), "online") == 0);
   sqlite3_finalize(stmt);
}

static void test_save_event_null_db_is_noop(void)
{
   clear_tables();
   sqlite3 *saved = db;
   db = NULL;
   db_save_event("x", "y");
   db = saved;
   CHECK("event: null db is noop — no rows inserted",
         count_rows("device_events") == 0);
}

/*---------------------------------------------------------------------------*/
/* db_query_rooms                                                             */
/*---------------------------------------------------------------------------*/
static void test_query_rooms_returns_seeded_row(void)
{
   clear_tables();
   db_begin();
   seed_room("living_room", 1, "open", "front");
   db_commit();

   struct RoomStatus out[10];
   int n = db_query_rooms(out, 10);
   CHECK("query_rooms: returns 1 row",
         n == 1);
   CHECK("query_rooms: room_name correct",
         strcmp(out[0].room_name, "living_room") == 0);
   CHECK("query_rooms: state correct",
         strcmp(out[0].state, "open") == 0);
}

static void test_query_rooms_returns_latest_state(void)
{
   clear_tables();
   db_begin();
   seed_room("bedroom", 2, "closed", "window");   /* older */
   seed_room("bedroom", 2, "open",   "window");   /* latest */
   db_commit();

   struct RoomStatus out[10];
   int n = db_query_rooms(out, 10);
   CHECK("query_rooms: deduplicates to 1 row per sensor",
         n == 1);
   CHECK("query_rooms: returns latest state not first",
         strcmp(out[0].state, "open") == 0);
}

static void test_query_rooms_respects_max_rooms(void)
{
   clear_tables();
   db_begin();
   for (int i = 1; i <= 8; i++) {
      char name[32];
      snprintf(name, sizeof(name), "room_%d", i);
      seed_room(name, i, "closed", "hall");
   }
   db_commit();

   struct RoomStatus out[3];
   int n = db_query_rooms(out, 3);
   CHECK("query_rooms: honours max_rooms cap",
         n <= 3);
}

static void test_query_rooms_null_db_returns_minus_one(void)
{
   sqlite3 *saved = db;
   db = NULL;
   struct RoomStatus out[10];
   int n = db_query_rooms(out, 10);
   db = saved;
   CHECK("query_rooms: null db returns -1",
         n == -1);
}

/*---------------------------------------------------------------------------*/
/* main                                                                       */
/*---------------------------------------------------------------------------*/
int main(void)
{
   printf("==============================================\n");
   printf(" db_manager write/query tests\n");
   printf("==============================================\n");

   db = NULL;
   db_open_and_init();
   if (NULL == db)
   {
      printf("FATAL: db_open_and_init() failed — cannot run tests\n");
      return 1;
   }

   test_save_motor_online_with_batt();
   test_save_motor_offline_null_batt();
   test_save_motor_null_db_is_noop();

   test_save_uart_with_batt();
   test_save_uart_null_batt();
   test_save_uart_null_db_is_noop();

   test_save_event_inserts_row();
   test_save_event_null_db_is_noop();

   test_query_rooms_returns_seeded_row();
   test_query_rooms_returns_latest_state();
   test_query_rooms_respects_max_rooms();
   test_query_rooms_null_db_returns_minus_one();

   printf("\n==============================================\n");
   printf(" Results: %d passed  %d failed\n", g_pass, g_fail);
   printf("==============================================\n\n");

   junit_write("db_write_tests", "junit_db_writes.xml");

   sqlite3_close(db);
   return g_fail;
}
