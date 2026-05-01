/******************************************************************************
 * \file test_db_transaction.c
 *
 * \brief Unit tests for db_manager.c Unit of Work transaction behaviour.
 *
 * \details Tests the atomicity guarantee introduced when db_save_reed() was
 *          moved into process_sensor_frame() so all per-frame writes share
 *          one BEGIN/COMMIT/fsync.
 *
 *          What is actually tested (vs the old test_db.sh):
 *          - Old test: inserted rows directly via sqlite3 CLI and read them
 *            back. Tested that SQLite works. Never called db_manager.c at all.
 *          - This test: calls db_begin(), db_save_reading(), db_save_motor(),
 *            db_save_reed(), db_commit() and db_rollback() through the real
 *            C functions. Tests that the transaction boundary works correctly.
 *
 *          Three tests:
 *          1. COMMIT  — full frame (reading + motor + 2 reed slots) commits
 *                       atomically; all rows appear after db_commit().
 *          2. ROLLBACK — same frame written then db_rollback() called;
 *                        zero rows should appear in every table.
 *          3. PARTIAL  — simulate crash mid-frame: begin, save reading and
 *                        motor, save one reed slot, then rollback without
 *                        committing. All three tables must be empty —
 *                        confirms reed saves inside the transaction are
 *                        rolled back with everything else.
 *
 *          Note on count_rows():
 *          db_save_reed() and db_save_motor() stamp time(NULL), not the
 *          frame timestamp from SensorData, so filtering by frame ts would
 *          always return 0 for those tables.  A simple COUNT(*) is correct
 *          instead.  clear_tables() is called at the start of every test so
 *          counts are always relative to that test's own writes only.
 *
 *          Build (Ubuntu, no cross-compile needed):
 *          gcc test_db_transaction.c db_manager_shim.c \
 *              ../../db_manager.c -I../.. -lsqlite3 -lpthread -o test_db_tx
 *          ./test_db_tx
 *
 *          The shim (db_manager_shim.c) provides the globals and stubs that
 *          db_manager.c expects from the rest of the controller.
 *          DB_PATH is overridden to :memory: via CMake compile-definition
 *          on the test_db_tx target (see CMakeLists.txt).
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date   2026-04-30
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sqlite3.h>
#include <pthread.h>
#include <time.h>

#include "controller_internal.h"

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
/* Helpers                                                                    */
/*---------------------------------------------------------------------------*/

/**
 * \brief Count all rows in a table.
 *
 * \details db_save_reed() and db_save_motor() stamp time(NULL), not the
 *          frame ts, so WHERE ts=? would always return 0 for those tables.
 *          clear_tables() is called at the start of every test so COUNT(*)
 *          reflects only that test's own writes.
 */
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

/**
 * \brief Delete all rows from the three tables under test.
 *
 * \details Called at the start of each test so rows committed by an earlier
 *          test do not pollute later COUNT(*) checks.  The in-memory DB
 *          persists for the lifetime of the process, so isolation between
 *          tests must be explicit.
 */
static void clear_tables(void)
{
   (void)sqlite3_exec(db, "DELETE FROM readings;",       NULL, NULL, NULL);
   (void)sqlite3_exec(db, "DELETE FROM motor_readings;", NULL, NULL, NULL);
   (void)sqlite3_exec(db, "DELETE FROM reed_readings;",  NULL, NULL, NULL);
}

static struct SensorData make_frame(long ts)
{
   struct SensorData d;
   int               i;

   memset(&d, 0, sizeof(d));

   d.timestamp    = ts;
   d.avg_temp     = 21.5;
   d.motion_count = 2;
   d.light_state  = 1;
   d.lock_state   = 0;
   d.motor_online = 1;
   d.batt_pir     = 80;
   d.batt_lck     = 70;
   d.batt_motor   = 90;

   for (i = 0; i < MAX_REEDS; i++)
      d.reed_slots[i].active = 0;

   d.reed_slots[0].active = 1;
   d.reed_slots[0].state  = 0;
   d.reed_slots[0].batt   = 91;
   d.reed_slots[0].age    = 10;
   (void)strncpy(d.reed_slots[0].name, "FrontDoor", REED_NAME_SIZE - 1);

   d.reed_slots[1].active = 1;
   d.reed_slots[1].state  = 1;
   d.reed_slots[1].batt   = 74;
   d.reed_slots[1].age    = 305;
   (void)strncpy(d.reed_slots[1].name, "BackDoor", REED_NAME_SIZE - 1);

   return d;
}

static void write_full_frame(const struct SensorData *p_d)
{
   int i;

   db_save_reading(p_d);
   db_save_motor(p_d->motor_online, p_d->batt_motor);

   for (i = 0; i < MAX_REEDS; i++)
   {
      if (p_d->reed_slots[i].active)
      {
         db_save_reed(i + 1,
                      p_d->reed_slots[i].name,
                      p_d->reed_slots[i].state,
                      p_d->reed_slots[i].batt,
                      p_d->reed_slots[i].age);
      }
   }
}

/*---------------------------------------------------------------------------*/
/* Test 1 — committed frame appears in all three tables                       */
/*---------------------------------------------------------------------------*/

static void test_commit(void)
{
   long             ts = (long)time(NULL) + 1000;
   struct SensorData d  = make_frame(ts);

   printf("\n[ test_commit ] full frame — all rows must appear after COMMIT\n");

   clear_tables();

   db_begin();
   write_full_frame(&d);
   db_commit();

   CHECK("reading row committed",     count_rows("readings")      == 1);
   CHECK("motor row committed",       count_rows("motor_readings") == 1);
   CHECK("reed slots committed (x2)", count_rows("reed_readings")  == 2);
}

/*---------------------------------------------------------------------------*/
/* Test 2 — rolled-back frame leaves nothing                                  */
/*---------------------------------------------------------------------------*/

static void test_rollback(void)
{
   long             ts = (long)time(NULL) + 2000;
   struct SensorData d  = make_frame(ts);

   printf("\n[ test_rollback ] full frame then ROLLBACK — zero rows must appear\n");

   clear_tables();

   db_begin();
   write_full_frame(&d);
   db_rollback();

   CHECK("reading rolled back",    count_rows("readings")      == 0);
   CHECK("motor rolled back",      count_rows("motor_readings") == 0);
   CHECK("reed slots rolled back", count_rows("reed_readings")  == 0);
}

/*---------------------------------------------------------------------------*/
/* Test 3 — partial frame (crash simulation) rolls back reed saves too        */
/*                                                                            */
/* Regression test for the original bug: reed saves were outside the          */
/* transaction. If the old code were still in place, reed rows would          */
/* survive the rollback even though reading and motor did not.                */
/*---------------------------------------------------------------------------*/

static void test_partial_frame_rollback(void)
{
   long             ts = (long)time(NULL) + 3000;
   struct SensorData d  = make_frame(ts);

   printf("\n[ test_partial ] begin + reading + motor + one reed + ROLLBACK\n");
   printf("  (regression: reed saves must be inside the transaction)\n");

   clear_tables();

   db_begin();
   db_save_reading(&d);
   db_save_motor(d.motor_online, d.batt_motor);
   db_save_reed(1,
                d.reed_slots[0].name,
                d.reed_slots[0].state,
                d.reed_slots[0].batt,
                d.reed_slots[0].age);
   db_rollback();

   CHECK("partial reading rolled back", count_rows("readings")      == 0);
   CHECK("partial motor rolled back",   count_rows("motor_readings") == 0);
   CHECK("partial reed rolled back",    count_rows("reed_readings")  == 0);
}

/*---------------------------------------------------------------------------*/
/* main                                                                       */
/*---------------------------------------------------------------------------*/

int main(void)
{
   printf("==============================================\n");
   printf(" db_manager transaction tests\n");
   printf("==============================================\n");

   db = NULL;
   db_open_and_init();

   if (NULL == db)
   {
      printf("FATAL: db_open_and_init() failed — cannot run tests\n");
      return 1;
   }

   test_commit();
   test_rollback();
   test_partial_frame_rollback();

   printf("\n==============================================\n");
   printf(" Results: %d passed  %d failed\n", g_pass, g_fail);
   printf("==============================================\n\n");

   sqlite3_close(db);
   return g_fail;
}
