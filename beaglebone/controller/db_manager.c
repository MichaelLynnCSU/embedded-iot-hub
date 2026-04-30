/******************************************************************************
 * \file db_manager.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief SQLite database manager for BeagleBone data controller.
 *
 * \details Manages all database operations for the data controller.
 *          Creates and maintains six tables:
 *          - readings:      main sensor frames (temp, motion, light, lock,
 *                           motor, battery SOC)
 *          - uart_readings: inbound UART frames from STM32 (PIR, LGT, LCK)
 *          - reed_readings: reed slot snapshots (state, battery, age)
 *          - motor_readings: motor controller state snapshots
 *          - device_events: device online/offline and generation change log
 *          - room_sensors:  room sensor states from ESP32 JSON
 *
 *          Design patterns applied:
 *          - Null Object:   every function guards on db == NULL and no-ops
 *          - Facade:        hides raw SQLite API from all callers
 *          - Repository:    db_query_rooms() returns domain structs, never
 *                           touches shm — caller owns shm writes
 *          - Unit of Work:  db_begin() / db_commit() / db_rollback() let
 *                           callers batch writes into one fsync
 *          - Decorator:     db_save_event_core() is pure; db_save_event()
 *                           wraps with timestamp and LOG
 *
 *          SQL injection:   all write functions use prepared statements
 *                           with sqlite3_bind_* — no string interpolation.
 *
 *          All functions are no-ops if db is NULL (open failed at init).
 ******************************************************************************/

#include "controller_internal.h"

#define ROOM_QUERY_LIMIT  10   /**< maximum rooms returned by db_query_rooms */

/******************************************************************************
 * \brief Open SQLite database and create tables if they do not exist.
 *
 * \return void
 *
 * \details Called from main() after log_fp is open. Sets db to NULL and
 *          logs a warning if the database cannot be opened. Enables WAL
 *          mode for better concurrent read performance on BeagleBone SD.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void db_open_and_init(void)
{
   char       *p_err = NULL; /**< SQLite error message pointer */
   const char *p_sql = NULL; /**< SQL statement string */

   if (SQLITE_OK != sqlite3_open(DB_PATH, &db))
   {
      LOG("WARNING: DB open failed, running without persistence");
      db = NULL;
      return;
   }

   LOG("Database opened: %s", DB_PATH);

   /* WAL mode — better concurrent reads, fewer fsyncs on SD card */
   (void)sqlite3_exec(db, "PRAGMA journal_mode=WAL;", NULL, NULL, NULL);

   p_sql =
      "CREATE TABLE IF NOT EXISTS readings ("
      "  id           INTEGER PRIMARY KEY AUTOINCREMENT,"
      "  ts           INTEGER NOT NULL,"
      "  source       TEXT    NOT NULL,"
      "  avg_temp     REAL    DEFAULT 0,"
      "  motion_count INTEGER DEFAULT 0,"
      "  light_state  INTEGER DEFAULT 0,"
      "  lock_state   INTEGER DEFAULT 0,"
      "  motor_online INTEGER DEFAULT 0,"
      "  batt_pir     INTEGER,"
      "  batt_lck     INTEGER,"
      "  batt_motor   INTEGER);"

      "CREATE TABLE IF NOT EXISTS uart_readings ("
      "  id     INTEGER PRIMARY KEY AUTOINCREMENT,"
      "  ts     INTEGER NOT NULL,"
      "  device TEXT    NOT NULL,"
      "  value  INTEGER NOT NULL,"
      "  batt   INTEGER);"

      "CREATE TABLE IF NOT EXISTS reed_readings ("
      "  id     INTEGER PRIMARY KEY AUTOINCREMENT,"
      "  ts     INTEGER NOT NULL,"
      "  slot   INTEGER NOT NULL,"
      "  name   TEXT,"
      "  state  INTEGER NOT NULL,"
      "  batt   INTEGER,"
      "  age    INTEGER);"

      "CREATE TABLE IF NOT EXISTS motor_readings ("
      "  id     INTEGER PRIMARY KEY AUTOINCREMENT,"
      "  ts     INTEGER NOT NULL,"
      "  online INTEGER NOT NULL,"
      "  batt   INTEGER);"

      "CREATE TABLE IF NOT EXISTS device_events ("
      "  id     INTEGER PRIMARY KEY AUTOINCREMENT,"
      "  ts     INTEGER NOT NULL,"
      "  device TEXT    NOT NULL,"
      "  event  TEXT    NOT NULL);"

      "CREATE TABLE IF NOT EXISTS room_sensors ("
      "  id        INTEGER PRIMARY KEY AUTOINCREMENT,"
      "  ts        INTEGER NOT NULL,"
      "  room_name TEXT    NOT NULL,"
      "  sensor_id INTEGER NOT NULL,"
      "  state     TEXT    NOT NULL CHECK(state IN ('open','closed')),"
      "  location  TEXT,"
      "  batt      INTEGER);"

      "CREATE INDEX IF NOT EXISTS idx_readings_ts"
      "  ON readings(ts);"
      "CREATE INDEX IF NOT EXISTS idx_uart_readings_dev"
      "  ON uart_readings(device,ts);"
      "CREATE INDEX IF NOT EXISTS idx_reed_readings_slot"
      "  ON reed_readings(slot,ts);"
      "CREATE INDEX IF NOT EXISTS idx_motor_readings_ts"
      "  ON motor_readings(ts);"
      "CREATE INDEX IF NOT EXISTS idx_device_events_dev"
      "  ON device_events(device,ts);"
      "CREATE INDEX IF NOT EXISTS idx_room_sensors_id"
      "  ON room_sensors(sensor_id,ts);";

   if (SQLITE_OK != sqlite3_exec(db, p_sql, NULL, NULL, &p_err))
   {
      LOG("db_open_and_init tables error: %s", p_err);
      sqlite3_free(p_err);
   }
   else
   {
      LOG("DB tables verified");
   }
}

/******************************************************************************
 * Unit of Work — transaction helpers
 ******************************************************************************/

/******************************************************************************
 * \brief Begin an explicit SQLite transaction.
 *
 * \return void
 *
 * \details Call before a batch of db_save_* calls to group them into one
 *          fsync. Must be paired with db_commit() or db_rollback().
 *          No-op if db is NULL.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void db_begin(void)
{
   if (NULL == db) { return; }
   (void)sqlite3_exec(db, "BEGIN;", NULL, NULL, NULL);
}

/******************************************************************************
 * \brief Commit the current SQLite transaction.
 *
 * \return void
 *
 * \details No-op if db is NULL.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void db_commit(void)
{
   if (NULL == db) { return; }
   (void)sqlite3_exec(db, "COMMIT;", NULL, NULL, NULL);
}

/******************************************************************************
 * \brief Roll back the current SQLite transaction.
 *
 * \return void
 *
 * \details No-op if db is NULL.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void db_rollback(void)
{
   if (NULL == db) { return; }
   (void)sqlite3_exec(db, "ROLLBACK;", NULL, NULL, NULL);
}

/******************************************************************************
 * Write functions — prepared statements, no string interpolation
 ******************************************************************************/

/******************************************************************************
 * \brief Save a UART sensor reading to the uart_readings table.
 *
 * \param p_dev_id - Null-terminated device identifier string (PIR/LGT/LCK).
 * \param val      - Sensor value to store.
 * \param batt     - Battery SOC percent, or -1 to store NULL.
 *
 * \return void
 *
 * \details Uses prepared statement — safe against SQL injection.
 *          No-op if db is NULL.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void db_save_uart(const char *p_dev_id, int val, int batt)
{
   sqlite3_stmt *p_stmt = NULL; /**< prepared statement handle */
   const char   *p_sql  =
      "INSERT INTO uart_readings (ts,device,value,batt) VALUES (?,?,?,?);";

   if (NULL == db) { return; }

   if (SQLITE_OK != sqlite3_prepare_v2(db, p_sql, -1, &p_stmt, NULL))
   {
      LOG("db_save_uart prepare failed: %s", sqlite3_errmsg(db));
      return;
   }

   sqlite3_bind_int64(p_stmt, 1, (sqlite3_int64)time(NULL));
   sqlite3_bind_text (p_stmt, 2, p_dev_id, -1, SQLITE_STATIC);
   sqlite3_bind_int  (p_stmt, 3, val);

   if (0 <= batt) { sqlite3_bind_int (p_stmt, 4, batt); }
   else           { sqlite3_bind_null(p_stmt, 4);        }

   if (SQLITE_DONE != sqlite3_step(p_stmt))
   {
      LOG("db_save_uart step failed: %s", sqlite3_errmsg(db));
   }
   else
   {
      LOG("db_save_uart dev=%s val=%d batt=%d", p_dev_id, val, batt);
   }

   sqlite3_finalize(p_stmt);
}

/******************************************************************************
 * \brief Core — insert one device event row (no logging, no timestamp).
 *
 * \param ts       - Unix timestamp for the event.
 * \param p_dev    - Null-terminated device name string.
 * \param p_event  - Null-terminated event description string.
 *
 * \return int - 0 on success, -1 on failure.
 *
 * \details Pure storage function — Decorator pattern inner function.
 *          Called only by db_save_event().
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static int db_save_event_core(long ts,
                               const char *p_dev,
                               const char *p_event)
{
   sqlite3_stmt *p_stmt = NULL; /**< prepared statement handle */
   const char   *p_sql  =
      "INSERT INTO device_events (ts,device,event) VALUES (?,?,?);";
   int           rc     = 0;    /**< return code */

   if (SQLITE_OK != sqlite3_prepare_v2(db, p_sql, -1, &p_stmt, NULL))
   {
      return -1;
   }

   sqlite3_bind_int64(p_stmt, 1, (sqlite3_int64)ts);
   sqlite3_bind_text (p_stmt, 2, p_dev,   -1, SQLITE_STATIC);
   sqlite3_bind_text (p_stmt, 3, p_event, -1, SQLITE_STATIC);

   rc = (SQLITE_DONE == sqlite3_step(p_stmt)) ? 0 : -1;
   sqlite3_finalize(p_stmt);
   return rc;
}

/******************************************************************************
 * \brief Save a device event — Decorator wrapper with timestamp and LOG.
 *
 * \param p_dev_name - Null-terminated device name string.
 * \param p_event    - Null-terminated event description string.
 *
 * \return void
 *
 * \details Stamps current time, calls db_save_event_core(), logs result.
 *          No-op if db is NULL.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void db_save_event(const char *p_dev_name, const char *p_event)
{
   long ts  = 0; /**< current unix timestamp */
   int  ret = 0; /**< core return code */

   if (NULL == db) { return; }

   ts  = (long)time(NULL);
   ret = db_save_event_core(ts, p_dev_name, p_event);

   LOG("db_save_event dev=%s event=%s ts=%ld rc=%d",
       p_dev_name, p_event, ts, ret);
}

/******************************************************************************
 * \brief Save a main sensor reading to the readings table.
 *
 * \param p_data - Pointer to sensor data struct with all fields.
 *
 * \return void
 *
 * \details Saves all sensor state in one row: temp, motion, light, lock,
 *          motor online, and all battery SOC values.
 *          Uses prepared statement — safe against SQL injection.
 *          No-op if db is NULL.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void db_save_reading(const struct SensorData *p_data)
{
   sqlite3_stmt *p_stmt = NULL; /**< prepared statement handle */
   const char   *p_sql  =
      "INSERT INTO readings "
      "(ts,source,avg_temp,motion_count,light_state,lock_state,"
      " motor_online,batt_pir,batt_lck,batt_motor) "
      "VALUES (?,?,?,?,?,?,?,?,?,?);";

   if (NULL == db) { return; }

   if (SQLITE_OK != sqlite3_prepare_v2(db, p_sql, -1, &p_stmt, NULL))
   {
      LOG("db_save_reading prepare failed: %s", sqlite3_errmsg(db));
      return;
   }

   sqlite3_bind_int64 (p_stmt, 1,  (sqlite3_int64)p_data->timestamp);
   sqlite3_bind_text  (p_stmt, 2,  "vroom", -1, SQLITE_STATIC);
   sqlite3_bind_double(p_stmt, 3,  p_data->avg_temp);
   sqlite3_bind_int   (p_stmt, 4,  p_data->motion_count);
   sqlite3_bind_int   (p_stmt, 5,  p_data->light_state);
   sqlite3_bind_int   (p_stmt, 6,  p_data->lock_state);
   sqlite3_bind_int   (p_stmt, 7,  p_data->motor_online);

   if (p_data->batt_pir   >= 0) { sqlite3_bind_int(p_stmt, 8,  p_data->batt_pir);   }
   else                         { sqlite3_bind_null(p_stmt, 8);                       }

   if (p_data->batt_lck   >= 0) { sqlite3_bind_int(p_stmt, 9,  p_data->batt_lck);   }
   else                         { sqlite3_bind_null(p_stmt, 9);                       }

   if (p_data->batt_motor >= 0) { sqlite3_bind_int(p_stmt, 10, p_data->batt_motor);  }
   else                         { sqlite3_bind_null(p_stmt, 10);                      }

   if (SQLITE_DONE != sqlite3_step(p_stmt))
   {
      LOG("db_save_reading step failed: %s", sqlite3_errmsg(db));
   }
   else
   {
      LOG("db_save_reading saved ts=%ld temp=%.1f motion=%d lgt=%d lck=%d mtr=%d",
          (long)p_data->timestamp,
          p_data->avg_temp,
          p_data->motion_count,
          p_data->light_state,
          p_data->lock_state,
          p_data->motor_online);
   }

   sqlite3_finalize(p_stmt);
}

/******************************************************************************
 * \brief Save one reed slot snapshot to the reed_readings table.
 *
 * \param slot  - 1-based reed slot index (DR1=1, DR2=2 ...).
 * \param p_name - Null-terminated device name string (may be NULL).
 * \param state - Reed state (0=closed, 1=open).
 * \param batt  - Battery SOC percent, or -1 to store NULL.
 * \param age   - Seconds since last update from device.
 *
 * \return void
 *
 * \details Uses prepared statement — safe against SQL injection.
 *          No-op if db is NULL.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void db_save_reed(int slot, const char *p_name, int state, int batt, int age)
{
   sqlite3_stmt *p_stmt = NULL; /**< prepared statement handle */
   const char   *p_sql  =
      "INSERT INTO reed_readings (ts,slot,name,state,batt,age) "
      "VALUES (?,?,?,?,?,?);";

   if (NULL == db) { return; }

   if (SQLITE_OK != sqlite3_prepare_v2(db, p_sql, -1, &p_stmt, NULL))
   {
      LOG("db_save_reed prepare failed: %s", sqlite3_errmsg(db));
      return;
   }

   sqlite3_bind_int64(p_stmt, 1, (sqlite3_int64)time(NULL));
   sqlite3_bind_int  (p_stmt, 2, slot);

   if (NULL != p_name) { sqlite3_bind_text(p_stmt, 3, p_name, -1, SQLITE_STATIC); }
   else                { sqlite3_bind_null(p_stmt, 3);                              }

   sqlite3_bind_int(p_stmt, 4, state);

   if (batt >= 0) { sqlite3_bind_int (p_stmt, 5, batt); }
   else           { sqlite3_bind_null(p_stmt, 5);        }

   sqlite3_bind_int(p_stmt, 6, age);

   if (SQLITE_DONE != sqlite3_step(p_stmt))
   {
      LOG("db_save_reed step failed: %s", sqlite3_errmsg(db));
   }
   else
   {
      LOG("db_save_reed slot=%d name=%s state=%d batt=%d age=%d",
          slot, p_name ? p_name : "?", state, batt, age);
   }

   sqlite3_finalize(p_stmt);
}

/******************************************************************************
 * \brief Save a motor controller snapshot to the motor_readings table.
 *
 * \param online - Motor online flag (0=offline, 1=online).
 * \param batt   - Battery SOC percent, or -1 to store NULL.
 *
 * \return void
 *
 * \details Uses prepared statement — safe against SQL injection.
 *          No-op if db is NULL.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void db_save_motor(int online, int batt)
{
   sqlite3_stmt *p_stmt = NULL; /**< prepared statement handle */
   const char   *p_sql  =
      "INSERT INTO motor_readings (ts,online,batt) VALUES (?,?,?);";

   if (NULL == db) { return; }

   if (SQLITE_OK != sqlite3_prepare_v2(db, p_sql, -1, &p_stmt, NULL))
   {
      LOG("db_save_motor prepare failed: %s", sqlite3_errmsg(db));
      return;
   }

   sqlite3_bind_int64(p_stmt, 1, (sqlite3_int64)time(NULL));
   sqlite3_bind_int  (p_stmt, 2, online);

   if (batt >= 0) { sqlite3_bind_int (p_stmt, 3, batt); }
   else           { sqlite3_bind_null(p_stmt, 3);        }

   if (SQLITE_DONE != sqlite3_step(p_stmt))
   {
      LOG("db_save_motor step failed: %s", sqlite3_errmsg(db));
   }
   else
   {
      LOG("db_save_motor online=%d batt=%d", online, batt);
   }

   sqlite3_finalize(p_stmt);
}

/******************************************************************************
 * \brief Query latest room sensor states from database — Repository pattern.
 *
 * \param p_out     - Output array of RoomStatus structs to fill.
 * \param max_rooms - Maximum number of rooms to return.
 *
 * \return int - Number of rooms filled, or -1 on error.
 *
 * \details Repository pattern: returns domain structs, never touches shm.
 *          Caller owns all shared memory writes.
 *          No-op returning -1 if db is NULL.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
int db_query_rooms(struct RoomStatus *p_out, int max_rooms)
{
   const char   *p_sql  = NULL;  /**< SQL query string */
   sqlite3_stmt *p_stmt = NULL;  /**< prepared statement handle */
   int           i      = 0;    /**< room index */

   if (NULL == db)
   {
      LOG("db_query_rooms failed (no DB)");
      return -1;
   }

   p_sql =
      "SELECT room_name, sensor_id, state, location, ts "
      "FROM room_sensors "
      "WHERE id IN (SELECT MAX(id) FROM room_sensors GROUP BY sensor_id) "
      "ORDER BY sensor_id LIMIT ?";

   if (SQLITE_OK != sqlite3_prepare_v2(db, p_sql, -1, &p_stmt, NULL))
   {
      LOG("db_query_rooms prepare failed: %s", sqlite3_errmsg(db));
      return -1;
   }

   sqlite3_bind_int(p_stmt, 1, max_rooms);

   while ((SQLITE_ROW == sqlite3_step(p_stmt)) && (i < max_rooms))
   {
      (void)strncpy(p_out[i].room_name,
                    (const char *)sqlite3_column_text(p_stmt, 0), 31);
      p_out[i].sensor_id = sqlite3_column_int   (p_stmt, 1);
      (void)strncpy(p_out[i].state,
                    (const char *)sqlite3_column_text(p_stmt, 2), 15);
      (void)strncpy(p_out[i].location,
                    (const char *)sqlite3_column_text(p_stmt, 3), 31);
      p_out[i].timestamp = sqlite3_column_int64 (p_stmt, 4);

      LOG("Room %d: %s (%s) = %s",
          p_out[i].sensor_id,
          p_out[i].room_name,
          p_out[i].location,
          p_out[i].state);

      i++;
   }

   sqlite3_finalize(p_stmt);
   LOG("db_query_rooms complete (%d rooms)", i);
   return i;
}
