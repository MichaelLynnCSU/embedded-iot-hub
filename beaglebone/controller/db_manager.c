/******************************************************************************
 * \file db_manager.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief SQLite database manager for BeagleBone data controller.
 *
 * \details Manages all database operations for the data controller.
 *          Creates and maintains three tables:
 *          - readings:      main sensor readings (temp, motion)
 *          - uart_readings: UART device readings with optional battery
 *          - device_events: device online/offline event log
 *
 *          All functions are no-ops if db is NULL (open failed at init).
 ******************************************************************************/

#include "controller_internal.h"

#define SQL_BUF_SIZE_SM   200  /**< small SQL statement buffer size */
#define SQL_BUF_SIZE_LG   256  /**< large SQL statement buffer size */
#define ROOM_QUERY_LIMIT  10   /**< maximum rooms returned by db_query_rooms */

/******************************************************************************
 * \brief Open SQLite database and create tables if they do not exist.
 *
 * \return void
 *
 * \details Called from main() after log_fp is open. Sets db to NULL and
 *          logs a warning if the database cannot be opened. Creates
 *          readings, uart_readings, and device_events tables.
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

   p_sql =
      "CREATE TABLE IF NOT EXISTS readings ("
      "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
      "  ts INTEGER, source TEXT, avg_temp REAL, motion_count INTEGER);"

      "CREATE TABLE IF NOT EXISTS uart_readings ("
      "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
      "  ts INTEGER NOT NULL,"
      "  device TEXT NOT NULL,"
      "  value  INTEGER NOT NULL,"
      "  batt   INTEGER);"

      "CREATE TABLE IF NOT EXISTS device_events ("
      "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
      "  ts INTEGER NOT NULL,"
      "  device TEXT NOT NULL,"
      "  event  TEXT NOT NULL);";

   if (SQLITE_OK != sqlite3_exec(db, p_sql, NULL, NULL, &p_err))
   {
      LOG("db_ensure_tables error: %s", p_err);
      sqlite3_free(p_err);
   }
   else
   {
      LOG("DB tables verified");
   }
}

/******************************************************************************
 * \brief Save a UART sensor reading to the uart_readings table.
 *
 * \param p_dev_id - Null-terminated device identifier string.
 * \param val      - Sensor value to store.
 * \param batt     - Battery SOC percent, or -1 to store NULL.
 *
 * \return void
 *
 * \details No-op if db is NULL. Stores battery as NULL in database
 *          when batt is negative.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void db_save_uart(const char *p_dev_id, int val, int batt)
{
   char  sql[SQL_BUF_SIZE_LG] = {0}; /**< SQL statement buffer */
   char *p_err                = NULL; /**< SQLite error message */

   if (NULL == db)
   {
      return;
   }

   if (0 <= batt)
   {
      (void)snprintf(sql, sizeof(sql),
                     "INSERT INTO uart_readings (ts,device,value,batt) "
                     "VALUES (%ld,'%s',%d,%d);",
                     (long)time(NULL), p_dev_id, val, batt);
   }
   else
   {
      (void)snprintf(sql, sizeof(sql),
                     "INSERT INTO uart_readings (ts,device,value,batt) "
                     "VALUES (%ld,'%s',%d,NULL);",
                     (long)time(NULL), p_dev_id, val);
   }

   if (SQLITE_OK != sqlite3_exec(db, sql, NULL, NULL, &p_err))
   {
      LOG("db_save_uart error: %s", p_err);
      sqlite3_free(p_err);
   }
}

/******************************************************************************
 * \brief Save a device online/offline event to the device_events table.
 *
 * \param p_dev_name - Null-terminated device name string.
 * \param p_event    - Null-terminated event description string.
 *
 * \return void
 *
 * \details No-op if db is NULL.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void db_save_event(const char *p_dev_name, const char *p_event)
{
   char  sql[SQL_BUF_SIZE_SM] = {0}; /**< SQL statement buffer */
   char *p_err                = NULL; /**< SQLite error message */

   if (NULL == db)
   {
      return;
   }

   (void)snprintf(sql, sizeof(sql),
                  "INSERT INTO device_events (ts,device,event) "
                  "VALUES (%ld,'%s','%s');",
                  (long)time(NULL), p_dev_name, p_event);

   if (SQLITE_OK != sqlite3_exec(db, sql, NULL, NULL, &p_err))
   {
      LOG("db_save_event error: %s", p_err);
      sqlite3_free(p_err);
   }
}

/******************************************************************************
 * \brief Save a main sensor reading to the readings table.
 *
 * \param ts     - Unix timestamp of the reading.
 * \param temp   - Average temperature in Celsius.
 * \param motion - Motion event count.
 *
 * \return void
 *
 * \details No-op if db is NULL.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void db_save_reading(long ts, double temp, int motion)
{
   char  sql[SQL_BUF_SIZE_LG] = {0}; /**< SQL statement buffer */
   char *p_err                = NULL; /**< SQLite error message */

   if (NULL == db)
   {
      return;
   }

   (void)snprintf(sql, sizeof(sql),
                  "INSERT INTO readings (ts,source,avg_temp,motion_count) "
                  "VALUES (%ld,'vroom',%.2f,%d);",
                  ts, temp, motion);

   if (SQLITE_OK != sqlite3_exec(db, sql, NULL, NULL, &p_err))
   {
      LOG("db_save_reading error: %s", p_err);
      sqlite3_free(p_err);
   }
   else
   {
      LOG("Main reading saved");
   }
}

/******************************************************************************
 * \brief Query latest room sensor states from database into shared memory.
 *
 * \return void
 *
 * \details Queries the latest entry per sensor_id from room_sensors table
 *          and copies results into shm_data->rooms. Sets command_result
 *          to -1 if db is NULL. Acquires shm_sem during shm write.
 *          Called by handle_get_room_status() in cmd_handler.c.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void db_query_rooms(void)
{
   const char    *p_sql = NULL;  /**< SQL query string */
   sqlite3_stmt  *p_stmt = NULL; /**< prepared statement handle */
   int            i     = 0;    /**< room index */

   if (NULL == db)
   {
      sem_wait(shm_sem);
      shm_data->command_result = -1;
      sem_post(shm_sem);
      LOG("CMD_GET_ROOM_STATUS failed (no DB)");
      return;
   }

   p_sql =
      "SELECT room_name, sensor_id, state, location, ts "
      "FROM room_sensors "
      "WHERE id IN (SELECT MAX(id) FROM room_sensors GROUP BY sensor_id) "
      "ORDER BY sensor_id LIMIT 10";

   if (SQLITE_OK != sqlite3_prepare_v2(db, p_sql, -1, &p_stmt, NULL))
   {
      LOG("db_query_rooms prepare failed");
      return;
   }

   sem_wait(shm_sem);

   while ((SQLITE_ROW == sqlite3_step(p_stmt)) &&
          (i < ROOM_QUERY_LIMIT))
   {
      (void)strncpy(shm_data->rooms[i].room_name,
                    (const char *)sqlite3_column_text(p_stmt, 0), 31);
      shm_data->rooms[i].sensor_id = sqlite3_column_int(p_stmt, 1);
      (void)strncpy(shm_data->rooms[i].state,
                    (const char *)sqlite3_column_text(p_stmt, 2), 15);
      (void)strncpy(shm_data->rooms[i].location,
                    (const char *)sqlite3_column_text(p_stmt, 3), 31);
      shm_data->rooms[i].timestamp = sqlite3_column_int64(p_stmt, 4);

      LOG("Room %d: %s (%s) = %s",
          shm_data->rooms[i].sensor_id,
          shm_data->rooms[i].room_name,
          shm_data->rooms[i].location,
          shm_data->rooms[i].state);

      i++;
   }

   shm_data->room_count     = i;
   shm_data->last_command   = CMD_GET_ROOM_STATUS;
   shm_data->command_result = 0;
   sem_post(shm_sem);

   sqlite3_finalize(p_stmt);
   LOG("CMD_GET_ROOM_STATUS complete (%d rooms)", i);
}
