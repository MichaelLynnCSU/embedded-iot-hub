/******************************************************************************
 * \file cmd_handler.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Command handler and sensor data receiver for BeagleBone controller.
 *
 * \details Owns the latest sensor data snapshot and processes commands
 *          from the LCD display via shared memory. Two threads run
 *          concurrently:
 *
 *          receive_data_thread — reads SensorData structs from sensor_pipe,
 *          updates latest_data, detects reed slot generation changes and
 *          online/offline transitions, writes to shared memory and DB.
 *
 *          command_handler_thread — reads CommandMsg structs from
 *          command_pipe and dispatches to process_command().
 *
 *          Thread safety:
 *          - data_mutex protects latest_data reads and writes
 *          - shm_data->shm_mutex (PTHREAD_PROCESS_SHARED) protects all
 *            shared memory access, replacing the previous shm_sem named
 *            semaphore
 *
 *          DB writes in process_sensor_frame() are batched in one
 *          Unit of Work transaction (db_begin/db_commit) to reduce
 *          SD card fsyncs on BeagleBone. All writes — readings, motor,
 *          and all active reed slots — share one BEGIN/COMMIT/fsync.
 *
 *          handle_get_room_status() owns the shm write — db_query_rooms()
 *          returns domain structs only (Repository pattern).
 *
 * \note    Semaphore → mutex (2026-05-22):
 *          All sem_wait(shm_sem) / sem_post(shm_sem) calls replaced with
 *          pthread_mutex_lock(&shm_data->shm_mutex) /
 *          pthread_mutex_unlock(&shm_data->shm_mutex).
 *          Four call sites updated: handle_get_latest,
 *          handle_get_device_status, handle_get_room_status,
 *          update_shm_rooms. No logic changes — lock granularity and
 *          critical section boundaries are identical.
 ******************************************************************************/

#include <sys/stat.h>
#include "../controller_internal.h"

#define REED_DEV_BUF_SIZE   16  /**< reed device name buffer size */
#define REED_EVENT_BUF_SIZE 64  /**< reed event string buffer size */
#define PIPE_REOPEN_DELAY_S  1  /**< seconds to wait before reopening pipe */

/** \brief Latest sensor data snapshot — owned here, read by uart_push_thread */
struct LatestData latest_data =
{
   .age_pir  = 0xFFFF,
   .age_lgt  = 0xFFFF,
   .age_lck  = 0xFFFF,
   .batt_pir = -1,
   .batt_lck = -1,
};

pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER; /**< latest_data mutex */

/******************************************************************************
 * \brief Push latest_data snapshot into shared memory.
 *
 * \param p_cmd - Pointer to command message (unused, for handler signature).
 *
 * \return void
 *
 * \details Acquires shm_data->shm_mutex and data_mutex, copies latest_data
 *          fields into shm_data, increments sequence counter, and sets
 *          command result.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void handle_get_latest(struct CommandMsg *p_cmd)
{
   (void)p_cmd;

   pthread_mutex_lock(&shm_data->shm_mutex);

   pthread_mutex_lock(&data_mutex);
   shm_data->current_temp      = latest_data.avg_temp;
   shm_data->current_motion    = latest_data.motion_count;
   shm_data->current_light     = latest_data.light_state;
   shm_data->current_lock      = latest_data.lock_state;
   shm_data->batt_motor        = latest_data.batt_motor;
   shm_data->current_timestamp = latest_data.timestamp;
   shm_data->data_valid        = latest_data.valid;
   shm_data->sequence++;
   pthread_mutex_unlock(&data_mutex);

   shm_data->last_command   = CMD_GET_LATEST;
   shm_data->command_result = 0;
   pthread_mutex_unlock(&shm_data->shm_mutex);

   LOG("CMD_GET_LATEST: %.1fC %d motions",
       shm_data->current_temp,
       shm_data->current_motion);
}

/******************************************************************************
 * \brief Push device online status snapshot into shared memory.
 *
 * \param p_cmd - Pointer to command message (unused, for handler signature).
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void handle_get_device_status(struct CommandMsg *p_cmd)
{
   (void)p_cmd;

   pthread_mutex_lock(&shm_data->shm_mutex);
   heartbeat_snapshot_online(shm_data->device_online, DEV_COUNT);
   shm_data->last_command   = CMD_GET_DEVICE_STATUS;
   shm_data->command_result = 0;
   shm_data->sequence++;
   pthread_mutex_unlock(&shm_data->shm_mutex);

   LOG("CMD_GET_DEVICE_STATUS served");
}

/******************************************************************************
 * \brief Query room sensor status from database into shared memory.
 *
 * \param p_cmd - Pointer to command message (unused, for handler signature).
 *
 * \return void
 *
 * \details Repository pattern: db_query_rooms() fills a local RoomStatus
 *          array — it never touches shm. This function owns the shm write.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void handle_get_room_status(struct CommandMsg *p_cmd)
{
   struct RoomStatus rooms[ROOM_BUF_SIZE]; /**< local room result buffer */
   int               count = 0;           /**< rooms returned from DB */
   int               i     = 0;           /**< loop index */

   (void)p_cmd;

   (void)memset(rooms, 0, sizeof(rooms));

   count = db_query_rooms(rooms, ROOM_BUF_SIZE);

   pthread_mutex_lock(&shm_data->shm_mutex);

   if (0 > count)
   {
      shm_data->command_result = -1;
   }
   else
   {
      for (i = 0; i < count; i++)
      {
         shm_data->rooms[i].sensor_id = rooms[i].sensor_id;
         shm_data->rooms[i].timestamp = rooms[i].timestamp;
         (void)strncpy(shm_data->rooms[i].room_name,
                       rooms[i].room_name,
                       ROOM_NAME_SZ - 1);
         (void)strncpy(shm_data->rooms[i].state,
                       rooms[i].state,
                       ROOM_STATE_SZ - 1);
         (void)strncpy(shm_data->rooms[i].location,
                       rooms[i].location,
                       ROOM_LOC_SZ - 1);
      }

      shm_data->room_count     = count;
      shm_data->last_command   = CMD_GET_ROOM_STATUS;
      shm_data->command_result = 0;
      shm_data->sequence++;
   }

   pthread_mutex_unlock(&shm_data->shm_mutex);

   LOG("CMD_GET_ROOM_STATUS complete (%d rooms)", count);
}

/******************************************************************************
 * \brief Check for reed slot generation change and log if detected.
 ******************************************************************************/
static void check_reed_generation(int slot, uint16_t old_gen, uint16_t new_gen)
{
   char ev[REED_EVENT_BUF_SIZE] = {0};
   char dev[REED_DEV_BUF_SIZE]  = {0};

   if ((old_gen > 0) && (new_gen != old_gen))
   {
      (void)snprintf(ev,  sizeof(ev),
                     "gen %u->%u (device replaced)",
                     old_gen, new_gen);
      (void)snprintf(dev, sizeof(dev), "REED%d", slot + 1);
      LOG("[EVENT] %s %s", dev, ev);
      db_save_event(dev, ev);
   }
}

/******************************************************************************
 * \brief Check for reed slot online/offline transition and log if detected.
 ******************************************************************************/
static void check_reed_online_state(int slot,
                                     uint8_t was_offline,
                                     uint8_t now_offline)
{
   char dev[REED_DEV_BUF_SIZE] = {0};

   (void)snprintf(dev, sizeof(dev), "REED%d", slot + 1);

   if (!was_offline && now_offline)
   {
      LOG("[EVENT] %s went offline", dev);
      db_save_event(dev, "offline");
   }
   else if (was_offline && !now_offline)
   {
      LOG("[EVENT] %s back online", dev);
      db_save_event(dev, "online");
   }
   else
   {
      /* no transition — no action */
   }
}

/******************************************************************************
 * \brief Update latest_data reed slots and detect state transitions.
 *
 * \details Must be called with data_mutex held.
 ******************************************************************************/
static void process_reed_slots(const struct SensorData *p_data)
{
   int      i           = 0;
   uint16_t old_gen     = 0;
   uint16_t new_gen     = 0;
   uint8_t  was_offline = 0;
   uint8_t  now_offline = 0;

   for (i = 0; i < MAX_REEDS; i++)
   {
      old_gen = latest_data.reed_slots[i].gen;
      new_gen = p_data->reed_slots[i].gen;

      if (p_data->reed_slots[i].active)
      {
         check_reed_generation(i, old_gen, new_gen);

         was_offline = latest_data.reed_slots[i].offline;
         now_offline = p_data->reed_slots[i].offline;
         check_reed_online_state(i, was_offline, now_offline);
      }

      if (p_data->reed_slots[i].active)
      {
         latest_data.reed_slots[i] = p_data->reed_slots[i];
      }
   }

   for (i = 0; i < MAX_REEDS; i++)
   {
      if (p_data->reed_slots[i].active)
      {
         LOG("Reed %d (%s): state=%d batt=%d%% age=%d",
             i + 1,
             p_data->reed_slots[i].name,
             p_data->reed_slots[i].state,
             p_data->reed_slots[i].batt,
             p_data->reed_slots[i].age);
      }
   }
}

/******************************************************************************
 * \brief Copy room data from sensor frame into shared memory.
 ******************************************************************************/
static void update_shm_rooms(const struct SensorData *p_data)
{
   int i = 0;

   pthread_mutex_lock(&shm_data->shm_mutex);
   shm_data->room_count = p_data->room_count;

   for (i = 0; (i < p_data->room_count) && (i < MAX_ROOMS); i++)
   {
      shm_data->rooms[i].sensor_id = p_data->rooms[i].sensor_id;
      (void)strncpy(shm_data->rooms[i].room_name,
                    p_data->rooms[i].room_name,
                    ROOM_NAME_SZ - 1);
      (void)strncpy(shm_data->rooms[i].state,
                    p_data->rooms[i].state,
                    ROOM_STATE_SZ - 1);
      (void)strncpy(shm_data->rooms[i].location,
                    p_data->rooms[i].location,
                    ROOM_LOC_SZ - 1);
   }

   pthread_mutex_unlock(&shm_data->shm_mutex);
}

/******************************************************************************
 * \brief Receive and process one complete sensor data frame.
 ******************************************************************************/
static void process_sensor_frame(const struct SensorData *p_data)
{
   struct CommandMsg auto_cmd = {.cmd = CMD_GET_LATEST};
   int               i        = 0;

   pthread_mutex_lock(&data_mutex);

   latest_data.avg_temp     = p_data->avg_temp;
   latest_data.motion_count = p_data->motion_count;
   latest_data.light_state  = p_data->light_state;
   latest_data.lock_state   = p_data->lock_state;
   latest_data.timestamp    = p_data->timestamp;
   latest_data.valid        = 1;
   latest_data.age_pir      = p_data->age_pir;
   latest_data.age_lgt      = p_data->age_lgt;
   latest_data.age_lck      = p_data->age_lck;
   latest_data.batt_pir     = p_data->batt_pir;
   latest_data.pir_occupied = p_data->pir_occupied;
   latest_data.batt_lck     = p_data->batt_lck;
   latest_data.batt_motor   = p_data->batt_motor;
   latest_data.motor_online = p_data->motor_online;
   latest_data.doorbell_pressed   = p_data->doorbell_pressed;
   latest_data.doorbell_device_id = p_data->doorbell_device_id;

   process_reed_slots(p_data);
   for (i = 0; i < MAX_PIRS; i++)
   {
      latest_data.pir_slots[i] = p_data->pir_slots[i];
   }
   latest_data.pir_count = p_data->pir_count;

   for (i = 0; i < MAX_TEMPS; i++)
   {
      latest_data.temp_slots[i] = p_data->temp_slots[i];
   }
   latest_data.temp_count = p_data->temp_count;

   pthread_mutex_unlock(&data_mutex);

   LOG("Sensor: temp=%.1f motion=%d occ=%d lgt=%d lck=%d mtr=%d "
       "ages pir=%d lgt=%d lck=%d "
       "batt pir=%d%% lck=%d%% motor=%d%% pirs=%d",
       p_data->avg_temp,
       p_data->motion_count,
       p_data->pir_occupied,
       p_data->light_state,
       p_data->lock_state,
       p_data->motor_online,
       p_data->age_pir,
       p_data->age_lgt,
       p_data->age_lck,
       p_data->batt_pir,
       p_data->batt_lck,
       p_data->batt_motor,
       p_data->pir_count);

   if (p_data->motor_online)
   {
      heartbeat_stamp(DEV_MOTOR);
      LOG("Motor online — heartbeat stamped");
   }

   handle_get_latest(&auto_cmd);

   if (p_data->doorbell_pressed)
   {
      pthread_mutex_lock(&shm_data->shm_mutex);
      shm_data->doorbell_pressed   = 1;
      shm_data->doorbell_device_id = p_data->doorbell_device_id;
      shm_data->doorbell_timestamp = p_data->timestamp;
      pthread_mutex_unlock(&shm_data->shm_mutex);
      LOG("Doorbell press device_id=%d — shm updated", p_data->doorbell_device_id);
   }

   update_shm_rooms(p_data);

   db_begin();
   db_save_reading(p_data);
   db_save_motor(p_data->motor_online, p_data->batt_motor);
   for (i = 0; i < MAX_REEDS; i++)
   {
      if (p_data->reed_slots[i].active)
      {
         db_save_reed(i + 1,
                      p_data->reed_slots[i].name,
                      p_data->reed_slots[i].state,
                      p_data->reed_slots[i].batt,
                      p_data->reed_slots[i].age);
      }
   }
   db_commit();
}

/******************************************************************************
 * \brief Thread — reads SensorData structs from sensor_pipe.
 ******************************************************************************/
void *receive_data_thread(void *p_arg)
{
   int               pipe_fd = -1;
   ssize_t           bytes   = 0;
   struct SensorData data;

   (void)p_arg;

   pipe_fd = open(SENSOR_PIPE, O_RDONLY);
   if (0 > pipe_fd)
   {
      LOG("Failed to open sensor pipe");
      return NULL;
   }

   LOG("Listening for sensor data");

   while (running)
   {
      bytes = read(pipe_fd, &data, sizeof(data));

      if (bytes == (ssize_t)sizeof(data))
      {
         process_sensor_frame(&data);
      }
      else if (0 == bytes)
      {
         LOG("Sensor pipe closed, reopening");
         close(pipe_fd);
         sleep(PIPE_REOPEN_DELAY_S);
         pipe_fd = open(SENSOR_PIPE, O_RDONLY);
      }
      else
      {
         /* partial read — ignore */
      }
   }

   close(pipe_fd);
   return NULL;
}

/******************************************************************************
 * \brief Thread — reads CommandMsg structs from command_pipe.
 ******************************************************************************/
void *command_handler_thread(void *p_arg)
{
   int               pipe_fd = -1;
   ssize_t           bytes   = 0;
   struct CommandMsg cmd;

   (void)p_arg;

   (void)unlink(COMMAND_PIPE);
   (void)mkfifo(COMMAND_PIPE, 0666);

   pipe_fd = open(COMMAND_PIPE, O_RDONLY);
   if (0 > pipe_fd)
   {
      LOG("Failed to open command pipe");
      return NULL;
   }

   LOG("Listening for commands");

   while (running)
   {
      bytes = read(pipe_fd, &cmd, sizeof(cmd));

      if (bytes == (ssize_t)sizeof(cmd))
      {
         LOG("Command %d from client %d", cmd.cmd, cmd.client_id);
         process_command(&cmd);
      }
      else if (0 == bytes)
      {
         LOG("Command pipe closed, reopening");
         close(pipe_fd);
         sleep(PIPE_REOPEN_DELAY_S);
         pipe_fd = open(COMMAND_PIPE, O_RDONLY);
      }
      else
      {
         /* partial read — ignore */
      }
   }

   close(pipe_fd);
   return NULL;
}
