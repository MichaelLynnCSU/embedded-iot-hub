/******************************************************************************
 * \file controller_internal.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Internal shared header for BeagleBone data controller subsystem.
 *
 * \details Shared by all controller translation units. Defines paths,
 *          sizing constants, wire format structs, logging macros, and
 *          all internal function prototypes.
 *
 * \warning SensorData struct layout must match sensor_server.c exactly —
 *          both sides of the named pipe use this as the wire format.
 *          ReedSlotData, PirSlotData, and TempSlotData are packed for
 *          stable pipe wire format.
 *
 * \note    Logging fixes (2026-04-29):
 *          localtime() replaced with localtime_r() — not thread-safe.
 *          log_mutex added — prevents timestamp/message interleave across
 *          concurrent threads. Log levels added: ERR/WRN/INF/DBG. LOG is
 *          an alias for LOG_INF so all existing call sites compile without
 *          change. Thread ID included in each line for post-mortem
 *          debugging. Log rotation: file is rolled to .old at
 *          LOG_MAX_BYTES (5 MB).
 *
 * \note    Lock sync fix (2026-04-29):
 *          uart_sync_lock_state() added — called on every TCP frame so
 *          g_lock_state in uart_controller.c tracks the authoritative
 *          lock state regardless of whether the last command came via
 *          BLE or STM32.
 *
 * \note    PIR slot array (2026-05-XX):
 *          PirSlotData struct and pir_slots[MAX_PIRS] added to SensorData
 *          and LatestData to support dynamic multi-PIR slot tracking,
 *          mirroring the ReedSlotData pattern. Must match sensor_types.h
 *          in the server exactly.
 *
 * \note    Semaphore → mutex + ring buffer (2026-05-22):
 *          shm_sem (named POSIX semaphore, binary) removed. All shared
 *          memory protection now uses shm_data->shm_mutex, a
 *          PTHREAD_PROCESS_SHARED mutex embedded in SharedSensorData.
 *          SEM_NAME constant removed — no named semaphore object exists.
 *          g_uart_frame_sem (counting semaphore) added. Producer:
 *          uart_parse_line() posts once per parsed UART frame after
 *          pushing onto g_uart_ring. Consumer: uart_push_thread() calls
 *          sem_timedwait() — blocks with zero CPU until a frame arrives
 *          or UART_PUSH_INTERVAL_SEC elapses. Semaphore value == frames
 *          in ring buffer at all times. UartFrame / uart_ring_t types
 *          defined here so all translation units share one definition.
 *
 * \note    cmd/ subsystem split (2026-06-09):
 *          cmd_handler.c refactored into four translation units:
 *
 *          state_registry.c — single writer authority for all sensor
 *          state. Owns central_ledger (LatestData) and state_mutex.
 *          No other translation unit holds or modifies these directly.
 *          All writes go through update_snapshot(), all reads through
 *          get_snapshot(). Replaces the former latest_data / data_mutex
 *          globals that were previously extern in this header.
 *
 *          uart_staging.c — UART protocol transition logic only. Owns
 *          reed slot generation detection and online/offline transition
 *          logging. Provides uart_stage_lock(), uart_stage_pir(), and
 *          uart_stage_light() as partial-update wrappers that delegate
 *          to state_registry internally. Does not own any state.
 *
 *          shm_updater.c — shared memory projection layer. Consumes
 *          frozen LatestData snapshots from get_snapshot() and writes
 *          into SharedSensorData under shm_mutex. Never touches
 *          central_ledger or state_mutex directly.
 *
 *          db_persist.c — Unit of Work persistence facade. Takes the
 *          raw SensorData wire frame and batches db_begin /
 *          db_save_reading / db_save_motor / db_save_reed / db_commit
 *          into one fsync. No shared memory access, no snapshot
 *          dependency — persistence of ingress data only.
 *
 *          cmd_handler.c — orchestration only. sensor_frame_dispatch()
 *          calls update_snapshot(), get_snapshot(), then fans out to
 *          shm_update_frame(), db_persist_frame(), uart_update_frame().
 *
 *          Net result: latest_data and data_mutex are no longer extern
 *          globals. They do not appear in this header.
 *
 * \note    uart_update_frame (2026-06-09):
 *          Synchronous outbound STM32 push added to uart_controller.c.
 *          Called from sensor_frame_dispatch() after get_snapshot() on
 *          every TCP ingress frame. Builds the full STM32 protocol
 *          payload from the frozen snapshot. uart_push_thread() remains
 *          for UART-ingress frames (PIR/LGT/LCK) which do not go
 *          through sensor_frame_dispatch(). uart_push_msg() is the only
 *          send primitive — uart_write_string and uart_printf do not
 *          exist in this codebase.
 ******************************************************************************/

#ifndef INCLUDE_CONTROLLER_INTERNAL_H_
#define INCLUDE_CONTROLLER_INTERNAL_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdint.h>
#include <sqlite3.h>
#include "cmd/commands.h"
#include "shared_data.h"

/******************************** CONSTANTS ***********************************/

/** \brief IPC and filesystem paths */
#define SHM_NAME           "/sensor_shm"                       /**< shared memory name        */
#define SENSOR_PIPE        "/tmp/sensor_pipe"                  /**< sensor data named pipe    */
#define COMMAND_PIPE       "/tmp/controller_cmd"               /**< command named pipe        */
#ifndef DB_PATH
#define DB_PATH            "/home/debian/db/sensors.db"        /**< SQLite database path      */
#endif
#define CONTROLLER_LOG     "/var/log/data_controller.log"      /**< log file path             */
#define CONTROLLER_LOG_OLD "/var/log/data_controller.log.old"  /**< rotated log               */
#define UART_DEV           "/dev/ttyS1"                        /**< UART device for STM32     */

/** \brief Sizing constants */
#define MAX_ROOMS       10  /**< maximum room sensors in SensorData                           */
#define MAX_REEDS        6  /**< must match ESP32 tcp_manager.c and ble_scan.c               */
#define MAX_PIRS         5  /**< must match ESP32 tcp_manager.c and sensor_types.h           */
#define MAX_TEMPS        4  /**< must match ESP32 and sensor_types.h                         */
#define TEMP_NAME_LEN   32  /**< temp sensor BLE name buffer size                            */
#define UART_LINE_LEN   64  /**< UART line buffer size bytes                                 */

/** \brief Ring buffer capacity — must be a power of two for mask wrapping */
#define UART_RING_SIZE  16  /**< max queued UART frames before oldest is dropped             */

/** \brief Room and reed field sizes */
#define ROOM_NAME_SIZE  32  /**< room name string buffer size                                */
#define ROOM_STATE_SIZE 16  /**< room state string buffer size                               */
#define ROOM_LOC_SIZE   32  /**< room location string buffer size                            */
#define REED_NAME_SIZE  16  /**< reed BLE name string buffer size                            */

/** \brief Log rotation threshold */
#define LOG_MAX_BYTES   (5 * 1024 * 1024)  /**< rotate log at 5 MB                          */

/********************************** LOGGING ***********************************/

extern FILE           *log_fp;    /**< log file handle — opened by data_controller.c        */
extern pthread_mutex_t log_mutex; /**< serialises log writes across all threads             */

/**
 * \brief Rotate log file if it has exceeded LOG_MAX_BYTES.
 *
 * \details Renames current log to .old, opens a fresh log file.
 *          Must be called with log_mutex held.
 *          No-op if log_fp is NULL or file is below the threshold.
 */
static inline void log_rotate_if_needed(void)
{
   long pos = 0;

   if (NULL == log_fp) { return; }

   pos = ftell(log_fp);
   if (pos < LOG_MAX_BYTES) { return; }

   fclose(log_fp);
   (void)rename(CONTROLLER_LOG, CONTROLLER_LOG_OLD);
   log_fp = fopen(CONTROLLER_LOG, "a");
}

/**
 * \brief Core timestamped log macro — do not call directly, use LOG_* variants.
 *
 * \details Format: [YYYY-MM-DD HH:MM:SS] [LEVEL] [tid=XXXXXXXX] <message>
 *          localtime_r() used — thread-safe, no shared static buffer.
 *          log_mutex held for the full timestamp+message write.
 *          Log rotated if file exceeds LOG_MAX_BYTES.
 *          No-op if log_fp is NULL.
 */
#define _LOG_CORE(level, fmt, ...) \
do \
{ \
   pthread_mutex_lock(&log_mutex); \
   if (log_fp) \
   { \
      time_t      _t   = time(NULL); \
      struct tm   _tm; \
      (void)localtime_r(&_t, &_tm); \
      log_rotate_if_needed(); \
      fprintf(log_fp, \
              "[%04d-%02d-%02d %02d:%02d:%02d] [%-3s] [tid=%08lx] " fmt "\n", \
              _tm.tm_year + 1900, _tm.tm_mon + 1, _tm.tm_mday, \
              _tm.tm_hour, _tm.tm_min, _tm.tm_sec, \
              (level), \
              (unsigned long)pthread_self(), \
              ##__VA_ARGS__); \
      fflush(log_fp); \
   } \
   pthread_mutex_unlock(&log_mutex); \
} while (0)

/** \brief Error — unexpected failures, data loss, hardware faults.        */
#define LOG_ERR(fmt, ...)  _LOG_CORE("ERR", fmt, ##__VA_ARGS__)

/** \brief Warning — degraded operation, retryable errors, unexpected state. */
#define LOG_WRN(fmt, ...)  _LOG_CORE("WRN", fmt, ##__VA_ARGS__)

/** \brief Info — normal lifecycle events (startup, shutdown, connections). */
#define LOG_INF(fmt, ...)  _LOG_CORE("INF", fmt, ##__VA_ARGS__)

/** \brief Debug — per-frame data, DB writes, query results.
 *         High volume — compile out in production with -DNDEBUG if needed. */
#ifndef NDEBUG
#define LOG_DBG(fmt, ...)  _LOG_CORE("DBG", fmt, ##__VA_ARGS__)
#else
#define LOG_DBG(fmt, ...)  do {} while (0)
#endif

/**
 * \brief Backward-compatible alias — all existing LOG() calls map to LOG_INF.
 *        Migrate noisy per-frame call sites to LOG_DBG at your own pace.
 */
#define LOG(fmt, ...)  LOG_INF(fmt, ##__VA_ARGS__)

/******************************* ENUMERATIONS *********************************/

typedef enum
{
   DEV_PIR   = 0,  /**< PIR motion sensor          */
   DEV_LIGHT = 1,  /**< smart light relay           */
   DEV_LOCK  = 2,  /**< smart lock                  */
   DEV_MOTOR = 3,  /**< motor thermostat controller */
   DEV_COUNT = 4   /**< number of tracked devices   */
} DEV_ID_E;

/************************ STRUCTURE/UNION DATA TYPES **************************/

/**
 * \brief Reed sensor slot — packed wire format.
 * \warning Must match sensor_types.h ReedSlotData exactly.
 */
struct __attribute__((packed)) ReedSlotData
{
   uint16_t age;              /*!< seconds since last advertisement  */
   int8_t   batt;             /*!< battery SOC percent, -1=unknown   */
   uint8_t  active;           /*!< 1=slot occupied                   */
   uint8_t  state;            /*!< reed state (0=closed, 1=open)     */
   uint8_t  offline;          /*!< 1=age exceeded offline threshold  */
   uint16_t gen;              /*!< generation counter — increments
                                   on device replacement             */
   char     name[REED_NAME_SIZE]; /*!< BLE advertised name           */
};

/**
 * \brief PIR sensor slot — packed wire format.
 * \warning Must match sensor_types.h PirSlotData exactly.
 */
struct __attribute__((packed)) PirSlotData
{
   uint32_t count;    /*!< cumulative motion event count             */
   uint16_t age;      /*!< seconds since last advertisement          */
   int8_t   batt;     /*!< battery SOC percent, -1=unknown           */
   uint8_t  active;   /*!< 1=slot occupied                           */
   int8_t   occupied; /*!< sliding-window occupancy flag             */
   uint8_t  offline;  /*!< 1=age exceeded offline threshold          */
};

/**
 * \brief Temperature sensor slot — packed wire format.
 * \warning Must match sensor_types.h TempSlotData exactly.
 */
struct __attribute__((packed)) TempSlotData
{
   int16_t  temp_decidegc;       /*!< temperature in tenths of °C   */
   uint16_t age;                 /*!< seconds since last adv        */
   int8_t   batt;                /*!< battery SOC percent,-1=unknown*/
   uint8_t  active;              /*!< 1=slot occupied               */
   uint8_t  offline;             /*!< 1=age exceeded offline thresh */
   uint8_t  _pad;                /*!< alignment padding             */
   char     name[TEMP_NAME_LEN]; /*!< BLE advertised name           */
};

/**
 * \brief Sensor data pipe wire format — ingress from sensor_server.
 *
 * \warning Must match sensor_types.h SensorData exactly. Layout change
 *          requires recompiling both sensor_server and data_controller.
 */
struct SensorData
{
   double   avg_temp;          /*!< average temperature across active temp slots */
   int      motion_count;      /*!< cumulative PIR motion event count            */
   int      light_state;       /*!< smart light relay state                      */
   int      lock_state;        /*!< smart lock state                             */
   long     timestamp;         /*!< Unix timestamp of frame                      */
   int      room_count;        /*!< number of valid room entries                 */
   uint8_t  doorbell_pressed;  /*!< 1 = press event in this frame               */
   uint8_t  doorbell_device_id;/*!< 0-3, which doorbell cam                     */

   struct
   {
      int  sensor_id;
      char room_name[ROOM_NAME_SIZE];
      char state[ROOM_STATE_SIZE];
      char location[ROOM_LOC_SIZE];
   } rooms[MAX_ROOMS];         /*!< room sensor states from ESP32 JSON           */

   uint16_t age_pir;           /*!< seconds since last PIR advertisement         */
   uint16_t age_lgt;           /*!< seconds since last light advertisement       */
   uint16_t age_lck;           /*!< seconds since last lock advertisement        */
   int8_t   batt_pir;          /*!< PIR battery SOC percent, -1=unknown          */
   int8_t   pir_occupied;      /*!< global PIR occupancy flag                    */
   int8_t   batt_lck;          /*!< lock battery SOC percent, -1=unknown         */
   int      batt_motor;        /*!< motor battery SOC percent, -1=unknown        */

   struct ReedSlotData reed_slots[MAX_REEDS]; /*!< reed sensor slot array        */
   uint8_t             motor_online;          /*!< 1=motor controller reachable  */

   struct PirSlotData  pir_slots[MAX_PIRS];   /*!< dynamic PIR slot array        */
   uint8_t             pir_count;             /*!< number of active PIR slots    */

   struct TempSlotData temp_slots[MAX_TEMPS]; /*!< dynamic temp slot array       */
   uint8_t             temp_count;            /*!< number of active temp slots   */
};

/**
 * \brief Canonical sensor read model — owned by state_registry.c.
 *
 * \details Written only via update_snapshot(). Read only via get_snapshot().
 *          No translation unit other than state_registry.c holds a pointer
 *          to the internal central_ledger instance of this struct.
 *          Consumers receive a value copy from get_snapshot().
 */
struct LatestData
{
   double   avg_temp;          /*!< average temperature across active temp slots */
   int      motion_count;      /*!< cumulative PIR motion event count            */
   int      light_state;       /*!< smart light relay state                      */
   int      lock_state;        /*!< smart lock state                             */
   long     timestamp;         /*!< Unix timestamp of last update                */
   int      valid;             /*!< 1 after first update_snapshot() call         */
   uint16_t age_pir;           /*!< seconds since last PIR advertisement         */
   uint16_t age_lgt;           /*!< seconds since last light advertisement       */
   uint16_t age_lck;           /*!< seconds since last lock advertisement        */
   int8_t   batt_pir;          /*!< PIR battery SOC percent, -1=unknown          */
   int8_t   pir_occupied;      /*!< global PIR occupancy flag                    */
   int8_t   batt_lck;          /*!< lock battery SOC percent, -1=unknown         */
   int      batt_motor;        /*!< motor battery SOC percent, -1=unknown        */

   struct ReedSlotData reed_slots[MAX_REEDS]; /*!< reed sensor slot array        */
   uint8_t             motor_online;          /*!< 1=motor controller reachable  */

   struct PirSlotData  pir_slots[MAX_PIRS];   /*!< dynamic PIR slot array        */
   uint8_t             pir_count;             /*!< number of active PIR slots    */

   struct TempSlotData temp_slots[MAX_TEMPS]; /*!< dynamic temp slot array       */
   uint8_t             temp_count;            /*!< number of active temp slots   */
   uint8_t             doorbell_pressed;      /*!< 1 = press received this frame */
   uint8_t             doorbell_device_id;    /*!< 0-3, which doorbell cam       */
};

/**
 * \brief One captured UART frame — stored in the ring buffer.
 *
 * \details uart_parse_line() fills one of these and pushes it onto
 *          g_uart_ring before calling sem_post(&g_uart_frame_sem).
 *          uart_push_thread() pops after sem_timedwait() succeeds.
 *          Fields cover the subset of sensor state that a UART frame
 *          can update: device index, parsed value, and battery SOC.
 */
typedef struct
{
   DEV_ID_E idx;  /*!< device that generated the frame */
   int      val;  /*!< parsed sensor value              */
   int      batt; /*!< battery SOC, -1 if absent        */
} UartFrame;

/**
 * \brief Single-producer / single-consumer ring buffer for UartFrame objects.
 *
 * \details head written only by uart_reader_thread (producer).
 *          tail written only by uart_push_thread (consumer).
 *          Both are read by both sides — declared volatile so the compiler
 *          does not cache them in registers across the sem_post /
 *          sem_timedwait barrier.
 *
 *          Capacity is UART_RING_SIZE frames. When full the oldest frame
 *          is overwritten and a warning is logged — last-write-wins is
 *          the correct overflow policy for sensor data.
 */
typedef struct
{
   UartFrame         frames[UART_RING_SIZE]; /*!< frame storage           */
   volatile unsigned head;                   /*!< producer write index    */
   volatile unsigned tail;                   /*!< consumer read index     */
   pthread_mutex_t   mutex;                  /*!< producer-side push guard*/
} uart_ring_t;

/*************************** SHARED GLOBALS ***********************************/

extern struct SharedSensorData *shm_data;        /**< process-shared memory region  */
extern sqlite3                 *db;              /**< SQLite database handle        */
extern volatile int             running;         /**< main loop / thread run flag   */
extern const char              *dev_names[DEV_COUNT]; /**< device name strings      */

/**
 * \brief Counting semaphore — pending UART frames in g_uart_ring.
 *
 * \details Invariant: sem value == number of frames between tail and head
 *          in g_uart_ring at all times. Initialized to 0 at startup.
 *          Producer increments via sem_post() after uart_ring_push().
 *          Consumer decrements via sem_timedwait() before uart_ring_pop().
 */
extern sem_t       g_uart_frame_sem;

/**
 * \brief Ring buffer shared between uart_reader_thread and uart_push_thread.
 */
extern uart_ring_t g_uart_ring;

/*************************** FUNCTION PROTOTYPES *****************************/

/* -- cmd/state_registry --------------------------------------------------- */
/* Single writer authority. All sensor state reads and writes go here.       */

void update_snapshot (const struct SensorData *p_data);
void get_snapshot    (struct LatestData *p_out);

/* -- cmd/uart_staging ----------------------------------------------------- */
/* UART protocol transition logic. Partial-update wrappers for UART-ingress  */
/* frames. Delegates all state writes to state_registry internally.          */

void uart_stage_lock  (int lock_state, int batt);
void uart_stage_pir   (int val);
void uart_stage_light (int val);
void uart_check_reeds (const struct SensorData *p_data);

/* -- cmd/shm_updater ------------------------------------------------------ */
/* Shared memory projection. Consumes frozen snapshots, writes shm_data.     */

void shm_update_frame         (const struct LatestData *p_snapshot,
                                const struct SensorData *p_data);
void handle_get_latest        (const struct LatestData *p_snapshot);
void handle_get_device_status (struct CommandMsg       *p_cmd);
void handle_get_room_status   (struct CommandMsg       *p_cmd);

/* -- cmd/db_persist ------------------------------------------------------- */
/* Unit of Work persistence facade. Raw ingress frame only — no snapshot.    */

void db_persist_frame (const struct SensorData *p_data);

/* -- cmd/cmd_handler ------------------------------------------------------ */
/* Orchestration only. Fans out to state_registry, shm, db, uart.           */

void *receive_data_thread   (void *p_arg);
void *command_handler_thread(void *p_arg);

/* -- cmd/commands --------------------------------------------------------- */

void process_command(struct CommandMsg *p_cmd);

/* -- heartbeat ------------------------------------------------------------ */

void  heartbeat_stamp           (DEV_ID_E idx);
void  heartbeat_snapshot_online (uint8_t *p_out, int count);
void *heartbeat_monitor_thread  (void *p_arg);

/* -- db_manager ----------------------------------------------------------- */

void db_open_and_init(void);
void db_begin(void);
void db_commit(void);
void db_rollback(void);
void db_save_uart    (const char *p_dev_id, int val, int batt);
void db_save_event   (const char *p_dev_name, const char *p_event);
void db_save_reading (const struct SensorData *p_data);
void db_save_reed    (int slot, const char *p_name, int state, int batt, int age);
void db_save_motor   (int online, int batt);
int  db_query_rooms  (struct RoomStatus *p_out, int max_rooms);

/* -- uart_controller ------------------------------------------------------ */

void *uart_reader_thread   (void *p_arg);
void *uart_push_thread     (void *p_arg);
void  uart_sync_lock_state (int state);
void  uart_update_frame    (const struct LatestData *p_snapshot,
                             const struct SensorData *p_data);
void  uart_ring_push       (const UartFrame *p_frame);
int   uart_ring_pop        (UartFrame *p_frame);

/* -- data_controller ------------------------------------------------------ */

int init_shared_memory(void);

/** \brief Per-device heartbeat timeout seconds — indexed by DEV_ID_E */
static const int hb_timeout_sec[DEV_COUNT] =
{
   30,   /**< DEV_PIR   — motion critical, short timeout          */
   300,  /**< DEV_LIGHT — 240s firmware heartbeat, 300s timeout   */
   300,  /**< DEV_LOCK  — 240s firmware heartbeat, 300s timeout   */
   300,  /**< DEV_MOTOR — on-demand TCP only                      */
};

#endif /* INCLUDE_CONTROLLER_INTERNAL_H_ */
