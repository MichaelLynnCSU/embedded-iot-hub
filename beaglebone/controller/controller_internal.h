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
 *          ReedSlotData and PirSlotData are packed for stable pipe wire
 *          format.
 *
 * \note    Logging fixes (2026-04-29):
 *          - localtime() replaced with localtime_r() — not thread-safe.
 *          - log_mutex added — prevents timestamp/message interleave across
 *            five concurrent threads.
 *          - Log levels added: ERR/WRN/INF/DBG. LOG is an alias for LOG_INF
 *            so all existing call sites compile without change.
 *          - Thread ID included in each line for post-mortem debugging.
 *          - Log rotation: file is rolled to .old at LOG_MAX_BYTES (5 MB).
 *
 * \note    Lock sync fix (2026-04-29):
 *          uart_sync_lock_state() added — called by process_sensor_frame()
 *          on every TCP frame so g_lock_state tracks the authoritative lock
 *          state regardless of whether the last command came via BLE or STM32.
 *
 * \note    PIR slot array (2026-05-XX):
 *          PirSlotData struct and pir_slots[MAX_PIRS] added to SensorData
 *          and LatestData to support dynamic multi-PIR slot tracking,
 *          mirroring the ReedSlotData pattern. Must match sensor_types.h
 *          in the server exactly.
 *
 * \note    Semaphore → mutex + ring buffer (2026-05-22):
 *          - shm_sem (named POSIX semaphore, binary) removed. All shared
 *            memory protection now uses shm_data->shm_mutex, a
 *            PTHREAD_PROCESS_SHARED mutex embedded in SharedSensorData.
 *          - SEM_NAME constant removed — no named semaphore object exists.
 *          - g_uart_frame_sem (counting semaphore) added. Producer:
 *            uart_parse_line() posts once per parsed UART frame after
 *            pushing the frame onto g_uart_ring. Consumer: uart_push_thread()
 *            calls sem_timedwait() — blocks with zero CPU until a frame
 *            arrives or UART_PUSH_INTERVAL_SEC elapses, then pops from
 *            g_uart_ring. Semaphore value == frames in ring buffer at all
 *            times — this invariant prevents payload loss and eliminates
 *            the fixed sleep() polling loop.
 *          - UartFrame / uart_ring_t ring buffer types added here so all
 *            translation units share one definition.
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
#include "ipc_lcd/commands.h"
#include "shared_data.h"

/******************************** CONSTANTS ***********************************/

/** \brief IPC and filesystem paths */
#define SHM_NAME           "/sensor_shm"                   /**< shared memory name */
#define SENSOR_PIPE        "/tmp/sensor_pipe"              /**< sensor data named pipe */
#define COMMAND_PIPE       "/tmp/controller_cmd"           /**< command named pipe */
#ifndef DB_PATH
#define DB_PATH            "/home/debian/db/sensors.db"   /**< SQLite database path */
#endif
#define CONTROLLER_LOG     "/var/log/data_controller.log"     /**< log file path */
#define CONTROLLER_LOG_OLD "/var/log/data_controller.log.old" /**< rotated log */
#define UART_DEV           "/dev/ttyS1"                   /**< UART device for STM32 */

/** \brief Sizing constants */
#define MAX_ROOMS       10   /**< maximum room sensors in SensorData */
#define MAX_REEDS       6    /**< must match ESP32 tcp_manager.c and ble_scan.c */
#define MAX_PIRS        5    /**< must match ESP32 tcp_manager.c and sensor_types.h */
#define UART_LINE_LEN   64   /**< UART line buffer size bytes */

/** \brief Ring buffer capacity — must be a power of two for mask wrapping */
#define UART_RING_SIZE  16   /**< max queued UART frames before oldest is dropped */

/** \brief Room name field sizes */
#define ROOM_NAME_SIZE  32   /**< room name string buffer size */
#define ROOM_STATE_SIZE 16   /**< room state string buffer size */
#define ROOM_LOC_SIZE   32   /**< room location string buffer size */
#define REED_NAME_SIZE  16   /**< reed BLE name string buffer size */

/** \brief Log rotation threshold — 5 MB */
#define LOG_MAX_BYTES   (5 * 1024 * 1024)

/********************************** LOGGING ***********************************/

extern FILE           *log_fp;    /**< log file handle — opened by data_controller.c */
extern pthread_mutex_t log_mutex; /**< serialises log writes across all threads */

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
 *          - localtime_r() used — thread-safe, no shared static buffer.
 *          - log_mutex held for the full timestamp+message write.
 *          - Log rotated if file exceeds LOG_MAX_BYTES.
 *          - No-op if log_fp is NULL.
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

/** \brief Error — unexpected failures, data loss, hardware faults. */
#define LOG_ERR(fmt, ...)  _LOG_CORE("ERR", fmt, ##__VA_ARGS__)

/** \brief Warning — degraded operation, retryable errors, unexpected state. */
#define LOG_WRN(fmt, ...)  _LOG_CORE("WRN", fmt, ##__VA_ARGS__)

/** \brief Info — normal lifecycle events (startup, shutdown, connections). */
#define LOG_INF(fmt, ...)  _LOG_CORE("INF", fmt, ##__VA_ARGS__)

/** \brief Debug — per-frame data, successful DB writes, query results.
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
   DEV_PIR   = 0,
   DEV_LIGHT = 1,
   DEV_LOCK  = 2,
   DEV_MOTOR = 3,
   DEV_COUNT = 4
} DEV_ID_E;

/************************ STRUCTURE/UNION DATA TYPES **************************/

/**
 * \brief Reed sensor slot — packed wire format.
 * \warning Must match sensor_types.h ReedSlotData exactly.
 */
struct __attribute__((packed)) ReedSlotData
{
   uint16_t age;
   int8_t   batt;
   uint8_t  active;
   uint8_t  state;
   uint8_t  offline;
   uint16_t gen;
   char     name[REED_NAME_SIZE];
};

/**
 * \brief PIR sensor slot — packed wire format.
 * \warning Must match sensor_types.h PirSlotData exactly.
 */
struct __attribute__((packed)) PirSlotData
{
   uint32_t count;    /*!< cumulative motion event count   */
   uint16_t age;      /*!< seconds since last adv          */
   int8_t   batt;     /*!< battery SOC percent, -1=unknown */
   uint8_t  active;   /*!< 1=slot occupied                 */
   int8_t   occupied; /*!< sliding-window occupancy flag   */
   uint8_t  offline;  /*!< age > PIR_OFFLINE_S             */
};

/**
 * \brief Sensor data pipe wire format.
 * \warning Must match sensor_types.h SensorData exactly.
 */
struct SensorData
{
   double   avg_temp;
   int      motion_count;
   int      light_state;
   int      lock_state;
   long     timestamp;
   int      room_count;

   struct
   {
      int  sensor_id;
      char room_name[ROOM_NAME_SIZE];
      char state[ROOM_STATE_SIZE];
      char location[ROOM_LOC_SIZE];
   } rooms[MAX_ROOMS];

   uint16_t age_pir;
   uint16_t age_lgt;
   uint16_t age_lck;
   int8_t   batt_pir;
   int8_t   pir_occupied;
   int8_t   batt_lck;
   int      batt_motor;

   struct ReedSlotData reed_slots[MAX_REEDS];
   uint8_t             motor_online;

   struct PirSlotData  pir_slots[MAX_PIRS]; /*!< dynamic PIR slot array */
   uint8_t             pir_count;           /*!< number of active PIR slots */
};

/**
 * \brief Latest sensor snapshot — updated by process_sensor_frame().
 */
struct LatestData
{
   double   avg_temp;
   int      motion_count;
   int      light_state;
   int      lock_state;
   long     timestamp;
   int      valid;
   uint16_t age_pir;
   uint16_t age_lgt;
   uint16_t age_lck;
   int8_t   batt_pir;
   int8_t   pir_occupied;
   int8_t   batt_lck;
   int      batt_motor;
   struct ReedSlotData reed_slots[MAX_REEDS];
   uint8_t             motor_online;

   struct PirSlotData  pir_slots[MAX_PIRS]; /*!< dynamic PIR slot array */
   uint8_t             pir_count;           /*!< number of active PIR slots */
};

/**
 * \brief One captured UART frame — stored in the ring buffer.
 *
 * \details uart_parse_line() fills one of these and pushes it onto
 *          g_uart_ring before calling sem_post(&g_uart_frame_sem).
 *          uart_push_thread() pops after sem_timedwait() succeeds.
 *          Fields mirror the subset of LatestData that a UART frame
 *          can update: device index, parsed value, and battery SOC.
 */
typedef struct
{
   DEV_ID_E idx;  /*!< device that generated the frame */
   int      val;  /*!< parsed sensor value              */
   int      batt; /*!< battery SOC, -1 if absent        */
} UartFrame;

/**
 * \brief Lock-free single-producer / single-consumer ring buffer for
 *        UartFrame objects.
 *
 * \details head is written only by the producer (uart_reader_thread).
 *          tail is written only by the consumer (uart_push_thread).
 *          Both are read by both sides — declared volatile so the
 *          compiler does not cache them in registers across the
 *          sem_post / sem_timedwait barrier.
 *
 *          Capacity is UART_RING_SIZE frames. When the buffer is full
 *          the producer overwrites the oldest frame and logs a warning
 *          — this is a deliberate last-write-wins policy for sensor
 *          data where the newest reading is always most relevant.
 */
typedef struct
{
   UartFrame        frames[UART_RING_SIZE]; /*!< frame storage              */
   volatile unsigned head;                  /*!< producer write index        */
   volatile unsigned tail;                  /*!< consumer read index         */
   pthread_mutex_t  mutex;                  /*!< producer-side push guard    */
} uart_ring_t;

/*************************** SHARED GLOBALS ***********************************/

extern struct LatestData        latest_data;
extern pthread_mutex_t          data_mutex;
extern struct SharedSensorData *shm_data;
extern sqlite3                 *db;
extern volatile int             running;
extern const char              *dev_names[DEV_COUNT];

/**
 * \brief Counting semaphore — pending UART frames in g_uart_ring.
 *
 * \details Invariant: sem value == number of frames between tail and head
 *          in g_uart_ring at all times.
 *          Initialized to 0 (no frames pending at startup).
 *          Producer increments via sem_post() after uart_ring_push().
 *          Consumer decrements via sem_timedwait() before uart_ring_pop().
 */
extern sem_t       g_uart_frame_sem;

/**
 * \brief Ring buffer shared between uart_reader_thread (producer) and
 *        uart_push_thread (consumer).
 */
extern uart_ring_t g_uart_ring;

/*************************** FUNCTION PROTOTYPES *****************************/

/* -- heartbeat ------------------------------------------------------------ */

void  heartbeat_stamp(DEV_ID_E idx);
void  heartbeat_snapshot_online(uint8_t *p_out, int count);
void *heartbeat_monitor_thread(void *p_arg);

/* -- db_manager ----------------------------------------------------------- */

void db_open_and_init(void);
void db_begin(void);
void db_commit(void);
void db_rollback(void);
void db_save_uart(const char *p_dev_id, int val, int batt);
void db_save_event(const char *p_dev_name, const char *p_event);
void db_save_reading(const struct SensorData *p_data);
void db_save_reed(int slot, const char *p_name, int state, int batt, int age);
void db_save_motor(int online, int batt);
int  db_query_rooms(struct RoomStatus *p_out, int max_rooms);

/* -- uart_controller ------------------------------------------------------ */

void *uart_reader_thread(void *p_arg);
void *uart_push_thread(void *p_arg);
void  uart_sync_lock_state(int state);
void  uart_ring_push(const UartFrame *p_frame);
int   uart_ring_pop(UartFrame *p_frame);

/* -- ipc_lcd/cmd_handler -------------------------------------------------- */

void  handle_get_latest(struct CommandMsg *p_cmd);
void  handle_get_room_status(struct CommandMsg *p_cmd);
void  handle_get_device_status(struct CommandMsg *p_cmd);
void *receive_data_thread(void *p_arg);
void *command_handler_thread(void *p_arg);

/* -- ipc_lcd/commands ----------------------------------------------------- */

void process_command(struct CommandMsg *p_cmd);

/* -- data_controller ------------------------------------------------------ */

int init_shared_memory(void);

/** \brief Per-device heartbeat timeout seconds */
static const int hb_timeout_sec[DEV_COUNT] =
{
   30,   /**< DEV_PIR   — motion critical, short timeout */
   300,  /**< DEV_LIGHT — 240s firmware HB, 300s timeout */
   300,  /**< DEV_LOCK  — 240s firmware HB, 300s timeout */
   300,  /**< DEV_MOTOR — on-demand TCP only */
};

#endif /* INCLUDE_CONTROLLER_INTERNAL_H_ */
