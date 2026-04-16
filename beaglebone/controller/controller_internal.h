/******************************************************************************
 * \file controller_internal.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Internal shared header for BeagleBone data controller subsystem.
 *
 * \details Shared by all controller translation units. Defines paths,
 *          sizing constants, wire format structs, logging macro, and
 *          all internal function prototypes.
 *
 * \warning SensorData struct layout must match sensor_server.c exactly —
 *          both sides of the named pipe use this as the wire format.
 *          ReedSlotData is packed for stable pipe wire format.
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
#include "commands.h"
#include "shared_data.h"

/******************************** CONSTANTS ***********************************/

/** \brief IPC and filesystem paths */
#define SHM_NAME        "/sensor_shm"             /**< shared memory name */
#define SEM_NAME        "/sensor_sem"             /**< shared semaphore name */
#define SENSOR_PIPE     "/tmp/sensor_pipe"        /**< sensor data named pipe */
#define COMMAND_PIPE    "/tmp/controller_cmd"     /**< command named pipe */
#define DB_PATH         "/home/debian/db/sensors.db" /**< SQLite database path */
#define CONTROLLER_LOG  "/var/log/data_controller.log" /**< log file path */
#define UART_DEV        "/dev/ttyS1"              /**< UART device for STM32 */

/** \brief Sizing constants */
#define MAX_ROOMS       10   /**< maximum room sensors in SensorData */
#define MAX_REEDS       6    /**< must match ESP32 tcp_manager.c and ble_scan.c */
#define UART_LINE_LEN   64   /**< UART line buffer size bytes */
#define HB_TIMEOUT_SEC  10   /**< heartbeat timeout in seconds */

/** \brief Room name field sizes */
#define ROOM_NAME_SIZE  32   /**< room name string buffer size */
#define ROOM_STATE_SIZE 16   /**< room state string buffer size */
#define ROOM_LOC_SIZE   32   /**< room location string buffer size */
#define REED_NAME_SIZE  16   /**< reed BLE name string buffer size */

/********************************** MACROS ************************************/

extern FILE *log_fp; /**< log file handle — opened by data_controller.c */

/**
 * \brief Timestamped log macro — writes to log_fp if open.
 *
 * \details Format: [YYYY-MM-DD HH:MM:SS] <message>
 *          No-op if log_fp is NULL.
 */
#define LOG(fmt, ...) \
do \
{ \
   if (log_fp) \
   { \
      time_t          _t  = time(NULL); \
      struct tm      *_tm = localtime(&_t); \
      fprintf(log_fp, \
              "[%04d-%02d-%02d %02d:%02d:%02d] " fmt "\n", \
              _tm->tm_year + 1900, _tm->tm_mon + 1, _tm->tm_mday, \
              _tm->tm_hour, _tm->tm_min, _tm->tm_sec, \
              ##__VA_ARGS__); \
      fflush(log_fp); \
   } \
} while (0)

/******************************* ENUMERATIONS *********************************/

/**
 * \brief Device index enumeration for heartbeat tracking.
 *
 * \details Reed sensors tracked separately via heartbeat_stamp_reed(slot).
 */
typedef enum
{
   DEV_PIR   = 0, /**< PIR motion sensor */
   DEV_LIGHT = 1, /**< smart light relay */
   DEV_LOCK  = 2, /**< smart lock */
   DEV_MOTOR = 3, /**< C3 motor controller */
   DEV_COUNT = 4  /**< total device count — must be last */
} DEV_ID_E;

/************************ STRUCTURE/UNION DATA TYPES **************************/

/**
 * \brief Reed sensor slot data — packed for stable pipe wire format.
 *
 * \warning Layout must not change without updating sensor_server.c.
 */
struct __attribute__((packed)) ReedSlotData
{
   uint16_t age;           /*!< seconds since last seen, 0xFFFF=never */
   int8_t   batt;          /*!< battery SOC percent, -1=unknown */
   uint8_t  active;        /*!< 1=slot occupied (ACTIVE or OFFLINE) */
   uint8_t  state;         /*!< 0=closed, 1=open, 0xFF=unknown */
   uint8_t  offline;       /*!< 1=SLOT_OFFLINE on ESP32, red dot, tile stays */
   uint16_t gen;           /*!< increments on device swap */
   char     name[REED_NAME_SIZE]; /*!< BLE device name e.g. "ReedSensor3" */
};

/**
 * \brief Sensor data pipe wire format.
 *
 * \warning Must be identical in sensor_server.c and controller_internal.h.
 *          Both sides of the named pipe use this struct directly.
 */
struct SensorData
{
   double   avg_temp;      /*!< average temperature in Celsius */
   int      motion_count;  /*!< PIR motion event count */
   int      light_state;   /*!< smart light relay state */
   int      lock_state;    /*!< smart lock state */
   long     timestamp;     /*!< Unix timestamp of reading */
   int      room_count;    /*!< number of valid room entries */

   struct
   {
      int  sensor_id;                /*!< room sensor identifier */
      char room_name[ROOM_NAME_SIZE]; /*!< room name string */
      char state[ROOM_STATE_SIZE];    /*!< current state string */
      char location[ROOM_LOC_SIZE];   /*!< physical location string */
   } rooms[MAX_ROOMS];               /*!< room sensor array */

   uint16_t age_pir;       /*!< PIR device age in seconds */
   uint16_t age_lgt;       /*!< light device age in seconds */
   uint16_t age_lck;       /*!< lock device age in seconds */
   int8_t   batt_pir;      /*!< PIR battery SOC percent */
   int8_t   batt_lck;      /*!< lock battery SOC percent */
   int      batt_motor;

   struct ReedSlotData reed_slots[MAX_REEDS]; /*!< dynamic reed slot array */
   uint8_t  motor_online;  /*!< 1 if C3 motor controller is online */
};

/**
 * \brief In-memory mirror of latest received sensor data.
 */
struct LatestData
{
   double   avg_temp;      /*!< average temperature in Celsius */
   int      motion_count;  /*!< PIR motion event count */
   int      light_state;   /*!< smart light relay state */
   int      lock_state;    /*!< smart lock state */
   long     timestamp;     /*!< Unix timestamp of last reading */
   int      valid;         /*!< 1 if data has been received at least once */
   uint16_t age_pir;       /*!< PIR device age in seconds */
   uint16_t age_lgt;       /*!< light device age in seconds */
   uint16_t age_lck;       /*!< lock device age in seconds */
   int8_t   batt_pir;      /*!< PIR battery SOC percent */
   int8_t   batt_lck;      /*!< lock battery SOC percent */
   int      batt_motor;    /*!< motor battery mV */
   struct ReedSlotData reed_slots[MAX_REEDS]; /*!< reed slot mirror */
   uint8_t  motor_online;  /*!< 1 if C3 motor controller is online */
};

/*************************** SHARED GLOBALS ***********************************/

extern struct LatestData        latest_data; /**< latest sensor snapshot */
extern pthread_mutex_t          data_mutex;  /**< latest_data mutex */
extern struct SharedSensorData *shm_data;    /**< shared memory data pointer */
extern sem_t                   *shm_sem;     /**< shared memory semaphore */
extern sqlite3                 *db;          /**< SQLite database handle */
extern volatile int             running;     /**< main loop run flag */
extern const char              *dev_names[DEV_COUNT]; /**< device name strings */

/*************************** FUNCTION PROTOTYPES *****************************/

/** \brief Stamp heartbeat timestamp for a device.
 *  \param idx - Device index from DEV_ID_E.
 *  \return void */
void heartbeat_stamp(DEV_ID_E idx);

/** \brief Open and initialize the SQLite database.
 *  \return void */
void db_open_and_init(void);

/** \brief Save a UART sensor reading to database.
 *  \param p_dev_id - Device identifier string.
 *  \param val      - Sensor value.
 *  \param batt     - Battery SOC percent, -1 if unknown.
 *  \return void */
void db_save_uart(const char *p_dev_id, int val, int batt);

/** \brief Save a device event to database.
 *  \param p_dev_name - Device name string.
 *  \param p_event    - Event description string.
 *  \return void */
void db_save_event(const char *p_dev_name, const char *p_event);

/** \brief Save a main sensor reading to database.
 *  \param ts     - Unix timestamp.
 *  \param temp   - Average temperature.
 *  \param motion - Motion event count.
 *  \return void */
void db_save_reading(long ts, double temp, int motion);

/** \brief Query room sensor states from database into shared memory.
 *  \return void */
void db_query_rooms(void);

/** \brief Thread — reads UART data from STM32.
 *  \param p_arg - Unused thread argument.
 *  \return void* */
void *uart_reader_thread(void *p_arg);

/** \brief Thread — pushes UART data to sensor pipe.
 *  \param p_arg - Unused thread argument.
 *  \return void* */
void *uart_push_thread(void *p_arg);

/** \brief Thread — monitors device heartbeat timeouts.
 *  \param p_arg - Unused thread argument.
 *  \return void* */
void *heartbeat_monitor_thread(void *p_arg);

/** \brief Snapshot current device online status into output array.
 *  \param p_out  - Output array of online flags.
 *  \param count  - Number of devices to snapshot.
 *  \return void */
void heartbeat_snapshot_online(uint8_t *p_out, int count);

/** \brief Push latest data snapshot into shared memory.
 *  \param p_cmd - Command message pointer (unused).
 *  \return void */
void handle_get_latest(struct CommandMsg *p_cmd);

/** \brief Query room status from database into shared memory.
 *  \param p_cmd - Command message pointer (unused).
 *  \return void */
void handle_get_room_status(struct CommandMsg *p_cmd);

/** \brief Push device online status into shared memory.
 *  \param p_cmd - Command message pointer (unused).
 *  \return void */
void handle_get_device_status(struct CommandMsg *p_cmd);

/** \brief Dispatch a command message to the appropriate handler.
 *  \param p_cmd - Pointer to command message.
 *  \return void */
void process_command(struct CommandMsg *p_cmd);

/** \brief Thread — reads SensorData frames from sensor pipe.
 *  \param p_arg - Unused thread argument.
 *  \return void* */
void *receive_data_thread(void *p_arg);

/** \brief Thread — reads CommandMsg frames from command pipe.
 *  \param p_arg - Unused thread argument.
 *  \return void* */
void *command_handler_thread(void *p_arg);

/** \brief Initialize POSIX shared memory and semaphore.
 *  \return int - 0 on success, -1 on failure. */
int init_shared_memory(void);

#endif /* INCLUDE_CONTROLLER_INTERNAL_H_ */
