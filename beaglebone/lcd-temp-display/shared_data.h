/******************************************************************************
 * \file shared_data.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief POSIX shared memory layout for BeagleBone controller IPC.
 *
 * \details Defines the shared memory structure used between the data
 *          controller and LCD display processes. Both processes must
 *          include this header — the layout must never change without
 *          recompiling both binaries.
 *
 * \warning SHM_NAME and SEM_NAME are also defined in controller_internal.h.
 *          shared_data.h is the authoritative definition — controller_internal.h
 *          includes this header and should not redefine them.
 ******************************************************************************/

#ifndef INCLUDE_SHARED_DATA_H_
#define INCLUDE_SHARED_DATA_H_

#include <time.h>
#include <stdint.h>

/******************************** CONSTANTS ***********************************/

#define DEVICE_COUNT      6   /**< number of tracked devices in device_online */
#define HISTORY_BUF_SIZE  100 /**< history ring buffer capacity */
#define ALERT_BUF_SIZE    10  /**< maximum simultaneous alerts */
#define ROOM_BUF_SIZE     10  /**< maximum room sensors in shared memory */
#define ALERT_MSG_SIZE    64  /**< alert message string buffer size */
#define ROOM_NAME_SZ      32  /**< room name string buffer size */
#define ROOM_STATE_SZ     16  /**< room state string buffer size */
#define ROOM_LOC_SZ       32  /**< room location string buffer size */

/** \brief Alert severity levels */
#define ALERT_SEVERITY_LOW    1  /**< low severity alert */
#define ALERT_SEVERITY_MED    2  /**< medium severity alert */
#define ALERT_SEVERITY_HIGH   3  /**< high severity alert */

/************************ STRUCTURE/UNION DATA TYPES **************************/

/** \brief Single historical sensor reading. */
struct HistoryPoint
{
   double temp;      /*!< temperature reading in Celsius */
   int    motion;    /*!< motion event count */
   long   timestamp; /*!< Unix timestamp of reading */
};

/** \brief System alert entry. */
struct Alert
{
   long timestamp;          /*!< Unix timestamp of alert */
   char message[ALERT_MSG_SIZE]; /*!< alert description string */
   int  severity;           /*!< severity level: 1=low, 2=medium, 3=high */
};

/** \brief Room sensor status entry. */
struct RoomStatus
{
   int  sensor_id;             /*!< unique room sensor identifier */
   char room_name[ROOM_NAME_SZ]; /*!< room name string */
   char state[ROOM_STATE_SZ];  /*!< current state e.g. "open" or "closed" */
   char location[ROOM_LOC_SZ]; /*!< physical location string */
   long timestamp;             /*!< Unix timestamp of last update */
};

/**
 * \brief POSIX shared memory layout — written by controller, read by LCD.
 *
 * \warning Both controller and LCD display must be compiled with the same
 *          version of this struct. Any layout change requires recompiling
 *          both binaries.
 */
struct SharedSensorData
{
   uint8_t device_online[DEVICE_COUNT]; /*!< online flags indexed by DEV_ID_E */

   /* Current sensor reading */
   double current_temp;      /*!< latest average temperature in Celsius */
   int    current_motion;    /*!< latest PIR motion event count */
   int    current_light;     /*!< latest smart light relay state */
   int    current_lock;      /*!< latest smart lock state */
   int    batt_motor;        /*!< motor battery percentage (0–100), -1=unknown */
   long   current_timestamp; /*!< Unix timestamp of latest reading */
   int    data_valid;        /*!< 1 if at least one reading has been received */

   /* Statistics */
   double temp_min;          /*!< minimum recorded temperature */
   double temp_max;          /*!< maximum recorded temperature */
   double temp_avg;          /*!< average recorded temperature */
   int    motion_total;      /*!< total motion event count */

   /* History ring buffer */
   struct HistoryPoint history[HISTORY_BUF_SIZE]; /*!< circular history buffer */
   int    history_count;     /*!< number of valid history entries */

   /* Trends */
   double temp_trend;        /*!< temperature trend direction */
   int    motion_trend;      /*!< motion trend direction */

   /* Peak activity */
   int    peak_motion_hour;  /*!< hour of peak motion activity (0-23) */
   double peak_temp_time;    /*!< time of peak temperature */

   /* Alerts */
   int          alert_count;           /*!< number of active alerts */
   struct Alert alerts[ALERT_BUF_SIZE]; /*!< active alert array */

   /* Room sensor status */
   int               room_count;            /*!< number of valid room entries */
   struct RoomStatus rooms[ROOM_BUF_SIZE];  /*!< room sensor status array */

   /* System information */
   int  total_records;       /*!< total database record count */
   long uptime_seconds;      /*!< controller process uptime in seconds */
   int  disk_usage_percent;  /*!< filesystem usage percent */

   /* IPC response metadata */
   int  last_command;        /*!< last command processed */
   int  command_result;      /*!< result of last command (0=success, -1=fail) */
   int  sequence;            /*!< increments on each shm update */
   long response_time_ms;    /*!< time taken to process last command ms */
};

#endif /* INCLUDE_SHARED_DATA_H_ */
