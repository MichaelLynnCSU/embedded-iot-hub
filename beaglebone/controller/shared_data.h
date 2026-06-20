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
 * \note    Semaphore → mutex migration (2026-05-22):
 *          shm_sem replaced by shm_mutex (pthread_mutex_t,
 *          PTHREAD_PROCESS_SHARED) embedded as the first field.
 *          Initialized once by data_controller.c:init_shared_memory().
 *
 * \note    Doorbell liveness (2026-06-09):
 *          doorbell_age_s[] and doorbell_online[] added. Projected from
 *          SensorData.doorbell_slots[] by shm_updater.c:shm_update_frame().
 *
 * \note    Inference camera liveness (2026-06-10):
 *          cam_age_s[] and cam_online[] added. CAM_COUNT=3.
 *          Projected from SensorData.cam_slots[] by shm_updater.c.
 *          Heartbeat stamped by esp32-cam, forwarded by hub TCP frame.
 *
 * \note    Structured event tracing — finish line (2026-06-16):
 *          event_id added to tail of SharedSensorData. Originally carried
 *          the VROOM-assigned correlation key from the last sensor frame,
 *          set by shm_updater.c:handle_get_latest() for LCD correlation
 *          logging only.
 *
 * \note    Per-device event_id logging (2026-06-19):
 *          SharedSensorData.event_id is NO LONGER WRITTEN by
 *          shm_updater.c. The rolled-up single event_id silently
 *          discarded every PIR/reed/temp event_id to represent an entire
 *          snapshot with one device's ID — see shm_updater.c header note
 *          "Per-device event_id logging (2026-06-19)" for rationale.
 *          Per-device EID_* lines are now emitted directly into the UART
 *          bundle by uart_controller.c instead. The event_id field is
 *          retained here for ABI compatibility with the LCD binary but
 *          is always 0 in current builds — do not write or read it for
 *          any correlation purpose. Remove it when LCD is next recompiled
 *          against a new layout.
 *
 * \warning Layout change — recompile both controller and LCD binaries.
 ******************************************************************************/

#ifndef INCLUDE_SHARED_DATA_H_
#define INCLUDE_SHARED_DATA_H_

#include <time.h>
#include <stdint.h>
#include <pthread.h>

/******************************** CONSTANTS ***********************************/

#define DEVICE_COUNT      9   /**< number of tracked devices in device_online */
#define HISTORY_BUF_SIZE  100 /**< history ring buffer capacity */
#define ALERT_BUF_SIZE    10  /**< maximum simultaneous alerts */
#define ROOM_BUF_SIZE     10  /**< maximum room sensors in shared memory */
#define ALERT_MSG_SIZE    64  /**< alert message string buffer size */
#define ROOM_NAME_SZ      32  /**< room name string buffer size */
#define ROOM_STATE_SZ     16  /**< room state string buffer size */
#define ROOM_LOC_SZ       32  /**< room location string buffer size */
#define MAX_DOORBELL_CAMS 4   /**< number of doorbell camera slots */
#define CAM_COUNT         3   /**< number of inference cameras */

/** \brief Alert severity levels */
#define ALERT_SEVERITY_LOW    1
#define ALERT_SEVERITY_MED    2
#define ALERT_SEVERITY_HIGH   3

/************************ STRUCTURE/UNION DATA TYPES **************************/

/** \brief Single historical sensor reading. */
struct HistoryPoint
{
   double temp;
   int    motion;
   long   timestamp;
};

/** \brief System alert entry. */
struct Alert
{
   long timestamp;
   char message[ALERT_MSG_SIZE];
   int  severity;
};

/** \brief Room sensor status entry. */
struct RoomStatus
{
   int  sensor_id;
   char room_name[ROOM_NAME_SZ];
   char state[ROOM_STATE_SZ];
   char location[ROOM_LOC_SZ];
   long timestamp;
};

/**
 * \brief POSIX shared memory layout — written by controller, read by LCD.
 *
 * \warning Any layout change requires recompiling both controller and LCD.
 *          shm_mutex must be first field for correct alignment.
 */
struct SharedSensorData
{
   pthread_mutex_t shm_mutex; /*!< process-shared mutex, init by controller  */

   uint8_t device_online[DEVICE_COUNT];

   double current_temp;
   int    current_motion;
   int    current_light;
   int    current_lock;
   int    batt_motor;
   long   current_timestamp;
   int    data_valid;

   double temp_min;
   double temp_max;
   double temp_avg;
   int    motion_total;

   struct HistoryPoint history[HISTORY_BUF_SIZE];
   int    history_count;

   double temp_trend;
   int    motion_trend;

   int    peak_motion_hour;
   double peak_temp_time;

   int          alert_count;
   struct Alert alerts[ALERT_BUF_SIZE];

   int               room_count;
   struct RoomStatus rooms[ROOM_BUF_SIZE];

   int  total_records;
   long uptime_seconds;
   int  disk_usage_percent;

   int  last_command;
   int  command_result;
   int  sequence;
   long response_time_ms;

   /* Doorbell press event — one-shot, cleared after each consumer reads */
   uint8_t  doorbell_pressed;
   uint8_t  doorbell_device_id;
   long     doorbell_timestamp;

   /* Doorbell per-cam liveness */
   uint16_t doorbell_age_s[MAX_DOORBELL_CAMS];
   uint8_t  doorbell_online[MAX_DOORBELL_CAMS];

   /* Inference camera liveness */
   uint16_t cam_age_s[CAM_COUNT];
   uint8_t  cam_online[CAM_COUNT];

   /* Structured event tracing finish line (2026-06-16):
    * Originally: VROOM-assigned event_id from the last sensor frame,
    * set by shm_updater.c:handle_get_latest() for LCD correlation logging.
    *
    * Per-device event_id logging (2026-06-19):
    * This field is NO LONGER WRITTEN by shm_updater.c. The rolled-up
    * single event_id silently discarded every PIR/reed/temp event_id to
    * represent an entire snapshot with one device's ID — see shm_updater.c
    * header note "Per-device event_id logging (2026-06-19)" for rationale.
    * Per-device EID_* lines are now emitted directly into the UART bundle
    * by uart_controller.c instead.
    *
    * Field retained here for ABI compatibility with the LCD binary.
    * Value is always 0 in current builds. Do not write or read it for
    * any correlation purpose — use the per-device EID_* UART lines instead.
    * Remove this field when LCD is next recompiled against a new layout. */
   uint64_t event_id;
};

#endif /* INCLUDE_SHARED_DATA_H_ */
