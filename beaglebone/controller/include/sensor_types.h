/******************************************************************************
 * \file sensor_types.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-05-10
 *
 * \brief Shared wire format structs for controller.
 *
 * \details Pipe ingress structs consumed by the BeagleBone controller.
 *          Must be byte-for-byte identical with
 *          beaglebone/server/sensor_types.h.
 *          Both files must be updated together if the layout changes.
 *
 * \note    Structured event tracing (2026-06-16):
 *          event_id added to ReedSlotData, PirSlotData, TempSlotData.
 *          lock_event_id and light_event_id added to SensorData and
 *          LatestData tails.
 ******************************************************************************/

#ifndef INCLUDE_SENSOR_TYPES_H_
#define INCLUDE_SENSOR_TYPES_H_

#include <stdint.h>
#include <pthread.h>
#include "config.h"

typedef enum
{
   DEV_PIR   = 0,
   DEV_LIGHT = 1,
   DEV_LOCK  = 2,
   DEV_MOTOR = 3,
   DEV_LCD   = 4,
   DEV_COUNT = 5
} DEV_ID_E;

/** \warning Must match sensor_types.h ReedSlotData exactly — packed wire format */
struct __attribute__((packed)) ReedSlotData
{
   uint16_t age;
   int8_t   batt;
   uint8_t  active;
   uint8_t  state;
   uint8_t  offline;
   uint16_t gen;
   char     name[REED_NAME_SIZE];
   uint64_t event_id;  /*!< last vroom event_id for this slot */
};

/** \warning Must match sensor_types.h PirSlotData exactly — packed wire format */
struct __attribute__((packed)) PirSlotData
{
   uint32_t count;
   uint16_t age;
   int8_t   batt;
   uint8_t  active;
   int8_t   occupied;
   uint8_t  offline;
   uint8_t  _pad[2];   /*!< alignment padding before event_id */
   uint64_t event_id;  /*!< last vroom event_id for this slot */
};

/** \brief Camera slot age transport — pipe wire format. age_s only; online/offline derived by consumer from age threshold. */
struct __attribute__((packed)) CamSlotData
{
   uint16_t age_s;   /*!< seconds since last heartbeat, 0xFFFF=never */
   uint8_t  _pad;
};

/** \brief Doorbell slot age transport — pipe wire format. age_s only; online/offline derived by consumer from age threshold. */
struct __attribute__((packed)) DoorbellSlotData
{
   uint16_t age_s;   /*!< seconds since last heartbeat/press, 0xFFFF=never */
   uint8_t  _pad;
};

/** \brief Camera manager pipe frame — written by camera_manager, read by controller. */
struct CamData
{
   struct CamSlotData      cam_slots[MAX_CAMS];
   struct DoorbellSlotData doorbell_slots[MAX_DOORBELL_CAMS];
};

/** \warning Must match sensor_types.h TempSlotData exactly — packed wire format */
struct __attribute__((packed)) TempSlotData
{
   int16_t  temp_decidegc;
   uint16_t age;
   int8_t   batt;
   uint8_t  active;
   uint8_t  offline;
   uint8_t  _pad;
   char     name[TEMP_NAME_LEN];
   uint64_t event_id;  /*!< last vroom event_id for this slot */
};



/**
 * \brief Pipe wire format — ingress from sensor_server.
 * \warning Layout must match sensor_types.h SensorData exactly.
 */
struct SensorData
{
   double   avg_temp;
   int      motion_count;
   int      light_state;
   int      lock_state;
   long     timestamp;
   int      room_count;
   uint8_t  doorbell_pressed;
   uint8_t  doorbell_device_id;

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

   struct ReedSlotData     reed_slots[MAX_REEDS];
   uint8_t                 motor_online;
   struct PirSlotData      pir_slots[MAX_PIRS];
   uint8_t                 pir_count;
   struct TempSlotData     temp_slots[MAX_TEMPS];
   uint8_t                 temp_count;

   uint64_t lock_event_id;   /*!< last vroom event_id for BLE lock  */
   uint64_t light_event_id;  /*!< last vroom event_id for BLE light */
   uint32_t frame_seq;       /*!< monotonic frame counter from server parse stage; join key for telemetry.log and events.log */
};

/**
 * \brief Canonical sensor read model — owned exclusively by state_registry.c.
 * \details Consumers receive a value copy via get_snapshot(). Never held by pointer.
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
   struct PirSlotData  pir_slots[MAX_PIRS];
   uint8_t             pir_count;
   struct TempSlotData temp_slots[MAX_TEMPS];
   uint8_t             temp_count;
   uint8_t             doorbell_pressed;
   uint8_t             doorbell_device_id;

   uint64_t lock_event_id;
   uint64_t light_event_id;
};

/** \brief One captured UART frame stored in the ring buffer. */
typedef struct
{
   DEV_ID_E idx;
   int      val;
   int      batt;
} UartFrame;

/**
 * \brief SPSC ring buffer for UartFrame objects.
 * \details head = producer (uart_reader_thread), tail = consumer (uart_push_thread).
 *          Overflow policy: oldest frame overwritten, warning logged.
 */
typedef struct
{
   UartFrame         frames[UART_RING_SIZE];
   volatile unsigned head;
   volatile unsigned tail;
   pthread_mutex_t   mutex;
} uart_ring_t;

#endif /* INCLUDE_SENSOR_TYPES_H_ */
