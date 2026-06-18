/******************************************************************************
 * \file sensor_types.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-05-10
 *
 * \brief Shared wire format structs for sensor server.
 *
 * \details Extracted from sensor_server.c (2026-05-10) so that
 *          json_parser.c and pipe_writer.c can both include SensorData
 *          and slot structs without depending on sensor_server.c.
 *
 * \warning All structs and SensorData layout must be byte-for-byte
 *          identical with beaglebone/controller/include/sensor_types.h.
 *          Both files must be updated together if the layout changes.
 *
 * \note    Doorbell liveness (2026-06-09):
 *          DoorbellSlotData added. doorbell_slots[MAX_DOORBELL_CAMS]
 *          and doorbell_count added to SensorData tail.
 *
 * \note    Inference camera liveness (2026-06-10):
 *          CamSlotData added. cam_slots[MAX_CAMS] added to SensorData
 *          tail. MAX_CAMS=3 (indoor/front/back). Heartbeat stamped by
 *          esp32-cam udp_device_ingress, projected to BBB via TCP frame.
 *
 * \note    Structured event tracing (2026-06-16):
 *          event_id added to ReedSlotData, PirSlotData, TempSlotData.
 *          lock_event_id and light_event_id added to SensorData tail.
 *          Matches ESP32 tcp_manager.c Phase 4A serialization.
 ******************************************************************************/

#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <stdint.h>

#define MAX_TEMPS          4   /**< must match ESP32 and controller */
#define TEMP_NAME_LEN      32  /**< temp sensor BLE name buffer size */
#define MAX_ROOMS          10  /**< max room sensors per frame */
#define MAX_REEDS          6   /**< must match ESP32 and controller */
#define MAX_PIRS           5   /**< must match ESP32 and controller */
#define MAX_DOORBELL_CAMS  4   /**< must match ESP32 and controller */
#define MAX_CAMS           3   /**< inference cameras — must match controller */

#define REED_NAME_LEN  16  /**< reed BLE name buffer size */
#define ROOM_NAME_LEN  32  /**< room name buffer size */
#define ROOM_STATE_LEN 16  /**< room state buffer size */
#define ROOM_LOC_LEN   32  /**< room location buffer size */

#define AGE_MAX     0xFFFEu /**< maximum reportable age */
#define AGE_UNKNOWN 0xFFFFu /**< age sentinel: never seen */

/** \brief Doorbell camera slot — packed wire format. */
struct __attribute__((packed)) DoorbellSlotData
{
   uint16_t age_s;   /*!< seconds since last heartbeat/press, 0xFFFF=never */
   uint8_t  online;  /*!< 1=alive within threshold, 0=stale or never seen  */
   uint8_t  _pad;
};

/** \brief Inference camera slot — packed wire format. */
struct __attribute__((packed)) CamSlotData
{
   uint16_t age_s;   /*!< seconds since last heartbeat, 0xFFFF=never */
   uint8_t  online;  /*!< 1=alive within threshold, 0=stale          */
   uint8_t  _pad;
};

/** \brief Reed sensor slot — packed wire format. */
struct __attribute__((packed)) ReedSlotData
{
   uint16_t age;
   int8_t   batt;
   uint8_t  active;
   uint8_t  state;
   uint8_t  offline;
   uint16_t gen;
   char     name[REED_NAME_LEN];
   uint64_t event_id;  /*!< last vroom event_id for this slot */
};

/** \brief PIR sensor slot — packed wire format. */
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

/** \brief Temperature sensor slot — packed wire format. */
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
 * \brief Sensor data pipe wire format — written by server, read by controller.
 *
 * \warning Layout must match beaglebone/controller/include/sensor_types.h
 *          SensorData exactly. Any field change requires recompiling both.
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
      char room_name[ROOM_NAME_LEN];
      char state[ROOM_STATE_LEN];
      char location[ROOM_LOC_LEN];
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
   struct PirSlotData  pir_slots[MAX_PIRS];
   uint8_t             pir_count;
   struct TempSlotData temp_slots[MAX_TEMPS];
   uint8_t             temp_count;

   struct DoorbellSlotData doorbell_slots[MAX_DOORBELL_CAMS];
   uint8_t                 doorbell_count;

   struct CamSlotData cam_slots[MAX_CAMS];

   uint64_t lock_event_id;   /*!< last vroom event_id for BLE lock  */
   uint64_t light_event_id;  /*!< last vroom event_id for BLE light */
};

#endif /* SENSOR_TYPES_H */
