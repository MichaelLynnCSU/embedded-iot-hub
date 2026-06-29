/******************************************************************************
 * \file sensor_types.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-28
 *
 * \brief Canonical wire-format definitions shared by BBB services.
 *
 * \details This header is the single source of truth for all inter-process
 *          ABI types on the BeagleBone. It is shared by:
 *
 *            - beaglebone/server        (pipe writer)
 *            - beaglebone/camera_manager (pipe writer)
 *            - beaglebone/controller    (pipe reader)
 *            - beaglebone/inference/app (MAX_DOORBELL_CAMS consumer)
 *
 * \warning STABILITY REQUIREMENT:
 *          These structures define the pipe and shared-memory ABI between
 *          processes. Field order, types, packing attributes, array sizes,
 *          and enum values must remain stable across all consumers.
 *          Any change requires recompiling ALL services simultaneously.
 *          Do NOT add helper functions, logging macros, or service-specific
 *          constants here — wire protocol only.
 *
 * \note    Revision history:
 *          2026-05-10  Extracted from sensor_server.c
 *          2026-06-09  DoorbellSlotData added
 *          2026-06-10  CamSlotData added; MAX_CAMS=3
 *          2026-06-16  event_id added to slot types; lock/light event_id
 *                      added to SensorData tail
 *          2026-06-28  Consolidated into single shared header;
 *                      eliminated duplicate definitions across services
 ******************************************************************************/

#ifndef BBB_SHARED_SENSOR_TYPES_H
#define BBB_SHARED_SENSOR_TYPES_H

#include <stdint.h>

/*---------------------------------------------------------------------------*/
/* Array size constants — wire protocol, must match ESP32 and all consumers  */
/*---------------------------------------------------------------------------*/
#define MAX_TEMPS          4    /**< temp sensor slots — must match ESP32 */
#define MAX_REEDS          6    /**< reed sensor slots — must match ESP32 */
#define MAX_PIRS           5    /**< PIR sensor slots  — must match ESP32 */
#define MAX_ROOMS          10   /**< room entries per SensorData frame    */
#define MAX_CAMS           3    /**< inference cameras (indoor/front/back) */
#define MAX_DOORBELL_CAMS  4    /**< doorbell camera slots                */

/*---------------------------------------------------------------------------*/
/* String buffer sizes — canonical names are _SIZE; _LEN aliases provided   */
/* for backward compatibility with existing server code                      */
/*---------------------------------------------------------------------------*/
#define TEMP_NAME_SIZE     32   /**< temp sensor BLE name buffer  */
#define REED_NAME_SIZE     16   /**< reed sensor BLE name buffer  */
#define ROOM_NAME_SIZE     32   /**< room name buffer             */
#define ROOM_STATE_SIZE    16   /**< room state buffer            */
#define ROOM_LOC_SIZE      32   /**< room location buffer         */

/* Backward-compat aliases used by server/sensor_types.h */
#define TEMP_NAME_LEN      TEMP_NAME_SIZE
#define REED_NAME_LEN      REED_NAME_SIZE
#define ROOM_NAME_LEN      ROOM_NAME_SIZE
#define ROOM_STATE_LEN     ROOM_STATE_SIZE
#define ROOM_LOC_LEN       ROOM_LOC_SIZE

/*---------------------------------------------------------------------------*/
/* Age sentinels                                                              */
/*---------------------------------------------------------------------------*/
#define AGE_MAX            0xFFFEu  /**< maximum reportable age (seconds) */
#define AGE_UNKNOWN        0xFFFFu  /**< never seen sentinel              */

/*---------------------------------------------------------------------------*/
/* Wire-format structs — sensor_server → controller pipe                     */
/*---------------------------------------------------------------------------*/

/** \brief Reed sensor slot — packed wire format.
 *  \warning sizeof must equal 32. Verified by _Static_assert below. */
struct __attribute__((packed)) ReedSlotData
{
   uint16_t age;
   int8_t   batt;
   uint8_t  active;
   uint8_t  state;
   uint8_t  offline;
   uint16_t gen;
   char     name[REED_NAME_SIZE];
   uint64_t event_id;   /*!< last wroom event_id for this slot */
};

/** \brief PIR sensor slot — packed wire format.
 *  \warning sizeof must equal 20. Verified by _Static_assert below. */
struct __attribute__((packed)) PirSlotData
{
   uint32_t count;
   uint16_t age;
   int8_t   batt;
   uint8_t  active;
   int8_t   occupied;
   uint8_t  offline;
   uint8_t  _pad[2];    /*!< alignment padding before event_id */
   uint64_t event_id;   /*!< last wroom event_id for this slot */
};

/** \brief Temperature sensor slot — packed wire format.
 *  \warning sizeof must equal 48. Verified by _Static_assert below. */
struct __attribute__((packed)) TempSlotData
{
   int16_t  temp_decidegc;
   uint16_t age;
   int8_t   batt;
   uint8_t  active;
   uint8_t  offline;
   uint8_t  _pad;
   char     name[TEMP_NAME_SIZE];
   uint64_t event_id;   /*!< last wroom event_id for this slot */
};

/*---------------------------------------------------------------------------*/
/* Wire-format structs — camera_manager → controller pipe                    */
/*---------------------------------------------------------------------------*/

/** \brief Camera slot age transport — packed wire format.
 *  \warning sizeof must equal 3. Verified by _Static_assert below.
 *  \note   online/offline derived by consumer from age threshold. */
struct __attribute__((packed)) CamSlotData
{
   uint16_t age_s;      /*!< seconds since last heartbeat, AGE_UNKNOWN=never */
   uint8_t  _pad;
};

/** \brief Doorbell slot age transport — packed wire format.
 *  \warning sizeof must equal 3. Verified by _Static_assert below.
 *  \note   online/offline derived by consumer from age threshold. */
struct __attribute__((packed)) DoorbellSlotData
{
   uint16_t age_s;      /*!< seconds since last heartbeat/press, AGE_UNKNOWN=never */
   uint8_t  _pad;
};

/** \brief Camera manager pipe frame — written by camera_manager, read by controller.
 *  \note  Not packed — contains only packed sub-structs, no padding risk. */
struct CamData
{
   struct CamSlotData      cam_slots[MAX_CAMS];
   struct DoorbellSlotData doorbell_slots[MAX_DOORBELL_CAMS];
};

/*---------------------------------------------------------------------------*/
/* Wire-format struct — sensor_server → controller pipe (full frame)         */
/*---------------------------------------------------------------------------*/

/**
 * \brief Sensor data pipe frame — written by sensor_server, read by controller.
 * \warning Layout must be byte-for-byte identical between writer and reader.
 *          Do not reorder fields, change types, or add fields without
 *          recompiling all consumers.
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

   uint64_t lock_event_id;    /*!< last wroom event_id for BLE lock  */
   uint64_t light_event_id;   /*!< last wroom event_id for BLE light */
   uint32_t frame_seq;        /*!< monotonic frame counter; join key for
                                   telemetry.log and events.log       */
};

/*---------------------------------------------------------------------------*/
/* Compile-time ABI verification                                              */
/* If any of these fire, a field was added, removed, or reordered.           */
/* Update the size constant AND all consumers before committing.             */
/*---------------------------------------------------------------------------*/
_Static_assert(sizeof(struct ReedSlotData)     == 32,
               "ReedSlotData ABI changed — update all pipe consumers");
_Static_assert(sizeof(struct PirSlotData)      == 20,
               "PirSlotData ABI changed — update all pipe consumers");
_Static_assert(sizeof(struct TempSlotData)     == 48,
               "TempSlotData ABI changed — update all pipe consumers");
_Static_assert(sizeof(struct CamSlotData)      == 3,
               "CamSlotData ABI changed — update all pipe consumers");
_Static_assert(sizeof(struct DoorbellSlotData) == 3,
               "DoorbellSlotData ABI changed — update all pipe consumers");

#endif /* BBB_SHARED_SENSOR_TYPES_H */
