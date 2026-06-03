/******************************************************************************
 * \file sensor_types.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-05-10
 *
 * \brief Shared wire format structs for sensor server.
 *
 * \details Extracted from sensor_server.c (2026-05-10) so that
 *          json_parser.c and pipe_writer.c can both include SensorData
 *          and ReedSlotData without depending on sensor_server.c.
 *
 * \warning SensorData and ReedSlotData must be byte-for-byte identical
 *          with controller_internal.h. Both files must be updated
 *          together if the layout changes.
 ******************************************************************************/

#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#include <stdint.h>

#define MAX_TEMPS      4   /**< must match ESP32 and controller */
#define TEMP_NAME_LEN  32  /**< temp sensor BLE name buffer size */
#define MAX_ROOMS      10  /**< max room sensors per frame */
#define MAX_REEDS      6   /**< must match ESP32 and controller */
#define MAX_PIRS       5   /**< must match ESP32 and controller */
#define REED_NAME_LEN  16  /**< reed BLE name buffer size */
#define ROOM_NAME_LEN  32  /**< room name buffer size */
#define ROOM_STATE_LEN 16  /**< room state buffer size */
#define ROOM_LOC_LEN   32  /**< room location buffer size */

#define AGE_MAX     0xFFFEu /**< maximum reportable age */
#define AGE_UNKNOWN 0xFFFFu /**< age sentinel: never seen */

/**
 * \brief Reed sensor slot -- packed wire format.
 *
 * \warning Must match controller_internal.h ReedSlotData exactly.
 */
struct __attribute__((packed)) ReedSlotData
{
   uint16_t age;                 /*!< seconds since last adv, 0xFFFF=never */
   int8_t   batt;                /*!< battery SOC percent, -1=unknown */
   uint8_t  active;              /*!< 1=slot occupied (ACTIVE or OFFLINE) */
   uint8_t  state;               /*!< 0=closed 1=open 0xFF=unknown */
   uint8_t  offline;             /*!< 1=SLOT_OFFLINE, red dot, tile stays */
   uint16_t gen;                 /*!< generation counter, increments on swap */
   char     name[REED_NAME_LEN]; /*!< BLE advertised name */
};

/**
 * \brief PIR sensor slot -- packed wire format.
 *
 * \warning Must match controller_internal.h PirSlotData exactly.
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
 * \brief Temperature sensor slot -- packed wire format.
 *
 * \warning Must match controller_internal.h TempSlotData exactly.
 */
struct __attribute__((packed)) TempSlotData
{
   int16_t  temp_decidegc; /*!< temperature in tenths of °C      */
   uint16_t age;           /*!< seconds since last adv           */
   int8_t   batt;          /*!< battery SOC percent, -1=unknown  */
   uint8_t  active;        /*!< 1=slot occupied                  */
   uint8_t  offline;       /*!< age > TEMP_OFFLINE_S             */
   uint8_t  _pad;          /*!< padding for alignment            */
   char     name[TEMP_NAME_LEN]; /*!< BLE advertised name        */
};

/**
 * \brief Sensor data pipe wire format.
 *
 * \warning Must match controller_internal.h SensorData exactly.
 */
struct SensorData
{
   double   avg_temp;     /*!< average temperature in Celsius */
   int      motion_count; /*!< PIR motion event count */
   int      light_state;  /*!< smart light relay state */
   int      lock_state;   /*!< smart lock state */
   long     timestamp;    /*!< Unix timestamp of reading */
   int      room_count;   /*!< number of valid room entries */

   struct
   {
      int  sensor_id;                /*!< room sensor identifier */
      char room_name[ROOM_NAME_LEN]; /*!< room name string */
      char state[ROOM_STATE_LEN];    /*!< current state string */
      char location[ROOM_LOC_LEN];   /*!< physical location string */
   } rooms[MAX_ROOMS];

   uint16_t age_pir;      /*!< PIR device age seconds (legacy flat) */
   uint16_t age_lgt;      /*!< light device age seconds */
   uint16_t age_lck;      /*!< lock device age seconds */
   int8_t   batt_pir;     /*!< PIR battery SOC percent (legacy flat) */
   int8_t   pir_occupied; /*!< 1=occupied, 0=empty */
   int8_t   batt_lck;     /*!< lock battery SOC percent */
   int      batt_motor;   /*!< motor battery SOC percent */

   struct ReedSlotData reed_slots[MAX_REEDS]; /*!< dynamic reed slot array */
   uint8_t  motor_online; /*!< 1 if C3 motor controller is online */

   struct PirSlotData pir_slots[MAX_PIRS]; /*!< dynamic PIR slot array */
   uint8_t            pir_count;           /*!< number of active PIR slots */

   struct TempSlotData temp_slots[MAX_TEMPS]; /*!< dynamic temp slot array */
   uint8_t             temp_count;            /*!< number of active temp slots */
};

#endif /* SENSOR_TYPES_H */
