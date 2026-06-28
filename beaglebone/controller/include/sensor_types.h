#ifndef INCLUDE_SENSOR_TYPES_H_
#define INCLUDE_SENSOR_TYPES_H_

#include <stdint.h>
#include <pthread.h>
#include "config.h"
#include "../../include/ipc_proto.h"

typedef enum
{
   DEV_PIR   = 0,
   DEV_LIGHT = 1,
   DEV_LOCK  = 2,
   DEV_MOTOR = 3,
   DEV_LCD   = 4,
   DEV_COUNT = 5
} DEV_ID_E;

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

typedef struct
{
   DEV_ID_E idx;
   int      val;
   int      batt;
} UartFrame;

typedef struct
{
   UartFrame         frames[UART_RING_SIZE];
   volatile unsigned head;
   volatile unsigned tail;
   pthread_mutex_t   mutex;
} uart_ring_t;

#endif /* INCLUDE_SENSOR_TYPES_H_ */
