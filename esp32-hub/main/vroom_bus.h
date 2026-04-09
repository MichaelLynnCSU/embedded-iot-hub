/******************************************************************************
 * \file vroom_bus.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Vroom event bus public interface for ESP32 hub node.
 *
 * \details Publish/subscribe event bus connecting BLE scan, UART, and
 *          TCP/AWS consumer tasks. Each sensor type has a dedicated
 *          FreeRTOS queue and a corresponding event bit doorbell.
 *
 *          Pattern: producer pushes payload to queue, then signals all
 *          subscriber event groups. Consumers wait on their own event
 *          group then drain the queue.
 *
 *          Call order:
 *          1. bus_init()                — create all queues (app_main)
 *          2. bus_register_subscriber() — register consumers (pre-task)
 *          3. bus_publish_*()           — called by producers at runtime
 *
 * \note    Motor battery field added (2026-04-08):
 *          MOTOR_PAYLOAD_T gains batt field (motor supply voltage in mV).
 *          bus_publish_motor() gains batt parameter to match.
 *          All callers updated — bus_publish_motor(0) becomes
 *          bus_publish_motor(0, -1) on disconnect paths.
 ******************************************************************************/

#ifndef INCLUDE_VROOM_BUS_H_
#define INCLUDE_VROOM_BUS_H_

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

/******************************** CONSTANTS ***********************************/

/** \brief Event bits — one per device/transport type */
#define EVT_BLE_PIR          (1 << 0) /**< PIR motion event */
#define EVT_BLE_REED         (1 << 1) /**< reed sensor event */
#define EVT_BLE_LOCK         (1 << 2) /**< smart lock event */
#define EVT_BLE_LIGHT        (1 << 3) /**< smart light event */
#define EVT_UART_TEMP        (1 << 4) /**< UART temperature event */
#define EVT_MOTOR_STATUS     (1 << 5) /**< motor controller status event */
#define EVT_TCP_CONNECTED    (1 << 6) /**< TCP connection established */
#define EVT_TCP_DISCONNECTED (1 << 7) /**< TCP connection lost */
#define EVT_AWS_CONNECTED    (1 << 8) /**< AWS connection established */
#define EVT_ALL_MASK         (0x1FF)  /**< mask of all event bits */

/** \brief Queue depths — sized for burst scenarios */
#define Q_DEPTH_PIR          4  /**< PIR queue depth */
#define Q_DEPTH_REED         8  /**< reed queue depth (two reeds bursting) */
#define Q_DEPTH_LOCK         4  /**< lock queue depth */
#define Q_DEPTH_LIGHT        4  /**< light queue depth */
#define Q_DEPTH_TEMP         4  /**< temperature queue depth */
#define Q_DEPTH_MOTOR        4  /**< motor queue depth */

/** \brief Maximum number of bus subscribers */
#define BUS_MAX_SUBSCRIBERS  4  /**< increase if more consumers are added */

/******************************* ENUMERATIONS *********************************/

/************************ STRUCTURE/UNION DATA TYPES **************************/

/** \brief PIR motion sensor bus payload. */
typedef struct
{
   uint32_t count; /*!< motion event count */
   int      batt;  /*!< battery SOC percent */
} PIR_PAYLOAD_T;

/** \brief Reed sensor bus payload. */
typedef struct
{
   uint8_t id;      /*!< slot ID 1=Reed1, 2=Reed2, ... */
   uint8_t state;   /*!< 0=closed, 1=open, 0xFF=unknown */
   int     batt;    /*!< battery SOC percent */
   uint8_t mac[6];  /*!< device MAC address */
} REED_PAYLOAD_T;

/** \brief Smart lock bus payload. */
typedef struct
{
   uint8_t state; /*!< lock state */
   int     batt;  /*!< battery SOC percent */
} LOCK_PAYLOAD_T;

/** \brief Smart light bus payload. */
typedef struct
{
   uint8_t state; /*!< relay state */
} LIGHT_PAYLOAD_T;

/** \brief UART temperature bus payload. */
typedef struct
{
   int avg_temp; /*!< average temperature in Celsius */
} TEMP_PAYLOAD_T;

/** \brief Motor controller status bus payload. */
typedef struct
{
   uint8_t online; /*!< 1=connected, 0=offline */
   int     batt;   /*!< supply voltage in mV, -1 if unknown */
} MOTOR_PAYLOAD_T;

/*************************** QUEUE HANDLES ************************************/

extern QueueHandle_t q_pir;   /**< PIR motion event queue */
extern QueueHandle_t q_reed;  /**< reed sensor event queue */
extern QueueHandle_t q_lock;  /**< smart lock event queue */
extern QueueHandle_t q_light; /**< smart light event queue */
extern QueueHandle_t q_temp;  /**< UART temperature event queue */
extern QueueHandle_t q_motor; /**< motor status event queue */

/*************************** FUNCTION PROTOTYPES *****************************/

/** \brief Initialize the vroom bus — create all queues.
 *  \return void
 *  \warning Call once from app_main before any tasks start. */
void bus_init(void);

/** \brief Register a new bus subscriber.
 *  \return EventGroupHandle_t - Subscriber event group, NULL if table full.
 *  \warning Not thread-safe. Call only from app_main before tasks start. */
EventGroupHandle_t bus_register_subscriber(void);

/** \brief Publish a PIR motion event.
 *  \param count - Motion event count.
 *  \param batt  - Battery SOC percent.
 *  \return void */
void bus_publish_pir(uint32_t count, int batt);

/** \brief Publish a reed sensor event.
 *  \param id    - Reed slot ID (1-based).
 *  \param state - Door state (0=closed, 1=open, 0xFF=unknown).
 *  \param batt  - Battery SOC percent.
 *  \param p_mac - Pointer to 6-byte MAC address, or NULL.
 *  \return void */
void bus_publish_reed(uint8_t id,
                      uint8_t state,
                      int batt,
                      const uint8_t *p_mac);

/** \brief Publish a smart lock event.
 *  \param state - Lock state.
 *  \param batt  - Battery SOC percent.
 *  \return void */
void bus_publish_lock(uint8_t state, int batt);

/** \brief Publish a smart light event.
 *  \param state - Light relay state.
 *  \return void */
void bus_publish_light(uint8_t state);

/** \brief Publish a UART temperature event.
 *  \param avg_temp - Average temperature in Celsius.
 *  \return void */
void bus_publish_temp(int avg_temp);

/** \brief Publish a motor controller status event.
 *  \param online - 1 if connected, 0 if offline.
 *  \param batt   - Supply voltage in mV, -1 if unknown or offline.
 *  \return void */
void bus_publish_motor(uint8_t online, int batt);

#endif /* INCLUDE_VROOM_BUS_H_ */
