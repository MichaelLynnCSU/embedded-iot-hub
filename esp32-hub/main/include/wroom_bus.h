/******************************************************************************
 * \file wroom_bus.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Vroom event bus public interface for ESP32 hub node.
 *
 * \details Publish/subscribe event bus connecting BLE scan, UART, and
 *          TCP/AWS consumer tasks. Each sensor type has a dedicated
 *          per-subscriber private mailbox (depth-1 queue). Publishers
 *          fan out payloads to all subscriber mailboxes at publish time,
 *          then signal only the subscribers that registered interest in
 *          that event bit.
 *
 *          Pattern: producer calls bus_publish_*() which writes payload
 *          to every interested subscriber's private mailbox, then sets
 *          the matching event bit on those subscriber event groups only.
 *          Consumers wait on their own event group then read from their
 *          own mailboxes — no shared queues, no races.
 *
 *          Call order:
 *          1. bus_init()                — initialise bus (app_main)
 *          2. bus_register_subscriber() — register consumers (pre-task)
 *          3. bus_publish_*()           — called by producers at runtime
 *
 * \note    Mailbox refactor (2026-05-05):
 *          Replaced single shared queues with per-subscriber private
 *          mailboxes (depth 1, xQueueOverwrite). Eliminates the race
 *          condition where tcp_manager and device_gateway competed for the
 *          same queue item. bus_register_subscriber() now takes an
 *          EventBits_t mask so each subscriber is only woken for the
 *          bits it cares about. Return type changed from
 *          EventGroupHandle_t to BUS_SUBSCRIBER_T.
 *          Callers in main.c updated accordingly.
 *
 * \note    BLE temp sensor (2026-06-02):
 *          EVT_BLE_TEMP event bit added. BLE_TEMP_PAYLOAD_T added.
 *          mb_ble_temp mailbox added to BUS_SUBSCRIBER_T.
 *          bus_publish_ble_temp() added. EVT_ALL_MASK widened to 0x7FF.
 *
 * \note    Structured event tracing — Phase 0 (2026-06-15):
 *          Normalized log prefixes across all subsystems:
 *          [BLE_PIR], [BLE_REED], [BLE_TEMP], [BLE_LOCK], [BLE_LIGHT],
 *          [WROOM], [TCP], [UART]. No struct or ABI changes.
 *
 * \note    Structured event tracing — Phase 1 (2026-06-15):
 *          Local sequence counters added. g_bus_seq in wroom_bus.c
 *          increments on every publish. g_frame_seq in uart_manager.c
 *          increments on every parsed UART frame. No cross-module
 *          propagation. No struct or ABI changes.
 *
 * \note    Structured event tracing — Phase 2 (2026-06-15):
 *          event_id generator introduced inside wroom_bus.c only.
 *          wroom_event_id_generate() is static — not exported.
 *          event_id logged at [WROOM] ingest point only.
 *          No external ABI changes. BLE/TCP/UART unaware.
 *
 * \note    Structured event tracing — Phase 3 (2026-06-15):
 *          BLE publish functions now return uint64_t event_id so BLE
 *          modules can log the correlation key at ingress.
 *          bus_publish_pir(), bus_publish_reed(), bus_publish_ble_temp(),
 *          bus_publish_lock(), bus_publish_light() return uint64_t.
 *          bus_publish_temp(), bus_publish_motor(), bus_publish_doorbell()
 *          remain void — UART and doorbell not in this phase.
 *          TCP/UART still unaware of event_id. No struct changes.
 ******************************************************************************/

#ifndef INCLUDE_WROOM_BUS_H_
#define INCLUDE_WROOM_BUS_H_

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

/******************************** CONSTANTS ***********************************/

/** \brief Event bits — one per device/transport type */
#define EVT_BLE_PIR          (1 << 0) /**< PIR motion event          */
#define EVT_BLE_REED         (1 << 1) /**< reed sensor event         */
#define EVT_BLE_LOCK         (1 << 2) /**< smart lock event          */
#define EVT_BLE_LIGHT        (1 << 3) /**< smart light event         */
#define EVT_UART_TEMP        (1 << 4) /**< UART temperature event    */
#define EVT_MOTOR_STATUS     (1 << 5) /**< motor controller event    */
#define EVT_TCP_CONNECTED    (1 << 6) /**< TCP connection established */
#define EVT_TCP_DISCONNECTED (1 << 7) /**< TCP connection lost        */
#define EVT_AWS_CONNECTED    (1 << 8) /**< AWS connection established */
#define EVT_BLE_TEMP         (1 << 9) /**< BLE temperature sensor event */
#define EVT_DOORBELL         (1 << 10) /**< doorbell press event      */
#define EVT_ALL_MASK         (0x7FF)   /**< widen from 0x3FF          */

/** \brief Maximum number of bus subscribers */
#define BUS_MAX_SUBSCRIBERS  4  /**< increase if more consumers are added */

/************************ STRUCTURE/UNION DATA TYPES **************************/

/** \brief PIR motion sensor bus payload. */
typedef struct
{
   uint8_t  id;     /*!< 1-based PIR slot index  */
   uint8_t  slot;   /*!< 0-based PIR slot index  */
   uint32_t count;  /*!< motion event count      */
   int      batt;   /*!< battery SOC percent     */
   uint64_t event_id;  /* wroom-assigned correlation key */
} PIR_PAYLOAD_T;

/** \brief Reed sensor bus payload. */
typedef struct
{
   uint8_t id;     /*!< slot ID 1=Reed1, 2=Reed2, ... */
   uint8_t state;  /*!< 0=closed, 1=open, 0xFF=unknown */
   int     batt;   /*!< battery SOC percent            */
   uint8_t mac[6]; /*!< device MAC address             */
   uint64_t event_id;  /* wroom-assigned correlation key */
} REED_PAYLOAD_T;

/** \brief Smart lock bus payload. */
typedef struct
{
   uint8_t state; /*!< lock state          */
   int     batt;  /*!< battery SOC percent */
   uint64_t event_id;  /* wroom-assigned correlation key */
} LOCK_PAYLOAD_T;

/** \brief Smart light bus payload. */
typedef struct
{
   uint8_t state; /*!< relay state */
   uint64_t event_id;  /* wroom-assigned correlation key */
} LIGHT_PAYLOAD_T;

/** \brief UART temperature bus payload. */
typedef struct
{
   int avg_temp; /*!< average temperature in Celsius */
   uint64_t event_id;  /* wroom-assigned correlation key */
} TEMP_PAYLOAD_T;

/** \brief BLE temperature sensor bus payload. */
typedef struct
{
   uint8_t  id;            /*!< 1-based temp slot index           */
   int16_t  temp_decidegc; /*!< temperature in tenths of °C       */
   int      batt;          /*!< battery SOC percent               */
   uint64_t event_id;
} BLE_TEMP_PAYLOAD_T;

/** \brief Doorbell press bus payload. */
typedef struct
{
   uint8_t  device_id;    /*!< 0-3, which doorbell cam     */
   uint64_t event_id;     /*!< correlation key for BBB     */
   uint64_t timestamp_ms; /*!< cam timestamp at press      */
} DOORBELL_PAYLOAD_T;

/** \brief Motor controller status bus payload. */
typedef struct
{
   uint8_t online; /*!< 1=connected, 0=offline              */
   int     batt;   /*!< supply voltage in mV, -1 if unknown */
} MOTOR_PAYLOAD_T;

/** \brief Per-subscriber mailbox set.
 *
 *  \details Each subscriber owns a private copy of every mailbox.
 *           Publishers write to all interested subscribers at publish
 *           time via xQueueOverwrite — depth-1 queues always hold the
 *           latest value and never block the publisher.
 *           The mask field controls which event bits wake this subscriber.
 */
typedef struct
{
   EventGroupHandle_t events;      /*!< wakeup signal — doorbell only        */
   EventBits_t        mask;        /*!< bits this subscriber cares about     */
   QueueHandle_t      mb_pir;      /*!< private PIR mailbox      (depth 1)   */
   QueueHandle_t      mb_reed;     /*!< private reed mailbox     (depth 1)   */
   QueueHandle_t      mb_lock;     /*!< private lock mailbox     (depth 1)   */
   QueueHandle_t      mb_light;    /*!< private light mailbox    (depth 1)   */
   QueueHandle_t      mb_temp;     /*!< private UART temp mailbox (depth 1)  */
   QueueHandle_t      mb_motor;    /*!< private motor mailbox    (depth 1)   */
   QueueHandle_t      mb_ble_temp; /*!< private BLE temp mailbox (depth 1)   */
   QueueHandle_t      mb_doorbell; /*!< private doorbell mailbox (depth 1)   */
} BUS_SUBSCRIBER_T;

/*************************** FUNCTION PROTOTYPES *****************************/

/** \brief Initialize the wroom bus.
 *  \return void
 *  \warning Call once from app_main before any tasks start. */
void bus_init(void);

/** \brief Register a new bus subscriber with an interest mask.
 *  \param mask - EVT_* bits this subscriber wants to be woken for.
 *  \return BUS_SUBSCRIBER_T - Subscriber handle. Check .events != NULL.
 *  \warning Not thread-safe. Call only from app_main before tasks start. */
BUS_SUBSCRIBER_T bus_register_subscriber(EventBits_t mask);

/** \brief Publish a PIR motion event.
 *  \param slot  - 1-based PIR slot index.
 *  \param count - Motion event count.
 *  \param batt  - Battery SOC percent.
 *  \return uint64_t - Assigned event_id for caller logging. */
uint64_t bus_publish_pir(uint8_t slot, uint32_t count, int batt);

/** \brief Publish a reed sensor event.
 *  \param id    - Reed slot ID (1-based).
 *  \param state - Door state (0=closed, 1=open, 0xFF=unknown).
 *  \param batt  - Battery SOC percent.
 *  \param p_mac - Pointer to 6-byte MAC address, or NULL.
 *  \return uint64_t - Assigned event_id for caller logging. */
uint64_t bus_publish_reed(uint8_t        id,
                           uint8_t        state,
                           int            batt,
                           const uint8_t *p_mac);

/** \brief Publish a smart lock event.
 *  \param state - Lock state.
 *  \param batt  - Battery SOC percent.
 *  \return uint64_t - Assigned event_id for caller logging. */
uint64_t bus_publish_lock(uint8_t state, int batt);

/** \brief Publish a smart light event.
 *  \param state - Light relay state.
 *  \return uint64_t - Assigned event_id for caller logging. */
uint64_t bus_publish_light(uint8_t state);

/** \brief Publish a UART temperature event.
 *  \param avg_temp - Average temperature in Celsius.
 *  \return void — UART not yet in event_id propagation phase. */
uint64_t bus_publish_temp(int avg_temp);

/** \brief Publish a motor controller status event.
 *  \param online - 1 if connected, 0 if offline.
 *  \param batt   - Supply voltage in mV, -1 if unknown or offline.
 *  \return void — motor not yet in event_id propagation phase. */
uint64_t bus_publish_motor(uint8_t online, int batt);

/** \brief Publish a BLE temperature sensor event.
 *  \param slot          - 1-based temp slot index.
 *  \param temp_decidegc - Temperature in tenths of °C (int16_t).
 *  \param batt          - Battery SOC percent.
 *  \return uint64_t - Assigned event_id for caller logging. */
uint64_t bus_publish_ble_temp(uint8_t slot, int16_t temp_decidegc, int batt);

/** \brief Publish a doorbell press event.
 *  \param device_id    - 0-3, which doorbell cam.
 *  \param event_id     - Correlation key matching TCP JPEG header.
 *  \param timestamp_ms - Cam timestamp at button press.
 *  \return void — doorbell not yet in event_id propagation phase. */
uint64_t bus_publish_doorbell(uint8_t  device_id,
                           uint64_t event_id,
                           uint64_t timestamp_ms);

#endif /* INCLUDE_WROOM_BUS_H_ */
