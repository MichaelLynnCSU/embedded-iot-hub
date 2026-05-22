/******************************************************************************
 * \file vroom_bus.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Vroom event bus implementation for ESP32 hub node.
 *
 * \details Provides a publish/subscribe event bus connecting BLE scan,
 *          UART, and TCP/AWS consumer tasks. Each subscriber owns a
 *          private mailbox set (one depth-1 queue per device type).
 *          Publishers fan out payloads to all interested subscriber
 *          mailboxes at publish time via xQueueOverwrite, then signal
 *          only the subscribers that registered interest in that bit.
 *
 *          Design principles:
 *          - Producers never block — xQueueOverwrite always succeeds
 *          - Latest value wins — depth-1 mailbox holds most recent payload
 *          - No shared queues — each subscriber owns its mailboxes
 *          - No races — fanout happens at publish time, not read time
 *          - Targeted wakeup — subscribers only wake for their mask bits
 *          - Subscriber registration is single-threaded at init time
 *          - No mutex needed on sub_count — registration before tasks start
 *
 *          Call order:
 *          1. bus_init()                — initialise bus (app_main)
 *          2. bus_register_subscriber() — register consumers (pre-task)
 *          3. bus_publish_*()           — called by producers at runtime
 *
 * \note    Mailbox refactor (2026-05-05):
 *          Replaced single shared queues (q_pir, q_reed, etc.) with
 *          per-subscriber private mailboxes. xQueueSend + drop-and-log
 *          replaced by xQueueOverwrite — publishers never drop, never
 *          block. bus_signal() now checks subscriber mask before setting
 *          bits so idle tasks are not woken unnecessarily.
 ******************************************************************************/

#include "vroom_bus.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "VROOM_BUS"; /**< ESP log tag */

static BUS_SUBSCRIBER_T g_subscribers[BUS_MAX_SUBSCRIBERS]; /**< subscriber table */
static int              g_sub_count = 0;                    /**< registered count  */

/******************************************************************************
 * \brief Register a new bus subscriber with an interest mask.
 *
 * \param mask - EVT_* bits this subscriber wants to be woken for.
 *               Use EVT_ALL_MASK to receive every event.
 *
 * \return BUS_SUBSCRIBER_T - Subscriber handle. Check .events != NULL
 *         before use — NULL indicates the subscriber table is full.
 *
 * \warning Not thread-safe. Call only from app_main before tasks start.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
BUS_SUBSCRIBER_T bus_register_subscriber(EventBits_t mask)
{
   BUS_SUBSCRIBER_T sub = {0}; /**< zeroed subscriber handle */

   if (g_sub_count >= BUS_MAX_SUBSCRIBERS)
   {
      ESP_LOGE(TAG, "Subscriber table full (max=%d)", BUS_MAX_SUBSCRIBERS);
      return sub;
   }

   sub.events   = xEventGroupCreate();
   sub.mask     = mask;
   sub.mb_pir   = xQueueCreate(1, sizeof(PIR_PAYLOAD_T));
   sub.mb_reed  = xQueueCreate(1, sizeof(REED_PAYLOAD_T));
   sub.mb_lock  = xQueueCreate(1, sizeof(LOCK_PAYLOAD_T));
   sub.mb_light = xQueueCreate(1, sizeof(LIGHT_PAYLOAD_T));
   sub.mb_temp  = xQueueCreate(1, sizeof(TEMP_PAYLOAD_T));
   sub.mb_motor = xQueueCreate(1, sizeof(MOTOR_PAYLOAD_T));

   if ((NULL == sub.events)   || (NULL == sub.mb_pir)  ||
       (NULL == sub.mb_reed)  || (NULL == sub.mb_lock) ||
       (NULL == sub.mb_light) || (NULL == sub.mb_temp) ||
       (NULL == sub.mb_motor))
   {
      ESP_LOGE(TAG, "Subscriber mailbox alloc failed — check heap");
      return sub;
   }

   g_subscribers[g_sub_count] = sub;
   g_sub_count++;

   ESP_LOGI(TAG, "Subscriber registered (%d/%d) mask=0x%03lX",
            g_sub_count, BUS_MAX_SUBSCRIBERS, (unsigned long)mask);

   return sub;
}

/******************************************************************************
 * \brief Initialize the vroom bus.
 *
 * \return void
 *
 * \details Must be called once from app_main before any publish or
 *          subscriber registration.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void bus_init(void)
{
   ESP_LOGI(TAG, "Bus initialized");
}

/******************************************************************************
 * \brief Signal subscribers that registered interest in these bits.
 *
 * \param bits - Event bits to fan out.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void bus_signal(EventBits_t bits)
{
   int i = 0; /**< loop index */

   for (i = 0; i < g_sub_count; i++)
   {
      if (0 != (g_subscribers[i].mask & bits))
      {
         (void)xEventGroupSetBits(g_subscribers[i].events, bits);
      }
   }
}

/******************************************************************************
 * \brief Publish a PIR motion event to the bus.
 *
 * \param count - Id
 * \param count - Motion event count.
 * \param batt  - PIR battery SOC percent.
 *
 * \return void
 *
 * \details Writes payload to every interested subscriber's private
 *          mailbox via xQueueOverwrite — always succeeds, latest value
 *          wins. Signals only subscribers with EVT_BLE_PIR in their mask.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void bus_publish_pir(uint8_t slot, uint32_t count, int batt)
{
   PIR_PAYLOAD_T p = {.id = slot, .count = count, .batt = batt }; /**< PIR payload */
   int           i = 0;                                 /**< loop index  */

   for (i = 0; i < g_sub_count; i++)
   {
      if (0 != (g_subscribers[i].mask & EVT_BLE_PIR))
      {
         (void)xQueueOverwrite(g_subscribers[i].mb_pir, &p);
      }
   }
   bus_signal(EVT_BLE_PIR);
}

/******************************************************************************
 * \brief Publish a reed sensor event to the bus.
 *
 * \param id    - Reed sensor slot ID (1-based).
 * \param state - Door state (0=closed, 1=open, 0xFF=unknown).
 * \param batt  - Battery SOC percent.
 * \param p_mac - Pointer to 6-byte MAC address, or NULL.
 *
 * \return void
 *
 * \details Writes payload to every interested subscriber's private
 *          mailbox via xQueueOverwrite — always succeeds, latest value
 *          wins. Signals only subscribers with EVT_BLE_REED in their mask.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void bus_publish_reed(uint8_t id, uint8_t state, int batt, const uint8_t *p_mac)
{
   REED_PAYLOAD_T p = { .id = id, .state = state, .batt = batt }; /**< reed payload */
   int            i = 0;                                           /**< loop index   */

   if (NULL != p_mac)
   {
      (void)memcpy(p.mac, p_mac, 6);
   }

   for (i = 0; i < g_sub_count; i++)
   {
      if (0 != (g_subscribers[i].mask & EVT_BLE_REED))
      {
         (void)xQueueOverwrite(g_subscribers[i].mb_reed, &p);
      }
   }
   bus_signal(EVT_BLE_REED);
}

/******************************************************************************
 * \brief Publish a smart lock event to the bus.
 *
 * \param state - Lock state.
 * \param batt  - Battery SOC percent.
 *
 * \return void
 *
 * \details Writes payload to every interested subscriber's private
 *          mailbox via xQueueOverwrite — always succeeds, latest value
 *          wins. Signals only subscribers with EVT_BLE_LOCK in their mask.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void bus_publish_lock(uint8_t state, int batt)
{
   LOCK_PAYLOAD_T p = { .state = state, .batt = batt }; /**< lock payload */
   int            i = 0;                                 /**< loop index   */

   for (i = 0; i < g_sub_count; i++)
   {
      if (0 != (g_subscribers[i].mask & EVT_BLE_LOCK))
      {
         (void)xQueueOverwrite(g_subscribers[i].mb_lock, &p);
      }
   }
   bus_signal(EVT_BLE_LOCK);
}

/******************************************************************************
 * \brief Publish a smart light event to the bus.
 *
 * \param state - Light relay state.
 *
 * \return void
 *
 * \details Writes payload to every interested subscriber's private
 *          mailbox via xQueueOverwrite — always succeeds, latest value
 *          wins. Signals only subscribers with EVT_BLE_LIGHT in their mask.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void bus_publish_light(uint8_t state)
{
   LIGHT_PAYLOAD_T p = { .state = state }; /**< light payload */
   int             i = 0;                  /**< loop index    */

   for (i = 0; i < g_sub_count; i++)
   {
      if (0 != (g_subscribers[i].mask & EVT_BLE_LIGHT))
      {
         (void)xQueueOverwrite(g_subscribers[i].mb_light, &p);
      }
   }
   bus_signal(EVT_BLE_LIGHT);
}

/******************************************************************************
 * \brief Publish a UART temperature event to the bus.
 *
 * \param avg_temp - Average temperature in Celsius from STM32.
 *
 * \return void
 *
 * \details Writes payload to every interested subscriber's private
 *          mailbox via xQueueOverwrite — always succeeds, latest value
 *          wins. Signals only subscribers with EVT_UART_TEMP in their mask.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void bus_publish_temp(int avg_temp)
{
   TEMP_PAYLOAD_T p = { .avg_temp = avg_temp }; /**< temp payload */
   int            i = 0;                         /**< loop index   */

   for (i = 0; i < g_sub_count; i++)
   {
      if (0 != (g_subscribers[i].mask & EVT_UART_TEMP))
      {
         (void)xQueueOverwrite(g_subscribers[i].mb_temp, &p);
      }
   }
   bus_signal(EVT_UART_TEMP);
}

/******************************************************************************
 * \brief Publish a motor controller status event to the bus.
 *
 * \param online - 1 if motor controller is connected, 0 if offline.
 * \param batt   - Supply voltage in mV, -1 if unknown or offline.
 *
 * \return void
 *
 * \details Writes payload to every interested subscriber's private
 *          mailbox via xQueueOverwrite — always succeeds, latest value
 *          wins. Signals only subscribers with EVT_MOTOR_STATUS in mask.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void bus_publish_motor(uint8_t online, int batt)
{
   MOTOR_PAYLOAD_T p = { .online = online, .batt = batt }; /**< motor payload */
   int             i = 0;                                   /**< loop index    */

   for (i = 0; i < g_sub_count; i++)
   {
      if (0 != (g_subscribers[i].mask & EVT_MOTOR_STATUS))
      {
         (void)xQueueOverwrite(g_subscribers[i].mb_motor, &p);
      }
   }
   bus_signal(EVT_MOTOR_STATUS);
}
