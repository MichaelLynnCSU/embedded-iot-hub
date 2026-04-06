/******************************************************************************
 * \file vroom_bus.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Vroom event bus implementation for ESP32 hub node.
 *
 * \details Provides a publish/subscribe event bus connecting BLE scan,
 *          UART, and TCP/AWS consumer tasks. Each sensor type has a
 *          dedicated FreeRTOS queue. Subscribers receive event group
 *          bits on each publish.
 *
 *          Design principles:
 *          - Producers never block — xQueueSend with timeout=0
 *          - Full queues drop and log — BLE scan callbacks never stall
 *          - Subscriber registration is single-threaded at init time
 *          - No mutex needed on sub_count — registration before tasks start
 *
 *          Call order:
 *          1. bus_init()               — create all queues
 *          2. bus_register_subscriber() — register consumers (pre-task)
 *          3. bus_publish_*()          — called by producers at runtime
 ******************************************************************************/

#include "vroom_bus.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "VROOM_BUS"; /**< ESP log tag */

QueueHandle_t q_pir;   /**< PIR motion event queue */
QueueHandle_t q_reed;  /**< reed sensor event queue */
QueueHandle_t q_lock;  /**< smart lock event queue */
QueueHandle_t q_light; /**< smart light event queue */
QueueHandle_t q_temp;  /**< UART temperature event queue */
QueueHandle_t q_motor; /**< motor status event queue */

static EventGroupHandle_t g_subscribers[BUS_MAX_SUBSCRIBERS]; /**< subscriber table */
static int                g_sub_count = 0;                    /**< registered count */

/******************************************************************************
 * \brief Register a new bus subscriber and return its event group handle.
 *
 * \return EventGroupHandle_t - Subscriber event group, or NULL if table full.
 *
 * \warning Not thread-safe. Call only from app_main before tasks start.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
EventGroupHandle_t bus_register_subscriber(void)
{
   EventGroupHandle_t grp = NULL; /**< new subscriber event group */

   if (g_sub_count >= BUS_MAX_SUBSCRIBERS)
   {
      ESP_LOGE(TAG, "Subscriber table full (max=%d)", BUS_MAX_SUBSCRIBERS);
      return NULL;
   }

   grp = xEventGroupCreate();
   g_subscribers[g_sub_count] = grp;
   g_sub_count++;

   ESP_LOGI(TAG, "Subscriber registered (%d/%d)",
            g_sub_count, BUS_MAX_SUBSCRIBERS);

   return grp;
}

/******************************************************************************
 * \brief Fan out event bits to all registered subscribers.
 *
 * \param bits - Event bits to set on each subscriber event group.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void bus_signal(uint32_t bits)
{
   int i = 0; /**< loop index */

   for (i = 0; i < g_sub_count; i++)
   {
      (void)xEventGroupSetBits(g_subscribers[i], bits);
   }
}

/******************************************************************************
 * \brief Initialize the vroom bus — create all queues.
 *
 * \return void
 *
 * \details Must be called once from app_main before any publish or
 *          subscriber registration. Logs error if any queue fails.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void bus_init(void)
{
   q_pir   = xQueueCreate(Q_DEPTH_PIR,   sizeof(PIR_PAYLOAD_T));
   q_reed  = xQueueCreate(Q_DEPTH_REED,  sizeof(REED_PAYLOAD_T));
   q_lock  = xQueueCreate(Q_DEPTH_LOCK,  sizeof(LOCK_PAYLOAD_T));
   q_light = xQueueCreate(Q_DEPTH_LIGHT, sizeof(LIGHT_PAYLOAD_T));
   q_temp  = xQueueCreate(Q_DEPTH_TEMP,  sizeof(TEMP_PAYLOAD_T));
   q_motor = xQueueCreate(Q_DEPTH_MOTOR, sizeof(MOTOR_PAYLOAD_T));

   if ((NULL == q_pir)   || (NULL == q_reed)  ||
       (NULL == q_lock)  || (NULL == q_light) ||
       (NULL == q_temp)  || (NULL == q_motor))
   {
      ESP_LOGE(TAG, "Queue creation failed — check heap");
   }
   else
   {
      ESP_LOGI(TAG, "Bus initialized, all queues ready");
   }
}

/******************************************************************************
 * \brief Publish a PIR motion event to the bus.
 *
 * \param count - Motion event count.
 * \param batt  - PIR battery SOC percent.
 *
 * \return void
 *
 * \details Drops event and logs warning if queue is full.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void bus_publish_pir(uint32_t count, int batt)
{
   PIR_PAYLOAD_T p = { .count = count, .batt = batt }; /**< PIR payload */

   if (pdTRUE != xQueueSend(q_pir, &p, 0))
   {
      ESP_LOGW(TAG, "[PIR] Queue full, dropping event");
   }
   else
   {
      bus_signal(EVT_BLE_PIR);
   }
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
 * \details Drops event and logs warning if queue is full.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void bus_publish_reed(uint8_t id, uint8_t state, int batt, const uint8_t *p_mac)
{
   REED_PAYLOAD_T p = { .id = id, .state = state, .batt = batt }; /**< reed payload */

   if (NULL != p_mac)
   {
      (void)memcpy(p.mac, p_mac, 6);
   }

   if (pdTRUE != xQueueSend(q_reed, &p, 0))
   {
      ESP_LOGW(TAG, "[REED] Queue full, dropping event");
   }
   else
   {
      bus_signal(EVT_BLE_REED);
   }
}

/******************************************************************************
 * \brief Publish a smart lock event to the bus.
 *
 * \param state - Lock state.
 * \param batt  - Battery SOC percent.
 *
 * \return void
 *
 * \details Drops event and logs warning if queue is full.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void bus_publish_lock(uint8_t state, int batt)
{
   LOCK_PAYLOAD_T p = { .state = state, .batt = batt }; /**< lock payload */

   if (pdTRUE != xQueueSend(q_lock, &p, 0))
   {
      ESP_LOGW(TAG, "[LOCK] Queue full, dropping event");
   }
   else
   {
      bus_signal(EVT_BLE_LOCK);
   }
}

/******************************************************************************
 * \brief Publish a smart light event to the bus.
 *
 * \param state - Light relay state.
 *
 * \return void
 *
 * \details Drops event and logs warning if queue is full.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void bus_publish_light(uint8_t state)
{
   LIGHT_PAYLOAD_T p = { .state = state }; /**< light payload */

   if (pdTRUE != xQueueSend(q_light, &p, 0))
   {
      ESP_LOGW(TAG, "[LIGHT] Queue full, dropping event");
   }
   else
   {
      bus_signal(EVT_BLE_LIGHT);
   }
}

/******************************************************************************
 * \brief Publish a UART temperature event to the bus.
 *
 * \param avg_temp - Average temperature in Celsius from STM32.
 *
 * \return void
 *
 * \details Drops event and logs warning if queue is full.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void bus_publish_temp(int avg_temp)
{
   TEMP_PAYLOAD_T p = { .avg_temp = avg_temp }; /**< temp payload */

   if (pdTRUE != xQueueSend(q_temp, &p, 0))
   {
      ESP_LOGW(TAG, "[TEMP] Queue full, dropping event");
   }
   else
   {
      bus_signal(EVT_UART_TEMP);
   }
}

/******************************************************************************
 * \brief Publish a motor controller status event to the bus.
 *
 * \param online - 1 if motor controller is connected, 0 if offline.
 *
 * \return void
 *
 * \details Drops event and logs warning if queue is full.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void bus_publish_motor(uint8_t online)
{
   MOTOR_PAYLOAD_T p = { .online = online }; /**< motor payload */

   if (pdTRUE != xQueueSend(q_motor, &p, 0))
   {
      ESP_LOGW(TAG, "[MOTOR] Queue full, dropping event");
   }
   else
   {
      bus_signal(EVT_MOTOR_STATUS);
   }
}
