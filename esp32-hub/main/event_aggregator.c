/******************************************************************************
 * \file event_aggregator.c
 * \brief Structured event log consumer for vroom_bus channels.
 *
 * \note Phase 4 (2026-06-16):
 *       event_id logged at every mailbox drain point.
 *       send_cam_trigger() on PIR 0->1 occupancy transition confirmed here.
 *       pir_window_update() removed — already called at BLE ingestion.
 ******************************************************************************/

#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "vroom_bus.h"
#include "cam_trigger.h"
#include "config.h"

#define AGG_TASK_STACK    4096
#define AGG_TASK_PRIO     5
#define AGG_LOOP_DELAY_MS 50

static const char *TAG = "EVENT_AGG";

static BUS_SUBSCRIBER_T g_sub;
static uint8_t          g_last_pir_occupied[MAX_PIRS] = {0};

static void event_aggregator_task(void *arg)
{
   PIR_PAYLOAD_T      pir      = {0};
   REED_PAYLOAD_T     reed     = {0};
   LOCK_PAYLOAD_T     lock     = {0};
   LIGHT_PAYLOAD_T    light    = {0};
   TEMP_PAYLOAD_T     temp     = {0};
   BLE_TEMP_PAYLOAD_T ble_temp = {0};
   DOORBELL_PAYLOAD_T doorbell = {0};
   MOTOR_PAYLOAD_T    motor    = {0};

   while (1)
   {
      EventBits_t bits = xEventGroupWaitBits(
         g_sub.events, g_sub.mask, pdTRUE, pdFALSE,
         pdMS_TO_TICKS(AGG_LOOP_DELAY_MS)
      );

      if (bits & EVT_BLE_PIR)
      {
         if (pdTRUE == xQueueReceive(g_sub.mb_pir, &pir, 0))
         {
            ESP_LOGI(TAG, "[AGG] event_id=%llu device=PIR slot=%u motion=%lu batt=%d",
                     (unsigned long long)pir.event_id,
                     pir.slot, pir.count, pir.batt);

            /* 0->1 occupancy transition — fire cam trigger */
            if (pir.slot < MAX_PIRS)
            {
               uint8_t occ = (pir.count > 0) ? 1 : 0;
               if ((0 == g_last_pir_occupied[pir.slot]) && (1 == occ))
               {
                  ESP_LOGI(TAG, "[AGG] event_id=%llu PIR slot=%u edge -> cam trigger",
                           (unsigned long long)pir.event_id, pir.slot);
                  send_cam_trigger();
               }
               g_last_pir_occupied[pir.slot] = occ;
            }
         }
      }

      if (bits & EVT_BLE_REED)
      {
         if (pdTRUE == xQueueReceive(g_sub.mb_reed, &reed, 0))
         {
            ESP_LOGI(TAG, "[AGG] event_id=%llu device=REED slot=%u state=%u batt=%d",
                     (unsigned long long)reed.event_id,
                     reed.id, reed.state, reed.batt);
         }
      }

      if (bits & EVT_BLE_LOCK)
      {
         if (pdTRUE == xQueueReceive(g_sub.mb_lock, &lock, 0))
         {
            ESP_LOGI(TAG, "[AGG] event_id=%llu device=LOCK state=%u batt=%d",
                     (unsigned long long)lock.event_id,
                     lock.state, lock.batt);
         }
      }

      if (bits & EVT_BLE_LIGHT)
      {
         if (pdTRUE == xQueueReceive(g_sub.mb_light, &light, 0))
         {
            ESP_LOGI(TAG, "[AGG] event_id=%llu device=LIGHT state=%u",
                     (unsigned long long)light.event_id,
                     light.state);
         }
      }

      if (bits & EVT_UART_TEMP)
      {
         if (pdTRUE == xQueueReceive(g_sub.mb_temp, &temp, 0))
         {
            ESP_LOGI(TAG, "[AGG] device=UART_TEMP avg_temp=%d", temp.avg_temp);
         }
      }

      if (bits & EVT_BLE_TEMP)
      {
         if (pdTRUE == xQueueReceive(g_sub.mb_ble_temp, &ble_temp, 0))
         {
            ESP_LOGI(TAG, "[AGG] event_id=%llu device=BLE_TEMP slot=%u temp_dc=%d batt=%d",
                     (unsigned long long)ble_temp.event_id,
                     ble_temp.id, ble_temp.temp_decidegc, ble_temp.batt);
         }
      }

      if (bits & EVT_DOORBELL)
      {
         if (pdTRUE == xQueueReceive(g_sub.mb_doorbell, &doorbell, 0))
         {
            ESP_LOGI(TAG, "[AGG] event_id=%llu device=DOORBELL cam_event_id=%llu device_id=%u",
                     (unsigned long long)doorbell.event_id,
                     (unsigned long long)doorbell.event_id,
                     doorbell.device_id);
         }
      }

      if (bits & EVT_MOTOR_STATUS)
      {
         if (pdTRUE == xQueueReceive(g_sub.mb_motor, &motor, 0))
         {
            ESP_LOGI(TAG, "[AGG] device=MOTOR online=%u batt=%d",
                     motor.online, motor.batt);
         }
      }
   }
}

void event_aggregator_start(void)
{
   g_sub = bus_register_subscriber(EVT_ALL_MASK);
   memset(g_last_pir_occupied, 0, sizeof(g_last_pir_occupied));
   xTaskCreate(event_aggregator_task, "event_agg", AGG_TASK_STACK, NULL, AGG_TASK_PRIO, NULL);
   ESP_LOGI(TAG, "[VROOM] Event aggregator started");
}
