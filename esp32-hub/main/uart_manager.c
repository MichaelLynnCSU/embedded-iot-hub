/******************************************************************************
 * \file uart_manager.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief UART manager for ESP32 hub node.
 *
 * \details Receives JSON temperature data from STM32 blue pill over
 *          UART2 at 115200 baud. Parses avg_temp field and publishes
 *          to vroom bus for consumption by TCP and AWS managers.
 *
 *          Message format: {"avg_temp": <int>}\n
 *          High temperature threshold: 100C — logged as EVENT: TEMP_HIGH
 *
 * \note    WDT fix (2026-03-21):
 *          trinity_wdt_kick() added at the top of the main while(1) loop.
 *          The loop already wakes every UART_READ_TIMEOUT_MS (100 ms) on
 *          timeout, so the kick fires well within the 5 s WDT window even
 *          when no UART data arrives.
 *          trinity_log_event() renamed to trinity_log_event() throughout.
 ******************************************************************************/

#include "uart_manager.h"
#include "config.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vroom_bus.h"
#include "trinity_log.h"

#define TEMP_HIGH_THRESHOLD  100  /**< temperature high event threshold C  */
#define UART_READ_TIMEOUT_MS 100  /**< UART byte read timeout ms           */
#define UART_READ_LEN        1    /**< bytes to read per call              */

static const char *TAG        = "UART_MGR";       /**< ESP log tag              */
static int         g_avg_temp = DEFAULT_AVG_TEMP; /**< last received avg temp   */

void uart_manager_init(void)
{
   ESP_LOGI(TAG, "UART manager initialized");
}

void uart_manager_task(void)
{
   uint8_t buf[UART_BUF_SIZE] = {0};
   uint8_t byte               = 0;
   int     pos                = 0;
   int     len                = 0;
   cJSON  *p_json             = NULL;
   cJSON  *p_avg              = NULL;

   ESP_LOGI(TAG, "UART task running");

   while (1)
   {
      /* ---- Trinity: kick WDT every loop iteration (~100 ms timeout) ---- */
      trinity_wdt_kick();

      len = uart_read_bytes(UART_STM32,
                             &byte,
                             UART_READ_LEN,
                             pdMS_TO_TICKS(UART_READ_TIMEOUT_MS));

      if (0 < len)
      {
         if (pos < (UART_BUF_SIZE - 1))
         {
            buf[pos] = byte;
            pos++;
         }

         if ('\n' == byte)
         {
            buf[pos] = 0;

            p_json = cJSON_Parse((char *)buf);

            if (NULL != p_json)
            {
               p_avg = cJSON_GetObjectItem(p_json, "avg_temp");

               if (NULL != p_avg)
               {
                  g_avg_temp = p_avg->valueint;

                  if (g_avg_temp > TEMP_HIGH_THRESHOLD)
                  {
                     trinity_log_event("EVENT: TEMP_HIGH\n");
                  }

                  bus_publish_temp(g_avg_temp);
                  ESP_LOGI(TAG, "Received avg_temp from STM32: %d", g_avg_temp);
               }

               cJSON_Delete(p_json);
               p_json = NULL;
            }
            else
            {
               ESP_LOGW(TAG, "Failed to parse JSON from STM32: %s", buf);
               trinity_log_event("EVENT: UART_PARSE_FAIL\n");
            }

            pos = 0;
         }
      }
   }
}

int uart_get_avg_temp(void)
{
   return g_avg_temp;
}
