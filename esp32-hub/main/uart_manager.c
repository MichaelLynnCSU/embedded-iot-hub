/******************************************************************************
 * \file uart_manager.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief UART manager for ESP32 hub node.
 *
 * \details Receives JSON temperature data from STM32 blue pill over
 *          UART2 at 115200 baud. Parses avg_temp and tx_id fields
 *          publishes to vroom bus for consumption by TCP and AWS managers.
 *
 *          Message format: {"avg_temp": <int>, "tx_id": <uint>}\n
 *          tx_id is a per-session monotonic counter incremented by the
 *          STM32 before each transmit. Defaults to 0 if field is absent
 *          (old firmware graceful degradation — no error, no crash).
 *          High temperature threshold: 100C — logged as EVENT: TEMP_HIGH
 *
 * \note    WDT fix (2026-03-21):
 *          trinity_wdt_kick() added at the top of the main while(1) loop.
 *          The loop already wakes every UART_READ_TIMEOUT_MS (100 ms) on
 *          timeout, so the kick fires well within the 5 s WDT window even
 *          when no UART data arrives.
 *          trinity_log_event() renamed to trinity_log_event() throughout.
 *
 * \note    tx_id tracing (2026-06-20):
 *          STM32 blue pill now stamps a per-session uint16_t tx_id counter
 *          into every outbound JSON frame:
 *
 *            {"avg_temp": 22, "tx_id": 7}
 *
 *          Hub extracts tx_id via cJSON_GetObjectItem and logs it alongside
 *          event_id:
 *
 *            [UART] tx_id=7 event_id=204 avg_temp=22
 *
 *          This closes the STM32 → hub trace gap: tx_id is device-generated
 *          and increments on every parsed frame with no relationship to the
 *          STM32 transmit counter. tx_id is device-generated and increments
 *          on every STM32 transmit attempt, making drops and retransmits
 *          visible as gaps in the tx_id sequence in the hub log.
 *
 *          Full trace chain:
 *            [STM32]  tx_id=7 avg_temp=22
 *            [UART]   tx_id=7 event_id=204 avg_temp=22
 *            [VROOM]  event_id=204 bus_seq=8 ingest type=UART_TEMP avg_temp=22
 *            [TCP]    event_id=204 ...
 *
 *          tx_id=0 in hub log indicates old STM32 firmware (field absent) —
 *          not an error. cJSON_GetObjectItem returns NULL for missing fields;
 *          tx_id defaults to 0u with no log noise.
 *
 * \note    Delta gate on publish (2026-06-20):
 *          bus_publish_temp() previously fired on every parsed UART frame
 *          regardless of whether avg_temp changed. Confirmed in hub firmware
 *          log: event_id=11 and event_id=14 both carried avg_temp=20 with
 *          no change between them (~5s apart, one per STM32 transmit cycle).
 *          This generated continuous UART_TEMP events in the vroom bus and
 *          flooded events[] on the wire protocol with no-change publishes.
 *
 *          bus_publish_temp() now fires only when new_temp != g_avg_temp.
 *          g_avg_temp is still updated on every frame so uart_get_avg_temp()
 *          always reflects the latest received value.
 *
 *          TEMP_HIGH threshold check is gated inside the delta block — it
 *          is a discrete state-transition event (cool->hot), not a
 *          persistent alarm, so firing it only on change is correct. If a
 *          persistent above-threshold alarm is ever needed, move the check
 *          outside the delta gate at that time.
 *
 *          No-change frames are logged at LOGD level so the trace gap is
 *          still visible for debugging without polluting the normal log:
 *
 *            [UART] tx_id=8 no_change avg_temp=22
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
   uint8_t  buf[UART_BUF_SIZE] = {0};
   uint8_t  byte               = 0;
   int      pos                = 0;
   int      len                = 0;
   cJSON   *p_json             = NULL;
   cJSON   *p_avg              = NULL;
   cJSON   *p_txid             = NULL;  /**< "tx_id" field, NULL if absent  */
   uint32_t tx_id              = 0u;   /**< device tx counter, 0 = old fw  */

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
               p_avg  = cJSON_GetObjectItem(p_json, "avg_temp");

               /* tx_id: present on new STM32 firmware, absent on old.
                * NULL return from cJSON_GetObjectItem is the expected
                * old-firmware path — default to 0, no warning logged. */
               p_txid = cJSON_GetObjectItem(p_json, "tx_id");
               tx_id  = (NULL != p_txid) ? (uint32_t)p_txid->valueint : 0u;

               if (NULL != p_avg)
               {
                  int new_temp = p_avg->valueint;

                  if (new_temp != g_avg_temp)
                  {
                     g_avg_temp = new_temp;

                     /* TEMP_HIGH is a state-transition event — gate on
                      * delta so it fires once on cool->hot, not every
                      * frame while above threshold. See file header note
                      * "Delta gate on publish (2026-06-20)". */
                     if (g_avg_temp > TEMP_HIGH_THRESHOLD)
                     {
                        trinity_log_event("EVENT: TEMP_HIGH\n");
                     }

                     uint64_t eid = bus_publish_temp(g_avg_temp);

                     ESP_LOGI(TAG, "[UART] tx_id=%u event_id=%llu avg_temp=%d",
                              (unsigned)tx_id,
                              (unsigned long long)eid,
                              g_avg_temp);
                  }
                  else
                  {
                     /* Temperature unchanged — update stored value to keep
                      * uart_get_avg_temp() fresh, skip publish. LOGD so
                      * tx_id gaps remain visible for debugging. */
                     g_avg_temp = new_temp;
                     ESP_LOGD(TAG, "[UART] tx_id=%u no_change avg_temp=%d",
                              (unsigned)tx_id, g_avg_temp);
                  }
               }

               cJSON_Delete(p_json);
               p_json = NULL;
            }
            else
            {
               ESP_LOGW(TAG, "[UART] parse_fail buf=%s", buf);
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
