/******************************************************************************
 * \file ble_temp.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-02
 *
 * \brief BLE temperature sensor handler for ESP32 hub node.
 *
 * \details Implements dynamic slot table for TempSensor* devices.
 *          Mirrors the reed sensor pattern in ble_scan.c.
 *
 *          MFG data layout (6 bytes, company ID 0xAE):
 *          [0] = 0xAE  (MFG_COMPANY_ID, consumed by find_mfg_data)
 *          [1] = temp_decidegc low byte   (int16 little-endian)
 *          [2] = temp_decidegc high byte
 *          [3] = batt_soc                 (0-100%)
 *          [4] = tx_id low byte           (little-endian)
 *          [5] = tx_id high byte          (little-endian)
 *
 *          Slot state machine:
 *          SLOT_EMPTY   — never seen or expired after TEMP_REMOVE_MS
 *          SLOT_ACTIVE  — advertising within TEMP_OFFLINE_MS
 *          SLOT_OFFLINE — last seen > TEMP_OFFLINE_MS, tile stays visible
 *
 * \note    Temperature is stored as int16_t tenths of °C (decidegC).
 *          Divide by 10 for display. e.g. 253 → 25.3°C, -100 → -10.0°C.
 *
 * \note    Structured event tracing — Phase 0 (2026-06-15):
 *          Log prefix normalized to [BLE_TEMP]. Removed ">>>" style.
 *
 * \note    Structured event tracing — Phase 3 (2026-06-15):
 *          bus_publish_ble_temp() now returns uint64_t event_id.
 *          Caller captures and logs event_id at BLE ingress for
 *          end-to-end correlation with [WROOM] ingest log.
 *
 * \note    Structured event tracing -- tx_id (2026-06-16):
 *          s_rx_seq removed. tx_id extracted from mfg_data[4..5]
 *          (little-endian) with mfg_len >= 6 guard. Defaults to 0 for
 *          old firmware (4-byte payload). Both log lines now emit tx_id=
 *          instead of rx_id=. Correlation key is (MAC + tx_id) within a
 *          bounded time window. Not a global unique message ID.
 *
 *            [BLE] tx_id=5 temp=25.3°C batt=87%
 *            [BLE_TEMP] tx_id=5 event_id=101 slot=0 temp=25.3°C batt=87%
 *
 * \note    Temperature event threshold (2026-06-20):
 *          bus_publish_ble_temp() is gated on abs(delta) >= 5 decidegC
 *          (0.5°C) rather than any change. Sensor noise causes 0.1°C
 *          oscillations between consecutive advertisements at stable
 *          temperatures, confirmed in sensor_server.log frames 96869-96870.
 *          Sub-threshold changes update the slot table silently.
 ******************************************************************************/

#include "ble_temp.h"
#include "ble_internal.h"
#include "config.h"
#include "wroom_bus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "BLE_TEMP"; /**< ESP log tag */

#define MFG_TEMP_LO_IDX       1   /**< temp low byte index in mfg payload  */
#define MFG_TEMP_HI_IDX       2   /**< temp high byte index in mfg payload */
#define MFG_TEMP_BATT_IDX     3   /**< battery SOC byte index              */
#define MFG_TEMP_MIN_LEN      4   /**< minimum mfg payload length          */
#define MFG_TEMP_TX_ID_LO_IDX 4   /**< tx_id low byte index                */
#define MFG_TEMP_TX_ID_HI_IDX 5   /**< tx_id high byte index               */

#define AGE_MAX_VALUE      0xFFFE  /**< max reportable age value            */

/******************************* ENUMERATIONS *********************************/

/** \brief Slot state machine states — mirrors reed pattern. */
typedef enum
{
   SLOT_EMPTY   = 0, /**< never seen or expired */
   SLOT_ACTIVE  = 1, /**< advertising within offline threshold */
   SLOT_OFFLINE = 2, /**< last seen > offline threshold ago */
} TEMP_SLOT_STATE_E;

/************************ STRUCTURE/UNION DATA TYPES **************************/

/** \brief Temperature sensor slot entry. */
typedef struct
{
   uint8_t            mac[6];                  /*!< device MAC address          */
   char               name[ADV_NAME_BUF_SIZE]; /*!< BLE device name             */
   int16_t            temp_decidegc;           /*!< temperature tenths of °C    */
   int                batt;                    /*!< battery SOC percent         */
   TEMP_SLOT_STATE_E  state;                   /*!< slot state machine state    */
   uint32_t           last_seen_ms;            /*!< timestamp of last adv       */
   uint16_t           generation;              /*!< increments on slot reuse    */
} TEMP_SLOT_T;

static TEMP_SLOT_T       g_temp_table[MAX_TEMPS];    /**< temp slot table   */
static StaticSemaphore_t g_temp_mutex_buf;           /**< static mutex buf  */
static SemaphoreHandle_t g_temp_mutex = NULL;        /**< temp table mutex  */

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Pre-initialize the temp scan module.
 *
 * \return void
 *
 * \details Creates temp mutex and zeroes the slot table.
 *          Must be called before ble_scan_start().
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_temp_preinit(void)
{
   g_temp_mutex = xSemaphoreCreateMutexStatic(&g_temp_mutex_buf);
   configASSERT(g_temp_mutex);
   (void)memset(g_temp_table, 0, sizeof(g_temp_table));
}

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Handle a TempSensor* BLE advertisement.
 *
 * \param p_mfg  - Pointer to manufacturer data payload (after company ID byte).
 * \param mfg_len - Length of manufacturer data in bytes.
 * \param p_mac  - Pointer to 6-byte device MAC address.
 * \param p_name - Null-terminated device name string.
 *
 * \return void
 *
 * \details Implements temp slot state machine. Known MACs update their
 *          slot. New MACs are allocated a slot if the table is not full.
 *          Publishes to wroom bus on first seen or data change.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_temp_handle(const uint8_t *p_mfg,
                     int            mfg_len,
                     const uint8_t *p_mac,
                     const char    *p_name)
{
   int16_t  temp_decidegc = 0;    /**< decoded temperature tenths °C */
   int      batt          = -1;   /**< battery SOC percent           */
   uint16_t tx_id         = 0;    /**< device-stamped sequence counter;
                                   *   0 = old firmware, no tx_id in payload */
   uint32_t now           = 0;    /**< current tick in ms            */
   int      slot          = -1;   /**< matched or allocated slot idx */
   bool     was_offline   = false; /**< slot was offline flag        */
   bool     changed       = false; /**< data changed flag            */
   uint16_t gen           = 0;    /**< slot generation counter       */
   uint64_t eid           = 0;    /**< wroom bus event id            */
   int      i             = 0;    /**< loop index                    */

   if ((NULL == p_mfg) || (mfg_len < MFG_TEMP_MIN_LEN))
   {
      return;
   }

   /* decode int16 little-endian from bytes [1] and [2] of mfg payload */
   temp_decidegc = (int16_t)((uint16_t)p_mfg[MFG_TEMP_LO_IDX] |
                             ((uint16_t)p_mfg[MFG_TEMP_HI_IDX] << 8));
   batt          = (int)p_mfg[MFG_TEMP_BATT_IDX];

   /* Extract device-stamped tx_id if payload is new format (6 bytes).
    * Older devices send 4 bytes — tx_id stays 0, hub continues normally. */
   if (mfg_len >= (MFG_TEMP_TX_ID_HI_IDX + 1))
   {
      tx_id = (uint16_t)p_mfg[MFG_TEMP_TX_ID_LO_IDX] |
              ((uint16_t)p_mfg[MFG_TEMP_TX_ID_HI_IDX] << 8);
   }

   now = xTaskGetTickCount() * portTICK_PERIOD_MS;

   if (NULL == g_temp_mutex)
   {
      return;
   }

   (void)xSemaphoreTake(g_temp_mutex, portMAX_DELAY);

   /* search for existing slot by MAC */
   for (i = 0; i < MAX_TEMPS; i++)
   {
      if ((SLOT_EMPTY != g_temp_table[i].state) &&
          (0 == memcmp(g_temp_table[i].mac, p_mac, 6)))
      {
         slot = i;
         break;
      }
   }

   if (0 <= slot)
   {
      was_offline = (SLOT_OFFLINE == g_temp_table[slot].state);
      changed = (abs((int)temp_decidegc - (int)g_temp_table[slot].temp_decidegc) >= 5) || (batt != g_temp_table[slot].batt);

      g_temp_table[slot].temp_decidegc = temp_decidegc;
      g_temp_table[slot].batt          = batt;
      g_temp_table[slot].last_seen_ms  = now;
      g_temp_table[slot].state         = SLOT_ACTIVE;

      (void)xSemaphoreGive(g_temp_mutex);

      if (was_offline)
      {
         ESP_LOGI(TAG, "[BLE_TEMP] slot=%d name=%s -> ACTIVE (recovered)",
                  slot, p_name);
      }

      if (changed)
      {
         eid = bus_publish_ble_temp((uint8_t)(slot + 1), temp_decidegc, batt);
         ESP_LOGI(TAG, "[BLE_TEMP] tx_id=%u event_id=%llu slot=%d "
                       "temp=%d.%d°C batt=%d%%",
                  (unsigned)tx_id, (unsigned long long)eid, slot,
                  (int)(temp_decidegc / 10),
                  (int)(temp_decidegc < 0 ?
                        -(temp_decidegc % 10) : temp_decidegc % 10),
                  batt);
      }
      else
      {
         ESP_LOGD(TAG, "[BLE_TEMP] tx_id=%u no_change slot=%d batt=%d",
                  (unsigned)tx_id, slot, batt);
      }

      return;
   }

   /* allocate new slot */
   for (i = 0; i < MAX_TEMPS; i++)
   {
      if (SLOT_EMPTY == g_temp_table[i].state)
      {
         slot = i;
         break;
      }
   }

   if (0 > slot)
   {
      (void)xSemaphoreGive(g_temp_mutex);
      ESP_LOGW(TAG, "[BLE_TEMP] table full (%d), ignoring %s", MAX_TEMPS, p_name);
      return;
   }

   (void)memcpy(g_temp_table[slot].mac, p_mac, 6);
   (void)strncpy(g_temp_table[slot].name, p_name,
                  sizeof(g_temp_table[slot].name) - 1);
   g_temp_table[slot].name[sizeof(g_temp_table[slot].name) - 1] = '\0';
   g_temp_table[slot].temp_decidegc = temp_decidegc;
   g_temp_table[slot].batt          = batt;
   g_temp_table[slot].state         = SLOT_ACTIVE;
   g_temp_table[slot].last_seen_ms  = now;
   g_temp_table[slot].generation++;
   gen = g_temp_table[slot].generation;

   (void)xSemaphoreGive(g_temp_mutex);

   ESP_LOGI(TAG, "[BLE_TEMP] slot=%d assigned name=%s gen=%u", slot, p_name, gen);
   eid = bus_publish_ble_temp((uint8_t)(slot + 1), temp_decidegc, batt);
   ESP_LOGI(TAG, "[BLE_TEMP] tx_id=%u event_id=%llu slot=%d "
                 "temp=%d.%d°C batt=%d%%",
            (unsigned)tx_id, (unsigned long long)eid, slot,
            (int)(temp_decidegc / 10),
            (int)(temp_decidegc < 0 ?
                  -(temp_decidegc % 10) : temp_decidegc % 10),
            batt);
}

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Expire stale temperature sensor slots based on age thresholds.
 *
 * \return void
 *
 * \details Transitions ACTIVE->OFFLINE after TEMP_OFFLINE_MS and
 *          OFFLINE->EMPTY after TEMP_REMOVE_MS. Call periodically.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_expire_temp_slots(void)
{
   uint32_t now = 0; /**< current tick in ms */
   uint32_t age = 0; /**< slot age in ms     */
   int      i   = 0; /**< loop index         */

   now = xTaskGetTickCount() * portTICK_PERIOD_MS;

   if (NULL == g_temp_mutex)
   {
      return;
   }

   (void)xSemaphoreTake(g_temp_mutex, portMAX_DELAY);

   for (i = 0; i < MAX_TEMPS; i++)
   {
      if (SLOT_EMPTY == g_temp_table[i].state)
      {
         continue;
      }

      age = now - g_temp_table[i].last_seen_ms;

      if (age > TEMP_REMOVE_MS)
      {
         ESP_LOGW(TAG, "Temp slot %d (%s gen=%u) expired — clearing",
                  i, g_temp_table[i].name, g_temp_table[i].generation);
         (void)memset(&g_temp_table[i], 0, sizeof(TEMP_SLOT_T));
      }
      else if ((age > TEMP_OFFLINE_MS) &&
               (SLOT_ACTIVE == g_temp_table[i].state))
      {
         g_temp_table[i].state = SLOT_OFFLINE;
         ESP_LOGI(TAG, "Temp slot %d (%s) -> OFFLINE",
                  i, g_temp_table[i].name);
      }
      else
      {
         /* slot within thresholds — no action */
      }
   }

   (void)xSemaphoreGive(g_temp_mutex);
}

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Get count of active or offline temperature sensor slots.
 *
 * \return int - Highest non-empty slot index + 1, or 0 if none.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
int ble_get_temp_count(void)
{
   int count = 0; /**< highest non-empty slot + 1 */
   int i     = 0; /**< loop index                 */

   if (NULL == g_temp_mutex)
   {
      return 0;
   }

   (void)xSemaphoreTake(g_temp_mutex, portMAX_DELAY);

   for (i = 0; i < MAX_TEMPS; i++)
   {
      if (SLOT_EMPTY != g_temp_table[i].state)
      {
         count = i + 1;
      }
   }

   (void)xSemaphoreGive(g_temp_mutex);

   return count;
}

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Get information for a temperature sensor slot.
 *
 * \param slot             - Slot index (0-based).
 * \param p_name_out       - Output buffer for device name, or NULL.
 * \param p_temp_out       - Output for temperature in tenths of °C, or NULL.
 * \param p_batt_out       - Output for battery SOC, or NULL.
 * \param p_age_out        - Output for age in seconds, or NULL.
 * \param p_gen_out        - Output for generation counter, or NULL.
 *
 * \return bool - true if slot is active or offline, false if empty or OOB.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
bool ble_get_temp_slot_info(int      slot,
                             char    *p_name_out,
                             int16_t *p_temp_out,
                             int     *p_batt_out,
                             uint16_t *p_age_out,
                             uint16_t *p_gen_out)
{
   uint32_t now_ms = 0; /**< current tick in ms  */
   uint32_t age_ms = 0; /**< slot age in ms      */
   uint32_t age_s  = 0; /**< slot age in seconds */

   if ((0 > slot) || (slot >= MAX_TEMPS) || (NULL == g_temp_mutex))
   {
      return false;
   }

   (void)xSemaphoreTake(g_temp_mutex, portMAX_DELAY);

   if (SLOT_EMPTY == g_temp_table[slot].state)
   {
      (void)xSemaphoreGive(g_temp_mutex);
      return false;
   }

   if (NULL != p_name_out)
   {
      (void)strncpy(p_name_out, g_temp_table[slot].name, SLOT_NAME_MAX);
   }

   if (NULL != p_temp_out)
   {
      *p_temp_out = g_temp_table[slot].temp_decidegc;
   }

   if (NULL != p_batt_out)
   {
      *p_batt_out = g_temp_table[slot].batt;
   }

   if (NULL != p_gen_out)
   {
      *p_gen_out = g_temp_table[slot].generation;
   }

   if (NULL != p_age_out)
   {
      now_ms     = xTaskGetTickCount() * portTICK_PERIOD_MS;
      age_ms     = now_ms - g_temp_table[slot].last_seen_ms;
      age_s      = age_ms / 1000;
      *p_age_out = (age_s > AGE_MAX_VALUE) ?
                   (uint16_t)AGE_MAX_VALUE : (uint16_t)age_s;
   }

   (void)xSemaphoreGive(g_temp_mutex);

   return true;
}
