/******************************************************************************
 * \file ble_pir.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-02
 *
 * \brief BLE PIR sensor slot table for ESP32 hub node.
 *
 * \details Implements dynamic slot table for PIR_* devices.
 *          Mirrors the temperature sensor pattern in ble_temp.c.
 *
 *          MFG data layout:
 *          [0] = company ID  (consumed by find_mfg_data in ble_scan.c)
 *          [1] = reserved
 *          [2..5] = motion count (uint32, big-endian)
 *          [6] = batt_soc    (0-100%)
 *          [7] = occupied    (0=empty, 1=occupied)
 *          [8] = tx_id low byte
 *          [9] = tx_id high byte
 *
 *          Slot state machine:
 *          SLOT_EMPTY   — never seen or expired after PIR_REMOVE_MS
 *          SLOT_ACTIVE  — advertising within PIR_OFFLINE_MS
 *          SLOT_OFFLINE — last seen > PIR_OFFLINE_MS, tile stays visible
 *
 *          Occupancy logic is delegated to pir_window_update() on every
 *          received advertisement.
 *
 * \note    PIR slot table (2026-05-20):
 *          handle_pir() replaced by dynamic slot table matching reed
 *          pattern. g_motion_count, g_pir_batt, g_pir_seen removed.
 *          Extracted from ble_scan.c (2026-06-02).
 *
 * \note    Structured event tracing — Phase 0 (2026-06-15):
 *          Log prefix normalized to [BLE_PIR]. Removed ">>>" style.
 *
 * \note    Structured event tracing — Phase 3 (2026-06-15):
 *          bus_publish_pir() now returns uint64_t event_id.
 *          Caller captures and logs event_id at BLE ingress so the
 *          correlation key is visible at both [BLE_PIR] and [VROOM].
 *
 * \note    Structured event tracing — Phase 3.5 (2026-06-16):
 *          s_rx_seq added. BLE adv ingress sequence counter logged as
 *          rx_id at every publish site. No struct or ABI changes.
 *
 * \note    Structured event tracing — tx_id (2026-06-16):
 *          s_rx_seq removed. tx_id extracted from mfg_data[8..9]
 *          (little-endian) — appended after existing 8-byte payload.
 *          Device stamps tx_id on every broadcast. Hub reads it out
 *          of raw bytes so both sides share the same correlation key:
 *
 *            [DEVICE]  tx_id=1844 count=42 batt=87%
 *            [BLE_PIR] tx_id=1844 event_id=101 slot=0 count=42 batt=87%
 *
 *          Graceful degradation: devices sending 8-byte payloads (old
 *          firmware without tx_id) produce tx_id=0 in the hub log.
 *          No crash, no behavior change — hub checks mfg_len >= 10.
 *
 *          tx_id is extracted before the changed check and logged only
 *          when bus_publish_pir() fires — no point logging a tx_id for
 *          an advertisement that did not trigger a publish.
 ******************************************************************************/

#include "ble_pir.h"
#include "ble_internal.h"
#include "ble_proto.h"
#include "config.h"
#include "vroom_bus.h"
#include "pir_window.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "BLE_PIR"; /**< ESP log tag */

#define MFG_PIR_MIN_LEN         6  /**< minimum mfg payload length          */
#define MFG_PIR_BATT_IDX        6  /**< battery SOC byte index              */
#define MFG_PIR_OCCUPIED_IDX    7  /**< occupied flag byte index            */
#define MFG_PIR_TX_ID_LO_IDX   8  /**< tx_id low byte index                */
#define MFG_PIR_TX_ID_HI_IDX   9  /**< tx_id high byte index               */

#define PIR_COUNT_BYTE0      2  /**< motion count MSB index              */
#define PIR_COUNT_BYTE1      3  /**< motion count byte 1 index           */
#define PIR_COUNT_BYTE2      4  /**< motion count byte 2 index           */
#define PIR_COUNT_BYTE3      5  /**< motion count LSB index              */


/******************************* ENUMERATIONS *********************************/

/** \brief Slot state machine states — mirrors reed/temp pattern. */
typedef enum
{
   SLOT_EMPTY   = 0, /**< never seen or expired */
   SLOT_ACTIVE  = 1, /**< advertising within offline threshold */
   SLOT_OFFLINE = 2, /**< last seen > offline threshold ago */
} PIR_SLOT_STATE_E;

/************************ STRUCTURE/UNION DATA TYPES **************************/

/** \brief PIR sensor slot entry. */
typedef struct
{
   uint8_t           mac[6];                  /*!< device MAC address      */
   char              name[ADV_NAME_BUF_SIZE]; /*!< BLE device name         */
   uint32_t          count;                   /*!< motion event count      */
   int               batt;                    /*!< battery SOC percent     */
   PIR_SLOT_STATE_E  state;                   /*!< slot state machine state */
   uint32_t          last_seen_ms;            /*!< timestamp of last adv   */
   uint16_t          generation;              /*!< increments on slot reuse */
} PIR_SLOT_T;

static PIR_SLOT_T        g_pir_table[MAX_PIRS];   /**< PIR slot table     */
static StaticSemaphore_t g_pir_mutex_buf;          /**< static mutex buf   */
static SemaphoreHandle_t g_pir_mutex = NULL;       /**< PIR table mutex    */

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Pre-initialize the PIR sensor module.
 *
 * \return void
 *
 * \details Creates PIR mutex and zeroes the slot table.
 *          Must be called before ble_scan_start().
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_pir_preinit(void)
{
   g_pir_mutex = xSemaphoreCreateMutexStatic(&g_pir_mutex_buf);
   configASSERT(g_pir_mutex);

   (void)memset(g_pir_table, 0, sizeof(g_pir_table));
}

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Handle a PIR_* BLE advertisement.
 *
 * \param p_mfg   - Pointer to manufacturer data payload (after company ID).
 * \param mfg_len - Length of manufacturer data in bytes.
 * \param p_mac   - Pointer to 6-byte device MAC address.
 * \param p_name  - Null-terminated device name string.
 *
 * \return void
 *
 * \details Implements PIR slot state machine. Known MACs update their
 *          slot. New MACs are allocated a slot if the table is not full.
 *          Publishes to vroom bus on first seen or data change.
 *          Delegates occupancy logic to pir_window_update().
 *          Captures returned event_id from bus_publish_pir() and logs
 *          it at BLE ingress for end-to-end correlation.
 *
 *          tx_id is extracted from mfg_data[8..9] before the changed
 *          check so it is always available when publish fires.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_pir_handle(const uint8_t *p_mfg,
                    int            mfg_len,
                    const uint8_t *p_mac,
                    const char    *p_name)
{
   uint32_t count    = 0;     /**< motion event count                     */
   int      batt     = -1;    /**< battery SOC percent                    */
   uint8_t  occupied = 0;     /**< occupied flag byte                     */
   uint16_t tx_id    = 0;     /**< device-stamped sequence counter;
                               *   0 = old firmware, no tx_id in payload  */
   uint32_t now      = 0;     /**< current tick in ms                     */
   int      slot     = -1;    /**< slot index                             */
   bool     changed  = false; /**< data changed flag                      */
   uint16_t gen      = 0;     /**< slot generation counter                */
   uint64_t eid      = 0;     /**< correlation event ID                   */
   int      i        = 0;     /**< loop index                             */

   if ((NULL == p_mfg) || (mfg_len < MFG_PIR_MIN_LEN))
   {
      return;
   }

   count = ((uint32_t)p_mfg[PIR_COUNT_BYTE0] << 24) |
           ((uint32_t)p_mfg[PIR_COUNT_BYTE1] << 16) |
           ((uint32_t)p_mfg[PIR_COUNT_BYTE2] <<  8) |
            (uint32_t)p_mfg[PIR_COUNT_BYTE3];

   if (mfg_len >= (MFG_PIR_BATT_IDX + 1))
   {
      batt = (int)p_mfg[MFG_PIR_BATT_IDX];
   }

   if (mfg_len >= (MFG_PIR_OCCUPIED_IDX + 1))
   {
      occupied = p_mfg[MFG_PIR_OCCUPIED_IDX];
   }

   /* Extract device-stamped tx_id if payload is new format (10 bytes).
    * Older devices send 8 bytes — tx_id stays 0, hub continues normally. */
   if (mfg_len >= (MFG_PIR_TX_ID_HI_IDX + 1))
   {
      tx_id = (uint16_t)p_mfg[MFG_PIR_TX_ID_LO_IDX] |
              ((uint16_t)p_mfg[MFG_PIR_TX_ID_HI_IDX] << 8);
   }

   now = xTaskGetTickCount() * portTICK_PERIOD_MS;

   if (NULL == g_pir_mutex)
   {
      return;
   }

   (void)xSemaphoreTake(g_pir_mutex, portMAX_DELAY);

   /* search for existing slot by MAC */
   for (i = 0; i < MAX_PIRS; i++)
   {
      if ((SLOT_EMPTY != g_pir_table[i].state) &&
          (0 == memcmp(g_pir_table[i].mac, p_mac, 6)))
      {
         slot = i;
         break;
      }
   }

   if (0 <= slot)
   {
      changed = (count != g_pir_table[slot].count) ||
                (batt  != g_pir_table[slot].batt);

      g_pir_table[slot].count        = count;
      g_pir_table[slot].batt         = batt;
      g_pir_table[slot].last_seen_ms = now;
      g_pir_table[slot].state        = SLOT_ACTIVE;

      (void)xSemaphoreGive(g_pir_mutex);

      stamp_device((BLE_DEV_IDX_E)(DEV_IDX_PIR + slot));
      pir_window_update(slot, now, (int)occupied);

      if (changed)
      {
         eid = bus_publish_pir((uint8_t)(slot + 1), count, batt);
         ESP_LOGI(TAG, "[BLE_PIR] tx_id=%u event_id=%llu slot=%d count=%u batt=%d%%",
                  tx_id, (unsigned long long)eid, slot, count, batt);
      }

      return;
   }

   /* allocate new slot */
   for (i = 0; i < MAX_PIRS; i++)
   {
      if (SLOT_EMPTY == g_pir_table[i].state)
      {
         slot = i;
         break;
      }
   }

   if (0 > slot)
   {
      (void)xSemaphoreGive(g_pir_mutex);
      ESP_LOGW(TAG, "PIR table full (%d), ignoring %s", MAX_PIRS, p_name);
      return;
   }

   (void)memcpy(g_pir_table[slot].mac, p_mac, 6);
   (void)strncpy(g_pir_table[slot].name, p_name,
                  sizeof(g_pir_table[slot].name) - 1);
   g_pir_table[slot].name[sizeof(g_pir_table[slot].name) - 1] = '\0';
   g_pir_table[slot].count        = count;
   g_pir_table[slot].batt         = batt;
   g_pir_table[slot].state        = SLOT_ACTIVE;
   g_pir_table[slot].last_seen_ms = now;
   g_pir_table[slot].generation++;
   gen = g_pir_table[slot].generation;

   (void)xSemaphoreGive(g_pir_mutex);

   ESP_LOGI(TAG, "[BLE_PIR] slot=%d assigned name=%s gen=%u", slot, p_name, gen);
   stamp_device((BLE_DEV_IDX_E)(DEV_IDX_PIR + slot));
   pir_window_update(slot, now, (int)occupied);

   eid = bus_publish_pir((uint8_t)(slot + 1), count, batt);
   ESP_LOGI(TAG, "[BLE_PIR] tx_id=%u event_id=%llu slot=%d count=%u batt=%d%%",
            tx_id, (unsigned long long)eid, slot, count, batt);
}

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Expire stale PIR sensor slots based on age thresholds.
 *
 * \return void
 *
 * \details Transitions ACTIVE->OFFLINE after PIR_OFFLINE_MS and
 *          OFFLINE->EMPTY after PIR_REMOVE_MS. Call periodically.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_expire_pir_slots(void)
{
   uint32_t now = 0; /**< current tick in ms */
   uint32_t age = 0; /**< slot age in ms     */
   int      i   = 0; /**< loop index         */

   now = xTaskGetTickCount() * portTICK_PERIOD_MS;

   if (NULL == g_pir_mutex)
   {
      return;
   }

   (void)xSemaphoreTake(g_pir_mutex, portMAX_DELAY);

   for (i = 0; i < MAX_PIRS; i++)
   {
      if (SLOT_EMPTY == g_pir_table[i].state)
      {
         continue;
      }

      age = now - g_pir_table[i].last_seen_ms;

      if (age > PIR_REMOVE_MS)
      {
         ESP_LOGW(TAG, "[BLE_PIR] slot=%d name=%s gen=%u expired — clearing",
                  i, g_pir_table[i].name, g_pir_table[i].generation);
         (void)memset(&g_pir_table[i], 0, sizeof(PIR_SLOT_T));
      }
      else if ((age > PIR_OFFLINE_MS) &&
               (SLOT_ACTIVE == g_pir_table[i].state))
      {
         g_pir_table[i].state = SLOT_OFFLINE;
         ESP_LOGI(TAG, "[BLE_PIR] slot=%d name=%s -> OFFLINE",
                  i, g_pir_table[i].name);
      }
      else
      {
         /* slot within thresholds — no action */
      }
   }

   (void)xSemaphoreGive(g_pir_mutex);
}

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Get count of active or offline PIR sensor slots.
 *
 * \return int - Highest non-empty slot index + 1, or 0 if none.
 *
 * \details Returns the count such that all slots 0..count-1 are visible
 *          on the dashboard. SLOT_EMPTY slots beyond the last active
 *          slot cause the count to stop.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
int ble_pir_get_count(void)
{
   int count = 0; /**< highest non-empty slot index + 1 */
   int i     = 0; /**< loop index                       */

   if (NULL == g_pir_mutex)
   {
      return 0;
   }

   (void)xSemaphoreTake(g_pir_mutex, portMAX_DELAY);

   for (i = 0; i < MAX_PIRS; i++)
   {
      if (SLOT_EMPTY != g_pir_table[i].state)
      {
         count = i + 1;
      }
   }

   (void)xSemaphoreGive(g_pir_mutex);

   return count;
}

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Get slot metrics for an active PIR sensor.
 *
 * \param slot        - Slot index (0-based).
 * \param p_count_out - Output for cumulative motion count, or NULL.
 * \param p_batt_out  - Output for battery SOC percent, or NULL.
 * \param p_age_out   - Output for age in seconds, or NULL.
 *
 * \return bool - true if slot is active or offline, false if empty or OOB.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
bool ble_pir_get_slot_info(int       slot,
                            uint32_t *p_count_out,
                            int      *p_batt_out,
                            uint16_t *p_age_out)
{
   uint32_t now_ms = 0;     /**< current tick in ms  */
   uint32_t age_ms = 0;     /**< slot age in ms      */
   uint32_t age_s  = 0;     /**< slot age in seconds */
   bool     active = false; /**< slot active flag    */

   if ((0 > slot) || (slot >= MAX_PIRS) || (NULL == g_pir_mutex))
   {
      return false;
   }

   (void)xSemaphoreTake(g_pir_mutex, portMAX_DELAY);

   if (SLOT_EMPTY != g_pir_table[slot].state)
   {
      if (NULL != p_count_out)
      {
         *p_count_out = g_pir_table[slot].count;
      }

      if (NULL != p_batt_out)
      {
         *p_batt_out = g_pir_table[slot].batt;
      }

      if (NULL != p_age_out)
      {
         now_ms     = xTaskGetTickCount() * portTICK_PERIOD_MS;
         age_ms     = now_ms - g_pir_table[slot].last_seen_ms;
         age_s      = age_ms / 1000u;
         *p_age_out = (age_s > AGE_MAX_VALUE) ?
                      (uint16_t)AGE_MAX_VALUE : (uint16_t)age_s;
      }

      active = true;
   }

   (void)xSemaphoreGive(g_pir_mutex);

   return active;
}
