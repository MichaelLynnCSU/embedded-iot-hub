/******************************************************************************
 * \file ble_reed.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-02
 *
 * \brief BLE reed sensor slot table for ESP32 hub node.
 *
 * \details Implements dynamic slot table for ReedSensor* devices.
 *          Mirrors the temperature sensor pattern in ble_temp.c.
 *
 *          MFG data layout:
 *          [0] = company ID  (consumed by find_mfg_data in ble_scan.c)
 *          [1] = door state  (0=closed, 1=open, 0xFF=unknown)
 *          [2] = batt_soc    (0-100%)
 *          [3] = tx_id low byte
 *          [4] = tx_id high byte
 *
 *          Slot state machine:
 *          SLOT_EMPTY   — never seen or expired after REED_REMOVE_MS
 *          SLOT_ACTIVE  — advertising within REED_OFFLINE_MS
 *          SLOT_OFFLINE — last seen > REED_OFFLINE_MS, tile stays visible
 *
 *          Thresholds:
 *          - OFFLINE:  150 s — matches BLE_AGE_THRESHOLD_S on STM32
 *          - REMOVE:  3600 s — 1 hour unseen, slot cleared, tile hidden
 *
 *          Cooldown window (BLE_COOLDOWN_MS) prevents a removed device
 *          from immediately re-claiming a slot on its next advertisement.
 *
 * \note    Structured event tracing — Phase 0 (2026-06-15):
 *          Log prefix normalized to [BLE_REED].
 *
 * \note    Structured event tracing — Phase 3 (2026-06-15):
 *          bus_publish_reed() now returns uint64_t event_id.
 *          Caller captures and logs event_id at BLE ingress for
 *          end-to-end correlation with [VROOM] ingest log.
 *
 * \note    Structured event tracing — Phase 3.5 (2026-06-16):
 *          s_rx_seq added. BLE adv ingress sequence counter logged as
 *          rx_id at every publish site. No struct or ABI changes.
 *
 * \note    Structured event tracing — tx_id (2026-06-16):
 *          s_rx_seq replaced by tx_id extracted from mfg_data[3..4].
 *          tx_id is stamped by the device on every broadcast so both
 *          device-side and hub-side logs share the same correlation key:
 *
 *            [DEVICE]   tx_id=1844 state=1 batt=87%
 *            [BLE_REED] tx_id=1844 event_id=101 slot=1 state=1 batt=87%
 *
 *          Graceful degradation: devices sending a 3-byte payload (old
 *          firmware without tx_id) produce tx_id=0 in the hub log.
 *          No crash, no behavior change — hub checks mfg_len >= 5 before
 *          reading bytes [3..4].
 *
 *          Template note for other device types:
 *          1. Add MFG_<TYPE>_TX_ID_LO_IDX and HI_IDX defines matching
 *             the byte positions chosen in that device's payload.
 *          2. Extract with the same mfg_len guard (>= HI_IDX + 1).
 *          3. Declare tx_id as uint16_t, default 0.
 *          4. Replace any s_rx_seq log reference with tx_id=.
 *          5. Remove s_rx_seq entirely — it is no longer needed once
 *             tx_id is available from the payload.
 ******************************************************************************/

#include "ble_reed.h"
#include "ble_internal.h"
#include "ble_proto.h"
#include "config.h"
#include "vroom_bus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "BLE_REED"; /**< ESP log tag */

#define MFG_REED_STATE_IDX      1  /**< door state byte index in mfg payload */
#define MFG_REED_BATT_IDX       2  /**< battery SOC byte index               */
#define MFG_REED_TX_ID_LO_IDX   3  /**< tx_id low byte index                 */
#define MFG_REED_TX_ID_HI_IDX   4  /**< tx_id high byte index                */


/******************************* ENUMERATIONS *********************************/

/** \brief Slot state machine states — mirrors reed pattern. */
typedef enum
{
   SLOT_EMPTY   = 0, /**< never seen or expired */
   SLOT_ACTIVE  = 1, /**< advertising within offline threshold */
   SLOT_OFFLINE = 2, /**< last seen > offline threshold ago */
} REED_SLOT_STATE_E;

/************************ STRUCTURE/UNION DATA TYPES **************************/

/** \brief Reed sensor slot entry. */
typedef struct
{
   uint8_t            mac[6];                  /*!< device MAC address          */
   char               name[ADV_NAME_BUF_SIZE]; /*!< BLE device name             */
   uint8_t            door_state;              /*!< 0=closed 1=open 0xFF=unknown */
   int                batt;                    /*!< battery SOC percent          */
   REED_SLOT_STATE_E  state;                   /*!< slot state machine state     */
   uint32_t           last_seen_ms;            /*!< timestamp of last adv        */
   uint16_t           generation;              /*!< increments on slot reuse     */
} REED_SLOT_T;

/** \brief Cooldown table entry — prevents immediate slot re-allocation. */
typedef struct
{
   uint8_t  mac[6];        /*!< MAC of recently removed device */
   uint32_t removed_at_ms; /*!< timestamp of removal           */
} COOLDOWN_ENTRY_T;

static REED_SLOT_T       g_reed_table[MAX_REEDS];          /**< reed slot table  */
static COOLDOWN_ENTRY_T  g_cooldown_table[COOLDOWN_COUNT]; /**< cooldown table   */
static StaticSemaphore_t g_reed_mutex_buf;                 /**< static mutex buf */
static SemaphoreHandle_t g_reed_mutex = NULL;              /**< reed table mutex */

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Add a MAC to the cooldown table to prevent immediate re-allocation.
 *
 * \param p_mac - Pointer to 6-byte MAC address.
 *
 * \return void
 *
 * \details Replaces the oldest cooldown entry if the table is full.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void cooldown_add(const uint8_t *p_mac)
{
   uint32_t now    = 0; /**< current tick in ms    */
   int      oldest = 0; /**< index of oldest entry */
   int      i      = 0; /**< loop index            */

   now = xTaskGetTickCount() * portTICK_PERIOD_MS;

   for (i = 1; i < COOLDOWN_COUNT; i++)
   {
      if (g_cooldown_table[i].removed_at_ms < g_cooldown_table[oldest].removed_at_ms)
      {
         oldest = i;
      }
   }

   (void)memcpy(g_cooldown_table[oldest].mac, p_mac, 6);
   g_cooldown_table[oldest].removed_at_ms = now;
}

/******************************************************************************
 * \brief Check if a MAC is currently in the cooldown window.
 *
 * \param p_mac - Pointer to 6-byte MAC address.
 *
 * \return bool - true if MAC is in cooldown, false otherwise.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static bool cooldown_check(const uint8_t *p_mac)
{
   uint32_t now = 0; /**< current tick in ms */
   int      i   = 0; /**< loop index         */

   now = xTaskGetTickCount() * portTICK_PERIOD_MS;

   for (i = 0; i < COOLDOWN_COUNT; i++)
   {
      if (0 == memcmp(g_cooldown_table[i].mac, p_mac, 6))
      {
         if ((now - g_cooldown_table[i].removed_at_ms) < BLE_COOLDOWN_MS)
         {
            return true;
         }
      }
   }

   return false;
}

/******************************************************************************
 * \brief Find a reed slot by MAC address.
 *
 * \param p_mac - Pointer to 6-byte MAC address.
 *
 * \return int - Slot index if found, -1 if not found.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static int find_slot_by_mac(const uint8_t *p_mac)
{
   int i = 0; /**< loop index */

   for (i = 0; i < MAX_REEDS; i++)
   {
      if ((SLOT_EMPTY != g_reed_table[i].state) &&
          (0 == memcmp(g_reed_table[i].mac, p_mac, 6)))
      {
         return i;
      }
   }

   return -1;
}

/******************************************************************************
 * \brief Find the first empty reed slot.
 *
 * \return int - Slot index if found, -1 if table is full.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static int find_empty_slot(void)
{
   int i = 0; /**< loop index */

   for (i = 0; i < MAX_REEDS; i++)
   {
      if (SLOT_EMPTY == g_reed_table[i].state)
      {
         return i;
      }
   }

   return -1;
}

/******************************************************************************
 * \brief Update room sensor state string for a reed slot.
 *
 * \param slot       - Reed slot index.
 * \param door_state - Door state byte (0=closed, 1=open, 0xFF=unknown).
 *
 * \return void
 *
 * \details Maps slot 0 to ROOM_SENSOR_SLOT0_ID and slot 1 to
 *          ROOM_SENSOR_SLOT1_ID. No-op for other slots.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void update_room_for_slot(int slot, uint8_t door_state)
{
   const char *p_state_str = NULL; /**< state string pointer */

   if (1 == door_state)
   {
      p_state_str = "open";
   }
   else if (0 == door_state)
   {
      p_state_str = "closed";
   }
   else
   {
      p_state_str = "unknown";
   }

   if (0 == slot)
   {
      ble_update_room_sensor(ROOM_SENSOR_SLOT0_ID, p_state_str);
   }
   else if (1 == slot)
   {
      ble_update_room_sensor(ROOM_SENSOR_SLOT1_ID, p_state_str);
   }
   else
   {
      /* no room mapping for this slot */
   }
}

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Pre-initialize the reed sensor module.
 *
 * \return void
 *
 * \details Creates reed mutex and zeroes slot and cooldown tables.
 *          Must be called before ble_scan_start().
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_reed_preinit(void)
{
   g_reed_mutex = xSemaphoreCreateMutexStatic(&g_reed_mutex_buf);
   configASSERT(g_reed_mutex);

   (void)memset(g_reed_table,     0, sizeof(g_reed_table));
   (void)memset(g_cooldown_table, 0, sizeof(g_cooldown_table));
}

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Handle a ReedSensor* BLE advertisement.
 *
 * \param p_mfg   - Pointer to manufacturer data payload (after company ID).
 * \param mfg_len - Length of manufacturer data in bytes.
 * \param p_mac   - Pointer to 6-byte device MAC address.
 * \param p_name  - Null-terminated device name string.
 *
 * \return void
 *
 * \details Implements reed slot state machine. Known MACs update their
 *          slot. New MACs are allocated a slot if not in cooldown and
 *          table is not full. Captures returned event_id from
 *          bus_publish_reed() and logs it at BLE ingress for
 *          end-to-end correlation with [VROOM] ingest log.
 *
 *          tx_id is extracted from mfg_data[3..4] (little-endian) if
 *          present (mfg_len >= 5). Old devices sending 3-byte payloads
 *          produce tx_id=0 — no error, no behavior change.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_reed_handle(const uint8_t *p_mfg,
                     int            mfg_len,
                     const uint8_t *p_mac,
                     const char    *p_name)
{
   uint8_t  door_state  = 0xFF;   /**< door state byte                        */
   int      batt        = -1;     /**< battery SOC percent                    */
   uint16_t tx_id       = 0;      /**< device-stamped sequence counter;
                                   *   0 = old firmware, no tx_id in payload  */
   uint32_t now         = 0;      /**< current tick in ms                     */
   int      slot        = -1;     /**< slot index                             */
   bool     was_offline = false;  /**< slot was offline flag                  */
   uint16_t gen         = 0;      /**< slot generation counter                */
   uint64_t eid         = 0;      /**< correlation event ID                   */

   if ((NULL != p_mfg) && (mfg_len >= (MFG_REED_STATE_IDX + 1)))
   {
      door_state = p_mfg[MFG_REED_STATE_IDX];
   }

   if ((NULL != p_mfg) && (mfg_len >= (MFG_REED_BATT_IDX + 1)))
   {
      batt = (int)p_mfg[MFG_REED_BATT_IDX];
   }

   /* Extract device-stamped tx_id if payload is new format (5 bytes).
    * Older devices send 3 bytes — tx_id stays 0, hub continues normally. */
   if ((NULL != p_mfg) && (mfg_len >= (MFG_REED_TX_ID_HI_IDX + 1)))
   {
      tx_id = (uint16_t)p_mfg[MFG_REED_TX_ID_LO_IDX] |
              ((uint16_t)p_mfg[MFG_REED_TX_ID_HI_IDX] << 8);
   }

   now = xTaskGetTickCount() * portTICK_PERIOD_MS;

   if (NULL == g_reed_mutex)
   {
      return;
   }

   (void)xSemaphoreTake(g_reed_mutex, portMAX_DELAY);

   slot = find_slot_by_mac(p_mac);

   if (0 <= slot)
   {
      was_offline = (SLOT_OFFLINE == g_reed_table[slot].state);
      g_reed_table[slot].last_seen_ms = now;
      g_reed_table[slot].state        = SLOT_ACTIVE;
      g_reed_table[slot].door_state   = door_state;
      g_reed_table[slot].batt         = batt;
      (void)xSemaphoreGive(g_reed_mutex);

      if (was_offline)
      {
         ESP_LOGI(TAG, "[BLE_REED] slot=%d name=%s -> ACTIVE (recovered)",
                  slot, p_name);
      }

      update_room_for_slot(slot, door_state);
      eid = bus_publish_reed((uint8_t)(slot + 1), door_state, batt, p_mac);
      ESP_LOGI(TAG, "[BLE_REED] tx_id=%u event_id=%llu slot=%d state=%d batt=%d%%",
               tx_id, (unsigned long long)eid, slot, (int)door_state, batt);
      return;
   }

   if (cooldown_check(p_mac))
   {
      (void)xSemaphoreGive(g_reed_mutex);
      return;
   }

   slot = find_empty_slot();
   if (0 > slot)
   {
      (void)xSemaphoreGive(g_reed_mutex);
      ESP_LOGW(TAG, "[BLE_REED] table full (%d), ignoring %s", MAX_REEDS, p_name);
      return;
   }

   (void)memcpy(g_reed_table[slot].mac, p_mac, 6);
   (void)strncpy(g_reed_table[slot].name, p_name,
                  sizeof(g_reed_table[slot].name) - 1);
   g_reed_table[slot].name[sizeof(g_reed_table[slot].name) - 1] = '\0';
   g_reed_table[slot].door_state   = door_state;
   g_reed_table[slot].batt         = batt;
   g_reed_table[slot].state        = SLOT_ACTIVE;
   g_reed_table[slot].last_seen_ms = now;
   g_reed_table[slot].generation++;
   gen = g_reed_table[slot].generation;

   (void)xSemaphoreGive(g_reed_mutex);

   ESP_LOGI(TAG, "[BLE_REED] slot=%d assigned name=%s gen=%u", slot, p_name, gen);
   update_room_for_slot(slot, door_state);
   eid = bus_publish_reed((uint8_t)(slot + 1), door_state, batt, p_mac);
   ESP_LOGI(TAG, "[BLE_REED] tx_id=%u event_id=%llu slot=%d state=%d batt=%d%%",
            tx_id, (unsigned long long)eid, slot, (int)door_state, batt);
}

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Expire stale reed sensor slots based on age thresholds.
 *
 * \return void
 *
 * \details Transitions ACTIVE->OFFLINE after REED_OFFLINE_MS and
 *          OFFLINE->EMPTY after REED_REMOVE_MS. Removed slots are added
 *          to the cooldown table. Call periodically (e.g. every 30 s).
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_expire_reed_slots(void)
{
   uint32_t now = 0; /**< current tick in ms */
   uint32_t age = 0; /**< slot age in ms     */
   int      i   = 0; /**< loop index         */

   now = xTaskGetTickCount() * portTICK_PERIOD_MS;

   if (NULL == g_reed_mutex)
   {
      return;
   }

   (void)xSemaphoreTake(g_reed_mutex, portMAX_DELAY);

   for (i = 0; i < MAX_REEDS; i++)
   {
      if (SLOT_EMPTY == g_reed_table[i].state)
      {
         continue;
      }

      age = now - g_reed_table[i].last_seen_ms;

      if (age > REED_REMOVE_MS)
      {
         ESP_LOGW(TAG, "[BLE_REED] slot=%d name=%s gen=%u expired — clearing",
                  i, g_reed_table[i].name, g_reed_table[i].generation);
         cooldown_add(g_reed_table[i].mac);
         (void)memset(&g_reed_table[i], 0, sizeof(REED_SLOT_T));
      }
      else if ((age > REED_OFFLINE_MS) &&
               (SLOT_ACTIVE == g_reed_table[i].state))
      {
         g_reed_table[i].state = SLOT_OFFLINE;
         ESP_LOGI(TAG, "[BLE_REED] slot=%d name=%s -> OFFLINE",
                  i, g_reed_table[i].name);
      }
      else
      {
         /* slot within thresholds — no action */
      }
   }

   (void)xSemaphoreGive(g_reed_mutex);
}

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Get count of active or offline reed sensor slots.
 *
 * \return int - Highest non-empty slot index + 1, or 0 if none.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
int ble_get_reed_count(void)
{
   int count = 0; /**< highest non-empty slot index + 1 */
   int i     = 0; /**< loop index                       */

   if (NULL == g_reed_mutex)
   {
      return 0;
   }

   (void)xSemaphoreTake(g_reed_mutex, portMAX_DELAY);

   for (i = 0; i < MAX_REEDS; i++)
   {
      if (SLOT_EMPTY != g_reed_table[i].state)
      {
         count = i + 1;
      }
   }

   (void)xSemaphoreGive(g_reed_mutex);

   return count;
}

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Get information for a reed sensor slot.
 *
 * \param slot        - Slot index (0-based).
 * \param p_name_out  - Output buffer for device name (31 chars max), or NULL.
 * \param p_batt_out  - Output for battery SOC percent, or NULL.
 * \param p_age_out   - Output for age in seconds, or NULL.
 * \param p_state_out - Output for door state byte, or NULL.
 * \param p_gen_out   - Output for generation counter, or NULL.
 *
 * \return bool - true if slot is active or offline, false if empty or OOB.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
bool ble_get_reed_slot_info(int       slot,
                             char     *p_name_out,
                             int      *p_batt_out,
                             uint16_t *p_age_out,
                             uint8_t  *p_state_out,
                             uint16_t *p_gen_out)
{
   uint32_t now_ms = 0; /**< current tick in ms  */
   uint32_t age_ms = 0; /**< slot age in ms      */
   uint32_t age_s  = 0; /**< slot age in seconds */

   if ((0 > slot) || (slot >= MAX_REEDS) || (NULL == g_reed_mutex))
   {
      return false;
   }

   (void)xSemaphoreTake(g_reed_mutex, portMAX_DELAY);

   if (SLOT_EMPTY == g_reed_table[slot].state)
   {
      (void)xSemaphoreGive(g_reed_mutex);
      return false;
   }

   if (NULL != p_name_out)
   {
      (void)strncpy(p_name_out, g_reed_table[slot].name, SLOT_NAME_MAX);
   }

   if (NULL != p_batt_out)
   {
      *p_batt_out = g_reed_table[slot].batt;
   }

   if (NULL != p_state_out)
   {
      *p_state_out = g_reed_table[slot].door_state;
   }

   if (NULL != p_gen_out)
   {
      *p_gen_out = g_reed_table[slot].generation;
   }

   if (NULL != p_age_out)
   {
      now_ms     = xTaskGetTickCount() * portTICK_PERIOD_MS;
      age_ms     = now_ms - g_reed_table[slot].last_seen_ms;
      age_s      = age_ms / 1000u;
      *p_age_out = (age_s > AGE_MAX_VALUE) ?
                   (uint16_t)AGE_MAX_VALUE : (uint16_t)age_s;
   }

   (void)xSemaphoreGive(g_reed_mutex);

   return true;
}
