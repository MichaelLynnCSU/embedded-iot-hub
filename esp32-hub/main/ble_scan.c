/******************************************************************************
 * \file ble_scan.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief BLE advertisement scanner for ESP32 hub node.
 *
 * \details Parses BLE advertisements and dispatches to per-device handlers.
 *          Manages dynamic reed sensor slot allocation with offline/removal
 *          state machine. Owns lock and light MAC discovery state.
 *
 *          Fixed devices (table-driven):
 *          - PIR_Motion  — motion count and battery
 *          - LightNF     — relay state, triggers GATT connect if pending
 *          - SmartLock   — lock state and battery, triggers GATT connect
 *
 *          Dynamic devices (prefix match):
 *          - ReedSensor* — up to MAX_REEDS slots, auto-allocated by MAC
 *
 *          Reed slot state machine:
 *          SLOT_EMPTY   — never seen or expired after REED_REMOVE_MS
 *          SLOT_ACTIVE  — advertising within REED_OFFLINE_MS
 *          SLOT_OFFLINE — last seen > REED_OFFLINE_MS, tile stays visible
 *
 *          Thresholds:
 *          - OFFLINE:  150s — matches BLE_AGE_THRESHOLD_S on STM32
 *          - REMOVE:  3600s — 1 hour unseen, slot cleared, tile hidden
 ******************************************************************************/

#include "config.h"
#include "esp_log.h"
#include "esp_gap_ble_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ble_manager.h"
#include "ble_internal.h"
#include "vroom_bus.h"
#include <string.h>

#define MAX_REEDS              6              /**< max concurrent reed sensors */
#define REED_OFFLINE_MS        (150  * 1000)  /**< offline threshold ms */
#define REED_REMOVE_MS         (3600 * 1000)  /**< removal threshold ms */
#define COOLDOWN_COUNT         MAX_REEDS      /**< cooldown table depth */
#define COOLDOWN_MS            (10   * 1000)  /**< re-discovery quiet window ms */
#define ADV_NAME_BUF_SIZE      32             /**< advertisement name buffer size */
#define REED_NAME_PREFIX       "ReedSensor"   /**< reed sensor name prefix */
#define REED_NAME_PREFIX_LEN   10             /**< length of REED_NAME_PREFIX */
#define ADV_TYPE_SHORT_NAME    0x08           /**< AD type: shortened local name */
#define ADV_TYPE_FULL_NAME     0x09           /**< AD type: complete local name */
#define REED_AGE_LOG_THRESHOLD 30             /**< seconds between periodic logs */
#define LOCK_AGE_LOG_THRESHOLD 30             /**< seconds between periodic logs */
#define MFG_PIR_MIN_LEN        6             /**< min PIR mfg data length */
#define MFG_PIR_BATT_IDX       6             /**< PIR battery byte index */
#define MFG_PIR_OCCUPIED_IDX   7             /* occupied flag byte index */
#define MFG_REED_STATE_IDX     1             /**< reed state byte index */
#define MFG_REED_BATT_IDX      2             /**< reed battery byte index */
#define MFG_LIGHT_STATE_IDX    2             /**< light state byte index */
#define MFG_LIGHT_MIN_LEN      2             /**< min light mfg data length */
#define MFG_LOCK_STATE_IDX     1             /**< lock state byte index */
#define MFG_LOCK_BATT_IDX      2             /**< lock battery byte index */
#define MFG_LOCK_MIN_LEN       3             /**< min lock mfg data length */
#define SLOT_NAME_MAX          31             /**< max slot name copy length */
#define PIR_COUNT_BYTE0        2             /**< PIR count MSB index */
#define PIR_COUNT_BYTE1        3             /**< PIR count byte 1 index */
#define PIR_COUNT_BYTE2        4             /**< PIR count byte 2 index */
#define PIR_COUNT_BYTE3        5             /**< PIR count LSB index */
#define ROOM_SENSOR_SLOT0_ID   1             /**< room sensor ID for slot 0 */
#define ROOM_SENSOR_SLOT1_ID   7             /**< room sensor ID for slot 1 */
#define AGE_MAX_VALUE          0xFFFE        /**< max reportable age value */
#define PIR_EVENT_BUF_SIZE  16u  /**< circular buffer depth for occ=1 timestamps */ 

static const char *TAG = "BLE_SCAN"; /**< ESP log tag */

uint8_t             lock_mac[6]     = {0};              /**< lock device MAC */
bool                lock_found      = false;            /**< lock MAC discovered */
esp_ble_addr_type_t lock_addr_type  = BLE_ADDR_TYPE_PUBLIC; /**< lock addr type */

uint8_t             light_mac[6]    = {0};              /**< light device MAC */
bool                light_found     = false;            /**< light MAC discovered */
esp_ble_addr_type_t light_addr_type = BLE_ADDR_TYPE_PUBLIC; /**< light addr type */

int g_motion_count = DEFAULT_MOTION_COUNT; /**< PIR motion event count */
int g_pir_batt     = -1;                  /**< PIR battery SOC percent */

/* TODO: remove extern aliases below when tcp_manager is refactored
 * to use ble_get_motion_count() and ble_get_pir_batt() accessors */
int motion_count = DEFAULT_MOTION_COUNT; /**< legacy alias for g_motion_count */
int pir_batt     = -1;                  /**< legacy alias for g_pir_batt */
int g_pir_occupied = 0;                  /* 0=empty, 1=occupied */
static uint32_t g_pir_event_buf[PIR_EVENT_BUF_SIZE]; /**< occ=1 event timestamps ms */
static int      g_pir_event_head    = 0;             /**< next write index */
static uint32_t g_pir_last_hold_ms  = 0;             /**< timestamp when hold last started */

static bool g_pir_seen   = false; /**< PIR first-seen flag */
static bool g_lock_seen  = false; /**< lock first-seen flag */
static bool g_light_seen = false; /**< light first-seen flag */

/******************************* ENUMERATIONS *********************************/

/** \brief Reed slot state machine states. */
typedef enum
{
   SLOT_EMPTY   = 0, /**< never seen or expired */
   SLOT_ACTIVE  = 1, /**< advertising within REED_OFFLINE_MS */
   SLOT_OFFLINE = 2, /**< last seen > REED_OFFLINE_MS ago */
} SLOT_STATE_E;

/************************ STRUCTURE/UNION DATA TYPES **************************/

/** \brief Reed sensor slot entry. */
typedef struct
{
   uint8_t      mac[6];        /*!< device MAC address */
   char         name[ADV_NAME_BUF_SIZE]; /*!< BLE device name */
   uint8_t      door_state;    /*!< 0=closed 1=open 0xFF=unknown */
   int          batt;          /*!< battery SOC percent */
   SLOT_STATE_E state;         /*!< slot state machine state */
   uint32_t     last_seen_ms;  /*!< timestamp of last advertisement */
   uint16_t     generation;    /*!< increments each time slot is reused */
} REED_SLOT_T;

/** \brief Cooldown table entry — prevents immediate slot re-allocation. */
typedef struct
{
   uint8_t  mac[6];           /*!< MAC of recently removed device */
   uint32_t removed_at_ms;    /*!< timestamp of removal */
} COOLDOWN_ENTRY_T;

/** \brief Advertisement handler function pointer type. */
typedef void (*adv_handler_t)(const uint8_t *p_adv,
                               int len,
                               const uint8_t *p_mac,
                               uint8_t addr_type);

/** \brief Fixed device dispatch table entry. */
typedef struct
{
   const char    *p_name;   /*!< exact BLE device name to match */
   adv_handler_t  handler;  /*!< handler function for this device */
} DEVICE_ENTRY_T;

static REED_SLOT_T     g_reed_table[MAX_REEDS];          /**< reed slot table */
static COOLDOWN_ENTRY_T g_cooldown_table[COOLDOWN_COUNT]; /**< cooldown table */
static StaticSemaphore_t g_reed_mutex_buf;               /**< static mutex buffer */
static SemaphoreHandle_t g_reed_mutex = NULL;            /**< reed table mutex */

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
   uint32_t now     = 0; /**< current tick in ms */
   int      oldest  = 0; /**< index of oldest cooldown entry */
   int      i       = 0; /**< loop index */

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
   int      i   = 0; /**< loop index */

   now = xTaskGetTickCount() * portTICK_PERIOD_MS;

   for (i = 0; i < COOLDOWN_COUNT; i++)
   {
      if (0 == memcmp(g_cooldown_table[i].mac, p_mac, 6))
      {
         if ((now - g_cooldown_table[i].removed_at_ms) < COOLDOWN_MS)
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
 * \brief Expire stale reed sensor slots based on age thresholds.
 *
 * \return void
 *
 * \details Transitions ACTIVE->OFFLINE after REED_OFFLINE_MS and
 *          OFFLINE->EMPTY after REED_REMOVE_MS. Removed slots are
 *          added to the cooldown table. Call periodically (e.g. every 30s).
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_expire_reed_slots(void)
{
   uint32_t now = 0; /**< current tick in ms */
   uint32_t age = 0; /**< slot age in ms */
   int      i   = 0; /**< loop index */

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
         ESP_LOGW(TAG, "Reed slot %d (%s gen=%u) expired — clearing",
                  i, g_reed_table[i].name, g_reed_table[i].generation);
         cooldown_add(g_reed_table[i].mac);
         (void)memset(&g_reed_table[i], 0, sizeof(REED_SLOT_T));
      }
      else if ((age > REED_OFFLINE_MS) &&
               (SLOT_ACTIVE == g_reed_table[i].state))
      {
         g_reed_table[i].state = SLOT_OFFLINE;
         ESP_LOGI(TAG, "Reed slot %d (%s) -> OFFLINE",
                  i, g_reed_table[i].name);
      }
      else
      {
         /* slot within thresholds — no action */
      }
   }

   (void)xSemaphoreGive(g_reed_mutex);
}

/******************************************************************************
 * \brief Extract BLE device name from advertisement payload.
 *
 * \param p_adv  - Pointer to raw advertisement data.
 * \param len    - Length of advertisement data in bytes.
 * \param p_out  - Output buffer for null-terminated name.
 * \param out_sz - Size of output buffer.
 *
 * \return bool - true if name was found and extracted, false otherwise.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static bool extract_name(const uint8_t *p_adv,
                          int len,
                          char *p_out,
                          int out_sz)
{
   int     i        = 0;    /**< advertisement parse index */
   uint8_t flen     = 0;    /**< AD structure length */
   uint8_t ftype    = 0;    /**< AD structure type */
   int     name_len = 0;    /**< extracted name length */

   while (i < len)
   {
      flen = p_adv[i];
      i++;

      if ((0 == flen) || ((i + flen) > len))
      {
         break;
      }

      ftype = p_adv[i];

      if ((ADV_TYPE_SHORT_NAME == ftype) || (ADV_TYPE_FULL_NAME == ftype))
      {
         name_len = flen - 1;

         if (name_len >= out_sz)
         {
            name_len = out_sz - 1;
         }

         (void)memcpy(p_out, &p_adv[i + 1], name_len);
         p_out[name_len] = '\0';
         return true;
      }

      i += flen;
   }

   return false;
}

/******************************************************************************
 * \brief Find manufacturer specific data in advertisement payload.
 *
 * \param p_adv    - Pointer to raw advertisement data.
 * \param len      - Length of advertisement data in bytes.
 * \param p_out_len - Output for manufacturer data length in bytes.
 *
 * \return const uint8_t* - Pointer to manufacturer data, or NULL if not found.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static const uint8_t *find_mfg_data(const uint8_t *p_adv,
                                     int len,
                                     int *p_out_len)
{
   int     i     = 0; /**< advertisement parse index */
   uint8_t flen  = 0; /**< AD structure length */
   uint8_t ftype = 0; /**< AD structure type */

   while (i < len)
   {
      flen = p_adv[i];
      i++;

      if ((0 == flen) || ((i + flen) > len))
      {
         break;
      }

      ftype = p_adv[i];

      if (ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE == ftype)
      {
         *p_out_len = flen - 1;
         return &p_adv[i + 1];
      }

      i += flen;
   }

   return NULL;
}

/******************************************************************************
 * \brief Update room sensor state string for a reed slot.
 *
 * \param slot       - Reed slot index.
 * \param door_state - Door state byte (0=closed, 1=open, 0xFF=unknown).
 *
 * \return void
 *
 * \details Maps slot 0 to room sensor ID ROOM_SENSOR_SLOT0_ID and
 *          slot 1 to ROOM_SENSOR_SLOT1_ID. No-op for other slots.
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

/******************************************************************************
 * \brief Update PIR sliding window and recompute g_pir_occupied.
 *
 * \param now_ms   - Current tick count in ms.
 * \param occ      - Raw occupied flag from BLE advertisement (0 or 1).
 *
 * \return void
 *
 * \details Implements a time-based sliding window over occ=1 events.
 *          On each call with occ=1, the current timestamp is written
 *          into a circular buffer. The window then counts how many
 *          buffered timestamps fall within the last PIR_WINDOW_SEC.
 *          If the count meets PIR_WINDOW_THRESHOLD, the hold timer is
 *          (re)started. g_pir_occupied is 1 for the duration of the
 *          hold and 0 once PIR_HOLD_SEC elapses with no new trigger.
 *
 *          Window size:  PIR_WINDOW_SEC       (60s)
 *          Threshold:    PIR_WINDOW_THRESHOLD  (2 events)
 *          Hold:         PIR_HOLD_SEC          (600s / 10 min)
 *
 *          The 10s device heartbeat (occ=0) is the resolution floor --
 *          the window cannot meaningfully be smaller than ~20-30s.
 *          motion_count delta can be used to detect missed occ=1 bursts
 *          during lossy BLE periods.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void pir_window_update(uint32_t now_ms, int occ)
{
    uint32_t window_ms  = 0u; /**< window width in ms */
    uint32_t hold_ms    = 0u; /**< hold duration in ms */
    int      count      = 0;  /**< events inside window */
    int      i          = 0;  /**< loop index */

    window_ms = PIR_WINDOW_SEC  * 1000u;
    hold_ms   = PIR_HOLD_SEC    * 1000u;

    /* Stamp event into circular buffer on occ=1 */
    if (1 == occ)
    {
        g_pir_event_buf[g_pir_event_head] = now_ms;
        g_pir_event_head = (g_pir_event_head + 1) % PIR_EVENT_BUF_SIZE;
    }

    /* Count events inside the sliding window */
    for (i = 0; i < PIR_EVENT_BUF_SIZE; i++)
    {
        if ((g_pir_event_buf[i] > 0u) &&
            ((now_ms - g_pir_event_buf[i]) <= window_ms))
        {
            count++;
        }
    }

    /* Restart hold timer if threshold met */
    if (count >= (int)PIR_WINDOW_THRESHOLD)
    {
        g_pir_last_hold_ms = now_ms;
    }

    /* Occupied for duration of hold after last threshold crossing */
    if ((g_pir_last_hold_ms > 0u) &&
        ((now_ms - g_pir_last_hold_ms) <= hold_ms))
    {
        g_pir_occupied = 1;
    }
    else
    {
        g_pir_occupied = 0;
    }
}

/******************************************************************************
 * \brief Handle PIR motion sensor advertisement.
 *
 * \param p_adv    - Pointer to raw advertisement data.
 * \param len      - Length of advertisement data.
 * \param p_mac    - Pointer to device MAC address (unused).
 * \param addr_type - BLE address type (unused).
 *
 * \return void
 *
 * \details Extracts motion count and battery SOC from manufacturer data.
 *          Publishes to vroom bus on first seen or count change.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void handle_pir(const uint8_t *p_adv,
                        int len,
                        const uint8_t *p_mac,
                        uint8_t addr_type)
{
   int             mfg_len    = 0;    /**< manufacturer data length */
   const uint8_t  *p_mfg     = NULL; /**< manufacturer data pointer */
   uint32_t        count      = 0;    /**< motion event count */
   bool            first_seen = false; /**< first advertisement flag */
   bool            changed    = false; /**< state changed flag */

   (void)p_mac;
   (void)addr_type;

   p_mfg = find_mfg_data(p_adv, len, &mfg_len);
   if ((NULL == p_mfg) || (mfg_len < MFG_PIR_MIN_LEN))
   {
      return;
   }

   count = ((uint32_t)p_mfg[PIR_COUNT_BYTE0] << 24) |
           ((uint32_t)p_mfg[PIR_COUNT_BYTE1] << 16) |
           ((uint32_t)p_mfg[PIR_COUNT_BYTE2] <<  8) |
            (uint32_t)p_mfg[PIR_COUNT_BYTE3];

   first_seen   = !g_pir_seen;
   changed      = ((int)count != motion_count);
   motion_count = (int)count;
   g_motion_count = motion_count;

   if (mfg_len >= MFG_PIR_BATT_IDX)
   {
      pir_batt   = (int)p_mfg[MFG_PIR_BATT_IDX];
      g_pir_batt = pir_batt;
   }

   if (mfg_len >= (MFG_PIR_OCCUPIED_IDX + 1))
   {
      uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
      pir_window_update(now_ms, (int)p_mfg[MFG_PIR_OCCUPIED_IDX]);
   }

   stamp_device(DEV_IDX_PIR);

   if (first_seen || changed)
   {
      g_pir_seen = true;
      bus_publish_pir(count, pir_batt);
      ESP_LOGI(TAG, ">>> PIR count=%u batt=%d%%", count, pir_batt);
   }
   ESP_LOGI(TAG, ">>> even if no change PIR count=%u batt=%d%%", count, pir_batt);
}

/******************************************************************************
 * \brief Handle dynamic reed sensor advertisement.
 *
 * \param p_adv    - Pointer to raw advertisement data.
 * \param len      - Length of advertisement data.
 * \param p_mac    - Pointer to device MAC address.
 * \param addr_type - BLE address type (unused).
 * \param p_name   - Null-terminated device name string.
 *
 * \return void
 *
 * \details Implements reed slot state machine. Known MACs update their
 *          slot. New MACs are allocated a slot if not in cooldown and
 *          table is not full.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void handle_reed_dynamic(const uint8_t *p_adv,
                                 int len,
                                 const uint8_t *p_mac,
                                 uint8_t addr_type,
                                 const char *p_name)
{
   int            mfg_len    = 0;    /**< manufacturer data length */
   const uint8_t *p_mfg     = NULL; /**< manufacturer data pointer */
   uint8_t        door_state = 0xFF; /**< door state byte */
   int            batt       = -1;  /**< battery SOC percent */
   uint32_t       now        = 0;   /**< current tick in ms */
   int            slot       = -1;  /**< slot index */
   bool           was_offline = false; /**< slot was offline flag */
   uint16_t       gen        = 0;   /**< slot generation counter */

   (void)addr_type;

   p_mfg = find_mfg_data(p_adv, len, &mfg_len);

   if ((NULL != p_mfg) && (mfg_len >= (MFG_REED_STATE_IDX + 1)))
   {
      door_state = p_mfg[MFG_REED_STATE_IDX];
   }

   if ((NULL != p_mfg) && (mfg_len >= (MFG_REED_BATT_IDX + 1)))
   {
      batt = (int)p_mfg[MFG_REED_BATT_IDX];
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
         ESP_LOGI(TAG, "Reed slot %d (%s) -> ACTIVE (recovered)",
                  slot, p_name);
      }

      update_room_for_slot(slot, door_state);
      bus_publish_reed((uint8_t)(slot + 1), door_state, batt, p_mac);
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
      ESP_LOGW(TAG, "Reed table full (%d), ignoring %s", MAX_REEDS, p_name);
      return;
   }

   (void)memcpy(g_reed_table[slot].mac, p_mac, 6);
   (void)strncpy(g_reed_table[slot].name,
                  p_name,
                  sizeof(g_reed_table[slot].name) - 1);
   g_reed_table[slot].door_state   = door_state;
   g_reed_table[slot].batt         = batt;
   g_reed_table[slot].state        = SLOT_ACTIVE;
   g_reed_table[slot].last_seen_ms = now;
   g_reed_table[slot].generation++;

   gen = g_reed_table[slot].generation;
   (void)xSemaphoreGive(g_reed_mutex);

   ESP_LOGI(TAG, "Reed slot %d assigned to %s (gen=%u)", slot, p_name, gen);

   update_room_for_slot(slot, door_state);
   bus_publish_reed((uint8_t)(slot + 1), door_state, batt, p_mac);
}

/******************************************************************************
 * \brief Handle LightNF smart light advertisement.
 *
 * \param p_adv    - Pointer to raw advertisement data.
 * \param len      - Length of advertisement data.
 * \param p_mac    - Pointer to device MAC address.
 * \param addr_type - BLE address type.
 *
 * \return void
 *
 * \details Extracts relay state from manufacturer data. Records MAC on
 *          first seen. Triggers GATT connect attempt if command pending.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void handle_light(const uint8_t *p_adv,
                          int len,
                          const uint8_t *p_mac,
                          uint8_t addr_type)
{
   int            mfg_len   = 0;    /**< manufacturer data length */
   const uint8_t *p_mfg    = NULL; /**< manufacturer data pointer */
   uint8_t        new_state = 0;   /**< extracted relay state */
   bool           first_seen = false; /**< first advertisement flag */
   bool           changed    = false; /**< state changed flag */
   bool           log_due    = false; /**< periodic log due flag */

   p_mfg = find_mfg_data(p_adv, len, &mfg_len);
   if ((NULL == p_mfg) || (mfg_len < MFG_LIGHT_MIN_LEN))
   {
      return;
   }

   new_state  = p_mfg[MFG_LIGHT_STATE_IDX];
   first_seen = !g_light_seen;
   changed    = !first_seen &&
                (new_state != (uint8_t)ble_get_light_state());
   log_due    = (ble_get_device_age_s(DEV_IDX_LIGHT) >=
                 REED_AGE_LOG_THRESHOLD);

   ble_light_update_adv(new_state);
   stamp_device(DEV_IDX_LIGHT);

   if (first_seen || changed || log_due)
   {
      g_light_seen = true;
      bus_publish_light(new_state);
      ESP_LOGI(TAG, ">>> LIGHT state=%d", new_state);
   }

   if (!light_found)
   {
      (void)memcpy(light_mac, p_mac, 6);
      light_found     = true;
      light_addr_type = addr_type;
   }

   ble_light_try_connect();
}

/******************************************************************************
 * \brief Handle SmartLock advertisement.
 *
 * \param p_adv    - Pointer to raw advertisement data.
 * \param len      - Length of advertisement data.
 * \param p_mac    - Pointer to device MAC address.
 * \param addr_type - BLE address type.
 *
 * \return void
 *
 * \details Extracts lock state and battery from manufacturer data.
 *          Records MAC on first seen. Triggers GATT connect if pending.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void handle_lock(const uint8_t *p_adv,
                         int len,
                         const uint8_t *p_mac,
                         uint8_t addr_type)
{
   int            mfg_len   = 0;    /**< manufacturer data length */
   const uint8_t *p_mfg    = NULL; /**< manufacturer data pointer */
   uint8_t        new_state = 0;   /**< extracted lock state */
   uint8_t        new_batt  = 0;   /**< extracted battery SOC */
   bool           first_seen = false; /**< first advertisement flag */
   bool           changed    = false; /**< state changed flag */
   bool           log_due    = false; /**< periodic log due flag */

   p_mfg = find_mfg_data(p_adv, len, &mfg_len);
   if ((NULL == p_mfg) || (mfg_len < MFG_LOCK_MIN_LEN))
   {
      return;
   }

   new_state  = p_mfg[MFG_LOCK_STATE_IDX];
   new_batt   = p_mfg[MFG_LOCK_BATT_IDX];
   first_seen = !g_lock_seen;
   changed    = !first_seen &&
                ((new_state != (uint8_t)ble_get_lock_state()) ||
                 (new_batt  != (uint8_t)ble_lock_get_batt()));
   log_due    = (ble_get_device_age_s(DEV_IDX_LOCK) >=
                 LOCK_AGE_LOG_THRESHOLD);

   ble_lock_update_adv(new_state, new_batt);
   stamp_device(DEV_IDX_LOCK);

   if (first_seen || changed || log_due)
   {
      g_lock_seen = true;
      bus_publish_lock(new_state, new_batt);
      ESP_LOGI(TAG, ">>> LOCK state=%d batt=%d%%", new_state, new_batt);
   }

   if (!lock_found)
   {
      (void)memcpy(lock_mac, p_mac, 6);
      lock_found     = true;
      lock_addr_type = addr_type;
   }

   ble_lock_try_connect();
}

static const DEVICE_ENTRY_T g_device_table[] =
{
   { "PIR_Motion", handle_pir   },
   { "LightNF",    handle_light },
   { "SmartLock",  handle_lock  },
};

#define DEVICE_TABLE_SIZE \
   (sizeof(g_device_table) / sizeof(g_device_table[0])) /**< device table count */

/******************************************************************************
 * \brief Parse a BLE advertisement and dispatch to the appropriate handler.
 *
 * \param p_adv    - Pointer to raw advertisement data.
 * \param len      - Length of advertisement data.
 * \param p_mac    - Pointer to device MAC address.
 * \param addr_type - BLE address type.
 *
 * \return void
 *
 * \details Reed sensors matched by name prefix. Fixed devices matched
 *          by exact name via g_device_table.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void parse_advertisement(const uint8_t *p_adv,
                                 int len,
                                 const uint8_t *p_mac,
                                 uint8_t addr_type)
{
   char name[ADV_NAME_BUF_SIZE] = {0}; /**< extracted device name */
   int  i = 0;                         /**< loop index */

   if (!extract_name(p_adv, len, name, sizeof(name)))
   {
      return;
   }

   if (0 == strncmp(name, REED_NAME_PREFIX, REED_NAME_PREFIX_LEN))
   {
      handle_reed_dynamic(p_adv, len, p_mac, addr_type, name);
      return;
   }

   for (i = 0; i < (int)DEVICE_TABLE_SIZE; i++)
   {
      if (0 == strcmp(name, g_device_table[i].p_name))
      {
         g_device_table[i].handler(p_adv, len, p_mac, addr_type);
         return;
      }
   }
}

/******************************************************************************
 * \brief GAP event handler for BLE scan results.
 *
 * \param event   - GAP event type.
 * \param p_param - Pointer to GAP event parameters.
 *
 * \return void
 *
 * \details Handles scan parameter set complete, scan start complete,
 *          and scan result events. Dispatches advertisements to
 *          parse_advertisement().
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                               esp_ble_gap_cb_param_t *p_param)
{
   switch (event)
   {
      case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
      {
         ESP_LOGI(TAG, "Scan params set, starting scan");
         (void)esp_ble_gap_start_scanning(0);
         break;
      }

      case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
      {
         if (ESP_BT_STATUS_SUCCESS == p_param->scan_start_cmpl.status)
         {
            ESP_LOGI(TAG, "Scan started");
         }
         else
         {
            ESP_LOGE(TAG, "Scan start failed: %d",
                     p_param->scan_start_cmpl.status);
         }

         break;
      }

      case ESP_GAP_BLE_SCAN_RESULT_EVT:
      {
         if (ESP_GAP_SEARCH_INQ_RES_EVT != p_param->scan_rst.search_evt)
         {
            break;
         }

         parse_advertisement(p_param->scan_rst.ble_adv,
                              p_param->scan_rst.adv_data_len,
                              p_param->scan_rst.bda,
                              p_param->scan_rst.ble_addr_type);
         break;
      }

      default:
      {
         /* unhandled event — ignore */
         break;
      }
   }
}

/******************************************************************************
 * \brief Get count of active or offline reed sensor slots.
 *
 * \return int - Highest non-empty slot index + 1, or 0 if none.
 *
 * \details Returns the count such that all slots 0..count-1 are visible
 *          on the dashboard. SLOT_EMPTY slots beyond the last active
 *          slot cause the count to stop.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
int ble_get_reed_count(void)
{
   int count = 0; /**< highest non-empty slot index + 1 */
   int i     = 0; /**< loop index */

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

/******************************************************************************
 * \brief Get information for a reed sensor slot.
 *
 * \param slot        - Slot index (0-based).
 * \param p_name_out  - Output buffer for device name (31 chars max), or NULL.
 * \param p_batt_out  - Output for battery SOC, or NULL.
 * \param p_age_out   - Output for age in seconds, or NULL.
 * \param p_state_out - Output for door state, or NULL.
 * \param p_gen_out   - Output for generation counter, or NULL.
 *
 * \return bool - true if slot is active or offline, false if empty or OOB.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
bool ble_get_reed_slot_info(int slot,
                             char     *p_name_out,
                             int      *p_batt_out,
                             uint16_t *p_age_out,
                             uint8_t  *p_state_out,
                             uint16_t *p_gen_out)
{
   uint32_t now_ms = 0; /**< current tick in ms */
   uint32_t age_ms = 0; /**< slot age in ms */
   uint32_t age_s  = 0; /**< slot age in seconds */

   if ((0 > slot) || (slot >= MAX_REEDS))
   {
      return false;
   }

   if (NULL == g_reed_mutex)
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
      now_ms  = xTaskGetTickCount() * portTICK_PERIOD_MS;
      age_ms  = now_ms - g_reed_table[slot].last_seen_ms;
      age_s   = age_ms / 1000;
      *p_age_out = (age_s > AGE_MAX_VALUE) ?
                   (uint16_t)AGE_MAX_VALUE : (uint16_t)age_s;
   }

   (void)xSemaphoreGive(g_reed_mutex);

   return true;
}

/******************************************************************************
 * \brief Start BLE scanning — called by ble_manager.c after ble_gattc_init().
 *
 * \return void
 *
 * \details Registers GAP callback and sets scan parameters. Scanning
 *          begins asynchronously via ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_scan_start(void)
{
   esp_err_t ret = ESP_OK; /**< esp return value */

   static esp_ble_scan_params_t scan_params =
   {
      .scan_type          = BLE_SCAN_TYPE_ACTIVE,
      .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
      .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
      .scan_interval      = BLE_SCAN_INTERVAL,
      .scan_window        = BLE_SCAN_WINDOW,
      .scan_duplicate     = BLE_SCAN_DUPLICATE,
   };

   configASSERT(g_reed_mutex);

   ret = esp_ble_gap_register_callback(gap_event_handler);
   if (ESP_OK != ret)
   {
      ESP_LOGE(TAG, "GAP callback register failed: %s",
               esp_err_to_name(ret));
      return;
   }

   ret = esp_ble_gap_set_scan_params(&scan_params);
   if (ESP_OK == ret)
   {
      ESP_LOGI(TAG, "Scan params configured");
   }
   else
   {
      ESP_LOGE(TAG, "Scan params failed: %s", esp_err_to_name(ret));
   }
}

/******************************************************************************
 * \brief Pre-initialize the BLE scan module.
 *
 * \return void
 *
 * \details Creates reed mutex and zeroes reed and cooldown tables.
 *          Must be called before ble_gattc_init() and ble_scan_start().
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void ble_scan_preinit(void)
{
   g_reed_mutex = xSemaphoreCreateMutexStatic(&g_reed_mutex_buf);
   configASSERT(g_reed_mutex);
   (void)memset(g_reed_table,     0, sizeof(g_reed_table));
   (void)memset(g_cooldown_table, 0, sizeof(g_cooldown_table));
}
