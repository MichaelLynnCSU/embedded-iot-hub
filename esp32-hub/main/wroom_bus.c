/******************************************************************************
 * \file wroom_bus.c
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
 *
 * \note    Structured event tracing — Phase 0 (2026-06-15):
 *          [WROOM] ingest log added to every bus_publish_* function.
 *          Normalized prefix style matches [BLE_*], [TCP], [UART].
 *          No struct or ABI changes.
 *
 * \note    Structured event tracing — Phase 1 (2026-06-15):
 *          g_bus_seq static counter added. Increments on every publish.
 *          Included in every [WROOM] ingest log line.
 *          No cross-module propagation. No struct or ABI changes.
 *
 * \note    Structured event tracing — Phase 2 (2026-06-15):
 *          g_event_seq static counter added. wroom_event_id_generate()
 *          static function assigns monotonic event_id per publish.
 *          event_id logged at [WROOM] ingest only — not exported.
 *          No external ABI changes.
 *
 * \note    Structured event tracing — Phase 3 (2026-06-15):
 *          bus_publish_pir(), bus_publish_reed(), bus_publish_ble_temp(),
 *          bus_publish_lock(), bus_publish_light() now return uint64_t
 *          event_id so BLE callers can log the correlation key at ingress.
 *          bus_publish_temp(), bus_publish_motor(), bus_publish_doorbell()
 *          remain void — UART/motor/doorbell not in this phase.
 *          TCP/UART still unaware of event_id. No struct changes.
 ******************************************************************************/

#include "wroom_bus.h"
#include "esp_log.h"
#include <string.h>
#include "nvs_flash.h"
#include "nvs.h"

static const char *TAG = "WROOM_BUS";

static BUS_SUBSCRIBER_T g_subscribers[BUS_MAX_SUBSCRIBERS];
static int              g_sub_count  = 0;
static uint32_t         g_bus_seq    = 0;  /* Phase 1: Local bus sequence counter */
static uint64_t         g_event_seq  = 0;  /* Phase 2: Authority event_id sequencer */
static portMUX_TYPE      g_event_seq_mux = portMUX_INITIALIZER_UNLOCKED; /**< guards g_event_seq */

/*
 * NOTE — RACE CONDITION (fixed):
 *          wroom_event_id_generate() originally did a bare "g_event_seq++"
 *          with no locking. bus_publish_pir/reed/lock/light/temp/etc. each
 *          call this from their own FreeRTOS task, so two tasks could read
 *          the same g_event_seq value, both increment locally, and both
 *          write back the same result — producing duplicate event_ids
 *          under concurrent publishes. Confirmed in the wild: event_id=9999
 *          was assigned to both a PIR event (5/29) and a REED event (5/31)
 *          with continuous uptime and no restarts anywhere in the chain,
 *          which ruled out wraparound/reset as the cause and pointed at
 *          a plain read-modify-write race instead.
 *
 *          Fixed by guarding the increment with taskENTER_CRITICAL/
 *          taskEXIT_CRITICAL + g_event_seq_mux, matching the portMUX_TYPE
 *          pattern already used for state mutexes in ble_light.c/ble_lock.c.
 *
 *          Any future counter/global added to this file that's touched
 *          from more than one task needs the same treatment — a bare
 *          increment/read-modify-write here is not safe by default.
 */
#define WROOM_NVS_NAMESPACE   "wroom_bus"
#define WROOM_NVS_SEQ_KEY     "event_seq"
#define WROOM_SEQ_SAVE_EVERY  50U   /* persist every N increments to limit flash wear */

static uint32_t g_seq_since_save = 0;

static void wroom_event_seq_save(uint64_t value)
{
   nvs_handle_t handle = 0;
   if (ESP_OK != nvs_open(WROOM_NVS_NAMESPACE, NVS_READWRITE, &handle)) { return; }
   (void)nvs_set_u64(handle, WROOM_NVS_SEQ_KEY, value);
   (void)nvs_commit(handle);
   nvs_close(handle);
}

static uint64_t wroom_event_seq_load(void)
{
   nvs_handle_t handle = 0;
   uint64_t     value  = 0;
   if (ESP_OK != nvs_open(WROOM_NVS_NAMESPACE, NVS_READONLY, &handle)) { return 0; }
   (void)nvs_get_u64(handle, WROOM_NVS_SEQ_KEY, &value);
   nvs_close(handle);
   return value;
}

static uint64_t wroom_event_id_generate(void)
{
   uint64_t id;
   bool     should_save;

   taskENTER_CRITICAL(&g_event_seq_mux);
   id = ++g_event_seq;
   g_seq_since_save++;
   should_save = (g_seq_since_save >= WROOM_SEQ_SAVE_EVERY);
   if (should_save) { g_seq_since_save = 0; }
   taskEXIT_CRITICAL(&g_event_seq_mux);

   if (should_save) { wroom_event_seq_save(id); }

   return id;
}

BUS_SUBSCRIBER_T bus_register_subscriber(EventBits_t mask)
{
   BUS_SUBSCRIBER_T sub = {0};

   if (g_sub_count >= BUS_MAX_SUBSCRIBERS) {
      return sub;
   }

   sub.events      = xEventGroupCreate();
   sub.mask        = mask;
   sub.mb_pir      = xQueueCreate(1, sizeof(PIR_PAYLOAD_T));
   sub.mb_reed     = xQueueCreate(1, sizeof(REED_PAYLOAD_T));
   sub.mb_lock     = xQueueCreate(1, sizeof(LOCK_PAYLOAD_T));
   sub.mb_light    = xQueueCreate(1, sizeof(LIGHT_PAYLOAD_T));
   sub.mb_temp     = xQueueCreate(1, sizeof(TEMP_PAYLOAD_T));
   sub.mb_motor    = xQueueCreate(1, sizeof(MOTOR_PAYLOAD_T));
   sub.mb_ble_temp = xQueueCreate(1, sizeof(BLE_TEMP_PAYLOAD_T));
   sub.mb_doorbell = xQueueCreate(1, sizeof(DOORBELL_PAYLOAD_T));

   g_subscribers[g_sub_count] = sub;
   g_sub_count++;

   return sub;
}

void bus_init(void)
{
   g_event_seq = wroom_event_seq_load();
   ESP_LOGI(TAG, "[WROOM] Bus initialized, resuming event_id from %llu",
            (unsigned long long)g_event_seq);
}

static void bus_signal(EventBits_t bits)
{
   for (int i = 0; i < g_sub_count; i++) {
      if (g_subscribers[i].mask & bits) {
         (void)xEventGroupSetBits(g_subscribers[i].events, bits);
      }
   }
}

uint64_t bus_publish_pir(uint8_t slot, uint32_t count, int batt)
{
   uint64_t eid = wroom_event_id_generate();
   PIR_PAYLOAD_T p = { .slot = slot, .id = slot, .count = count,
                       .batt = batt, .event_id = eid };

   g_bus_seq++;

   for (int i = 0; i < g_sub_count; i++) {
      if (g_subscribers[i].mask & EVT_BLE_PIR) {
         (void)xQueueOverwrite(g_subscribers[i].mb_pir, &p);
      }
   }

   // Normalized Log Format Matching Spec
   ESP_LOGI(TAG, "[WROOM] event_id=%llu bus_seq=%u ingest type=BLE_PIR device_id=%d motion=%u",
            (unsigned long long)eid, (unsigned)g_bus_seq, (int)slot, (unsigned)count);

   bus_signal(EVT_BLE_PIR);
   return eid;
}

uint64_t bus_publish_reed(uint8_t id, uint8_t state, int batt, const uint8_t *p_mac)
{
   uint64_t eid = wroom_event_id_generate();
   REED_PAYLOAD_T p = { .id = id, .state = state, .batt = batt, .event_id = eid };
   if (p_mac) { memcpy(p.mac, p_mac, 6); }
   g_bus_seq++;

   for (int i = 0; i < g_sub_count; i++) {
      if (g_subscribers[i].mask & EVT_BLE_REED) {
         (void)xQueueOverwrite(g_subscribers[i].mb_reed, &p);
      }
   }

   ESP_LOGI(TAG, "[WROOM] event_id=%llu bus_seq=%u ingest type=BLE_REED device_id=%d state=%d",
            (unsigned long long)eid, (unsigned)g_bus_seq, (int)id, (int)state);

   bus_signal(EVT_BLE_REED);
   return eid;
}

uint64_t bus_publish_lock(uint8_t state, int batt)
{
   uint64_t eid = wroom_event_id_generate();
   LOCK_PAYLOAD_T p = { .state = state, .batt = batt, .event_id = eid };

   g_bus_seq++;

   for (int i = 0; i < g_sub_count; i++) {
      if (g_subscribers[i].mask & EVT_BLE_LOCK) {
         (void)xQueueOverwrite(g_subscribers[i].mb_lock, &p);
      }
   }

   ESP_LOGI(TAG, "[WROOM] event_id=%llu bus_seq=%u ingest type=BLE_LOCK state=%d",
            (unsigned long long)eid, (unsigned)g_bus_seq, (int)state);

   bus_signal(EVT_BLE_LOCK);
   return eid;
}

uint64_t bus_publish_light(uint8_t state)
{
   uint64_t eid = wroom_event_id_generate();
   LIGHT_PAYLOAD_T p = { .state = state, .event_id = eid };

   g_bus_seq++;

   for (int i = 0; i < g_sub_count; i++) {
      if (g_subscribers[i].mask & EVT_BLE_LIGHT) {
         (void)xQueueOverwrite(g_subscribers[i].mb_light, &p);
      }
   }

   ESP_LOGI(TAG, "[WROOM] event_id=%llu bus_seq=%u ingest type=BLE_LIGHT state=%d",
            (unsigned long long)eid, (unsigned)g_bus_seq, (int)state);

   bus_signal(EVT_BLE_LIGHT);
   return eid;
}

uint64_t bus_publish_temp(int avg_temp)
{
   uint64_t eid = wroom_event_id_generate();
   TEMP_PAYLOAD_T p = { .avg_temp = avg_temp };

   g_bus_seq++;

   for (int i = 0; i < g_sub_count; i++) {
      if (g_subscribers[i].mask & EVT_UART_TEMP) {
         (void)xQueueOverwrite(g_subscribers[i].mb_temp, &p);
      }
   }

   ESP_LOGI(TAG, "[WROOM] event_id=%llu bus_seq=%u ingest type=UART_TEMP avg_temp=%d",
            (unsigned long long)eid, (unsigned)g_bus_seq, avg_temp);

   bus_signal(EVT_UART_TEMP);
   return eid;
}

uint64_t bus_publish_motor(uint8_t online, int batt)
{
   uint64_t eid = wroom_event_id_generate();
   MOTOR_PAYLOAD_T p = { .online = online, .batt = batt };

   g_bus_seq++;

   for (int i = 0; i < g_sub_count; i++) {
      if (g_subscribers[i].mask & EVT_MOTOR_STATUS) {
         (void)xQueueOverwrite(g_subscribers[i].mb_motor, &p);
      }
   }

   ESP_LOGI(TAG, "[WROOM] event_id=%llu bus_seq=%u ingest type=MOTOR online=%d",
            (unsigned long long)eid, (unsigned)g_bus_seq, (int)online);

   bus_signal(EVT_MOTOR_STATUS);
   return eid;
}

uint64_t bus_publish_ble_temp(uint8_t slot, int16_t temp_decidegc, int batt)
{
   uint64_t eid = wroom_event_id_generate();
   BLE_TEMP_PAYLOAD_T p = { .id = slot, .temp_decidegc = temp_decidegc,
                             .batt = batt, .event_id = eid };

   g_bus_seq++;

   for (int i = 0; i < g_sub_count; i++) {
      if (g_subscribers[i].mask & EVT_BLE_TEMP) {
         (void)xQueueOverwrite(g_subscribers[i].mb_ble_temp, &p);
      }
   }

   ESP_LOGI(TAG, "[WROOM] event_id=%llu bus_seq=%u ingest type=BLE_TEMP device_id=%d temp_dc=%d",
            (unsigned long long)eid, (unsigned)g_bus_seq, (int)slot, (int)temp_decidegc);

   bus_signal(EVT_BLE_TEMP);
   return eid;
}

uint64_t bus_publish_doorbell(uint8_t device_id, uint64_t event_id, uint64_t timestamp_ms)
{
   uint64_t eid = wroom_event_id_generate();
   DOORBELL_PAYLOAD_T p = { .device_id = device_id, .event_id = event_id, .timestamp_ms = timestamp_ms };

   g_bus_seq++;

   for (int i = 0; i < g_sub_count; i++) {
      if (g_subscribers[i].mask & EVT_DOORBELL) {
         (void)xQueueOverwrite(g_subscribers[i].mb_doorbell, &p);
      }
   }

   ESP_LOGI(TAG, "[WROOM] event_id=%llu bus_seq=%u ingest type=DOORBELL device_id=%d cam_event_id=%llu",
            (unsigned long long)eid, (unsigned)g_bus_seq, (int)device_id, (unsigned long long)event_id);

   bus_signal(EVT_DOORBELL);
   return eid;
}
