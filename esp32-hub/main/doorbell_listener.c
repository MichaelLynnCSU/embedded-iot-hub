/******************************************************************************
 * \file doorbell_listener.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief UDP doorbell event listener for ESP32 hub node.
 *
 * \details Binds UDP socket on HUB_DOORBELL_UDP_PORT, receives JSON
 *          envelopes from ESP32 doorbell cam, parses device_id /
 *          event_id / timestamp_ms, publishes to vroom bus.
 *
 *          JSON envelope (sent by cam):
 *          {"device_id":N,"device_type":"doorbell","event_type":"press",
 *           "event_id":"XXXXXXXXXXXXXXXX","timestamp_ms":NNNN}
 *
 *          Lane A only — this path is independent of JPEG delivery.
 *          Neither lane depends on the other for correctness.
 *
 * \note    event_id is a 16-char hex string matching the event_id embedded
 *          in the TCP JPEG header (cam_header_t). BBB uses it as a join key
 *          to correlate press events with image payloads — no time-window
 *          heuristics required.
 ******************************************************************************/

#include "config.h"
#include "doorbell_listener.h"
#include "vroom_bus.h"
#include "trinity_log.h"
#include "network_config.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "freertos/event_groups.h"
static EventGroupHandle_t g_wifi_eg = NULL;
#include "wifi_manager.h"

static const char *TAG = "DOORBELL_RX";

#define DOORBELL_RX_BUF_SIZE   256   /**< max UDP payload we accept          */
#define DOORBELL_TASK_STACK   4096   /**< task stack in bytes                */
#define DOORBELL_TASK_PRIO       5   /**< same priority as cam_trigger path  */

static uint32_t g_doorbell_last_seen_ms[MAX_DOORBELL_CAMS] = {0};

static void doorbell_stamp(uint8_t device_id)
{
    g_doorbell_last_seen_ms[device_id] = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

uint16_t doorbell_get_age_s(uint8_t device_id)
{
    uint32_t now = 0;

    if (device_id >= MAX_DOORBELL_CAMS)          { return 0xFFFF; }
    if (0 == g_doorbell_last_seen_ms[device_id]) { return 0xFFFF; }

    now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return (uint16_t)((now - g_doorbell_last_seen_ms[device_id]) / 1000);
}

bool doorbell_is_alive(uint8_t device_id)
{
    uint32_t now = 0;

    if (device_id >= MAX_DOORBELL_CAMS)          { return false; }
    if (0 == g_doorbell_last_seen_ms[device_id]) { return false; }

    now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return (now - g_doorbell_last_seen_ms[device_id]) < DOORBELL_HEARTBEAT_MS;
}

/*---------------------------------------------------------------------------*/
/* JSON field parsers — no cJSON dependency, fields are small and fixed      */
/*---------------------------------------------------------------------------*/

/**
 * \brief Extract a uint64 hex value from a JSON string field.
 *
 * Looks for "key":"HEXVALUE" and parses the hex string.
 * Returns 0 on parse failure or missing field.
 */
static uint64_t parse_hex_field(const char *p_json, const char *p_key)
{
   char        search[40]; /**< key search string */
   const char *p = NULL;   /**< cursor            */

   snprintf(search, sizeof(search), "\"%s\":\"", p_key);
   p = strstr(p_json, search);
   if (NULL == p) { return 0ULL; }
   p += strlen(search);
   return (uint64_t)strtoull(p, NULL, 16);
}

/**
 * \brief Extract a uint32 decimal value from a JSON numeric field.
 *
 * Looks for "key":VALUE and parses as unsigned long.
 * Returns 0 on parse failure or missing field.
 */
static uint32_t parse_uint_field(const char *p_json, const char *p_key)
{
   char        search[40]; /**< key search string */
   const char *p = NULL;   /**< cursor            */

   snprintf(search, sizeof(search), "\"%s\":", p_key);
   p = strstr(p_json, search);
   if (NULL == p) { return 0UL; }
   p += strlen(search);
   return (uint32_t)strtoul(p, NULL, 10);
}

/**
 * \brief Extract a uint64 decimal value from a JSON numeric field.
 *
 * Returns 0 on parse failure or missing field.
 */
static uint64_t parse_u64_field(const char *p_json, const char *p_key)
{
   char        search[40]; /**< key search string */
   const char *p = NULL;   /**< cursor            */

   snprintf(search, sizeof(search), "\"%s\":", p_key);
   p = strstr(p_json, search);
   if (NULL == p) { return 0ULL; }
   p += strlen(search);
   return (uint64_t)strtoull(p, NULL, 10);
}

/*---------------------------------------------------------------------------*/
/* Listener task                                                               */
/*---------------------------------------------------------------------------*/

static void doorbell_listener_task(void *p_arg)
{
   char               rx_buf[DOORBELL_RX_BUF_SIZE]; /**< receive buffer     */
   int                sock      = -1;               /**< UDP socket fd      */
   uint8_t            device_id = 0;                /**< parsed device id   */
   uint64_t           event_id  = 0;                /**< parsed event id    */
   uint64_t           ts_ms     = 0;                /**< parsed timestamp   */
   char               log_buf[64];                  /**< trinity log buffer */
   int                n         = 0;                /**< recvfrom result    */
   struct sockaddr_in bind_addr = {0};              /**< bind address       */
   struct sockaddr_in src_addr  = {0};              /**< sender address     */
   socklen_t          src_len   = sizeof(src_addr); /**< sender addr len    */

   while (!(xEventGroupGetBits(g_wifi_eg) & WIFI_CONNECTED_BIT))
   {
      vTaskDelay(pdMS_TO_TICKS(500));
   }

   sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
   if (sock < 0)
   {
      ESP_LOGE(TAG, "socket() failed");
      vTaskDelete(NULL);
      return;
   }

   bind_addr.sin_family      = AF_INET;
   bind_addr.sin_port        = htons(HUB_DOORBELL_UDP_PORT);
   bind_addr.sin_addr.s_addr = INADDR_ANY;

   if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0)
   {
      ESP_LOGE(TAG, "bind() failed on port %d", HUB_DOORBELL_UDP_PORT);
      close(sock);
      vTaskDelete(NULL);
      return;
   }

   ESP_LOGI(TAG, "Listening on UDP port %d", HUB_DOORBELL_UDP_PORT);

   while (1)
   {
      n = recvfrom(sock, rx_buf, sizeof(rx_buf) - 1, 0,
                   (struct sockaddr *)&src_addr, &src_len);
      if (n <= 0) { continue; }
      rx_buf[n] = '\0';

      if (NULL != strstr(rx_buf, "\"event_type\":\"press\""))
      {
         device_id = (uint8_t)parse_uint_field(rx_buf, "device_id");
         event_id  = parse_hex_field(rx_buf,            "event_id");
         ts_ms     = parse_u64_field(rx_buf,            "timestamp_ms");

         if (device_id >= MAX_DOORBELL_CAMS) { continue; }

         doorbell_stamp(device_id);

         ESP_LOGI(TAG, ">>> DOORBELL press device_id=%d event_id=%08lx%08lx",
                  device_id,
                  (unsigned long)(event_id >> 32),
                  (unsigned long)(event_id & 0xFFFFFFFFUL));

         snprintf(log_buf, sizeof(log_buf),
                  "EVENT: DOORBELL_PRESS device_id=%d\n", device_id);
         trinity_log_event(log_buf);
         bus_publish_doorbell(device_id, event_id, ts_ms);
      }
      else if (NULL != strstr(rx_buf, "\"event_type\":\"heartbeat\""))
      {
         device_id = (uint8_t)parse_uint_field(rx_buf, "device_id");

         if (device_id >= MAX_DOORBELL_CAMS) { continue; }

         doorbell_stamp(device_id);

         ESP_LOGI(TAG, "HEARTBEAT device_id=%d age_s=%d",
                  device_id, doorbell_get_age_s(device_id));
      }
      else
      {
         ESP_LOGW(TAG, "Unknown event_type — ignoring");
      }
   }
}

/*---------------------------------------------------------------------------*/


void doorbell_listener_start(EventGroupHandle_t wifi_eg)
{
   g_wifi_eg = wifi_eg;
   xTaskCreate(doorbell_listener_task, "doorbell_rx",
               DOORBELL_TASK_STACK, NULL, DOORBELL_TASK_PRIO, NULL);
   ESP_LOGI(TAG, "Doorbell listener task started");
}
