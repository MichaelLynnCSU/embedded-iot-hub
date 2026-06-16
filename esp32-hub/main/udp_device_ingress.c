/******************************************************************************
 * \file udp_device_ingress.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief UDP device ingress — doorbell presses, camera heartbeats,
 *        device liveness events.
 *
 * \details Single UDP socket on HUB_DOORBELL_UDP_PORT receives JSON
 *          envelopes from all ESP32 devices. Routes by device_type field.
 *
 *          Handles:
 *          - doorbell presses    (device_type:"doorbell", event_type:"press")
 *          - doorbell heartbeats (device_type:"doorbell", event_type:"heartbeat")
 *          - camera heartbeats   (device_type:"cam",      event_type:"heartbeat")
 *
 *          Liveness model:
 *          last_seen[slot] stamped on valid packet receipt.
 *          online derived at read time: (now - last_seen) < threshold.
 *          No online flag stored — callers derive at query time.
 *
 * \note    Renamed from doorbell_listener.c (2026-06-10).
 *          Extended to handle camera heartbeats alongside doorbell events.
 *          doorbell_get_age_s() and doorbell_is_alive() retain their names
 *          for compatibility with existing tcp_manager.c call sites.
 ******************************************************************************/

/*
 * UDP Device Ingress
 * Handles: doorbell presses, doorbell heartbeats, camera heartbeats
 * Grep: udp ingress heartbeat doorbell cam liveness
 */

#include "config.h"
#include "udp_device_ingress.h"
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
#include "wifi_manager.h"

static EventGroupHandle_t g_wifi_eg = NULL;

static const char *TAG = "UDP_DEV_INGRESS";
static uint32_t    rx_seq = 0;

#define INGRESS_RX_BUF_SIZE   256
#define INGRESS_TASK_STACK   4096
#define INGRESS_TASK_PRIO       5
#define MAX_CAMS                3u

/************************ DOORBELL LIVENESS ***********************************/

static uint32_t g_doorbell_last_seen_ms[MAX_DOORBELL_CAMS] = {0};

static void doorbell_stamp(uint8_t device_id)
{
   g_doorbell_last_seen_ms[device_id] =
      xTaskGetTickCount() * portTICK_PERIOD_MS;
}

uint16_t doorbell_get_age_s(uint8_t device_id)
{
   uint32_t now = 0;

   if (device_id >= MAX_DOORBELL_CAMS)          { return 0xFFFF; }
   if (0 == g_doorbell_last_seen_ms[device_id]) { return 0xFFFF; }

   now = xTaskGetTickCount() * portTICK_PERIOD_MS;
   return (uint16_t)((now - g_doorbell_last_seen_ms[device_id]) / 1000u);
}

bool doorbell_is_alive(uint8_t device_id)
{
   uint32_t now = 0;

   if (device_id >= MAX_DOORBELL_CAMS)          { return false; }
   if (0 == g_doorbell_last_seen_ms[device_id]) { return false; }

   now = xTaskGetTickCount() * portTICK_PERIOD_MS;
   return (now - g_doorbell_last_seen_ms[device_id]) < DOORBELL_HEARTBEAT_MS;
}

/************************ CAMERA LIVENESS *************************************/

static uint32_t g_cam_last_seen_ms[MAX_CAMS] = {0};

static void cam_stamp(uint8_t slot)
{
   if (slot >= MAX_CAMS) { return; }
   g_cam_last_seen_ms[slot] = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

uint16_t cam_get_age_s(uint8_t slot)
{
   uint32_t now = 0;

   if (slot >= MAX_CAMS)              { return 0xFFFF; }
   if (0 == g_cam_last_seen_ms[slot]) { return 0xFFFF; }

   now = xTaskGetTickCount() * portTICK_PERIOD_MS;
   return (uint16_t)((now - g_cam_last_seen_ms[slot]) / 1000u);
}

bool cam_is_alive(uint8_t slot)
{
   uint32_t now = 0;

   if (slot >= MAX_CAMS)              { return false; }
   if (0 == g_cam_last_seen_ms[slot]) { return false; }

   now = xTaskGetTickCount() * portTICK_PERIOD_MS;
   return (now - g_cam_last_seen_ms[slot]) < DOORBELL_HEARTBEAT_MS;
}

/*---------------------------------------------------------------------------*/
/* JSON field parsers — no cJSON dependency                                   */
/*---------------------------------------------------------------------------*/

static uint64_t parse_hex_field(const char *p_json, const char *p_key)
{
   char        search[40];
   const char *p = NULL;

   snprintf(search, sizeof(search), "\"%s\":\"", p_key);
   p = strstr(p_json, search);
   if (NULL == p) { return 0ULL; }
   p += strlen(search);
   return (uint64_t)strtoull(p, NULL, 16);
}

static uint32_t parse_uint_field(const char *p_json, const char *p_key)
{
   char        search[40];
   const char *p = NULL;

   snprintf(search, sizeof(search), "\"%s\":", p_key);
   p = strstr(p_json, search);
   if (NULL == p) { return 0UL; }
   p += strlen(search);
   return (uint32_t)strtoul(p, NULL, 10);
}

static uint64_t parse_u64_field(const char *p_json, const char *p_key)
{
   char        search[40];
   const char *p = NULL;

   snprintf(search, sizeof(search), "\"%s\":", p_key);
   p = strstr(p_json, search);
   if (NULL == p) { return 0ULL; }
   p += strlen(search);
   return (uint64_t)strtoull(p, NULL, 10);
}

/*---------------------------------------------------------------------------*/
/* Ingress task                                                                */
/*---------------------------------------------------------------------------*/

static void udp_device_ingress_task(void *p_arg)
{
   char               rx_buf[INGRESS_RX_BUF_SIZE];
   int                sock      = -1;
   uint8_t            device_id = 0;
   uint64_t           event_id  = 0;
   uint64_t           ts_ms     = 0;
   char               log_buf[64];
   int                n         = 0;
   struct sockaddr_in bind_addr = {0};
   struct sockaddr_in src_addr  = {0};
   socklen_t          src_len   = sizeof(src_addr);

   (void)p_arg;

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
         uint64_t pipeline_eid = 0;

         device_id = (uint8_t)parse_uint_field(rx_buf, "device_id");
         event_id  = parse_hex_field(rx_buf,            "event_id");
         ts_ms     = parse_u64_field(rx_buf,            "timestamp_ms");

         if (device_id >= MAX_DOORBELL_CAMS) { continue; }

         // Increment pipeline sequence counter on verified reception
         rx_seq++;

         doorbell_stamp(device_id);

         // Publish and extract returned tracking ID from vroom_bus
         pipeline_eid = bus_publish_doorbell(device_id, event_id, ts_ms);

         ESP_LOGI(TAG, "[DOORBELL] rx_id=%u event_id=%llu device_id=%d cam_event_id=%llu",
                  (unsigned)rx_seq,
                  (unsigned long long)pipeline_eid,
                  (int)device_id,
                  (unsigned long long)event_id);


         snprintf(log_buf, sizeof(log_buf),
                  "EVENT: DOORBELL_PRESS device_id=%d\n", device_id);
         trinity_log_event(log_buf);
         bus_publish_doorbell(device_id, event_id, ts_ms);
      }
      else if (NULL != strstr(rx_buf, "\"event_type\":\"heartbeat\""))
      {
         device_id = (uint8_t)parse_uint_field(rx_buf, "device_id");

         if (NULL != strstr(rx_buf, "\"device_type\":\"doorbell\""))
         {
            if (device_id >= MAX_DOORBELL_CAMS) { continue; }
            doorbell_stamp(device_id);
            ESP_LOGI(TAG, "DOORBELL heartbeat device_id=%d age_s=%d",
                     device_id, doorbell_get_age_s(device_id));
         }
         else if (NULL != strstr(rx_buf, "\"device_type\":\"cam\""))
         {
            if (device_id >= MAX_CAMS) { continue; }
            cam_stamp(device_id);
            ESP_LOGI(TAG, "CAM heartbeat slot=%d age_s=%d",
                     device_id, cam_get_age_s(device_id));
         }
         else
         {
            ESP_LOGW(TAG, "Heartbeat unknown device_type — ignoring");
         }
      }
      else
      {
         ESP_LOGW(TAG, "Unknown event_type — ignoring");
      }
   }
}

/*---------------------------------------------------------------------------*/

void udp_device_ingress_start(EventGroupHandle_t wifi_eg)
{
   g_wifi_eg = wifi_eg;
   xTaskCreate(udp_device_ingress_task, "udp_dev_ingress",
               INGRESS_TASK_STACK, NULL, INGRESS_TASK_PRIO, NULL);
   ESP_LOGI(TAG, "UDP device ingress started");
}
