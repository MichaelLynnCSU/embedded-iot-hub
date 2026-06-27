/******************************************************************************
 * \file camera_manager.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-27
 *
 * \brief BBB camera manager — UDP ingress for all camera devices.
 *
 * \details Listens on CAMERA_MANAGER_PORT (9094) for JSON envelopes
 *          from all ESP32 camera devices on the camera subnet (10.0.1.0/24).
 *          Routes by device_type and event_type fields.
 *
 *          Handles:
 *          - cam heartbeats      (device_type:"cam",      event_type:"heartbeat")
 *          - doorbell heartbeats (device_type:"doorbell", event_type:"heartbeat")
 *          - doorbell presses    (device_type:"doorbell", event_type:"press")
 *
 *          Liveness model:
 *          last_seen[slot] stamped on valid packet receipt.
 *          age_s derived at read time: (now - last_seen).
 *          online derived at read time: age_s < CAMERA_ONLINE_THRESHOLD_S.
 *
 *          Press events are forwarded to the hub ESP32 via UDP on the
 *          main network (HUB_HOST:HUB_DOORBELL_UDP_PORT) so the hub
 *          can publish the doorbell event to the BeagleBone controller
 *          pipeline as before.
 *
 * \note    Architecture (2026-06-27):
 *          All camera devices report only to the BBB camera manager.
 *          Hub ESP32 no longer receives camera UDP traffic directly.
 *          See GitHub issue: camera telemetry architecture shift.
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <errno.h>
#include <stdarg.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/*---------------------------------------------------------------------------*/
/* Config                                                                      */
/*---------------------------------------------------------------------------*/

#define CAMERA_MANAGER_PORT       9094
#define MAX_CAMS                  3
#define MAX_DOORBELL_CAMS         4
#define CAMERA_ONLINE_THRESHOLD_S 90     /**< seconds before slot goes offline */
#define RX_BUF_SIZE               256
#define LOG_PATH                  "/var/log/camera_manager.log"

/** Hub ESP32 — press events forwarded here for pipeline injection */
#define HUB_HOST                  "10.0.0.190"
#define HUB_DOORBELL_UDP_PORT     9092

/*---------------------------------------------------------------------------*/
/* Liveness tables                                                             */
/*---------------------------------------------------------------------------*/

static time_t          g_cam_last_seen[MAX_CAMS]          = {0};
static time_t          g_doorbell_last_seen[MAX_DOORBELL_CAMS] = {0};
static pthread_mutex_t g_liveness_mutex = PTHREAD_MUTEX_INITIALIZER;

/*---------------------------------------------------------------------------*/
/* Logging                                                                     */
/*---------------------------------------------------------------------------*/

static FILE           *g_log      = NULL;
static pthread_mutex_t g_log_mutex = PTHREAD_MUTEX_INITIALIZER;

static void log_msg(const char *fmt, ...)
{
   va_list  ap;
   time_t   now = time(NULL);
   char     ts[32];
   struct tm tm_buf;

   strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", gmtime_r(&now, &tm_buf));

   pthread_mutex_lock(&g_log_mutex);
   if (NULL != g_log)
   {
      fprintf(g_log, "[%s] ", ts);
      va_start(ap, fmt);
      vfprintf(g_log, fmt, ap);
      va_end(ap);
      fprintf(g_log, "\n");
   }
   va_start(ap, fmt);
   vprintf(fmt, ap);
   va_end(ap);
   printf("\n");
   pthread_mutex_unlock(&g_log_mutex);
}

/*---------------------------------------------------------------------------*/
/* JSON field parsers — no cJSON dependency                                   */
/*---------------------------------------------------------------------------*/

static uint32_t parse_uint_field(const char *p_json, const char *p_key)
{
   char        search[48];
   const char *p = NULL;

   snprintf(search, sizeof(search), "\"%s\":", p_key);
   p = strstr(p_json, search);
   if (NULL == p) { return 0UL; }
   p += strlen(search);
   return (uint32_t)strtoul(p, NULL, 10);
}

static int parse_str_field(const char *p_json, const char *p_key,
                           char *out, size_t out_len)
{
   char        search[48];
   const char *p   = NULL;
   const char *end = NULL;
   size_t      len = 0;

   snprintf(search, sizeof(search), "\"%s\":\"", p_key);
   p = strstr(p_json, search);
   if (NULL == p) { return -1; }
   p += strlen(search);
   end = strchr(p, '"');
   if (NULL == end) { return -1; }
   len = (size_t)(end - p);
   if (len >= out_len) { len = out_len - 1; }
   memcpy(out, p, len);
   out[len] = '\0';
   return 0;
}

/*---------------------------------------------------------------------------*/
/* Liveness accessors                                                          */
/*---------------------------------------------------------------------------*/

static void cam_stamp(uint8_t slot)
{
   if (slot >= MAX_CAMS) { return; }
   pthread_mutex_lock(&g_liveness_mutex);
   g_cam_last_seen[slot] = time(NULL);
   pthread_mutex_unlock(&g_liveness_mutex);
}

static void doorbell_stamp(uint8_t device_id)
{
   if (device_id >= MAX_DOORBELL_CAMS) { return; }
   pthread_mutex_lock(&g_liveness_mutex);
   g_doorbell_last_seen[device_id] = time(NULL);
   pthread_mutex_unlock(&g_liveness_mutex);
}

uint16_t cam_get_age_s(uint8_t slot)
{
   time_t   last = 0;
   time_t   now  = time(NULL);

   if (slot >= MAX_CAMS) { return 0xFFFF; }
   pthread_mutex_lock(&g_liveness_mutex);
   last = g_cam_last_seen[slot];
   pthread_mutex_unlock(&g_liveness_mutex);
   if (0 == last) { return 0xFFFF; }
   return (uint16_t)(now - last);
}

uint16_t doorbell_get_age_s(uint8_t device_id)
{
   time_t   last = 0;
   time_t   now  = time(NULL);

   if (device_id >= MAX_DOORBELL_CAMS) { return 0xFFFF; }
   pthread_mutex_lock(&g_liveness_mutex);
   last = g_doorbell_last_seen[device_id];
   pthread_mutex_unlock(&g_liveness_mutex);
   if (0 == last) { return 0xFFFF; }
   return (uint16_t)(now - last);
}

int cam_is_online(uint8_t slot)
{
   return cam_get_age_s(slot) < CAMERA_ONLINE_THRESHOLD_S;
}

int doorbell_is_online(uint8_t device_id)
{
   return doorbell_get_age_s(device_id) < CAMERA_ONLINE_THRESHOLD_S;
}

/*---------------------------------------------------------------------------*/
/* Press event forwarding to hub                                               */
/*---------------------------------------------------------------------------*/

static void forward_press_to_hub(const char *p_json)
{
   struct sockaddr_in addr = {0};
   int                sock = -1;
   size_t             len  = 0;

   addr.sin_family = AF_INET;
   addr.sin_port   = htons(HUB_DOORBELL_UDP_PORT);
   inet_pton(AF_INET, HUB_HOST, &addr.sin_addr);

   sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
   if (sock < 0)
   {
      log_msg("[PRESS] forward socket() failed errno=%d", errno);
      return;
   }

   len = strlen(p_json);
   if (sendto(sock, p_json, len, 0,
              (struct sockaddr *)&addr, sizeof(addr)) < 0)
   {
      log_msg("[PRESS] forward sendto() failed errno=%d", errno);
   }
   else
   {
      log_msg("[PRESS] forwarded to hub %s:%d", HUB_HOST, HUB_DOORBELL_UDP_PORT);
   }

   close(sock);
}

/*---------------------------------------------------------------------------*/
/* Ingress loop                                                                */
/*---------------------------------------------------------------------------*/

static volatile int g_running = 1;

static void sig_handler(int sig)
{
   (void)sig;
   g_running = 0;
}

static void ingress_loop(int sock)
{
   char               rx_buf[RX_BUF_SIZE];
   struct sockaddr_in src   = {0};
   socklen_t          src_len = sizeof(src);
   int                n     = 0;
   char               device_type[32];
   char               event_type[32];
   uint32_t           device_id = 0;

   while (g_running)
   {
      n = recvfrom(sock, rx_buf, sizeof(rx_buf) - 1, 0,
                   (struct sockaddr *)&src, &src_len);
      if (n <= 0) { continue; }
      rx_buf[n] = '\0';

      if (parse_str_field(rx_buf, "device_type", device_type,
                          sizeof(device_type)) < 0) { continue; }
      if (parse_str_field(rx_buf, "event_type",  event_type,
                          sizeof(event_type))  < 0) { continue; }

      device_id = parse_uint_field(rx_buf, "device_id");

      if (0 == strcmp(device_type, "cam"))
      {
         if (device_id >= MAX_CAMS)
         {
            log_msg("[CAM] device_id=%u out of range", device_id);
            continue;
         }
         cam_stamp((uint8_t)device_id);
         log_msg("[CAM] heartbeat slot=%u age_s=%u online=%d",
                 device_id,
                 (unsigned)cam_get_age_s((uint8_t)device_id),
                 cam_is_online((uint8_t)device_id));
      }
      else if (0 == strcmp(device_type, "doorbell"))
      {
         if (device_id >= MAX_DOORBELL_CAMS)
         {
            log_msg("[DOORBELL] device_id=%u out of range", device_id);
            continue;
         }

         if (0 == strcmp(event_type, "heartbeat"))
         {
            doorbell_stamp((uint8_t)device_id);
            log_msg("[DOORBELL] heartbeat device_id=%u age_s=%u online=%d",
                    device_id,
                    (unsigned)doorbell_get_age_s((uint8_t)device_id),
                    doorbell_is_online((uint8_t)device_id));
         }
         else if (0 == strcmp(event_type, "press"))
         {
            doorbell_stamp((uint8_t)device_id);
            log_msg("[DOORBELL] press device_id=%u — forwarding to hub",
                    device_id);
            forward_press_to_hub(rx_buf);
         }
         else
         {
            log_msg("[DOORBELL] unknown event_type=%s", event_type);
         }
      }
      else
      {
         log_msg("[INGRESS] unknown device_type=%s", device_type);
      }
   }
}

/*---------------------------------------------------------------------------*/
/* main                                                                        */
/*---------------------------------------------------------------------------*/

int main(void)
{
   int                sock     = -1;
   struct sockaddr_in bind_addr = {0};

   signal(SIGINT,  sig_handler);
   signal(SIGTERM, sig_handler);

   g_log = fopen(LOG_PATH, "a");
   if (NULL == g_log)
   {
      fprintf(stderr, "Warning: cannot open log %s: %s\n",
              LOG_PATH, strerror(errno));
   }

   log_msg("[CAMERA_MGR] starting on UDP port %d", CAMERA_MANAGER_PORT);
   log_msg("[CAMERA_MGR] max_cams=%d max_doorbells=%d online_threshold=%ds",
           MAX_CAMS, MAX_DOORBELL_CAMS, CAMERA_ONLINE_THRESHOLD_S);

   sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
   if (sock < 0)
   {
      log_msg("[CAMERA_MGR] socket() failed errno=%d", errno);
      return 1;
   }

   bind_addr.sin_family      = AF_INET;
   bind_addr.sin_port        = htons(CAMERA_MANAGER_PORT);
   bind_addr.sin_addr.s_addr = INADDR_ANY;

   if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0)
   {
      log_msg("[CAMERA_MGR] bind() failed on port %d errno=%d",
              CAMERA_MANAGER_PORT, errno);
      close(sock);
      return 1;
   }

   log_msg("[CAMERA_MGR] listening on 0.0.0.0:%d", CAMERA_MANAGER_PORT);

   ingress_loop(sock);

   close(sock);
   log_msg("[CAMERA_MGR] shutdown");
   if (NULL != g_log) { fflush(g_log); fclose(g_log); }
   return 0;
}
