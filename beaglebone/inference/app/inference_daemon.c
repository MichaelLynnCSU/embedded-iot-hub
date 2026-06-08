/******************************************************************************
 * \file inference_daemon.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief BeagleBone PIR inference daemon — TCP server on port 9090.
 *
 * \details ESP32-CAM connects and pushes JPEGs on PIR trigger.
 *          Inference runs on each received JPEG via inference_worker.
 *          Results saved to /data/pir.
 *
 *          Wire protocol (port 9090):
 *          [jpeg_size:4] followed by jpeg_size bytes of JPEG payload.
 *          (Simple 4-byte header — legacy PIR cam protocol.)
 *
 * \note    Inference logic extracted to inference_worker.c (2026-06-07)
 *          so doorbell_daemon can share the same perception pipeline.
 *          Transport parsing (4-byte header, TCP accept loop) stays here.
 *          Storage path changed from /data/pending to /data/pir.
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <stdarg.h>
#include <stdint.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include "inference_worker.h"
#include "inference_core.h"

/******************************** CONFIG **************************************/
#define MODEL_PATH    "/opt/inference/models/detect.tflite"
#define LABEL_PATH    "/opt/inference/models/labelmap.txt"
#define PIR_DIR       "/data/pir"
#define LOG_PATH      "/var/log/inference.log"
#define LISTEN_PORT   9090

/******************************** GLOBALS *************************************/
static volatile int  g_running   = 1;    /**< main loop run flag  */
static FILE         *g_log       = NULL; /**< log file handle     */
static int           g_server_fd = -1;  /**< TCP listen socket   */
static int           g_client_fd = -1;  /**< active client fd    */

/******************************** LOGGING *************************************/
static void log_msg(const char *fmt, ...)
{
   va_list   ap;
   time_t    now = time(NULL);
   char      ts[32];
   struct tm tm_buf;

   localtime_r(&now, &tm_buf);
   strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tm_buf);

   if (NULL != g_log)
   {
      fprintf(g_log, "[%s] ", ts);
      va_start(ap, fmt);
      vfprintf(g_log, fmt, ap);
      va_end(ap);
      fprintf(g_log, "\n");
      fflush(g_log);
   }

   fprintf(stderr, "[%s] ", ts);
   va_start(ap, fmt);
   vfprintf(stderr, fmt, ap);
   va_end(ap);
   fprintf(stderr, "\n");
}

/******************************** SIGNAL **************************************/
static void sig_handler(int sig) { (void)sig; g_running = 0; }

/******************************** TCP SERVER **********************************/
static void tcp_server_init(void)
{
   struct sockaddr_in addr = {0};
   int                opt  = 1;

   addr.sin_family      = AF_INET;
   addr.sin_port        = htons(LISTEN_PORT);
   addr.sin_addr.s_addr = INADDR_ANY;

   g_server_fd = socket(AF_INET, SOCK_STREAM, 0);
   setsockopt(g_server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
   bind(g_server_fd, (struct sockaddr *)&addr, sizeof(addr));
   listen(g_server_fd, 1);
   log_msg("TCP server listening on port %d", LISTEN_PORT);
}

/******************************** JPEG RECEIVE ********************************/
static uint8_t *receive_jpeg(size_t *out_len)
{
   uint8_t  hdr[JPEG_HDR_SIZE]; /**< 4-byte length header  */
   uint32_t jpeg_len = 0;       /**< JPEG payload length   */
   uint8_t *buf      = NULL;    /**< JPEG payload buffer   */
   ssize_t  n        = 0;       /**< recv return value     */
   size_t   received = 0;       /**< bytes received so far */

   if (g_client_fd < 0)
   {
      log_msg("Waiting for ESP32-CAM connection...");
      g_client_fd = accept(g_server_fd, NULL, NULL);
      if (g_client_fd < 0) { return NULL; }
      log_msg("ESP32-CAM connected");

      int keepalive = 1, keepidle = 30, keepintvl = 5, keepcnt = 3;
      setsockopt(g_client_fd, SOL_SOCKET,  SO_KEEPALIVE, &keepalive, sizeof(keepalive));
      setsockopt(g_client_fd, IPPROTO_TCP, TCP_KEEPIDLE,  &keepidle,  sizeof(keepidle));
      setsockopt(g_client_fd, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
      setsockopt(g_client_fd, IPPROTO_TCP, TCP_KEEPCNT,   &keepcnt,   sizeof(keepcnt));
   }

   n = recv(g_client_fd, hdr, JPEG_HDR_SIZE, MSG_WAITALL);
   if (n != JPEG_HDR_SIZE)
   {
      log_msg("Connection lost, waiting for reconnect");
      close(g_client_fd);
      g_client_fd = -1;
      return NULL;
   }

   jpeg_len = infer_unpack_jpeg_len(hdr);

   if (!infer_jpeg_len_valid(jpeg_len))
   {
      log_msg("Bad jpeg_len %u", jpeg_len);
      close(g_client_fd);
      g_client_fd = -1;
      return NULL;
   }

   buf = malloc(jpeg_len);
   if (NULL == buf) { log_msg("ERROR: malloc"); return NULL; }

   received = 0;
   while (received < jpeg_len)
   {
      n = recv(g_client_fd, buf + received, jpeg_len - received, 0);
      if (n <= 0)
      {
         log_msg("Receive truncated");
         free(buf);
         close(g_client_fd);
         g_client_fd = -1;
         return NULL;
      }
      received += (size_t)n;
   }

   *out_len = jpeg_len;
   log_msg("Received JPEG %zu bytes", jpeg_len);
   return buf;
}

/******************************** MAIN ****************************************/
int main(void)
{
   signal(SIGINT,  sig_handler);
   signal(SIGTERM, sig_handler);

   g_log = fopen(LOG_PATH, "a");
   log_msg("inference_daemon starting on port %d", LISTEN_PORT);

   if (inference_worker_init(MODEL_PATH, LABEL_PATH) < 0)
   {
      log_msg("ERROR: inference_worker_init failed");
      return 1;
   }

   tcp_server_init();

   while (g_running)
   {
      size_t   jpeg_len = 0;
      uint8_t *jpeg     = receive_jpeg(&jpeg_len);
      if (NULL == jpeg) { usleep(100000); continue; }

      int   detected = 0;
      float conf     = 0.0f;
      inference_worker_run(jpeg, jpeg_len, &detected, &conf);
      log_msg("Result: person=%d confidence=%.2f", detected, conf);

      inference_worker_save(jpeg, jpeg_len, PIR_DIR, detected, conf, NULL);
      free(jpeg);
   }

   log_msg("inference_daemon shutting down");
   if (g_client_fd >= 0) { close(g_client_fd); }
   if (g_server_fd >= 0) { close(g_server_fd); }
   inference_worker_shutdown();
   if (NULL != g_log) { fclose(g_log); }
   return 0;
}
