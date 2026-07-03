/******************************************************************************
 * \file doorbell_daemon.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief BeagleBone doorbell ingest daemon.
 *
 * \details TCP server on port 9091. Accepts connections from ESP32 doorbell
 *          cams (up to MAX_DOORBELL_CAMS=4). Each connection sends a 24-byte
 *          cam_header_t followed by a JPEG payload. Runs inference via the
 *          shared inference_worker and saves results to /data/doorbell.
 *
 *          Wire protocol (port 9091):
 *          [magic:4][version:1][device_id:1][reserved:2][event_id:8][jpeg_size:4]
 *          followed immediately by jpeg_size bytes of JPEG payload.
 *
 *          Lane B — image payload path. Independent of Lane A (UDP event
 *          to hub). event_id from header is the join key; it matches the
 *          UDP envelope sent by the cam before this TCP connection.
 *
 *          Saved filename format:
 *          /data/doorbell/<timestamp>_<person|noperson>_dev<N>_<event_id>_<conf>.jpg
 *
 * \note    One accept loop, one client at a time per-connection.
 *          Doorbell is low-frequency (human-initiated), so a single-threaded
 *          accept loop is sufficient. Extend to per-connection threads if
 *          simultaneous multi-cam doorbell events become a requirement.
 *
 * \note    Doorbell result SHM publish (2026-06-14):
 *          New /doorbell_result segment (doorbell_result_shm.h) created
 *          here and published after each completed inference run. This
 *          daemon is the sole creator/owner of the segment — see
 *          doorbell_result_shm.h for the publish/consume contract.
 *          Reader is uart_controller (beaglebone/controller). No changes
 *          to SharedSensorData or SensorData.
 *
 * \note    event_id tracing (2026-06-15):
 *          All log lines that carry an event_id use the shared
 *          EVENT_ID_FMT/EVENT_ID_ARG macros from doorbell_result_shm.h,
 *          giving a single canonical %08lx%08lx representation across
 *          this daemon and uart_controller's data_controller.log.
 *
 *          Logging contract (commit 1, 2026-06-15):
 *          Internal lifecycle events use a flat [DOORBELL] tag:
 *            [DOORBELL] rx          — header validated, JPEG received
 *            [DOORBELL] infer_start — inference about to run
 *            [DOORBELL] infer_done  — inference completed successfully
 *            [DOORBELL] infer_failed — inference or save failed
 *          Cross-process boundary uses a directional tag:
 *            [DOORBELL] -> [SHM] publish       — result written to shm
 *            [DOORBELL] -> [SHM] publish_failed — shm not available
 *          This mirrors the consume-side contract in doorbell_result_reader.c
 *          ([SHM] -> [CONTROLLER]) and uart_controller.c
 *          ([CONTROLLER] -> [UART]), making the full pipeline traceable
 *          with a single grep <event_id_hex> across all daemon logs.
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
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "inference_worker.h"
#include "inference_core.h"
#include "build_info.h"
#include "doorbell_result_shm.h"

/******************************** CONFIG **************************************/
#define MODEL_PATH        "/opt/inference/models/detect.tflite"
#define LABEL_PATH        "/opt/inference/models/labelmap.txt"
#define DOORBELL_DIR      "/data/doorbell"
#define LOG_PATH          "/var/log/doorbell_daemon.log"
#define LISTEN_PORT       9091
#define CAM_MAGIC         0xCAFEBABEu
#define CAM_HEADER_SIZE   24
#define CAM_HEADER_VER    2
#define MAX_DOORBELL_CAMS 4
#define MAX_JPEG_BYTES    500000

/******************************** GLOBALS *************************************/
static volatile int  g_running   = 1;   /**< main loop run flag    */
static FILE         *g_log       = NULL; /**< log file handle       */
static int           g_server_fd = -1;  /**< TCP listen socket     */

/**< /doorbell_result shm — created and owned by this daemon */
static struct DoorbellResult *g_result = NULL;

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
static void sig_handler(int sig)
{
   (void)sig;
   g_running = 0;
}

/******************************** RESULT SHM **********************************/

/**
 * \brief Create and map the /doorbell_result shared memory segment.
 *
 * \details This daemon is the sole creator/owner of the segment (see
 *          doorbell_result_shm.h). Zeroes the segment on creation so
 *          event_id starts at 0 ("no event yet") for the reader.
 *
 * \return 0 on success, -1 on failure (daemon continues without publish —
 *         logged as a warning, not fatal, so inference/save still work).
 */
static int result_shm_init(void)
{
   int fd = -1;

   fd = shm_open(DOORBELL_RESULT_SHM_NAME, O_CREAT | O_RDWR, 0666);
   if (0 > fd)
   {
      log_msg("WARNING: shm_open(%s) failed: %s — doorbell result publish disabled",
              DOORBELL_RESULT_SHM_NAME, strerror(errno));
      return -1;
   }

   if (0 != ftruncate(fd, sizeof(struct DoorbellResult)))
   {
      log_msg("WARNING: ftruncate(%s) failed: %s — doorbell result publish disabled",
              DOORBELL_RESULT_SHM_NAME, strerror(errno));
      close(fd);
      return -1;
   }

   g_result = mmap(NULL, sizeof(struct DoorbellResult),
                    PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
   close(fd);

   if (MAP_FAILED == g_result)
   {
      log_msg("WARNING: mmap(%s) failed: %s — doorbell result publish disabled",
              DOORBELL_RESULT_SHM_NAME, strerror(errno));
      g_result = NULL;
      return -1;
   }

   memset(g_result, 0, sizeof(struct DoorbellResult));

   log_msg("Doorbell result shm initialized (%s, %zu bytes)",
           DOORBELL_RESULT_SHM_NAME, sizeof(struct DoorbellResult));
   return 0;
}

/**
 * \brief Publish one inference result to /doorbell_result.
 *
 * \details Writes payload fields, issues a memory barrier, then publishes
 *          by setting event_id last. See doorbell_result_shm.h for the
 *          publish/consume contract. Emits exactly one log line at the
 *          cross-process boundary: "[DOORBELL] -> [SHM] publish ...".
 *          If the segment is unavailable, emits "[DOORBELL] -> [SHM]
 *          publish_failed ..." instead. Callers must not duplicate either
 *          line — this is the single log point for the shm handoff.
 *
 * \param event_id   Correlation key from cam_header_t (non-zero).
 * \param device_id  Doorbell cam ID (0-3).
 * \param person     1 if a person was detected, else 0.
 * \param conf_pct   Confidence, 0-100 (represents 0.00-1.00).
 * \param asset      Timestamp string from inference_worker_save()'s out_ts,
 *                   "%Y%m%dT%H%M%SZ" (e.g. "20260614T182513Z"). Display
 *                   identifier only — not a filename. See
 *                   doorbell_result_shm.h.
 */
static void result_shm_publish(uint64_t event_id, uint8_t device_id,
                                uint8_t person, uint8_t conf_pct,
                                const char *asset)
{
   if (NULL == g_result)
   {
      log_msg("[DOORBELL] -> [SHM] publish_failed event_id=" EVENT_ID_FMT,
              EVENT_ID_ARG(event_id));
      return;
   }

   g_result->device_id = device_id;
   g_result->person    = person;
   g_result->conf_pct  = conf_pct;
   strncpy(g_result->asset, asset, sizeof(g_result->asset) - 1);
   g_result->asset[sizeof(g_result->asset) - 1] = '\0';

   __sync_synchronize();

   g_result->event_id = event_id;

   log_msg("[DOORBELL] -> [SHM] publish event_id=" EVENT_ID_FMT
           " device_id=%d person=%d conf=%d asset=%s",
           EVENT_ID_ARG(event_id),
           device_id, person, conf_pct, asset);
}

/******************************** HEADER UNPACK *******************************/

/**
 * \brief Unpacked cam_header_t fields.
 */
typedef struct
{
   uint32_t magic;      /**< must equal CAM_MAGIC     */
   uint8_t  version;    /**< header format version    */
   uint8_t  device_id;  /**< doorbell cam ID (0-3)    */
   uint64_t event_id;   /**< correlation key for hub  */
   uint32_t jpeg_size;  /**< JPEG payload bytes       */
   uint32_t seq;        /**< BBB telemetry frame_seq. Always 0 for
                         *   doorbell — device-initiated interrupt with
                         *   no BBB frame correlation (see esp32-doorbell/
                         *   main/main.c send_jpeg_to_bbb() note). */
} doorbell_header_t;

/**
 * \brief Unpack a 24-byte wire header into doorbell_header_t.
 *
 * \param buf  CAM_HEADER_SIZE byte input buffer.
 * \param out  Output struct.
 */
static void unpack_header(const uint8_t *buf, doorbell_header_t *out)
{
   out->magic     = ((uint32_t)buf[0]  << 24) | ((uint32_t)buf[1]  << 16) |
                    ((uint32_t)buf[2]  <<  8) | ((uint32_t)buf[3]);
   out->version   = buf[4];
   out->device_id = buf[5];
   /* buf[6], buf[7] = reserved */
   out->event_id  = ((uint64_t)buf[8]  << 56) | ((uint64_t)buf[9]  << 48) |
                    ((uint64_t)buf[10] << 40) | ((uint64_t)buf[11] << 32) |
                    ((uint64_t)buf[12] << 24) | ((uint64_t)buf[13] << 16) |
                    ((uint64_t)buf[14] <<  8) | ((uint64_t)buf[15]);
   out->jpeg_size = ((uint32_t)buf[16] << 24) | ((uint32_t)buf[17] << 16) |
                    ((uint32_t)buf[18] <<  8) | ((uint32_t)buf[19]);
   out->seq       = ((uint32_t)buf[20] << 24) | ((uint32_t)buf[21] << 16) |
                    ((uint32_t)buf[22] <<  8) | ((uint32_t)buf[23]);
}

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
   listen(g_server_fd, MAX_DOORBELL_CAMS);
   log_msg("TCP server listening on port %d", LISTEN_PORT);
}

/******************************** CONNECTION HANDLER **************************/

/**
 * \brief Handle a single doorbell cam connection.
 *
 * \details Reads all complete header+JPEG frames from fd until connection
 *          closes or an error occurs. Each frame runs inference, saves
 *          to DOORBELL_DIR, and publishes the result to /doorbell_result.
 *
 *          Logging contract (flat [DOORBELL] tags for intra-process stages,
 *          directional [DOORBELL] -> [SHM] tag for the publish boundary;
 *          result_shm_publish() owns the boundary log — do not duplicate):
 *            [DOORBELL] rx          — after header+JPEG validated and received
 *            [DOORBELL] infer_start — immediately before inference_worker_run()
 *            [DOORBELL] infer_done  — after successful inference + save
 *            [DOORBELL] infer_failed — on JPEG recv truncation or malloc failure
 *
 * \param client_fd  Accepted client socket fd.
 * \param client_ip  Client IP string for logging.
 */
static void handle_connection(int client_fd, const char *client_ip)
{
   uint8_t            hdr_buf[CAM_HEADER_SIZE]; /**< raw header bytes       */
   doorbell_header_t  hdr;                      /**< unpacked header        */
   uint8_t           *jpeg     = NULL;          /**< JPEG payload buffer    */
   ssize_t            n        = 0;             /**< recv return value      */
   size_t             received = 0;             /**< bytes received so far  */
   int                detected = 0;             /**< person detected flag   */
   float              conf     = 0.0f;          /**< detection confidence   */
   char               tag[48];                  /**< filename tag string    */
   char               event_str[32];            /**< event_id hex string    */
   char               ts_str[20] = {0};         /**< saved-file timestamp,
                                                   * "%Y%m%dT%H%M%SZ" (17 chars
                                                   * + nul), from
                                                   * inference_worker_save()  */
   uint8_t            conf_pct = 0;             /**< confidence as 0-100    */

   /* keepalive */
   int keepalive = 1, keepidle = 30, keepintvl = 5, keepcnt = 3;
   setsockopt(client_fd, SOL_SOCKET,  SO_KEEPALIVE, &keepalive, sizeof(keepalive));
   setsockopt(client_fd, IPPROTO_TCP, TCP_KEEPIDLE,  &keepidle,  sizeof(keepidle));
   setsockopt(client_fd, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
   setsockopt(client_fd, IPPROTO_TCP, TCP_KEEPCNT,   &keepcnt,   sizeof(keepcnt));

   log_msg("Connection from %s", client_ip);

   /* Drain all frames until connection closes */
   while (g_running)
   {
      /* Read 24-byte header */
      n = recv(client_fd, hdr_buf, CAM_HEADER_SIZE, MSG_WAITALL);
      if (n != CAM_HEADER_SIZE)
      {
         if (0 == n)
            log_msg("Client %s disconnected", client_ip);
         else
            log_msg("Header recv error from %s: %s", client_ip, strerror(errno));
         break;
      }

      unpack_header(hdr_buf, &hdr);

      /* Validate magic */
      if (hdr.magic != CAM_MAGIC)
      {
         log_msg("Bad magic 0x%08X from %s — dropping connection",
                 hdr.magic, client_ip);
         break;
      }

      /* Validate version — rejects mismatched firmware/daemon builds
       * instead of silently misparsing a header of the wrong size. */
      if (hdr.version != CAM_HEADER_VER)
      {
         log_msg("Bad version %d (expected %d) from %s — dropping connection",
                 hdr.version, CAM_HEADER_VER, client_ip);
         break;
      }

      /* Validate device_id */
      if (hdr.device_id >= MAX_DOORBELL_CAMS)
      {
         log_msg("device_id %d out of range from %s — dropping",
                 hdr.device_id, client_ip);
         break;
      }

      /* Validate jpeg_size */
      if (0 == hdr.jpeg_size || hdr.jpeg_size > MAX_JPEG_BYTES)
      {
         log_msg("Bad jpeg_size %u from %s — dropping",
                 hdr.jpeg_size, client_ip);
         break;
      }

      /* Receive JPEG payload */
      jpeg = malloc(hdr.jpeg_size);
      if (NULL == jpeg)
      {
         log_msg("[DOORBELL] infer_failed event_id=" EVENT_ID_FMT
                 " reason=malloc_failed",
                 EVENT_ID_ARG(hdr.event_id));
         break;
      }

      received = 0;
      while (received < hdr.jpeg_size)
      {
         n = recv(client_fd, jpeg + received,
                  hdr.jpeg_size - received, 0);
         if (n <= 0)
         {
            log_msg("[DOORBELL] infer_failed event_id=" EVENT_ID_FMT
                    " reason=jpeg_recv_truncated",
                    EVENT_ID_ARG(hdr.event_id));
            free(jpeg);
            jpeg = NULL;
            break;
         }
         received += (size_t)n;
      }

      if (NULL == jpeg) { break; }

      log_msg("[DOORBELL] rx event_id=" EVENT_ID_FMT
              " device_id=%d jpeg_bytes=%zu",
              EVENT_ID_ARG(hdr.event_id), hdr.device_id, received);

      /* Run inference via shared worker */
      detected = 0;
      conf     = 0.0f;
      log_msg("[DOORBELL] infer_start event_id=" EVENT_ID_FMT,
              EVENT_ID_ARG(hdr.event_id));

      inference_worker_run(jpeg, received, &detected, &conf);

      /* Build tag: dev<N>_<event_id_hex> */
      snprintf(event_str, sizeof(event_str), EVENT_ID_FMT,
               EVENT_ID_ARG(hdr.event_id));
      snprintf(tag, sizeof(tag), "dev%d_%s", hdr.device_id, event_str);

      /* Save to /data/doorbell. ts_str receives the exact timestamp
       * string embedded in the saved filename, so the result publish
       * below uses an asset identifier guaranteed to match the file
       * (no independent time(NULL) call, no drift). */
      inference_worker_save(jpeg, received, DOORBELL_DIR,
                            detected, conf, tag,
                            ts_str, sizeof(ts_str));

      conf_pct = (uint8_t)(conf * 100.0f + 0.5f);

      log_msg("[DOORBELL] infer_done event_id=" EVENT_ID_FMT
              " device_id=%d person=%d conf=%d asset=%s",
              EVENT_ID_ARG(hdr.event_id),
              hdr.device_id, detected, conf_pct, ts_str);

      /* Cross-process boundary — result_shm_publish() emits the single
       * canonical [DOORBELL] -> [SHM] publish log line. Do not log here. */
      result_shm_publish(hdr.event_id, hdr.device_id,
                          (uint8_t)detected, conf_pct, ts_str);

      free(jpeg);
      jpeg = NULL;
   }

   if (NULL != jpeg) { free(jpeg); }
   close(client_fd);
}

/******************************** MAIN ****************************************/
int main(void)
{
   struct sockaddr_in client_addr;           /**< accepted client address  */
   socklen_t          client_len;            /**< client address length    */
   int                client_fd  = -1;      /**< accepted client fd       */
   char               client_ip[INET_ADDRSTRLEN]; /**< client IP string   */

   signal(SIGINT,  sig_handler);
   signal(SIGTERM, sig_handler);

   g_log = fopen(LOG_PATH, "a");
   log_msg("doorbell_daemon starting on port %d", LISTEN_PORT);
   fprintf(stdout, "[BOOT] %s\n", bbb_build_date);
   fprintf(stdout, "[BOOT] %s\n", bbb_build_target);

   if (inference_worker_init(MODEL_PATH, LABEL_PATH) < 0)
   {
      log_msg("ERROR: inference_worker_init failed");
      return 1;
   }

   /* Non-fatal: if this fails, daemon still ingests/saves/infers normally,
    * just without publishing results to the STM32 path. */
   (void)result_shm_init();

   tcp_server_init();

   while (g_running)
   {
      client_len = sizeof(client_addr);
      client_fd  = accept(g_server_fd,
                          (struct sockaddr *)&client_addr,
                          &client_len);
      if (client_fd < 0)
      {
         if (g_running) { log_msg("accept() error: %s", strerror(errno)); }
         continue;
      }

      inet_ntop(AF_INET, &client_addr.sin_addr,
                client_ip, sizeof(client_ip));

      handle_connection(client_fd, client_ip);
   }

   log_msg("doorbell_daemon shutting down");
   if (g_server_fd >= 0) { close(g_server_fd); }
   if (NULL != g_result)
   {
      munmap(g_result, sizeof(struct DoorbellResult));
   }
   inference_worker_shutdown();
   if (NULL != g_log) { fclose(g_log); }
   return 0;
}
