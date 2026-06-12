/******************************************************************************
 * \file inference_daemon.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief BeagleBone PIR inference daemon — TCP server on port 9090.
 *
 * \details ESP32-CAM connects and pushes a JPEG clip on PIR trigger.
 *          Each connection delivers a stream of [len:4][jpeg] frames
 *          until the ESP32 closes the connection (end of clip).
 *          Frames are saved as individual JPEGs to a temp directory,
 *          then assembled into an MJPEG .avi via ffmpeg.
 *          Inference runs on each frame; best confidence is recorded.
 *          DB record inserted directly via sqlite3 after .avi is written.
 *
 *          Wire protocol (port 9090):
 *          [jpeg_size:4] followed by jpeg_size bytes of JPEG payload.
 *          Repeated for each frame. Connection close = end of clip.
 *          (Simple 4-byte header — legacy PIR cam protocol.)
 *
 * \note    Inference logic extracted to inference_worker.c (2026-06-07)
 *          so doorbell_daemon can share the same perception pipeline.
 *          Transport parsing (4-byte header, TCP accept loop) stays here.
 *
 * \note    Clip streaming (2026-06-11):
 *          Changed from single JPEG receive to multi-frame clip receive.
 *          ESP32 streams frames for CAM_CLIP_DURATION_MS then closes.
 *          BBB saves frames as JPEGs, assembles .avi via ffmpeg,
 *          runs inference per-frame, inserts DB record via sqlite3.
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
#include <sys/stat.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <sqlite3.h>
#include "inference_worker.h"
#include "inference_core.h"

/******************************** CONFIG **************************************/
#define MODEL_PATH      "/opt/inference/models/detect.tflite"
#define LABEL_PATH      "/opt/inference/models/labelmap.txt"
#define CLIPS_DIR       "/data/clips"
#define DB_PATH         "/home/debian/db/sensors.db"
#define LOG_PATH        "/var/log/inference.log"
#define LISTEN_PORT     9090
#define MAX_CLIP_FRAMES 64        /**< hard cap on frames per clip           */
#define FRAME_FPS       2         /**< nominal FPS for ffmpeg encoding       */

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
      fprintf(g_log, "[%s] [indoor] ", ts);
      va_start(ap, fmt);
      vfprintf(g_log, fmt, ap);
      va_end(ap);
      fprintf(g_log, "\n");
      fflush(g_log);
   }

   fprintf(stderr, "[%s] [indoor] ", ts);
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

/******************************** DB INSERT ***********************************/

/**
 * \brief Insert a clip record into the sensors.db clips table.
 *
 * \param clip_path   Absolute path to the .avi file.
 * \param ts          Unix timestamp of clip start.
 * \param frame_count Number of frames in clip.
 * \param person      1 if person detected, 0 otherwise.
 * \param confidence  Best confidence across all frames.
 * \param duration_ms Clip duration in ms.
 */
static void db_insert_clip(const char *clip_path,
                            time_t      ts,
                            int         frame_count,
                            int         person,
                            float       confidence,
                            int         duration_ms)
{
   sqlite3      *db   = NULL;
   sqlite3_stmt *stmt = NULL;
   const char   *sql  =
       "INSERT INTO clips "
       "(ts, camera, clip_path, frame_count, person, confidence, duration_ms) "
       "VALUES (?, 'indoor', ?, ?, ?, ?, ?);";

   if (SQLITE_OK != sqlite3_open(DB_PATH, &db))
   {
      log_msg("ERROR: db open failed: %s", sqlite3_errmsg(db));
      if (db) { sqlite3_close(db); }
      return;
   }

   if (SQLITE_OK != sqlite3_prepare_v2(db, sql, -1, &stmt, NULL))
   {
      log_msg("ERROR: db prepare failed: %s", sqlite3_errmsg(db));
      sqlite3_close(db);
      return;
   }

   sqlite3_bind_int64 (stmt, 1, (sqlite3_int64)ts);
   sqlite3_bind_text  (stmt, 2, clip_path, -1, SQLITE_STATIC);
   sqlite3_bind_int   (stmt, 3, frame_count);
   sqlite3_bind_int   (stmt, 4, person);
   sqlite3_bind_double(stmt, 5, (double)confidence);
   sqlite3_bind_int   (stmt, 6, duration_ms);

   if (SQLITE_DONE != sqlite3_step(stmt))
      log_msg("ERROR: db insert failed: %s", sqlite3_errmsg(db));
   else
      log_msg("DB record inserted for %s", clip_path);

   sqlite3_finalize(stmt);
   sqlite3_close(db);
}

/******************************** CLIP RECEIVE ********************************/

/**
 * \brief Accept a connection and receive all frames until connection closes.
 *
 * \param out_frames      Output: array of frame buffers (caller frees each).
 * \param out_lens        Output: array of frame lengths.
 * \param out_n_frames    Output: number of frames received.
 * \param out_start_ts    Output: unix timestamp when connection was accepted.
 * \param out_duration_ms Output: elapsed ms from accept to connection close.
 * \return                0 on success, -1 on error.
 */
static int receive_clip(uint8_t ***out_frames,
                        size_t   **out_lens,
                        int       *out_n_frames,
                        time_t    *out_start_ts,
                        int       *out_duration_ms)
{
   uint8_t        **frames   = NULL;
   size_t          *lens     = NULL;
   int              n_frames = 0;
   struct timespec  t0, t1;

   frames = calloc(MAX_CLIP_FRAMES, sizeof(uint8_t *));
   lens   = calloc(MAX_CLIP_FRAMES, sizeof(size_t));
   if (NULL == frames || NULL == lens)
   {
      log_msg("ERROR: calloc clip arrays");
      free(frames); free(lens);
      return -1;
   }

   log_msg("Waiting for ESP32-CAM connection...");
   g_client_fd = accept(g_server_fd, NULL, NULL);
   if (g_client_fd < 0)
   {
      log_msg("accept() failed: %s", strerror(errno));
      free(frames); free(lens);
      return -1;
   }

   *out_start_ts = time(NULL);
   clock_gettime(CLOCK_MONOTONIC, &t0);
   log_msg("ESP32-CAM connected — receiving clip");

   int keepalive = 1, keepidle = 30, keepintvl = 5, keepcnt = 3;
   setsockopt(g_client_fd, SOL_SOCKET,  SO_KEEPALIVE, &keepalive, sizeof(keepalive));
   setsockopt(g_client_fd, IPPROTO_TCP, TCP_KEEPIDLE,  &keepidle,  sizeof(keepidle));
   setsockopt(g_client_fd, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
   setsockopt(g_client_fd, IPPROTO_TCP, TCP_KEEPCNT,   &keepcnt,   sizeof(keepcnt));

   while (g_running)
   {
      uint8_t  hdr[JPEG_HDR_SIZE];
      uint32_t jpeg_len;
      uint8_t *buf;
      ssize_t  n;
      size_t   received;

      n = recv(g_client_fd, hdr, JPEG_HDR_SIZE, MSG_WAITALL);
      if (n == 0) { log_msg("Clip complete — connection closed by ESP32"); break; }
      if (n != JPEG_HDR_SIZE) { log_msg("Header recv error: %s", strerror(errno)); break; }

      jpeg_len = infer_unpack_jpeg_len(hdr);
      if (!infer_jpeg_len_valid(jpeg_len))
      {
         log_msg("Bad jpeg_len %u — dropping connection", jpeg_len);
         break;
      }

      if (n_frames >= MAX_CLIP_FRAMES)
      {
         log_msg("WARN: MAX_CLIP_FRAMES reached — draining frame");
         uint8_t drain[512];
         size_t  drained = 0;
         while (drained < jpeg_len)
         {
            size_t chunk = jpeg_len - drained;
            if (chunk > sizeof(drain)) { chunk = sizeof(drain); }
            n = recv(g_client_fd, drain, chunk, 0);
            if (n <= 0) { break; }
            drained += (size_t)n;
         }
         continue;
      }

      buf = malloc(jpeg_len);
      if (NULL == buf) { log_msg("ERROR: malloc frame"); break; }

      received = 0;
      while (received < jpeg_len)
      {
         n = recv(g_client_fd, buf + received, jpeg_len - received, 0);
         if (n <= 0)
         {
            log_msg("Frame recv truncated at frame %d", n_frames);
            free(buf); buf = NULL; break;
         }
         received += (size_t)n;
      }
      if (NULL == buf) { break; }

      frames[n_frames] = buf;
      lens[n_frames]   = jpeg_len;
      n_frames++;
      log_msg("Frame %d received (%u bytes)", n_frames, jpeg_len);
   }

   clock_gettime(CLOCK_MONOTONIC, &t1);
   *out_duration_ms = (int)(((t1.tv_sec  - t0.tv_sec)  * 1000) +
                             ((t1.tv_nsec - t0.tv_nsec) / 1000000));

   close(g_client_fd);
   g_client_fd = -1;

   *out_frames   = frames;
   *out_lens     = lens;
   *out_n_frames = n_frames;
   return (n_frames > 0) ? 0 : -1;
}

/******************************** CLIP PROCESS ********************************/

/**
 * \brief Save frames, run inference, encode .avi via ffmpeg, insert DB record.
 *
 * \param frames      Array of JPEG frame buffers.
 * \param lens        Array of frame lengths.
 * \param n_frames    Number of frames.
 * \param start_ts    Unix timestamp of clip start.
 * \param duration_ms Clip duration in ms.
 */
static void process_clip(uint8_t **frames,
                         size_t   *lens,
                         int       n_frames,
                         time_t    start_ts,
                         int       duration_ms)
{
   char      ts_str[32];
   char      tmp_dir[256];
   char      avi_path[320];
   char      frame_path[320];
   char      cmd[1024];
   int       best_detected = 0;
   float     best_conf     = 0.0f;
   int       i;
   struct tm tm_buf;
   FILE     *f;

   mkdir(CLIPS_DIR, 0755);

   localtime_r(&start_ts, &tm_buf);
   strftime(ts_str, sizeof(ts_str), "%Y%m%dT%H%M%SZ", &tm_buf);

   /* Temp directory for JPEG frames */
   snprintf(tmp_dir, sizeof(tmp_dir), "%s/tmp_%s", CLIPS_DIR, ts_str);
   mkdir(tmp_dir, 0755);

   /* Save frames + run inference */
   for (i = 0; i < n_frames; i++)
   {
      int   detected = 0;
      float conf     = 0.0f;

      snprintf(frame_path, sizeof(frame_path), "%s/frame_%03d.jpg", tmp_dir, i);
      f = fopen(frame_path, "wb");
      if (NULL != f) { fwrite(frames[i], 1, lens[i], f); fclose(f); }

      inference_worker_run(frames[i], lens[i], &detected, &conf);
      log_msg("Frame %d: person=%d confidence=%.2f", i + 1, detected, conf);

      if (detected && conf > best_conf) { best_detected = 1; best_conf = conf; }
   }

   log_msg("Clip result: person=%d best_confidence=%.2f frames=%d duration=%dms",
           best_detected, best_conf, n_frames, duration_ms);

   /* Build .avi path */
   snprintf(avi_path, sizeof(avi_path), "%s/%s_indoor_%s_%03d.avi",
            CLIPS_DIR, ts_str,
            best_detected ? "person" : "noperson",
            (int)(best_conf * 100));

   /* Encode .avi via ffmpeg */
   snprintf(cmd, sizeof(cmd),
            "ffmpeg -y -framerate %d -i %s/frame_%%03d.jpg "
            "-c:v mjpeg -q:v 3 %s 2>/dev/null",
            FRAME_FPS, tmp_dir, avi_path);

   if (system(cmd) != 0)
   {
      log_msg("ERROR: ffmpeg failed for clip %s", avi_path);
   }
   else
   {
      log_msg("Clip saved: %s", avi_path);
      db_insert_clip(avi_path, start_ts, n_frames,
                     best_detected, best_conf, duration_ms);
   }

   /* Clean up temp frames */
   for (i = 0; i < n_frames; i++)
   {
      snprintf(frame_path, sizeof(frame_path), "%s/frame_%03d.jpg", tmp_dir, i);
      unlink(frame_path);
   }
   rmdir(tmp_dir);
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
      uint8_t **frames      = NULL;
      size_t   *lens        = NULL;
      int       n_frames    = 0;
      time_t    start_ts    = 0;
      int       duration_ms = 0;

      if (receive_clip(&frames, &lens, &n_frames,
                       &start_ts, &duration_ms) < 0)
      {
         usleep(100000);
         continue;
      }

      process_clip(frames, lens, n_frames, start_ts, duration_ms);

      for (int i = 0; i < n_frames; i++) { free(frames[i]); }
      free(frames);
      free(lens);
   }

   log_msg("inference_daemon shutting down");
   if (g_client_fd >= 0) { close(g_client_fd); }
   if (g_server_fd >= 0) { close(g_server_fd); }
   inference_worker_shutdown();
   if (NULL != g_log) { fclose(g_log); }
   return 0;
}
