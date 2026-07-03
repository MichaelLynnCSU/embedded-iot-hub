/******************************************************************************
 * \file inference_daemon.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief BeagleBone PIR inference daemon — TCP server on port 9090.
 *
 * \details ESP32-CAM connects and pushes a JPEG clip on PIR trigger.
 *          Each connection delivers a stream of [header:20][jpeg] frames
 *          until the ESP32 closes the connection (end of clip).
 *          Frames are saved as individual JPEGs to a temp directory,
 *          then assembled into an MJPEG .avi via ffmpeg.
 *          Inference runs on each frame; best confidence is recorded.
 *          DB record inserted directly via sqlite3 after .avi is written.
 *
 *          Wire protocol (port 9090):
 *          [magic:4][version:1][device_id:1][reserved:2][event_id:8][jpeg_size:4]
 *          followed by jpeg_size bytes of JPEG payload. Repeated for each
 *          frame. Connection close = end of clip. Same header layout as
 *          the doorbell path (port 9091) — see inference_core.h.
 *
 * \note    Inference logic extracted to inference_worker.c (2026-06-07)
 *          so doorbell_daemon can share the same perception pipeline.
 *          Transport parsing (header unpack, TCP accept loop) stays here.
 *
 * \note    event_id header (2026-07-02):
 *          Wire protocol upgraded from a bare 4-byte big-endian JPEG
 *          length prefix (no correlation key) to the 20-byte
 *          indoor_header_t layout above, matching cam_header_t on the
 *          esp32-cam side (esp32-cam/main/cam_logic.h). magic/version are
 *          validated on every header via infer_header_valid() so a peer
 *          still running the old 4-byte-only firmware is rejected
 *          cleanly (its first 4 bytes essentially never equal CAM_MAGIC)
 *          instead of being silently misparsed as a header.
 *
 *          event_id is relayed unchanged from the UDP CAPTURE trigger
 *          all the way through camera_manager's [TRIGGER] sent line and
 *          esp32-cam's [UDP_CAM_RX]/[CAM] lines (see esp32-cam/main/main.c)
 *          into this daemon, which now logs it on every clip lifecycle
 *          line via EVENT_ID_FMT/EVENT_ID_ARG (inference_core.h) — the
 *          same convention doorbell_daemon.c already uses. One event_id,
 *          grep-traceable end to end:
 *              grep <event_id_hex> /var/log/inference.log
 *          See the updated logging contract note below.
 *
 * \note    Clip streaming (2026-06-11):
 *          Changed from single JPEG receive to multi-frame clip receive.
 *          ESP32 streams frames for CAM_CLIP_DURATION_MS then closes.
 *          BBB saves frames as JPEGs, assembles .avi via ffmpeg,
 *          runs inference per-frame, inserts DB record via sqlite3.
 *
 * \note    ffmpeg re-encode removed / -c:v copy (2026-06-13):
 *          Previous ffmpeg command was:
 *              ffmpeg -framerate 2 -i frame_%03d.jpg -c:v mjpeg -q:v 3 out.avi
 *          This caused two distinct problems:
 *
 *          1) Double-encode quality loss. The ESP32 sends JPEG frames
 *             already compressed by the OV3660 sensor at quality=6
 *             (esp32-camera scale). ffmpeg was decoding those JPEGs to
 *             raw YUV, then re-encoding as MJPEG at -q:v 3. Every
 *             decode+re-encode cycle permanently destroys detail that
 *             cannot be recovered — the saved AVI looked visibly softer
 *             and lower resolution than the source frames even though the
 *             pixel dimensions were unchanged. This was the root cause of
 *             the "smooth low pixel" appearance reported in field clips.
 *
 *          2) FPS mismatch. FRAME_FPS was set to 2, but the ESP32 sends
 *             frames every CAM_CLIP_FRAME_MS = ~530ms (~1.9fps actual).
 *             A small mismatch, but the correct value derived from the
 *             actual ESP32 config is 2 fps (floor of 1000/530), so
 *             FRAME_FPS stays at 2 — it is now explicitly documented as
 *             derived from CAM_CLIP_FRAME_MS on the ESP32 side and should
 *             be updated if that constant changes.
 *
 *          Fix: replaced -c:v mjpeg -q:v 3 with -c:v copy. ffmpeg now
 *          muxes the source JPEGs directly into the AVI container with
 *          zero re-encoding. The frames in the .avi are bit-for-bit
 *          identical to what the OV3660 produced. AVI/MJPEG supports
 *          JPEG passthrough natively so no container change is needed.
 *          ffmpeg encode time also drops significantly since there is no
 *          decode/encode cycle.
 *
 *          If FRAME_FPS ever needs to change, derive it as:
 *              FRAME_FPS = 1000 / CAM_CLIP_FRAME_MS   (from main.c)
 *          and update the constant here to match.
 *
 * \note    Logging contract (commit 2, 2026-07-02):
 *          event_id now flows through this daemon (see note above). No
 *          cross-process shm boundaries. Flat [INDOOR] tags are used for
 *          all lifecycle events, with event_id (EVENT_ID_FMT/EVENT_ID_ARG)
 *          on every line from clip_start onward — a single connection can
 *          span multiple frames, all sharing the one event_id read off
 *          the first frame's header:
 *
 *          Transport layer (receive_clip):
 *            [INDOOR] clip_start event_id=<hex>
 *                                              — first header received, clip beginning
 *            [INDOOR] rx event_id=<hex> frame=%d bytes=%u
 *                                              — single frame received
 *            [INDOOR] recv_failed event_id=<hex> frame=%d reason=...
 *                                              — recv/header truncated or invalid (transport error)
 *            [INDOOR] clip_done event_id=<hex> frames=%d duration_ms=%d
 *                                              — connection closed by ESP32
 *
 *          Pipeline layer (process_clip):
 *            [INDOOR] infer_start event_id=<hex> frame=%d
 *                                              — inference about to run
 *            [INDOOR] infer_done  event_id=<hex> frame=%d person=%d conf=%.2f
 *            [INDOOR] infer_skip  event_id=<hex> frame=%d reason=interval_gate every_n=%d
 *            [INDOOR] clip_saved  event_id=<hex> path=%s — ffmpeg mux succeeded
 *            [INDOOR] clip_failed event_id=<hex> path=%s — ffmpeg mux failed
 *            [INDOOR] db_insert   event_id=<hex> path=%s — DB record inserted
 *
 *          This mirrors the DOORBELL daemon contract structure:
 *            clip_start  ↔ doorbell rx
 *            infer_start ↔ doorbell infer_start
 *            infer_done  ↔ doorbell infer_done
 *            clip_saved  ↔ doorbell [DOORBELL] -> [SHM] publish
 *          event_id is read from the first frame's header only; if a
 *          later frame in the same clip somehow carried a different
 *          event_id (should never happen — one trigger, one TCP
 *          connection, one event_id) the first frame's value wins and
 *          is what the whole clip is logged/saved under.
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
#include "build_info.h"

/******************************** CONFIG **************************************/
#define MODEL_PATH      "/opt/inference/models/detect.tflite"
#define LABEL_PATH      "/opt/inference/models/labelmap.txt"
#define CLIPS_DIR       "/data/clips"
#define DB_PATH         "/home/debian/db/sensors.db"
#define LOG_PATH        "/var/log/inference.log"
#define LISTEN_PORT     9090
#define MAX_CLIP_FRAMES 64        /**< hard cap on frames per clip           */

/* Nominal FPS for ffmpeg mux. Derived from CAM_CLIP_FRAME_MS in main.c:
 *   ESP32 sends one frame every CAM_CLIP_FRAME_MS ms (~530ms = ~1.9fps).
 *   floor(1000 / 530) = 1, but 2fps is the closest standard value and
 *   matches observed inter-frame timing (~530ms). Update this if
 *   CAM_CLIP_FRAME_MS changes on the ESP32 side. */
#define FRAME_FPS       2

/* Minimum interval between inference calls in process_clip().
 * Frames are still saved to disk and included in the AVI regardless.
 * Only inference_worker_run() is skipped for frames within the interval.
 *
 * At 2fps (one frame every ~530ms), INFER_INTERVAL_MS=500 means inference
 * runs on approximately every frame. Raise to 1000+ to halve CPU load.
 * Set to 0 to disable gating (every-frame inference, original behaviour). */
#define INFER_INTERVAL_MS  500

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
 * \param event_id    Correlation key from the clip's frame headers, logged
 *                    on the [INDOOR] db_insert line (not a DB column —
 *                    the clips table schema is unchanged; event_id lives
 *                    only in the logs for now, same as doorbell's asset
 *                    field is a display hint rather than a stored key).
 */
static void db_insert_clip(const char *clip_path,
                            time_t      ts,
                            int         frame_count,
                            int         person,
                            float       confidence,
                            int         duration_ms,
                            uint64_t    event_id)
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
   {
      log_msg("ERROR: db insert failed: %s", sqlite3_errmsg(db));
   }
   else
   {
      log_msg("[INDOOR] db_insert event_id=" EVENT_ID_FMT " path=%s",
              EVENT_ID_ARG(event_id), clip_path);
   }

   sqlite3_finalize(stmt);
   sqlite3_close(db);
}

/******************************** CLIP RECEIVE ********************************/

/**
 * \brief Accept a connection and receive all frames until connection closes.
 *
 * \details Reads an indoor_header_t (CAM_HEADER_SIZE bytes) before each
 *          JPEG payload. event_id is read off the first frame's header
 *          and reused for every subsequent log line in this clip — one
 *          TCP connection corresponds to one PIR trigger, so all frames
 *          in it share the same event_id (the ESP32 side only ever opens
 *          a new connection per trigger; see esp32-cam/main/main.c
 *          capture_task()).
 *
 *          Logs [INDOOR] clip_start on the first valid header, [INDOOR]
 *          rx on each frame received, [INDOOR] recv_failed on transport
 *          error or a bad/mismatched header, and [INDOOR] clip_done on
 *          clean connection close.
 *
 * \param out_frames      Output: array of frame buffers (caller frees each).
 * \param out_lens        Output: array of frame lengths.
 * \param out_n_frames    Output: number of frames received.
 * \param out_start_ts    Output: unix timestamp when connection was accepted.
 * \param out_duration_ms Output: elapsed ms from accept to connection close.
 * \param out_event_id    Output: event_id read from the first frame's
 *                        header (0 if no frame was ever received).
 * \return                0 on success, -1 on error.
 */
static int receive_clip(uint8_t ***out_frames,
                        size_t   **out_lens,
                        int       *out_n_frames,
                        time_t    *out_start_ts,
                        int       *out_duration_ms,
                        uint64_t  *out_event_id,
                        uint32_t  *out_seq)
{
   uint8_t        **frames   = NULL;
   size_t          *lens     = NULL;
   int              n_frames = 0;
   uint64_t         event_id = 0;
   uint32_t         seq      = 0;
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

   int keepalive = 1, keepidle = 30, keepintvl = 5, keepcnt = 3;
   setsockopt(g_client_fd, SOL_SOCKET,  SO_KEEPALIVE, &keepalive, sizeof(keepalive));
   setsockopt(g_client_fd, IPPROTO_TCP, TCP_KEEPIDLE,  &keepidle,  sizeof(keepidle));
   setsockopt(g_client_fd, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
   setsockopt(g_client_fd, IPPROTO_TCP, TCP_KEEPCNT,   &keepcnt,   sizeof(keepcnt));

   while (g_running)
   {
      uint8_t          hdr_buf[CAM_HEADER_SIZE];
      indoor_header_t  hdr;
      uint8_t         *buf;
      ssize_t          n;
      size_t           received;

      n = recv(g_client_fd, hdr_buf, CAM_HEADER_SIZE, MSG_WAITALL);
      if (n == 0)
      {
         /* Clean close by ESP32 — normal end of clip */
         break;
      }
      if (n != CAM_HEADER_SIZE)
      {
         log_msg("[INDOOR] recv_failed event_id=" EVENT_ID_FMT
                 " frame=%d reason=header_truncated",
                 EVENT_ID_ARG(event_id), n_frames);
         break;
      }

      infer_unpack_header(hdr_buf, &hdr);

      if (!infer_header_valid(&hdr))
      {
         log_msg("[INDOOR] recv_failed event_id=" EVENT_ID_FMT
                 " frame=%d reason=bad_magic magic=0x%08X",
                 EVENT_ID_ARG(event_id), n_frames, hdr.magic);
         break;
      }

      if (0 == n_frames)
      {
         /* First frame of this connection — this is the event_id for
          * the whole clip. */
         event_id = hdr.event_id;
         seq      = hdr.seq;
         log_msg("[INDOOR] clip_start event_id=" EVENT_ID_FMT
                 " seq=%u device_id=%d",
                 EVENT_ID_ARG(event_id), seq, hdr.device_id);
      }
      else if (hdr.event_id != event_id)
      {
         /* Should never happen — one trigger, one connection, one
          * event_id (see doc comment above). Log it but keep the
          * clip's original event_id rather than switching mid-clip. */
         log_msg("[INDOOR] WARNING event_id mismatch mid-clip: "
                 "clip=" EVENT_ID_FMT " frame=" EVENT_ID_FMT " frame=%d",
                 EVENT_ID_ARG(event_id), EVENT_ID_ARG(hdr.event_id),
                 n_frames);
      }

      if (!infer_jpeg_len_valid(hdr.jpeg_size))
      {
         log_msg("[INDOOR] recv_failed event_id=" EVENT_ID_FMT
                 " frame=%d reason=bad_jpeg_len len=%u",
                 EVENT_ID_ARG(event_id), n_frames, hdr.jpeg_size);
         break;
      }

      if (n_frames >= MAX_CLIP_FRAMES)
      {
         log_msg("WARN: MAX_CLIP_FRAMES reached — draining frame");
         uint8_t drain[512];
         size_t  drained = 0;
         while (drained < hdr.jpeg_size)
         {
            size_t chunk = hdr.jpeg_size - drained;
            if (chunk > sizeof(drain)) { chunk = sizeof(drain); }
            n = recv(g_client_fd, drain, chunk, 0);
            if (n <= 0) { break; }
            drained += (size_t)n;
         }
         continue;
      }

      buf = malloc(hdr.jpeg_size);
      if (NULL == buf)
      {
         log_msg("[INDOOR] recv_failed event_id=" EVENT_ID_FMT
                 " frame=%d reason=malloc_failed bytes=%u",
                 EVENT_ID_ARG(event_id), n_frames, hdr.jpeg_size);
         break;
      }

      received = 0;
      while (received < hdr.jpeg_size)
      {
         n = recv(g_client_fd, buf + received, hdr.jpeg_size - received, 0);
         if (n <= 0)
         {
            log_msg("[INDOOR] recv_failed event_id=" EVENT_ID_FMT
                    " frame=%d reason=payload_truncated",
                    EVENT_ID_ARG(event_id), n_frames);
            free(buf); buf = NULL; break;
         }
         received += (size_t)n;
      }
      if (NULL == buf) { break; }

      frames[n_frames] = buf;
      lens[n_frames]   = hdr.jpeg_size;
      n_frames++;

      log_msg("[INDOOR] rx event_id=" EVENT_ID_FMT " seq=%u frame=%d bytes=%u",
              EVENT_ID_ARG(event_id), seq, n_frames, hdr.jpeg_size);
   }

   clock_gettime(CLOCK_MONOTONIC, &t1);
   *out_duration_ms = (int)(((t1.tv_sec  - t0.tv_sec)  * 1000) +
                             ((t1.tv_nsec - t0.tv_nsec) / 1000000));

   close(g_client_fd);
   g_client_fd = -1;

   log_msg("[INDOOR] clip_done event_id=" EVENT_ID_FMT
           " seq=%u frames=%d duration_ms=%d",
           EVENT_ID_ARG(event_id), seq, n_frames, *out_duration_ms);

   *out_frames   = frames;
   *out_lens     = lens;
   *out_n_frames = n_frames;
   *out_event_id = event_id;
   *out_seq      = seq;
   return (n_frames > 0) ? 0 : -1;
}

/******************************** CLIP PROCESS ********************************/

/**
 * \brief Save frames, run inference, encode .avi via ffmpeg, insert DB record.
 *
 * \details Logs [INDOOR] infer_start before each inference call,
 *          [INDOOR] infer_done after a result, [INDOOR] infer_skip for
 *          gated frames. Logs [INDOOR] clip_saved or [INDOOR] clip_failed
 *          after ffmpeg, and [INDOOR] db_insert after DB write (via
 *          db_insert_clip).
 *
 * \param frames      Array of JPEG frame buffers.
 * \param lens        Array of frame lengths.
 * \param n_frames    Number of frames.
 * \param start_ts    Unix timestamp of clip start.
 * \param duration_ms Clip duration in ms.
 * \param event_id    Correlation key from the clip's frame headers —
 *                    logged on every pipeline-layer line so this clip's
 *                    processing is traceable back to receive_clip()'s
 *                    transport-layer lines with the same event_id.
 */
static void process_clip(uint8_t **frames,
                         size_t   *lens,
                         int       n_frames,
                         time_t    start_ts,
                         int       duration_ms,
                         uint64_t  event_id,
                         uint32_t  seq)
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

   /* Save frames + run inference (time-gated).
    *
    * Inference runs on every Nth frame where N = INFER_INTERVAL_MS / frame_period.
    * All frames are saved to disk and included in the AVI regardless.
    * Frame 0 is always inferred so no clip starts blind.
    *
    * Example: 10s clip, 20 frames -> frame_period = 500ms.
    *   INFER_INTERVAL_MS=500  -> every frame  (N=1)
    *   INFER_INTERVAL_MS=1000 -> every 2nd    (N=2)
    *   INFER_INTERVAL_MS=2000 -> every 4th    (N=4)
    *
    * Set INFER_INTERVAL_MS=0 to disable gating (original every-frame behaviour). */
   int frame_period_ms = (n_frames > 0 && duration_ms > 0) ? (duration_ms / n_frames) : 500;
   int infer_every_n;

   if (INFER_INTERVAL_MS > 0 && frame_period_ms > 0)
   {
      infer_every_n = INFER_INTERVAL_MS / frame_period_ms;
   }
   else
   {
      infer_every_n = 1; /* fallback: infer every frame */
   }
   if (infer_every_n < 1) { infer_every_n = 1; }

   for (i = 0; i < n_frames; i++)
   {
      int   detected = 0;
      float conf     = 0.0f;

      /* Save frame to disk unconditionally */
      snprintf(frame_path, sizeof(frame_path), "%s/frame_%03d.jpg", tmp_dir, i);
      f = fopen(frame_path, "wb");
      if (NULL != f) { fwrite(frames[i], 1, lens[i], f); fclose(f); }

      /* Run inference on every Nth frame */
      if (i % infer_every_n == 0)
      {
         log_msg("[INDOOR] infer_start event_id=" EVENT_ID_FMT " seq=%u frame=%d",
                 EVENT_ID_ARG(event_id), seq, i + 1);
         inference_worker_run(frames[i], lens[i], &detected, &conf);
         log_msg("[INDOOR] infer_done event_id=" EVENT_ID_FMT
                 " seq=%u frame=%d person=%d conf=%.2f",
                 EVENT_ID_ARG(event_id), seq, i + 1, detected, conf);
         if (detected && conf > best_conf) { best_detected = 1; best_conf = conf; }
      }
      else
      {
         log_msg("[INDOOR] infer_skip event_id=" EVENT_ID_FMT
                 " frame=%d reason=interval_gate every_n=%d",
                 EVENT_ID_ARG(event_id), i + 1, infer_every_n);
      }
   }

   /* Encode .avi via ffmpeg.
    *
    * -c:v copy: mux source JPEGs directly into the AVI container with no
    * re-encoding. Frames are bit-for-bit identical to what the OV3660
    * produced. Previously -c:v mjpeg -q:v 3 was used, which decoded each
    * JPEG to raw YUV then re-encoded it, destroying detail on every frame.
    * See file header note for full explanation.
    *
    * -framerate %d: set to FRAME_FPS (2), matching the ~530ms inter-frame
    * interval from CAM_CLIP_FRAME_MS on the ESP32 side. */
   snprintf(avi_path, sizeof(avi_path), "%s/%s.avi", CLIPS_DIR, ts_str);
   snprintf(cmd, sizeof(cmd),
            "ffmpeg -y -framerate %d -i %s/frame_%%03d.jpg "
            "-c:v copy %s 2>/dev/null",
            FRAME_FPS, tmp_dir, avi_path);

   if (system(cmd) != 0)
   {
      log_msg("[INDOOR] clip_failed event_id=" EVENT_ID_FMT " seq=%u path=%s",
              EVENT_ID_ARG(event_id), seq, avi_path);
   }
   else
   {
      log_msg("[INDOOR] clip_saved event_id=" EVENT_ID_FMT " seq=%u path=%s",
              EVENT_ID_ARG(event_id), seq, avi_path);
      db_insert_clip(avi_path, start_ts, n_frames,
                     best_detected, best_conf, duration_ms, event_id);
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
   fprintf(stdout, "[BOOT] %s\n", bbb_build_date);
   fprintf(stdout, "[BOOT] %s\n", bbb_build_target);

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
      uint64_t  event_id    = 0;
      uint32_t  seq         = 0;

      if (receive_clip(&frames, &lens, &n_frames,
                       &start_ts, &duration_ms, &event_id, &seq) < 0)
      {
          /* SIGTERM/SIGINT sets g_running=0 and interrupts any blocking
          * syscall (accept, recv) with EINTR, causing receive_clip() to
          * return -1. The while(g_running) condition would catch it on
          * the next iteration, but checking here avoids the usleep(100ms)
          * delay and exits immediately on the same pass. */
         if (!g_running) { break; }
         usleep(100000);
         continue;
      }

      process_clip(frames, lens, n_frames, start_ts, duration_ms, event_id, seq);

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
