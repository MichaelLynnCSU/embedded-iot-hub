/******************************************************************************
 * \file doorbell_stream_daemon.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief BeagleBone doorbell MJPEG stream daemon.
 *
 * \details TCP server on port 9093. Accepts connections from ESP32 doorbell
 *          cams in stream mode. Each connection sends a 1-byte device_id
 *          followed by a continuous stream of [len:4][jpeg] frames until
 *          the ESP32 closes the connection. Each connected device gets its
 *          own ffmpeg process which pushes an RTSP stream to mediamtx at
 *          rtsp://localhost:8554/doorbell<N>.
 *
 *          Wire protocol (port 9093):
 *          [device_id:1] — sent once on connect
 *          [jpeg_size:4][jpeg bytes] — repeated for each frame
 *          Connection close = end of stream.
 *
 *          One thread per connection so multiple doorbell cams can stream
 *          simultaneously without blocking each other.
 *
 * \note    No inference, no file saving, no sqlite. This daemon's only
 *          job is to relay MJPEG frames from ESP32 to mediamtx via ffmpeg.
 *          Inference on doorbell stills is handled by doorbell_daemon on
 *          port 9091 (one-shot JPEG + header, separate wire protocol).
 *
 * \note    SIGPIPE is ignored globally. If ffmpeg exits mid-stream,
 *          fwrite() returns an error instead of killing the daemon.
 *
 * \note    Truncation fix (2026-06-14):
 *          Root cause: ESP32 stream_send_all() treated SO_SNDTIMEO-induced
 *          send() failures (lwIP returns -1/ETIMEDOUT under backpressure)
 *          as fatal socket errors, closing the connection mid-frame. The
 *          BBB recv_all() saw the resulting TCP FIN mid-frame and logged
 *          "JPEG recv truncated", collapsing clean peer-close and transport
 *          errors into one bucket.
 *
 *          BBB fix (this file): recv_all() now returns -2 on clean TCP FIN
 *          (r == 0) and -1 on transport error (r < 0), so conn_thread()
 *          can log the correct cause. "connection closed mid-frame" means
 *          the ESP32 dropped the socket (almost always due to the send-side
 *          ETIMEDOUT misclassification). "JPEG recv truncated" means a real
 *          transport error occurred.
 *
 *          ESP32 fix (stream_send_all() in main.c): ETIMEDOUT added to the
 *          transient-retry set alongside EAGAIN/EWOULDBLOCK, and errno is
 *          only evaluated when send() returns < 0 (lwIP only guarantees
 *          errno validity on failure). This prevents the soft-stall ->
 *          socket-close -> BBB-truncation chain.
 *
 * \note    Corrupt frame / mediamtx timeout investigation (2026-06-14):
 *          Observed: ffmpeg publishes to mediamtx successfully but mediamtx
 *          kills the publisher after ~1-3 minutes with "i/o timeout". ffmpeg
 *          log showed frames flowing (non-monotonic DTS warnings) but
 *          mediamtx still timing out. Root cause identified via log
 *          correlation: ESP32 WiFi jitter causes irregular frame delivery.
 *          When the daemon blocks in recv_all() waiting for a slow frame,
 *          fwrite() to ffmpeg's stdin pipe is also stalled. ffmpeg blocks on
 *          stdin read, stops sending RTP packets to mediamtx, and mediamtx
 *          declares the publisher inactive after its inactivity window.
 *          ffmpeg only detects the dead mediamtx socket ~90s later on the
 *          next write attempt (EPIPE), creating the misleading delayed log.
 *
 *          The non-monotonic DTS warnings are a side-effect of irregular
 *          frame timing from WiFi jitter — they are harmless and do NOT
 *          cause the mediamtx timeout.
 *
 *          Attempted: "-c:v mjpeg -q:v 3" (re-encode). No improvement.
 *          Attempted: "-flush_packets 1", "-stimeout". Cosmetic only.
 *          Root cause is architectural: backpressure propagation from
 *          ESP32 jitter through the daemon pipe into ffmpeg's RTSP output.
 *
 * \note    Producer/consumer queue architecture (2026-06-14):
 *          Fix: decouple the ESP32 receiver from the ffmpeg writer using a
 *          bounded lock-free-style ring queue with drop-oldest policy.
 *
 *          Thread model per connection:
 *          - conn_thread (producer): receives frames from ESP32 over TCP,
 *            pushes to queue. Never blocks on ffmpeg.
 *          - ffmpeg_thread (consumer): pulls from queue, writes to ffmpeg
 *            stdin. Waits on condvar with timeout so it never blocks
 *            indefinitely — preventing mediamtx inactivity timeout even
 *            during ESP32 jitter gaps.
 *
 *          Queue spec:
 *          - FRAME_QUEUE_DEPTH slots (4) — absorbs ~100-200ms WiFi bursts
 *          - Drop-oldest when full: doorbell live view always wants the
 *            freshest frame, never a stale backlog. Blocking the producer
 *            would recreate the stall cascade at a higher level.
 *          - Mutex + condvar for producer/consumer sync.
 *          - Mutex never held during fwrite() — only during queue slot
 *            access.
 *
 *          Result: ESP32 jitter is absorbed by the queue. ffmpeg sees a
 *          steady frame supply. mediamtx sees continuous RTP activity.
 *          The entire backpressure propagation chain is broken.
 *
 * \note    WiFi architecture (2026-06-15, committed to Yocto meta-bbb):
 *          The BBB WiFi workload was split across two dedicated interfaces:
 *          - ESP-01 (UART bridge) → handles sensor/hub traffic to router
 *            via the main network. No change.
 *          - Dedicated USB dual-band dongle (RTL8821CU, wlu1) → runs
 *            hostapd AP on 2.4GHz ch6, dedicated exclusively to camera
 *            traffic. dnsmasq provides DHCP on 10.0.1.0/24. All ESP32
 *            cams connect directly to the BBB AP — no router in the camera
 *            path. SSID: cams_2.4.
 *
 *          Result: 7-second stalls eliminated. FPS stabilized at 27.8 with
 *          signal -56 dBm to BBB AP. Remaining stalls (100-400ms, periodic)
 *          identified as TCP backpressure from the BBB receive thread
 *          blocking on synchronous eMMC writes in log_msg() fflush() — see
 *          logging note below.
 *
 *          Observed failure mode under human presence: a person standing
 *          near the camera absorbs/reflects 2.4GHz RF sufficiently to
 *          degrade the ESP32→BBB link. MJPEG frame sizes increase from
 *          ~2600 bytes (idle scene) to 4000-5000 bytes (person present),
 *          compounding the RF degradation. Send stalls of 500-740ms
 *          observed, FPS dropping to 10-11. Post-person RF settling takes
 *          10-30 seconds, during which stalls continue at baseline frame
 *          sizes.
 *
 * \note    Logging contract and fflush fix (2026-06-15):
 *          All lifecycle events use flat [STREAM] tags. No cross-process
 *          boundaries exist in this daemon (TCP in, ffmpeg pipe out is
 *          intra-process), so no directional arrow tags are used.
 *
 *          Transport layer (recv path):
 *            [STREAM] connect      device_id=%u ip=%s
 *            [STREAM] recv_closed  ip=%s        (clean TCP FIN mid-frame)
 *            [STREAM] recv_error   ip=%s errno=%d (transport error)
 *            [STREAM] decode_failed ip=%s reason=bad_len|malloc_failed
 *            [STREAM] disconnect   device_id=%u ip=%s
 *
 *          Pipeline layer (ffmpeg path):
 *            [STREAM] ffmpeg_start  device_id=%u url=...
 *            [STREAM] ffmpeg_failed device_id=%u
 *            [STREAM] write_failed  ip=%s
 *
 *          Queue starvation diagnostics:
 *            [STREAM] queue_empty    ip=%s  (logged once on transition to empty)
 *            [STREAM] queue_refilled ip=%s  (logged once on transition back)
 *          These transition-only logs allow correlation of queue starvation
 *          windows against ESP32 slow-send timestamps without flooding the
 *          log at QUEUE_WAIT_MS frequency.
 *
 *          Failure taxonomy is grep-deterministic:
 *            recv_closed    = ESP32 closed socket (expected on stream end or
 *                             send-side timeout — see truncation fix note)
 *            recv_error     = BBB-side transport error (unexpected)
 *            decode_failed  = frame rejected before queue push (bad length
 *                             or allocation failure)
 *            write_failed   = ffmpeg exited or pipe broken
 *            queue_empty    = consumer starved; no frames for QUEUE_WAIT_MS
 *            queue_refilled = consumer recovered after starvation
 *
 *          fflush fix (2026-06-25):
 *          Root cause of residual 100-400ms slow sends: log_msg() called
 *          fflush(g_log) unconditionally on every call. At 27.8fps with
 *          an rx_frame log line on every frame, this produced 27 synchronous
 *          eMMC flush operations per second. The BBB eMMC regularly takes
 *          80-184ms per write cycle (measured via /proc/diskstats at 100ms
 *          resolution). When a flush landed during a slow write window,
 *          conn_thread blocked in log_msg(), stalling the TCP recv loop,
 *          filling the ESP32 TCP send buffer, and causing the ESP32 to
 *          report a slow send with retries=0 (TCP backpressure, not RF).
 *
 *          Fix: fflush(g_log) removed from log_msg(). Explicit fflush()
 *          calls added only at lifecycle event sites (connect, disconnect,
 *          ffmpeg_start, ffmpeg_failed, write_failed, recv_error,
 *          queue_empty, queue_refilled). The per-frame rx_frame log line
 *          is disabled entirely — it was the primary driver of flush
 *          frequency and is not needed for production diagnosis. Re-enable
 *          temporarily for frame-level debugging if needed.
 *
 * \note    H.264 encode — NOT VIABLE on AM335x (2026-06-25):
 *          libx264 software encode was evaluated and rejected for this
 *          platform. The motivation was real: MJPEG frame size scales
 *          directly with scene complexity, and a person at the door
 *          increases frame size from ~2600 to 4000-5000 bytes exactly when
 *          the 2.4GHz RF link is already degraded by body absorption.
 *          H.264 at crf=28 would deliver ~800-1200 bytes/frame regardless
 *          of scene complexity — a compelling ~4x reduction at worst case.
 *
 *          However, the AM335x (single Cortex-A8 core) cannot sustain
 *          libx264 encode at 27.8fps even at -preset ultrafast. Measured
 *          result on the target hardware:
 *            speed=0.000473x   drop=2354 frames
 *          That is 0.047% of realtime — a fundamental throughput ceiling
 *          violation, not a tuning problem. The encode budget per frame at
 *          27.8fps is ~36ms. libx264 ultrafast on AM335x at 320x240
 *          exceeds this budget by roughly 3 orders of magnitude due to:
 *          - Single core with no parallelism available
 *          - No hardware media blocks (no VPU, no DSP path for H.264)
 *          - Memory bandwidth saturation on MJPEG decode + H.264 encode path
 *          - x264 GOP structure serializes encoding even with thread flags
 *
 *          Adding ffmpeg thread flags (-threads N) has no effect: the
 *          bottleneck is instruction throughput on a single in-order core,
 *          not thread-level parallelism. Extra threads add overhead without
 *          increasing throughput.
 *
 *          The pipeline failure mode when x264 is used:
 *            ffmpeg falls behind → drop count climbs → RTP output stalls
 *            → mediamtx i/o timeout after 60s → session destroyed.
 *          This is indistinguishable from queue starvation in mediamtx logs
 *          but has a completely different root cause.
 *
 *          Production config: -c:v copy (MJPEG passthrough). CPU cost is
 *          near zero — ffmpeg becomes a pure RTP packetizer. The RF
 *          degradation under human presence remains a known limitation of
 *          this hardware class. H.264 encode requires either a SoC with a
 *          hardware VPU (e.g. AM5729, i.MX8, RK3399) or offloading encode
 *          to the ESP32-S3 before transmission.
 *
 * \note    Queue starvation instrumentation (2026-06-25):
 *          Transition-only logging added to ffmpeg_thread to correlate
 *          queue starvation windows against ESP32 slow-send timestamps.
 *          queue_empty is logged once when the queue first goes empty
 *          (fq_pop timeout). queue_refilled is logged once when the next
 *          frame arrives after a starvation period. This answers whether
 *          mediamtx timeouts coincide with queue starvation (RF dropout)
 *          or occur while the queue is healthy (investigate ffmpeg pipeline).
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
#include <pthread.h>
#include "build_info.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

/******************************** CONFIG **************************************/
#define LISTEN_PORT        9093
#define BACKLOG            8
#define MAX_FRAME_BYTES    (1024u * 1024u)  /**< 1 MB per-frame sanity cap   */
#define LOG_PATH           "/var/log/doorbell_stream_daemon.log"
#define RTSP_BASE          "rtsp://localhost:8554/doorbell"
#define FFMPEG_LOG_PATH    "/var/log/ffmpeg_doorbell.log"

/**
 * \brief Queue depth — number of frame slots in the ring buffer.
 *
 * \details 4 slots absorbs ~100-200ms of WiFi jitter at typical VGA MJPEG
 *          frame rates (~5-10 fps over this link), without accumulating
 *          enough latency to make the doorbell feel delayed.
 */
#define FRAME_QUEUE_DEPTH  4

/**
 * \brief ffmpeg consumer wait timeout in milliseconds.
 *
 * \details If the queue is empty for this long, the consumer wakes up and
 *          loops. This prevents the ffmpeg writer thread from blocking
 *          indefinitely on an empty queue, which would cause mediamtx to
 *          time out the RTSP session during ESP32 jitter gaps.
 *          500ms is well under mediamtx's default inactivity window.
 */
#define QUEUE_WAIT_MS      500

/******************************** GLOBALS *************************************/
static volatile int  g_running   = 1;    /**< main loop run flag             */
static FILE         *g_log       = NULL; /**< log file handle                */
static int           g_server_fd = -1;  /**< TCP listen socket              */

/******************************** LOGGING *************************************/

/**
 * \brief Write a timestamped log line to file and stderr.
 *
 * \details fflush() is intentionally NOT called here. Flushing on every
 *          log call caused synchronous eMMC writes at 27fps, blocking
 *          conn_thread for 80-184ms per flush and creating TCP backpressure
 *          visible as ESP32 slow sends. Callers that need guaranteed flush
 *          (lifecycle events) call fflush(g_log) explicitly after log_msg().
 *
 * \param fmt  printf-style format string.
 */
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
      fprintf(g_log, "[%s] [stream] ", ts);
      va_start(ap, fmt);
      vfprintf(g_log, fmt, ap);
      va_end(ap);
      fprintf(g_log, "\n");
      /* NOTE: fflush deliberately omitted — see file header logging note */
   }

   fprintf(stderr, "[%s] [stream] ", ts);
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

/******************************** RECV HELPER *********************************/

/**
 * \brief Receive exactly n bytes from fd, blocking until all arrive.
 *
 * \param fd   Socket file descriptor.
 * \param buf  Destination buffer (must be at least n bytes).
 * \param n    Number of bytes to receive.
 * \return      0 on success,
 *             -1 on transport error (recv() < 0),
 *             -2 on clean TCP FIN mid-frame (recv() == 0, peer closed).
 *
 * \note    -2 vs -1 matters: -2 means the ESP32 closed the socket (likely
 *          due to a send-side error on its end), not a BBB-side recv fault.
 *          Callers log these as recv_closed vs recv_error respectively so
 *          the true cause is visible in the grep taxonomy.
 */
static int recv_all(int fd, void *buf, size_t n)
{
   size_t  got = 0;
   ssize_t r;

   while (got < n)
   {
      r = recv(fd, (char *)buf + got, n - got, 0);
      if (r == 0) { return -2; }   /* clean TCP FIN — peer closed mid-frame */
      if (r <  0) { return -1; }   /* transport error                       */
      got += (size_t)r;
   }
   return 0;
}

/******************************** FRAME QUEUE *********************************/

/**
 * \brief One slot in the frame ring buffer.
 */
typedef struct
{
   uint8_t *buf;    /**< heap-allocated JPEG data, NULL if slot is empty     */
   size_t   len;    /**< length of JPEG data in buf                          */
} frame_slot_t;

/**
 * \brief Bounded ring buffer decoupling the ESP32 receiver from ffmpeg.
 *
 * \details Producer (conn_thread) pushes frames. Consumer (ffmpeg_thread)
 *          pulls them. Drop-oldest policy when full: the producer overwrites
 *          the oldest unconsumed slot rather than blocking, so ESP32 jitter
 *          never propagates into ffmpeg's write path.
 *
 *          Invariant: head == tail means empty.
 *          (tail - head) % FRAME_QUEUE_DEPTH == number of queued frames.
 *
 *          Mutex must be held to read or write head/tail/slots.
 *          fwrite() to ffmpeg is done OUTSIDE the mutex.
 *
 * \note    done flag signals the consumer to exit after the producer closes
 *          the connection. The consumer drains remaining frames first.
 */
typedef struct
{
   frame_slot_t    slots[FRAME_QUEUE_DEPTH]; /**< ring buffer slots          */
   int             head;                     /**< consumer reads from here   */
   int             tail;                     /**< producer writes here       */
   int             count;                    /**< frames currently queued    */
   int             done;                     /**< producer has finished      */
   pthread_mutex_t mu;                       /**< protects all fields        */
   pthread_cond_t  cv;                       /**< signals consumer on push   */
} frame_queue_t;

/**
 * \brief Initialise a frame_queue_t. Must be called before use.
 */
static void fq_init(frame_queue_t *q)
{
   memset(q, 0, sizeof(*q));
   pthread_mutex_init(&q->mu, NULL);
   pthread_cond_init(&q->cv, NULL);
}

/**
 * \brief Destroy a frame_queue_t, freeing any queued frame buffers.
 */
static void fq_destroy(frame_queue_t *q)
{
   int i;
   for (i = 0; i < FRAME_QUEUE_DEPTH; i++)
   {
      free(q->slots[i].buf);
      q->slots[i].buf = NULL;
   }
   pthread_mutex_destroy(&q->mu);
   pthread_cond_destroy(&q->cv);
}

/**
 * \brief Push a frame into the queue (producer side).
 *
 * \details If the queue is full, the oldest slot (at head) is evicted and
 *          its buffer freed before the new frame takes its place. This is
 *          the drop-oldest policy: live view always gets the freshest frame.
 *
 *          Takes ownership of buf — caller must not free it after push.
 *          Signals the consumer condvar after every successful push.
 *
 * \param q    Queue to push into.
 * \param buf  Heap-allocated JPEG buffer (ownership transferred).
 * \param len  Length of buf in bytes.
 */
static void fq_push(frame_queue_t *q, uint8_t *buf, size_t len)
{
   pthread_mutex_lock(&q->mu);

   if (q->count == FRAME_QUEUE_DEPTH)
   {
      /* Queue full — evict oldest (head slot) */
      free(q->slots[q->head].buf);
      q->slots[q->head].buf = NULL;
      q->head = (q->head + 1) % FRAME_QUEUE_DEPTH;
      q->count--;
   }

   q->slots[q->tail].buf = buf;
   q->slots[q->tail].len = len;
   q->tail  = (q->tail + 1) % FRAME_QUEUE_DEPTH;
   q->count++;

   pthread_cond_signal(&q->cv);
   pthread_mutex_unlock(&q->mu);
}

/**
 * \brief Pop a frame from the queue (consumer side), waiting if empty.
 *
 * \details Waits on condvar with QUEUE_WAIT_MS timeout. Returns the frame
 *          buffer and length via out parameters. Caller owns the returned
 *          buffer and must free it after use.
 *
 *          Returns 0 if a frame was popped, -1 if the queue is empty after
 *          timeout (caller should loop), -2 if done and queue is empty
 *          (caller should exit).
 *
 *          Timeout deadline is recomputed inside the loop on every iteration.
 *          Computing it once before the loop causes spurious wakeups to pass
 *          an already-expired ts to pthread_cond_timedwait, which returns
 *          ETIMEDOUT immediately and spins the consumer at ~100% CPU during
 *          idle periods.
 *
 * \param q    Queue to pop from.
 * \param buf  Output: pointer to heap-allocated JPEG buffer (caller frees).
 * \param len  Output: length of buffer in bytes.
 * \return     0 on success, -1 on timeout (retry), -2 on done+empty (exit).
 */
static int fq_pop(frame_queue_t *q, uint8_t **buf, size_t *len)
{
   struct timespec ts;

   pthread_mutex_lock(&q->mu);

   while (q->count == 0 && !q->done)
   {
      /* Recompute deadline on every iteration so spurious wakeups do not
       * cause an already-expired ts to spin pthread_cond_timedwait. */
      clock_gettime(CLOCK_REALTIME, &ts);
      ts.tv_sec  += QUEUE_WAIT_MS / 1000;
      ts.tv_nsec += (QUEUE_WAIT_MS % 1000) * 1000000L;
      if (ts.tv_nsec >= 1000000000L)
      {
         ts.tv_sec++;
         ts.tv_nsec -= 1000000000L;
      }

      int rc = pthread_cond_timedwait(&q->cv, &q->mu, &ts);
      if (rc == ETIMEDOUT)
      {
         int done = q->done;
         pthread_mutex_unlock(&q->mu);
         return done ? -2 : -1;
      }
   }

   if (q->count == 0)
   {
      /* done == 1 and queue empty — consumer should exit */
      pthread_mutex_unlock(&q->mu);
      return -2;
   }

   *buf = q->slots[q->head].buf;
   *len = q->slots[q->head].len;
   q->slots[q->head].buf = NULL;
   q->slots[q->head].len = 0;
   q->head  = (q->head + 1) % FRAME_QUEUE_DEPTH;
   q->count--;

   pthread_mutex_unlock(&q->mu);
   return 0;
}

/**
 * \brief Signal the consumer that the producer is done.
 *
 * \details Called by conn_thread after the ESP32 connection closes.
 *          Consumer will drain remaining frames then exit.
 */
static void fq_done(frame_queue_t *q)
{
   pthread_mutex_lock(&q->mu);
   q->done = 1;
   pthread_cond_signal(&q->cv);
   pthread_mutex_unlock(&q->mu);
}

/******************************** CONN STRUCT *********************************/

/**
 * \brief Arguments passed to each per-connection thread pair.
 *
 * \details Shared between conn_thread (producer) and ffmpeg_thread
 *          (consumer). conn_thread fills the queue; ffmpeg_thread drains it.
 *          Both threads are spawned per ESP32 connection.
 */
typedef struct
{
   int                client_fd;              /**< accepted socket fd         */
   struct sockaddr_in peer;                   /**< remote address             */
   frame_queue_t      queue;                  /**< producer/consumer queue    */
   FILE              *ffmpeg;                 /**< ffmpeg stdin pipe          */
   char               peer_ip[INET_ADDRSTRLEN]; /**< dotted-decimal peer IP  */
   uint8_t            device_id;             /**< doorbell device ID (0-3)   */
} conn_state_t;

/******************************** FFMPEG THREAD *******************************/

/**
 * \brief Consumer thread: drain queue and write frames to ffmpeg stdin.
 *
 * \details Pops frames from the queue with a bounded wait (QUEUE_WAIT_MS).
 *          On timeout with empty queue, logs a queue_empty transition (once)
 *          and loops — keeping the thread alive during ESP32 jitter gaps.
 *          When the next frame arrives after a starvation period, logs
 *          queue_refilled (once). These transition logs allow correlation
 *          of starvation windows against ESP32 slow-send timestamps without
 *          flooding the log at QUEUE_WAIT_MS frequency.
 *          Exits when fq_pop() returns -2 (producer done, queue empty).
 *
 *          fwrite() to ffmpeg is done outside the queue mutex so the producer
 *          is never blocked by a slow ffmpeg write.
 *
 * \param arg  conn_state_t pointer (not freed here — owned by conn_thread).
 * \return     NULL always.
 */
static void *ffmpeg_thread(void *arg)
{
   conn_state_t *cs               = (conn_state_t *)arg;
   uint8_t      *buf              = NULL;
   size_t        len              = 0;
   int           queue_empty_logged = 0;  /**< starvation transition flag    */

   while (1)
   {
      int rc = fq_pop(&cs->queue, &buf, &len);

      if (-1 == rc)
      {
         /* Timeout — queue empty, ESP32 jitter gap.
          * Log the transition to empty once so the starvation window start
          * is visible in the log without flooding at QUEUE_WAIT_MS rate. */
         if (!queue_empty_logged)
         {
            log_msg("[STREAM] queue_empty ip=%s", cs->peer_ip);
            fflush(g_log);
            queue_empty_logged = 1;
         }
         continue;
      }

      if (-2 == rc)
      {
         /* Producer done and queue drained — exit. */
         break;
      }

      /* Frame received — log recovery from starvation if applicable. */
      if (queue_empty_logged)
      {
         log_msg("[STREAM] queue_refilled ip=%s", cs->peer_ip);
         fflush(g_log);
         queue_empty_logged = 0;
      }

      /* Write frame to ffmpeg stdin — outside mutex.
       * fflush(cs->ffmpeg) intentionally omitted: forcing a pipe flush
       * syscall per frame at 27fps prevents ffmpeg's internal buffering
       * and adds unnecessary context switching jitter. ffmpeg manages its
       * own stdin read buffering; the pipe kernel buffer handles delivery. */
      if (fwrite(buf, 1, len, cs->ffmpeg) != len)
      {
         log_msg("[STREAM] write_failed ip=%s", cs->peer_ip);
         fflush(g_log);
         free(buf);
         break;
      }
      free(buf);
   }

   return NULL;
}

/******************************** CONN THREAD *********************************/

/**
 * \brief Per-connection producer thread: read device_id, spawn ffmpeg and
 *        ffmpeg_thread, receive frames from ESP32 and push to queue.
 *
 * \details Spawns ffmpeg via popen() and a ffmpeg_thread to consume the
 *          queue. Then loops receiving [len:4][jpeg] frames from the ESP32
 *          and pushing them into the queue. On exit (ESP32 disconnect or
 *          error), signals fq_done() and joins ffmpeg_thread before cleanup.
 *
 *          ffmpeg command uses MJPEG passthrough (-c:v copy). H.264 software
 *          encode via libx264 was evaluated and rejected — see file header
 *          H.264 encode note for full rationale. Short version: libx264
 *          runs at speed=0.000473x on AM335x (0.047% realtime), dropping
 *          thousands of frames and triggering mediamtx i/o timeout within
 *          60 seconds. Not a tuning problem; a hardware ceiling.
 *
 *          Logging contract (flat [STREAM] tags — no cross-process boundary
 *          in this daemon so no directional arrow tags are used):
 *            [STREAM] connect       — device_id and IP known, stream starting
 *            [STREAM] ffmpeg_start  — ffmpeg process spawned successfully
 *            [STREAM] ffmpeg_failed — popen() or pthread_create() failed
 *            [STREAM] decode_failed — frame rejected (bad_len, malloc_failed)
 *            [STREAM] recv_closed   — clean TCP FIN from ESP32 mid-frame
 *            [STREAM] recv_error    — BBB-side transport error
 *            [STREAM] write_failed  — fwrite to ffmpeg stdin failed (in
 *                                     ffmpeg_thread, not here)
 *            [STREAM] queue_empty   — consumer starved (transition log)
 *            [STREAM] queue_refilled — consumer recovered (transition log)
 *            [STREAM] disconnect    — connection fully torn down
 *
 *          Per-frame rx_frame logging is intentionally disabled. It was the
 *          primary driver of synchronous eMMC flushes that caused TCP
 *          backpressure. Re-enable temporarily for frame-level debugging.
 *
 * \param arg  Heap-allocated conn_state_t (this function frees it).
 * \return     NULL always.
 */
static void *conn_thread(void *arg)
{
   conn_state_t *cs        = (conn_state_t *)arg;
   int           fd        = cs->client_fd;
   uint8_t      *frame_buf = NULL;
   size_t        frame_cap = 0;
   char          cmd[512];
   pthread_t     ffmpeg_tid;
   int           ffmpeg_tid_valid = 0;

   /* TCP keepalive */
   {
      int keepalive = 1, keepidle = 30, keepintvl = 5, keepcnt = 3;
      setsockopt(fd, SOL_SOCKET,  SO_KEEPALIVE, &keepalive, sizeof(keepalive));
      setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE,  &keepidle,  sizeof(keepidle));
      setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
      setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT,   &keepcnt,   sizeof(keepcnt));
   }

   /* --- 1. Read 1-byte device_id ------------------------------------ */
   if (recv_all(fd, &cs->device_id, 1) < 0)
   {
      log_msg("[STREAM] recv_error ip=%s errno=%d reason=device_id_recv_failed",
              cs->peer_ip, errno);
      fflush(g_log);
      goto cleanup;
   }

   log_msg("[STREAM] connect device_id=%u ip=%s",
           (unsigned)cs->device_id, cs->peer_ip);
   fflush(g_log);

   /* --- 2. Spawn ffmpeg --------------------------------------------- */
   /* MJPEG passthrough (-c:v copy): validated production config for AM335x.
    *
    * H.264 software encode (libx264) was evaluated and rejected.
    * See file header "H.264 encode — NOT VIABLE on AM335x" note.
    * Do not attempt to re-enable x264 on this hardware without a measured
    * encode budget. The failure mode (mediamtx i/o timeout after 60s) is
    * not obviously attributable to encode overhead in the mediamtx log.
    *
    * Low-latency flags:
    * -fflags nobuffer   — pass frames to muxer immediately
    * -flags low_delay   — disable muxer look-ahead buffering
    * -flush_packets 1   — flush RTP packet buffer after every frame */
   snprintf(cmd, sizeof(cmd),
         "ffmpeg -loglevel info"
         " -fflags nobuffer"
         " -flags low_delay"
         " -analyzeduration 0 -probesize 32"
         " -f mjpeg"
         " -i pipe:0"
         " -c:v copy"
         " -flush_packets 1"
         " -rtsp_transport tcp"
         " -f rtsp %s%u"
         " 2>>%s",
         RTSP_BASE, (unsigned)cs->device_id, FFMPEG_LOG_PATH);

   cs->ffmpeg = popen(cmd, "w");
   if (NULL == cs->ffmpeg)
   {
      log_msg("[STREAM] ffmpeg_failed device_id=%u reason=popen errno=%d",
              (unsigned)cs->device_id, errno);
      fflush(g_log);
      goto cleanup;
   }

   log_msg("[STREAM] ffmpeg_start device_id=%u url=%s%u",
           (unsigned)cs->device_id, RTSP_BASE, (unsigned)cs->device_id);
   fflush(g_log);

   /* --- 3. Spawn ffmpeg consumer thread ----------------------------- */
   if (pthread_create(&ffmpeg_tid, NULL, ffmpeg_thread, cs) != 0)
   {
      log_msg("[STREAM] ffmpeg_failed device_id=%u reason=pthread_create errno=%d",
              (unsigned)cs->device_id, errno);
      fflush(g_log);
      goto cleanup;
   }
   ffmpeg_tid_valid = 1;

   /* --- 4. Frame receive loop (producer) ---------------------------- */
   while (g_running)
   {
      uint8_t  hdr[4];
      uint32_t frame_len;
      uint8_t *slot_buf;

      /* 4-byte big-endian frame length */
      {
         int rc = recv_all(fd, hdr, 4);
         if (-2 == rc)
         {
            /* Clean FIN — ESP32 ended the stream normally */
            break;
         }
         if (rc < 0)
         {
            log_msg("[STREAM] recv_error ip=%s errno=%d",
                    cs->peer_ip, errno);
            fflush(g_log);
            break;
         }
      }

      frame_len = ((uint32_t)hdr[0] << 24) |
                  ((uint32_t)hdr[1] << 16) |
                  ((uint32_t)hdr[2] <<  8) |
                   (uint32_t)hdr[3];

      if (0u == frame_len || frame_len > MAX_FRAME_BYTES)
      {
         log_msg("[STREAM] decode_failed ip=%s reason=bad_len len=%u",
                 cs->peer_ip, frame_len);
         fflush(g_log);
         break;
      }

      /* Receive JPEG payload into temp buffer */
      if (frame_len > frame_cap)
      {
         free(frame_buf);
         frame_buf = malloc(frame_len);
         if (NULL == frame_buf)
         {
            log_msg("[STREAM] decode_failed ip=%s reason=malloc_failed bytes=%u",
                    cs->peer_ip, frame_len);
            fflush(g_log);
            break;
         }
         frame_cap = frame_len;
      }

      {
         int rc = recv_all(fd, frame_buf, frame_len);
         if (-2 == rc)
         {
            log_msg("[STREAM] recv_closed ip=%s",
                    cs->peer_ip);
            fflush(g_log);
            break;
         }
         if (rc < 0)
         {
            log_msg("[STREAM] recv_error ip=%s errno=%d",
                    cs->peer_ip, errno);
            fflush(g_log);
            break;
         }
      }

      /* Copy frame into a fresh heap buffer for the queue.
       * The queue takes ownership; fq_push() will free it on eviction,
       * and ffmpeg_thread frees it after fwrite(). */
      slot_buf = malloc(frame_len);
      if (NULL == slot_buf)
      {
         log_msg("[STREAM] decode_failed ip=%s reason=malloc_failed bytes=%u",
                 cs->peer_ip, frame_len);
         fflush(g_log);
         break;
      }
      memcpy(slot_buf, frame_buf, frame_len);

      /* rx_frame per-frame logging intentionally disabled.
       * Re-enable for frame-level debugging only:
       * log_msg("[STREAM] rx_frame device_id=%u bytes=%u",
       *         (unsigned)cs->device_id, frame_len);
       * Do NOT add fflush() here — see file header logging note. */

      fq_push(&cs->queue, slot_buf, frame_len);
   }

   /* --- 5. Cleanup -------------------------------------------------- */
cleanup:
   /* Signal consumer that production is done, then join it */
   fq_done(&cs->queue);
   if (ffmpeg_tid_valid) { pthread_join(ffmpeg_tid, NULL); }

   free(frame_buf);
   if (cs->ffmpeg) { pclose(cs->ffmpeg); }
   close(fd);

   log_msg("[STREAM] disconnect device_id=%u ip=%s",
           (unsigned)cs->device_id, cs->peer_ip);
   fflush(g_log);

   fq_destroy(&cs->queue);
   free(cs);
   return NULL;
}

/******************************** TCP SERVER **********************************/

/**
 * \brief Initialise the TCP listen socket and bind to LISTEN_PORT.
 */
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
   listen(g_server_fd, BACKLOG);
   log_msg("TCP server listening on port %d", LISTEN_PORT);
   fflush(g_log);
}

/******************************** MAIN ****************************************/
int main(void)
{
   signal(SIGINT,  sig_handler);
   signal(SIGTERM, sig_handler);
   signal(SIGPIPE, SIG_IGN);   /**< fwrite to dead ffmpeg returns error, not kill */

   g_log = fopen(LOG_PATH, "a");
   log_msg("doorbell_stream_daemon starting on port %d", LISTEN_PORT);
   fflush(g_log);
   fprintf(stdout, "[BOOT] %s\n", bbb_build_date);
   fprintf(stdout, "[BOOT] %s\n", bbb_build_target);

   tcp_server_init();

   while (g_running)
   {
      conn_state_t *cs;
      socklen_t     peer_len;
      pthread_t     tid;

      cs = malloc(sizeof(conn_state_t));
      if (NULL == cs)
      {
         log_msg("ERROR: malloc conn_state_t");
         fflush(g_log);
         continue;
      }
      memset(cs, 0, sizeof(*cs));
      fq_init(&cs->queue);
      cs->ffmpeg = NULL;

      peer_len      = sizeof(cs->peer);
      cs->client_fd = accept(g_server_fd,
                             (struct sockaddr *)&cs->peer,
                             &peer_len);
      if (cs->client_fd < 0)
      {
         fq_destroy(&cs->queue);
         free(cs);
         if (g_running)
         {
            log_msg("accept() error: %s", strerror(errno));
            fflush(g_log);
         }
         continue;
      }

      inet_ntop(AF_INET, &cs->peer.sin_addr,
                cs->peer_ip, sizeof(cs->peer_ip));

      if (pthread_create(&tid, NULL, conn_thread, cs) != 0)
      {
         log_msg("pthread_create failed: %s", strerror(errno));
         fflush(g_log);
         close(cs->client_fd);
         fq_destroy(&cs->queue);
         free(cs);
         continue;
      }
      pthread_detach(tid);
   }

   log_msg("doorbell_stream_daemon shutting down");
   fflush(g_log);
   if (g_server_fd >= 0) { close(g_server_fd); }
   if (NULL != g_log)    { fclose(g_log); }
   return 0;
}
