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
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

/******************************** CONFIG **************************************/
#define LISTEN_PORT      9093
#define BACKLOG          8
#define MAX_FRAME_BYTES  (1024u * 1024u)   /**< 1 MB per-frame sanity cap  */
#define LOG_PATH         "/var/log/doorbell_stream_daemon.log"
#define RTSP_BASE        "rtsp://localhost:8554/doorbell"

/******************************** GLOBALS *************************************/
static volatile int  g_running   = 1;    /**< main loop run flag           */
static FILE         *g_log       = NULL; /**< log file handle              */
static int           g_server_fd = -1;  /**< TCP listen socket            */

/******************************** LOGGING *************************************/

/**
 * \brief Write a timestamped log line to file and stderr.
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
      fflush(g_log);
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
 * \return     0 on success, -1 on connection close or error.
 */
static int recv_all(int fd, void *buf, size_t n)
{
   size_t  got = 0;
   ssize_t r;

   while (got < n)
   {
      r = recv(fd, (char *)buf + got, n - got, 0);
      if (r <= 0) { return -1; }
      got += (size_t)r;
   }
   return 0;
}

/******************************** CONN STRUCT *********************************/

/**
 * \brief Arguments passed to each per-connection thread.
 */
typedef struct
{
   int                client_fd;              /**< accepted socket fd        */
   struct sockaddr_in peer;                   /**< remote address            */
} conn_args_t;

/******************************** CONN THREAD *********************************/

/**
 * \brief Per-connection thread: read device_id, spawn ffmpeg, relay frames.
 *
 * \details Reads the 1-byte device_id, opens an ffmpeg process targeting
 *          rtsp://localhost:8554/doorbell<N>, then loops receiving
 *          [len:4][jpeg] frames and writing them to ffmpeg stdin until
 *          the ESP32 closes the connection or an error occurs.
 *
 * \param arg  Heap-allocated conn_args_t (this function frees it).
 * \return     NULL always.
 */
static void *conn_thread(void *arg)
{
   conn_args_t *ca        = (conn_args_t *)arg;
   int          fd        = ca->client_fd;
   uint8_t     *frame_buf = NULL;
   size_t       frame_cap = 0;
   uint8_t      device_id = 0;
   char         peer_ip[INET_ADDRSTRLEN];
   char         cmd[256];
   FILE        *ffmpeg    = NULL;

   inet_ntop(AF_INET, &ca->peer.sin_addr, peer_ip, sizeof(peer_ip));
   free(ca);
   ca = NULL;

   /* TCP keepalive — match doorbell_daemon settings */
   {
      int keepalive = 1, keepidle = 30, keepintvl = 5, keepcnt = 3;
      setsockopt(fd, SOL_SOCKET,  SO_KEEPALIVE, &keepalive, sizeof(keepalive));
      setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE,  &keepidle,  sizeof(keepidle));
      setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &keepintvl, sizeof(keepintvl));
      setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT,   &keepcnt,   sizeof(keepcnt));
   }

   log_msg("Connection from %s", peer_ip);

   /* --- 1. Read 1-byte device_id ------------------------------------ */
   if (recv_all(fd, &device_id, 1) < 0)
   {
      log_msg("device_id recv failed from %s: %s", peer_ip, strerror(errno));
      close(fd);
      return NULL;
   }
   log_msg("%s: device_id=%u", peer_ip, (unsigned)device_id);

   /* --- 2. Spawn ffmpeg --------------------------------------------- */
   snprintf(cmd, sizeof(cmd),
            "ffmpeg -loglevel warning"
            " -f mjpeg -i pipe:0"
            " -c:v copy"
            " -f rtsp %s%u",
            RTSP_BASE, (unsigned)device_id);

   ffmpeg = popen(cmd, "w");
   if (NULL == ffmpeg)
   {
      log_msg("popen ffmpeg failed for device_id=%u: %s",
              (unsigned)device_id, strerror(errno));
      close(fd);
      return NULL;
   }
   log_msg("ffmpeg started → %s%u", RTSP_BASE, (unsigned)device_id);

   /* --- 3. Frame relay loop ----------------------------------------- */
   while (g_running)
   {
      uint8_t  hdr[4];
      uint32_t frame_len;

      /* 4-byte big-endian frame length */
      if (recv_all(fd, hdr, 4) < 0) { break; }

      frame_len = ((uint32_t)hdr[0] << 24) |
                  ((uint32_t)hdr[1] << 16) |
                  ((uint32_t)hdr[2] <<  8) |
                   (uint32_t)hdr[3];

      if (0u == frame_len || frame_len > MAX_FRAME_BYTES)
      {
         log_msg("%s: bad frame_len=%u — dropping connection",
                 peer_ip, frame_len);
         break;
      }

      /* Grow frame buffer if needed */
      if (frame_len > frame_cap)
      {
         free(frame_buf);
         frame_buf = malloc(frame_len);
         if (NULL == frame_buf)
         {
            log_msg("%s: malloc %u bytes failed", peer_ip, frame_len);
            break;
         }
         frame_cap = frame_len;
      }

      /* Receive JPEG payload */
      if (recv_all(fd, frame_buf, frame_len) < 0)
      {
         log_msg("%s: JPEG recv truncated", peer_ip);
         break;
      }

      /* Write to ffmpeg stdin */
      if (fwrite(frame_buf, 1, frame_len, ffmpeg) != frame_len)
      {
         log_msg("%s: fwrite to ffmpeg failed — ffmpeg may have exited",
                 peer_ip);
         break;
      }
      fflush(ffmpeg);
   }

   /* --- 4. Cleanup -------------------------------------------------- */
   free(frame_buf);
   pclose(ffmpeg);
   close(fd);
   log_msg("%s: device_id=%u stream ended", peer_ip, (unsigned)device_id);
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
}

/******************************** MAIN ****************************************/
int main(void)
{
   signal(SIGINT,  sig_handler);
   signal(SIGTERM, sig_handler);
   signal(SIGPIPE, SIG_IGN);   /**< fwrite to dead ffmpeg returns error, not kill */

   g_log = fopen(LOG_PATH, "a");
   log_msg("doorbell_stream_daemon starting on port %d", LISTEN_PORT);

   tcp_server_init();

   while (g_running)
   {
      conn_args_t *ca;
      socklen_t    peer_len;
      pthread_t    tid;

      ca = malloc(sizeof(conn_args_t));
      if (NULL == ca)
      {
         log_msg("ERROR: malloc conn_args_t");
         continue;
      }

      peer_len       = sizeof(ca->peer);
      ca->client_fd  = accept(g_server_fd,
                              (struct sockaddr *)&ca->peer,
                              &peer_len);
      if (ca->client_fd < 0)
      {
         free(ca);
         if (g_running) { log_msg("accept() error: %s", strerror(errno)); }
         continue;
      }

      if (pthread_create(&tid, NULL, conn_thread, ca) != 0)
      {
         log_msg("pthread_create failed: %s", strerror(errno));
         close(ca->client_fd);
         free(ca);
         continue;
      }
      pthread_detach(tid);
   }

   log_msg("doorbell_stream_daemon shutting down");
   if (g_server_fd >= 0) { close(g_server_fd); }
   if (NULL != g_log)    { fclose(g_log); }
   return 0;
}
