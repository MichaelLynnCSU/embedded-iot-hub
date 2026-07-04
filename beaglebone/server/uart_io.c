/******************************************************************************
 * \file uart_io.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-05-10
 *
 * \brief UART transport layer for sensor server.
 *
 * \details Manages UART open/close and the double buffer accumulation
 *          loop. Knows nothing about JSON parsing or the controller pipe.
 *
 *          Split from sensor_server.c (2026-05-10).
 *          See sensor_server.c file header for full double buffer rationale.
 *
 *          Owns:
 *          - UART file descriptor and init
 *          - g_active[] accumulation buffer (producer side)
 *          - find_json_frame() frame boundary detection
 *          - uart_io_next_frame() copy-and-compact
 *
 *          Does not own:
 *          - JSON parsing (json_parser.c)
 *          - pipe IPC (pipe_writer.c)
 *          - global run flag or signal handling (sensor_server.c)
 *
 *          Log taxonomy (2026-06-15):
 *          All log lines use the [UART] domain prefix with bytes=<n>
 *          as the correlation key. Timestamp and frame_seq are never
 *          read or mutated here; those are [PARSE]'s responsibility.
 *
 *          Events emitted:
 *            [UART] rx_frame        bytes=<n>  -- complete frame passed upstream
 *            [UART] overflow        bytes=<n>  -- active buffer reset after overrun
 *            [UART] frame_too_large bytes=<n>  -- frame exceeds out buf, discarded
 *            [UART] disconnect      bytes=<n>  -- read returned 0 or error
 *
 *          Pattern stack (four levels, each at a different abstraction):
 *
 *          Double buffer   g_active[] accumulates (producer side);
 *                          sensor_server.c owns the ready buffer
 *                          (consumer side). Producer and consumer
 *                          never touch the same buffer simultaneously.
 *
 *          Sliding window  g_active[] grows as bytes arrive; memmove()
 *                          compacts it forward after each frame is
 *                          extracted, preserving bytes that belong to
 *                          the next frame.
 *
 *          Two pointers    find_json_frame() uses p_start and p_end
 *                          to locate frame boundaries without
 *                          modifying the buffer.
 *
 *          Depth counter   find_json_frame() tracks brace nesting
 *                          depth to handle nested JSON objects
 *                          correctly ({} inside {}).
 *
 * \note    Architecture clarification — hub transport (2026-07-03):
 *          It is easy to misread this file as talking directly to the
 *          ESP32 hub. It does not. The actual chain is:
 *
 *            ESP32 hub --(WiFi/TCP client, tcp_manager.c)-->
 *            ESP-01 module --(AT firmware, acts as TCP server + UART bridge)-->
 *            /dev/ttyS4 (this file) --> sensor_server --> data_controller
 *
 *          The ESP-01 is configured once at boot by a separate one-shot
 *          script (beaglebone/wifi/esp01_tcp_server_setup.py, run by
 *          esp01-tcp-server.service, Type=oneshot, exits after setup —
 *          it is NOT a long-running process and has no ongoing role).
 *          After that script exits, the ESP-01's own AT firmware does
 *          all the TCP<->UART bridging in hardware; nothing in this
 *          repo owns that bridge at runtime.
 *
 *          Consequences that have caused real mistakes before:
 *          - The hub's own reboot boot banner (e.g. "rst:") is printed
 *            on the hub's USB debug console only. It does NOT cross
 *            the ESP-01 bridge and will NEVER appear in g_active[]
 *            here, no matter how this file is instrumented.
 *          - sensor_server (this file included) has no socket code at
 *            all and never will unless this architecture changes —
 *            don't go looking for a TCP accept loop on the BBB side to
 *            hook a hub-connect event; it doesn't exist.
 *          - The ESP-01's own AT firmware may emit its own link-state
 *            text (e.g. "CONNECT"/"CLOSED") into this same byte stream
 *            when the hub's TCP link to the ESP-01 flaps. This has not
 *            been confirmed live as of this note. If pursuing this
 *            signal, verify it first — do not assume the string or
 *            its exact behavior.
 *          - Hub-reboot detection was ultimately solved upstream of
 *            this file entirely, via a "boot_marker" field in the JSON
 *            telemetry payload itself (see json_parser.c and
 *            tcp_manager.c). That is the current source of truth for
 *            "did the hub reboot?" — not anything in this file.
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <stdbool.h>
#include "uart_io.h"
#include "server_log.h"

#define UART_PORT      "/dev/ttyS4" /**< ESP32 hub UART device */
#define BAUDRATE       B115200      /**< UART baud rate */
#define BUFFER_SIZE    4096         /**< active accumulation buffer size */
#define READ_BUF_SIZE  256          /**< per-read chunk size */

static int  g_uart_fd    = -1;           /**< UART file descriptor */
static char g_active[BUFFER_SIZE];       /**< UART accumulation buffer */
static int  g_active_pos = 0;            /**< write cursor into g_active[] */

/******************************************************************************
 * \brief Find the first complete JSON object boundaries in a buffer.
 *
 * \param p_buf       - Null-terminated buffer to search.
 * \param p_start_out - Output: pointer to opening '{' within p_buf.
 * \param p_end_out   - Output: pointer to closing '}' within p_buf.
 *
 * \return bool - true if a complete JSON object was found.
 *
 * \details Returns pointers into p_buf without modifying it. Caller is
 *          responsible for copying the frame before compacting the buffer.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static bool find_json_frame(char *p_buf,
                             char **p_start_out,
                             char **p_end_out)
{
   char *p_start = NULL; /**< pointer to opening brace */
   char *p       = NULL; /**< scan pointer */
   int   depth   = 0;    /**< brace nesting depth */

   p_start = strchr(p_buf, '{');
   if (NULL == p_start)
   {
      return false;
   }

   for (p = p_start; '\0' != *p; p++)
   {
      if ('{' == *p)
      {
         depth++;
      }
      else if ('}' == *p)
      {
         depth--;
         if (0 == depth)
         {
            *p_start_out = p_start;
            *p_end_out   = p;
            return true;
         }
      }
      else
      {
         /* non-brace character -- continue */
      }
   }

   return false;
}

/******************************************************************************
 * \brief Initialize UART at 115200 8N1.
 *
 * \return int - 0 on success, -1 on failure.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
int uart_io_init(void)
{
   struct termios tty; /**< terminal settings */

   g_uart_fd = open(UART_PORT, O_RDWR | O_NOCTTY);
   if (0 > g_uart_fd)
   {
      log_msg("ERROR opening UART: %s", strerror(errno));
      return -1;
   }

   (void)tcgetattr(g_uart_fd, &tty);
   (void)cfsetospeed(&tty, BAUDRATE);
   (void)cfsetispeed(&tty, BAUDRATE);

   tty.c_cflag &= ~PARENB;
   tty.c_cflag &= ~CSTOPB;
   tty.c_cflag &= ~CSIZE;
   tty.c_cflag |=  CS8;
   tty.c_cflag &= ~CRTSCTS;
   tty.c_cflag |=  CREAD | CLOCAL;
   tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
   tty.c_iflag &= ~(IXON | IXOFF | IXANY);
   tty.c_oflag &= ~OPOST;
   tty.c_cc[VTIME] = 10;
   tty.c_cc[VMIN]  = 0;

   (void)tcsetattr(g_uart_fd, TCSANOW, &tty);
   (void)tcflush(g_uart_fd, TCIOFLUSH);

   log_msg("UART initialized on %s @115200", UART_PORT);
   return 0;
}

/******************************************************************************
 * \brief Close UART file descriptor.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void uart_io_close(void)
{
   if (0 <= g_uart_fd)
   {
      close(g_uart_fd);
      g_uart_fd = -1;
   }
}

/******************************************************************************
 * \brief Read available bytes from UART into the active accumulation buffer.
 *
 * \return void
 *
 * \details Appends up to READ_BUF_SIZE bytes per call. Resets active
 *          buffer on overflow and emits [UART] overflow bytes=<n>.
 *          Emits [UART] disconnect bytes=<n> when read returns <= 0
 *          and bytes were buffered at the time (fd went silent mid-frame).
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void uart_io_read(void)
{
   char tmp[READ_BUF_SIZE] = {0}; /**< per-read chunk */
   int  n                  = 0;   /**< bytes read */

   if (0 > g_uart_fd)
   {
      return;
   }

   n = (int)read(g_uart_fd, tmp, sizeof(tmp) - 1);
   if (n <= 0)
   {
      if (g_active_pos > 0)
      {
         /* Bytes were buffered when the fd went silent -- log how many
          * were in flight so the disconnect is traceable. */
         log_msg("[UART] disconnect bytes=%d", g_active_pos);
      }
      return;
   }

   tmp[n] = '\0';

   if ((g_active_pos + n) >= BUFFER_SIZE)
   {
      log_msg("[UART] overflow bytes=%d", g_active_pos + n);
      g_active_pos = 0;
      g_active[0]  = '\0';  /* BUFFER_RESET: overflow discard -- do not
                             * confuse with BUFFER_APPEND below; these
                             * are two different operations that happen
                             * to look similar at a glance. */
      return;
   }

   (void)memcpy(g_active + g_active_pos, tmp, n);
   g_active_pos           += n;
   g_active[g_active_pos]  = '\0';  /* BUFFER_APPEND: normal accumulation --
                                     * this is the line to anchor on when
                                     * adding logic that should run after
                                     * every successful read. Do not match
                                     * on "g_active[...] = '\\0';" alone in
                                     * any future sed/grep here -- see
                                     * BUFFER_RESET above, which matches
                                     * the same shape but does something
                                     * different. */

   /* END_uart_io_read -- any new logic that should fire on every
    * successful accumulation must go ABOVE this comment and INSIDE
    * the function's closing brace above. A previous edit attempt
    * landed code after this brace by mistake (dead, non-compiling
    * code sitting between functions) -- this marker exists so that
    * doesn't happen again. */
}

/******************************************************************************
 * \brief Extract the next complete JSON frame from the active buffer.
 *
 * \param p_out  - Output buffer to copy frame into.
 * \param out_sz - Size of output buffer in bytes.
 *
 * \return bool - true if a complete frame was found and copied into p_out.
 *
 * \details Finds frame boundaries without modifying g_active[]. Copies
 *          the frame into p_out (stable copy), emits [UART] rx_frame
 *          bytes=<n>, then compacts g_active[] with memmove to preserve
 *          any bytes belonging to the next frame.
 *          Call in a loop until false to drain back-to-back frames.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
bool uart_io_next_frame(char *p_out, int out_sz)
{
   char *p_start    = NULL; /**< pointer to opening brace in g_active */
   char *p_end      = NULL; /**< pointer to closing brace in g_active */
   int   frame_len  = 0;    /**< length of frame including braces */
   int   tail_start = 0;    /**< offset of bytes after frame */
   int   tail_len   = 0;    /**< number of bytes after frame */

   if (!find_json_frame(g_active, &p_start, &p_end))
   {
      return false;
   }

   frame_len  = (int)(p_end - p_start) + 1;
   tail_start = (int)(p_end - g_active) + 1;
   tail_len   = g_active_pos - tail_start;

   if (frame_len >= out_sz)
   {
      log_msg("[UART] frame_too_large bytes=%d", frame_len);
      g_active_pos = 0;
      g_active[0]  = '\0';
      return false;
   }

   /* Copy frame into caller's stable buffer -- must happen before
    * memmove compacts g_active[], as p_start points into g_active[]. */
   (void)memcpy(p_out, p_start, frame_len);
   p_out[frame_len] = '\0';

   log_msg("[UART] rx_frame bytes=%d", frame_len);

   /* Compact active buffer -- preserve bytes after the frame */
   if (tail_len > 0)
   {
      (void)memmove(g_active, g_active + tail_start, tail_len);
   }

   g_active_pos       = tail_len;
   g_active[tail_len] = '\0';

   return true;
}
