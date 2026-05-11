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
 *          buffer on overflow and logs a warning.
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
      return;
   }

   tmp[n] = '\0';

   if ((g_active_pos + n) >= BUFFER_SIZE)
   {
      log_msg("Active buffer overflow, reset");
      g_active_pos = 0;
      g_active[0]  = '\0';
      return;
   }

   (void)memcpy(g_active + g_active_pos, tmp, n);
   g_active_pos           += n;
   g_active[g_active_pos]  = '\0';
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
 *          the frame into p_out (stable copy), then compacts g_active[]
 *          with memmove to preserve any bytes belonging to the next frame.
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
      log_msg("Frame too large (%d bytes), discarding", frame_len);
      g_active_pos = 0;
      g_active[0]  = '\0';
      return false;
   }

   /* Copy frame into caller's stable buffer -- must happen before
    * memmove compacts g_active[], as p_start points into g_active[]. */
   (void)memcpy(p_out, p_start, frame_len);
   p_out[frame_len] = '\0';

   /* Compact active buffer -- preserve bytes after the frame */
   if (tail_len > 0)
   {
      (void)memmove(g_active, g_active + tail_start, tail_len);
   }

   g_active_pos       = tail_len;
   g_active[tail_len] = '\0';

   return true;
}
