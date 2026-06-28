/******************************************************************************
 * \file pipe_writer.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-05-10
 *
 * \brief Controller pipe IPC for sensor server.
 *
 * \details Owns named pipe open, reconnect, and SensorData write.
 *          Knows nothing about UART or JSON parsing.
 *
 *          Split from sensor_server.c (2026-05-10).
 *
 *          Log taxonomy (2026-06-15):
 *          All log lines use the [PIPE] domain prefix with frame_seq=<n>
 *          as the correlation key (passed in by the caller; assigned and
 *          owned by [PARSE]). [PIPE] never reads or mutates timestamp.
 *
 *          Events emitted:
 *            [PIPE] connect      frame_seq=<n>  -- pipe opened successfully
 *            [PIPE] write        frame_seq=<n>  -- SensorData written to pipe
 *            [PIPE] write_failed frame_seq=<n>  -- write short/error, will reconnect
 *            [PIPE] reconnect    frame_seq=<n>  -- pipe was closed, retry attempted
 *
 *          CHICKEN-AND-EGG NOTE (named FIFO startup ordering):
 *          A named FIFO open(O_RDONLY) blocks until a writer opens the other end.
 *          A named FIFO open(O_WRONLY|O_NONBLOCK) returns ENXIO if no reader is
 *          present yet. To break the deadlock:
 *            - data-controller creates sensor_pipe and opens the read end in
 *              pipe_reader_thread before sensor-server starts.
 *            - sensor-server.service uses ExecStartPre to poll until the pipe
 *              file exists, guaranteeing the reader is blocking on open() before
 *              sensor-server attempts its first write.
 *          Once open, g_pipe_fd is kept alive for the process lifetime.
 *          On EPIPE/write failure the fd is closed and reopened on next frame.
 *          EAGAIN (buffer full) drops the frame but keeps the fd open.
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include "pipe_writer.h"
#include "sensor_types.h"
#include "server_log.h"

#define SENSOR_PIPE    "/tmp/sensor_pipe" /**< named pipe to controller */
#define PIPE_RETRY_SEC 3                  /**< pipe reconnect interval s */

static int    g_pipe_fd         = -1; /**< pipe file descriptor */
static time_t g_last_pipe_retry = 0;  /**< last retry timestamp */

/******************************************************************************
 * \brief Attempt to open the sensor named pipe in non-blocking write mode.
 *
 * \return void
 *
 * \details Emits [PIPE] connect on success. frame_seq is unknown at
 *          initial open time (called before any frame arrives), so 0
 *          is used as a sentinel meaning "pre-frame connection attempt".
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void pipe_writer_try_open(void)
{
   g_pipe_fd = open(SENSOR_PIPE, O_WRONLY | O_NONBLOCK);
   if (0 <= g_pipe_fd)
   {
      log_msg("[PIPE] connect frame_seq=0");
   }
}

/******************************************************************************
 * \brief Close the pipe file descriptor.
 *
 * \return void
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void pipe_writer_close(void)
{
   if (0 <= g_pipe_fd)
   {
      close(g_pipe_fd);
      g_pipe_fd = -1;
   }
}

/******************************************************************************
 * \brief Ensure pipe is connected, retrying at PIPE_RETRY_SEC intervals.
 *
 * \return void
 *
 * \details Emits [PIPE] reconnect frame_seq=<n> when a retry open
 *          succeeds mid-stream. frame_seq=0 is used when called before
 *          any frame arrives (startup path); callers pass the current
 *          frame_seq so reconnects are traceable to the frame that
 *          triggered them.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void pipe_ensure_connected(void)
{
   time_t now = 0; /**< current timestamp */

   if (0 <= g_pipe_fd)
   {
      return;
   }

   now = time(NULL);
   if ((now - g_last_pipe_retry) < PIPE_RETRY_SEC)
   {
      return;
   }

   g_last_pipe_retry = now;

   g_pipe_fd = open(SENSOR_PIPE, O_WRONLY | O_NONBLOCK);
   if (0 <= g_pipe_fd)
   {
      log_msg("[PIPE] reconnect frame_seq=0");
   }
   else
   {
      log_msg("Controller pipe not available, retrying in %ds",
              PIPE_RETRY_SEC);
   }
}

/******************************************************************************
 * \brief Write a SensorData struct to the named pipe.
 *
 * \param p_data   - Pointer to SensorData struct to write.
 * \param frame_seq - Monotonic frame counter assigned by [PARSE] at
 *                    decode_start; threaded here for log correlation.
 *
 * \return void
 *
 * \details Emits [PIPE] write on success, [PIPE] write_failed on error.
 *          Closes and resets g_pipe_fd on write failure so the next
 *          call to pipe_ensure_connected() will attempt reconnect.
 *          Never reads or mutates p_data->timestamp.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void pipe_write(struct SensorData *p_data, uint32_t frame_seq)
{
   ssize_t w = 0; /**< bytes written */

   if (0 > g_pipe_fd)
   {
      return;
   }
   p_data->frame_seq = frame_seq;
   w = write(g_pipe_fd, p_data, sizeof(*p_data));
   if (w == (ssize_t)sizeof(*p_data))
   {
      log_msg("[PIPE] write frame_seq=%u", frame_seq);
   }
   else if (w < 0)
   {
      switch (errno)
      {
         case EAGAIN: /* FIFO buffer full — transient, drop frame, keep fd */
         case EINTR:  /* interrupted by signal — drop frame, keep fd      */
            log_msg("[PIPE] write_dropped frame_seq=%u errno=%d", frame_seq, errno);
            break;
         case EPIPE:  /* reader closed its end                            */
         case ENXIO:  /* no reader present                                */
         default:     /* unknown error — close and reconnect next frame   */
            log_msg("[PIPE] write_failed frame_seq=%u errno=%d", frame_seq, errno);
            close(g_pipe_fd);
            g_pipe_fd = -1;
            break;
      }
   }
   else
   {
      /* partial write — should not happen on a FIFO, treat as error */
      log_msg("[PIPE] write_partial frame_seq=%u bytes=%zd", frame_seq, w);
      close(g_pipe_fd);
      g_pipe_fd = -1;
   }
}
