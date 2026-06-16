/******************************************************************************
 * \file pipe_reader.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-09
 *
 * \brief Sensor pipe read transport for BeagleBone controller.
 *
 * \details Owns the sensor named pipe fd lifecycle, blocking read(),
 *          frame size validation, and reopen on EOF. Calls
 *          sensor_frame_dispatch() on every valid frame.
 *          Knows nothing about what dispatch does — that is
 *          sensor_dispatch.c's responsibility.
 *
 *          Mirrors pipe_writer.c on the server side:
 *            server/pipe_writer.c  →  /tmp/sensor_pipe  →  controller/pipe_reader.c
 *
 *          Dependency direction:
 *            pipe_reader.c -> sensor_dispatch.c -> shm/db/uart subsystems
 *
 * \note    Logging taxonomy (2026-06-16):
 *          [PIPE] open_ok, open_failed, reopen, frame_received, read_partial
 ******************************************************************************/
#include <unistd.h>
#include <fcntl.h>
#include "config.h"
#include "globals.h"
#include "log.h"
#include "sensor_types.h"
#include "sensor_dispatch.h"
#define PIPE_REOPEN_DELAY_S  1  /**< seconds to wait before reopening pipe */
/******************************************************************************
 * \brief Thread — reads SensorData structs from sensor_pipe.
 *
 * \param p_arg - Unused thread argument.
 *
 * \return void*
 *
 * \details Blocks on read(). On full-frame receipt calls
 *          sensor_frame_dispatch(). On EOF closes and reopens the pipe
 *          after PIPE_REOPEN_DELAY_S. Partial reads are logged and ignored —
 *          the next read will realign on the next frame boundary.
 ******************************************************************************/
void *receive_data_thread(void *p_arg)
{
   int               pipe_fd = -1;
   ssize_t           bytes   = 0;
   struct SensorData data;
   (void)p_arg;
   pipe_fd = open(SENSOR_PIPE, O_RDONLY);
   if (0 > pipe_fd)
   {
      LOG("[PIPE] open_failed path=%s", SENSOR_PIPE);
      return NULL;
   }
   LOG("[PIPE] open_ok path=%s", SENSOR_PIPE);
   while (running)
   {
      bytes = read(pipe_fd, &data, sizeof(data));
      if (bytes == (ssize_t)sizeof(data))
      {
         LOG("[PIPE] frame_received bytes=%zd", bytes);
         sensor_frame_dispatch(&data);
      }
      else if (0 == bytes)
      {
         LOG("[PIPE] reopen path=%s", SENSOR_PIPE);
         close(pipe_fd);
         sleep(PIPE_REOPEN_DELAY_S);
         pipe_fd = open(SENSOR_PIPE, O_RDONLY);
      }
      else
      {
         LOG("[PIPE] read_partial bytes=%zd expected=%zu", bytes, sizeof(data));
      }
   }
   close(pipe_fd);
   return NULL;
}
