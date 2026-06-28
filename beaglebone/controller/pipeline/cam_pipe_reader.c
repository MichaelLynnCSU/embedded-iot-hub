/******************************************************************************
 * \file cam_pipe_reader.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-27
 *
 * \brief Camera pipe read transport for BeagleBone controller.
 *
 * \details Owns the camera named pipe fd lifecycle, blocking read(),
 *          frame size validation, and reopen on EOF. Calls
 *          cam_frame_dispatch() on every valid frame.
 *
 *          Mirrors pipe_reader.c:
 *            camera_manager  →  /tmp/cam_pipe  →  cam_pipe_reader.c
 *
 *          Dependency direction:
 *            cam_pipe_reader.c -> cam_dispatch.c -> shm/uart subsystems
 *
 *          CHICKEN-AND-EGG NOTE (named FIFO startup ordering):
 *          A named FIFO open(O_RDONLY) blocks until a writer opens the
 *          other end. To avoid deadlocking with camera_manager which uses
 *          O_WRONLY|O_NONBLOCK (returns ENXIO if no reader present), the
 *          read end is opened with O_RDONLY|O_NONBLOCK so it returns
 *          immediately regardless of whether a writer exists yet.
 *          The read loop then uses poll() to block efficiently until data
 *          arrives, avoiding a busy-spin on EAGAIN.
 ******************************************************************************/
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include "config.h"
#include "globals.h"
#include "log.h"
#include "sensor_types.h"
#include "cam_dispatch.h"

#define CAM_PIPE_REOPEN_DELAY_S  1
#define CAM_PIPE_POLL_MS         500

/******************************************************************************
 * \brief Thread — reads CamData structs from cam_pipe.
 ******************************************************************************/
void *cam_pipe_reader_thread(void *p_arg)
{
   int            pipe_fd = -1;
   ssize_t        bytes   = 0;
   struct CamData data;
   struct pollfd  pfd;
   int            ret     = 0;
   (void)p_arg;

   pipe_fd = open(CAM_PIPE, O_RDONLY | O_NONBLOCK);
   if (0 > pipe_fd)
   {
      LOG("[CAM_PIPE] open_failed path=%s", CAM_PIPE);
      return NULL;
   }
   LOG("[CAM_PIPE] open_ok path=%s", CAM_PIPE);

   while (running)
   {
      pfd.fd     = pipe_fd;
      pfd.events = POLLIN;
      ret = poll(&pfd, 1, CAM_PIPE_POLL_MS);

      if (ret < 0)
      {
         if (errno == EINTR) { continue; }
         LOG("[CAM_PIPE] poll_error errno=%d", errno);
         break;
      }

      if (ret == 0) { continue; } /* timeout — check running flag */

      if (!(pfd.revents & POLLIN)) { continue; }

      bytes = read(pipe_fd, &data, sizeof(data));

      if (bytes == (ssize_t)sizeof(data))
      {
         LOG("[CAM_PIPE] frame_received bytes=%zd", bytes);
         cam_frame_dispatch(&data);
      }
      else if (bytes == 0)
      {
         /* EOF — writer closed its end, reopen */
         LOG("[CAM_PIPE] eof_reopen path=%s", CAM_PIPE);
         close(pipe_fd);
         sleep(CAM_PIPE_REOPEN_DELAY_S);
         pipe_fd = open(CAM_PIPE, O_RDONLY | O_NONBLOCK);
         if (0 > pipe_fd)
         {
            LOG("[CAM_PIPE] reopen_failed path=%s", CAM_PIPE);
            return NULL;
         }
      }
      else if (bytes < 0)
      {
         if (errno == EAGAIN || errno == EINTR) { continue; }
         LOG("[CAM_PIPE] read_error errno=%d", errno);
         break;
      }
      else
      {
         LOG("[CAM_PIPE] read_partial bytes=%zd expected=%zu", bytes, sizeof(data));
      }
   }

   close(pipe_fd);
   return NULL;
}
