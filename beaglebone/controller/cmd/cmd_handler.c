/******************************************************************************
 * \file cmd_handler.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Command pipe handler for BeagleBone controller.
 *
 * \details Owns the command named pipe lifecycle and dispatches inbound
 *          CommandMsg structs to process_command().
 *
 *          Single responsibility: command IPC transport only.
 *          Sensor pipe read moved to pipe_reader.c.
 *          Sensor frame fanout moved to sensor_dispatch.c.
 *
 *          Pipe layout:
 *            /tmp/sensor_pipe   — inbound sensor frames  (pipe_reader.c)
 *            /tmp/command_pipe  — inbound commands       (this file)
 ******************************************************************************/

#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include "config.h"
#include "globals.h"
#include "log.h"
#include "commands.h"

#define PIPE_REOPEN_DELAY_S  1  /**< seconds to wait before reopening pipe */

/******************************************************************************
 * \brief Thread — reads CommandMsg structs from command_pipe.
 *
 * \param p_arg - Unused thread argument.
 *
 * \return void*
 *
 * \details Creates the command pipe on first run (unlink + mkfifo so a
 *          stale pipe from a previous run is always replaced). Blocks on
 *          read(). Dispatches each valid CommandMsg to process_command().
 *          Reopens on EOF after PIPE_REOPEN_DELAY_S.
 ******************************************************************************/
void *command_handler_thread(void *p_arg)
{
   int               pipe_fd = -1;
   ssize_t           bytes   = 0;
   struct CommandMsg cmd;

   (void)p_arg;

   (void)unlink(COMMAND_PIPE);
   (void)mkfifo(COMMAND_PIPE, 0666);

   pipe_fd = open(COMMAND_PIPE, O_RDONLY);
   if (0 > pipe_fd)
   {
      LOG("Failed to open command pipe");
      return NULL;
   }

   LOG("Listening for commands");

   while (running)
   {
      bytes = read(pipe_fd, &cmd, sizeof(cmd));

      if (bytes == (ssize_t)sizeof(cmd))
      {
         LOG("Command %d from client %d", cmd.cmd, cmd.client_id);
         process_command(&cmd);
      }
      else if (0 == bytes)
      {
         LOG("Command pipe closed, reopening");
         close(pipe_fd);
         sleep(PIPE_REOPEN_DELAY_S);
         pipe_fd = open(COMMAND_PIPE, O_RDONLY);
      }
      else
      {
         /* partial read — ignore */
      }
   }

   close(pipe_fd);
   return NULL;
}
