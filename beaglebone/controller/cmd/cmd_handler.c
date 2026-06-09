/******************************************************************************
 * \file cmd_handler.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Command handler and sensor data receiver for BeagleBone controller.
 *
 * \details Processes commands from pipes and routes unified sensor frames 
 * down decoupled subsystem pipelines. Two threads run concurrently:
 *
 * receive_data_thread — reads SensorData structs from sensor_pipe
 * and pushes the frames down the stage, shm, and DB pipelines.
 *
 * command_handler_thread — reads CommandMsg structs from
 * command_pipe and dispatches to process_command().
 ******************************************************************************/

#include <sys/stat.h>
#include "../controller_internal.h"

#define PIPE_REOPEN_DELAY_S  1  /**< seconds to wait before reopening pipe */

/******************************************************************************
 * \brief Receive and process one complete sensor data frame.
 ******************************************************************************/
static void sensor_frame_dispatch(const struct SensorData *p_data)
{
    if (p_data->motor_online)
    {
       heartbeat_stamp(DEV_MOTOR);
       LOG("Motor online — heartbeat stamped");
    }

    // Allocate the frozen read-model copy on the stack
    struct LatestData snapshot;

    // Mutate canonical state (Formerly uart_stage_frame)
    update_snapshot(p_data);

    // Read the frozen state model cleanly
    get_snapshot(&snapshot);

    // Pass BOTH the historical snapshot and the raw wire data
    shm_update_frame(&snapshot, p_data);
    db_persist_frame(p_data);
    uart_update_frame(&snapshot, p_data); // Changed from uart_stage_frame

    LOG("Sensor: temp=%.1f motion=%d occ=%d lgt=%d lck=%d mtr=%d "
        "ages pir=%d lgt=%d lck=%d "
        "batt pir=%d%% lck=%d%% motor=%d%% pirs=%d",
        p_data->avg_temp,
        p_data->motion_count,
        p_data->pir_occupied,
        p_data->light_state,
        p_data->lock_state,
        p_data->motor_online,
        p_data->age_pir,
        p_data->age_lgt,
        p_data->age_lck,
        p_data->batt_pir,
        p_data->batt_lck,
        p_data->batt_motor,
        p_data->pir_count);
}

/******************************************************************************
 * \brief Thread — reads SensorData structs from sensor_pipe.
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
      LOG("Failed to open sensor pipe");
      return NULL;
   }

   LOG("Listening for sensor data");

   while (running)
   {
      bytes = read(pipe_fd, &data, sizeof(data));

      if (bytes == (ssize_t)sizeof(data))
      {
         sensor_frame_dispatch(&data);
      }
      else if (0 == bytes)
      {
         LOG("Sensor pipe closed, reopening");
         close(pipe_fd);
         sleep(PIPE_REOPEN_DELAY_S);
         pipe_fd = open(SENSOR_PIPE, O_RDONLY);
      }
      else
      {
         /* partial read — ignore */
      }
   }

   close(pipe_fd);
   return NULL;
}

/******************************************************************************
 * \brief Thread — reads CommandMsg structs from command_pipe.
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
