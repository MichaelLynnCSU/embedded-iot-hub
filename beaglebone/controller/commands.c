/******************************************************************************
 * \file commands.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Command dispatcher for BeagleBone data controller.
 *
 * \details Receives CommandMsg structs and dispatches to the appropriate
 *          handler. Unimplemented commands are logged as stubs.
 *
 * \warning Many commands are currently stubbed with printf logging.
 *          See TODO tags for commands requiring full implementation.
 ******************************************************************************/

#include <stdio.h>
#include <semaphore.h>
#include "commands.h"
#include "shared_data.h"

extern struct SharedSensorData *shm_data; /**< shared memory data pointer */
extern sem_t                   *shm_sem;  /**< shared memory semaphore */

void handle_get_latest(struct CommandMsg *p_cmd);
void handle_get_device_status(struct CommandMsg *p_cmd);
void handle_get_room_status(struct CommandMsg *p_cmd);

/******************************************************************************
 * \brief Dispatch a command message to the appropriate handler.
 *
 * \param p_cmd - Pointer to command message to process.
 *
 * \return void
 *
 * \details Routes CMD_GET_DEVICE_STATUS and CMD_GET_ROOM_STATUS to their
 *          handlers. All other commands are currently stubbed with logging.
 *          No-op if p_cmd is NULL.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void process_command(struct CommandMsg *p_cmd)
{
   if (NULL == p_cmd)
   {
      return;
   }

   switch (p_cmd->cmd)
   {
      case CMD_GET_LATEST:
      {
         handle_get_latest(p_cmd);
         break;
      }

      case CMD_GET_DEVICE_STATUS:
      {
         handle_get_device_status(p_cmd);
         break;
      }

      case CMD_GET_ROOM_STATUS:
      {
         handle_get_room_status(p_cmd);
         break;
      }

      /* TODO: implement remaining commands */
      case CMD_GET_HISTORY:
      case CMD_GET_STATS_DAILY:
      case CMD_GET_STATS_WEEKLY:
      case CMD_GET_STATS_MONTHLY:
      case CMD_GET_MIN_MAX:
      case CMD_GET_AVERAGE:
      case CMD_GET_TREND:
      case CMD_GET_PEAK_TIMES:
      case CMD_GET_MOTION_HEATMAP:
      case CMD_GET_HOUR:
      case CMD_GET_DAY:
      case CMD_GET_WEEK:
      case CMD_GET_MONTH:
      case CMD_GET_ALERTS:
      case CMD_CHECK_THRESHOLD:
      case CMD_GET_RECORD_COUNT:
      case CMD_GET_SYSTEM_STATUS:
      case CMD_GET_DISK_USAGE:
      case CMD_GET_UPTIME:
      case CMD_GET_OPEN_DOORS:
      case CMD_GET_ROOM_HISTORY:
      {
         /* Fall through intentional — all stubbed */
         printf("process_command: cmd=%d client=%d (stub)\n",
                p_cmd->cmd, p_cmd->client_id);
         break;
      }

      default:
      {
         printf("process_command: unknown command %d\n", p_cmd->cmd);
         break;
      }
   }
}
