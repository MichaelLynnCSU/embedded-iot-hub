/******************************************************************************
 * \file commands.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Command definitions for BeagleBone data controller IPC.
 *
 * \details Defines the command enumeration and message structure used
 *          for communication between the LCD display process and the
 *          data controller via the command pipe.
 ******************************************************************************/

#ifndef INCLUDE_COMMANDS_H_
#define INCLUDE_COMMANDS_H_

/******************************* ENUMERATIONS *********************************/

/**
 * \brief IPC command types for data controller requests.
 */
typedef enum
{
   /* Display commands */
   CMD_GET_LATEST          = 1,  /**< get latest sensor snapshot */
   CMD_GET_HISTORY         = 2,  /**< get historical data */
   CMD_GET_STATS_DAILY     = 3,  /**< get daily statistics */
   CMD_GET_STATS_WEEKLY    = 4,  /**< get weekly statistics */
   CMD_GET_STATS_MONTHLY   = 5,  /**< get monthly statistics */

   /* Analysis commands */
   CMD_GET_MIN_MAX         = 6,  /**< get min/max values */
   CMD_GET_AVERAGE         = 7,  /**< get average values */
   CMD_GET_TREND           = 8,  /**< get trend data */
   CMD_GET_PEAK_TIMES      = 9,  /**< get peak activity times */
   CMD_GET_MOTION_HEATMAP  = 10, /**< get motion heatmap data */

   /* Time-range queries */
   CMD_GET_HOUR            = 11, /**< get last hour data */
   CMD_GET_DAY             = 12, /**< get last day data */
   CMD_GET_WEEK            = 13, /**< get last week data */
   CMD_GET_MONTH           = 14, /**< get last month data */

   /* Alert and threshold commands */
   CMD_GET_ALERTS          = 15, /**< get active alerts */
   CMD_CHECK_THRESHOLD     = 16, /**< check threshold violations */

   /* System commands */
   CMD_GET_RECORD_COUNT    = 17, /**< get database record count */
   CMD_GET_SYSTEM_STATUS   = 18, /**< get system status */
   CMD_GET_DISK_USAGE      = 19, /**< get disk usage */
   CMD_GET_UPTIME          = 20, /**< get system uptime */

   /* Room sensor commands */
   CMD_GET_ROOM_STATUS     = 21, /**< get room sensor states */
   CMD_GET_OPEN_DOORS      = 22, /**< get list of open doors */
   CMD_GET_ROOM_HISTORY    = 23, /**< get room sensor history */

   /* Device heartbeat and online status */
   CMD_GET_DEVICE_STATUS   = 24, /**< get device online status */
} COMMAND_E;

/************************ STRUCTURE/UNION DATA TYPES **************************/

/**
 * \brief IPC command message — written to command pipe by LCD display.
 */
struct CommandMsg
{
   COMMAND_E cmd;       /*!< command type */
   int       client_id; /*!< requesting client identifier */
   int       param1;    /*!< command-specific parameter 1 */
   int       param2;    /*!< command-specific parameter 2 */
   long      param3;    /*!< command-specific parameter 3 */
};

/*************************** FUNCTION PROTOTYPES *****************************/

/** \brief Dispatch a command message to the appropriate handler.
 *  \param p_cmd - Pointer to command message to process.
 *  \return void */
void process_command(struct CommandMsg *p_cmd);

#endif /* INCLUDE_COMMANDS_H_ */
