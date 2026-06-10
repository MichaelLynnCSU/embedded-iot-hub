/******************************************************************************
 * \file cmd_handler.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief Command pipe handler interface for BeagleBone data controller.
 *
 * \details Owns the command named pipe lifecycle and dispatches inbound
 *          CommandMsg structs to process_command(). Single responsibility:
 *          command IPC transport only.
 ******************************************************************************/
#ifndef INCLUDE_CMD_CMD_HANDLER_H_
#define INCLUDE_CMD_CMD_HANDLER_H_

void *command_handler_thread(void *p_arg);

#endif
