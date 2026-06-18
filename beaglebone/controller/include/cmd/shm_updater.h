/******************************************************************************
 * \file shm_updater.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-09
 *
 * \brief Shared memory projection layer for BeagleBone data controller.
 *
 * \details Consumes frozen LatestData snapshots from get_snapshot() and
 *          writes into SharedSensorData under shm_mutex. Never touches
 *          central_ledger or state_mutex directly. Also handles command
 *          responses for CMD_GET_LATEST, CMD_GET_DEVICE_STATUS, and
 *          CMD_GET_ROOM_STATUS.
 *
 *          SHM architecture — two writers, two readers:
 *
 *            [SHM] transport=sensor_shm event_id=M write src=pipe_ingress
 *            [SHM] transport=sensor_shm event_id=M write src=uart_ingress
 *            [SHM] transport=sensor_shm event_id=M read  dst=blackpill_lcd
 *            [SHM] transport=sensor_shm event_id=M read  dst=thermostat_lcd
 *
 *          pipe_ingress:    sensor_frame_dispatch() → shm_update_frame()
 *                           → handle_get_latest(p_snapshot, "pipe_ingress")
 *          uart_ingress:    uart_parse_line() → handle_get_latest()
 *                           → handle_get_latest(p_snapshot, "uart_ingress")
 *          blackpill_lcd:   uart_push_thread() / uart_update_frame()
 *          thermostat_lcd:  thermostat_lcd.c display update loop
 ******************************************************************************/
#ifndef INCLUDE_CMD_SHM_UPDATER_H_
#define INCLUDE_CMD_SHM_UPDATER_H_
#include "../sensor_types.h"
#include "../globals.h"
#include "../../shared_data.h"
#include "cmd/commands.h"
void shm_update_frame         (const struct LatestData *p_snapshot,
                                const struct SensorData *p_data);
void handle_get_latest        (const struct LatestData *p_snapshot,
                                const char              *p_src);
void handle_get_device_status (struct CommandMsg *p_cmd);
void handle_get_room_status   (struct CommandMsg *p_cmd);
#endif
