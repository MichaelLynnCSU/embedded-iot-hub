/******************************************************************************
 * \file pipe_writer.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-05-10
 *
 * \brief Controller pipe IPC interface for sensor server.
 *
 * \details Owns named pipe open, reconnect, and write.
 *          Split from sensor_server.c (2026-05-10).
 ******************************************************************************/

#ifndef PIPE_WRITER_H
#define PIPE_WRITER_H

#include <stdint.h>
#include "../include/ipc_proto.h"

/******************************************************************************
 * \brief Attempt to open the sensor named pipe in non-blocking write mode.
 *
 * \return void
 ******************************************************************************/
void pipe_writer_try_open(void);

/******************************************************************************
 * \brief Close the pipe file descriptor.
 *
 * \return void
 ******************************************************************************/
void pipe_writer_close(void);

/******************************************************************************
 * \brief Ensure pipe is connected, retrying at PIPE_RETRY_SEC intervals.
 *
 * \return void
 ******************************************************************************/
void pipe_ensure_connected(void);

/******************************************************************************
 * \brief Write a SensorData struct to the named pipe.
 *
 * \param p_data - Pointer to SensorData struct to write.
 *
 * \return void
 ******************************************************************************/
void pipe_write(struct SensorData *p_data, uint32_t frame_seq);

#endif /* PIPE_WRITER_H */
