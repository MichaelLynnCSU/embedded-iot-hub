/******************************************************************************
 * \file pipe_reader.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-09
 *
 * \brief Sensor pipe read transport interface for BeagleBone data controller.
 *
 * \details Owns the sensor named pipe fd lifecycle, blocking read(),
 *          frame size validation, and reopen on EOF. Calls
 *          sensor_frame_dispatch() on every valid SensorData frame.
 *          Knows nothing about what dispatch does.
 ******************************************************************************/
#ifndef INCLUDE_CMD_PIPE_READER_H_
#define INCLUDE_CMD_PIPE_READER_H_

void *receive_data_thread(void *p_arg);

#endif /* INCLUDE_CMD_PIPE_READER_H_ */
