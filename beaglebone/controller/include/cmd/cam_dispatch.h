/******************************************************************************
 * \file cam_dispatch.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-27
 *
 * \brief Camera frame dispatch public interface.
 ******************************************************************************/
#ifndef INCLUDE_CMD_CAM_DISPATCH_H_
#define INCLUDE_CMD_CAM_DISPATCH_H_
#include "../sensor_types.h"
void cam_frame_dispatch(const struct CamData *p_data);
#endif
