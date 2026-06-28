/******************************************************************************
 * \file cam_dispatch.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-27
 *
 * \brief Camera frame dispatch for BeagleBone controller.
 *
 * \details Owns cam_frame_dispatch() — receives a validated CamData
 *          struct from cam_pipe_reader.c and projects it into SHM.
 *
 *          Dependency direction:
 *            cam_pipe_reader.c -> cam_dispatch.c -> shm subsystem
 ******************************************************************************/
#include "config.h"
#include "sensor_types.h"
#include "log.h"
#include "globals.h"
#include "../../shared_data.h"
#include "cam_dispatch.h"

/******************************************************************************
 * \brief Receive and process one complete camera data frame.
 *
 * \param p_data - Pointer to validated CamData frame from cam_pipe_reader.c.
 ******************************************************************************/
void cam_frame_dispatch(const struct CamData *p_data)
{
   int i = 0;

   pthread_mutex_lock(&shm_data->shm_mutex);

   for (i = 0; i < MAX_CAMS; i++)
   {
      shm_data->cam_age_s[i]  = p_data->cam_slots[i].age_s;
      shm_data->cam_online[i] = p_data->cam_slots[i].online;
   }

   for (i = 0; i < MAX_DOORBELL_CAMS; i++)
   {
      shm_data->doorbell_age_s[i]  = p_data->doorbell_slots[i].age_s;
      shm_data->doorbell_online[i] = p_data->doorbell_slots[i].online;
   }

   pthread_mutex_unlock(&shm_data->shm_mutex);

   for (i = 0; i < MAX_CAMS; i++)
   {
      LOG("[CAM_DISPATCH] cam slot=%d age_s=%d online=%d",
          i, p_data->cam_slots[i].age_s, p_data->cam_slots[i].online);
   }
   for (i = 0; i < MAX_DOORBELL_CAMS; i++)
   {
      LOG("[CAM_DISPATCH] doorbell device_id=%d age_s=%d online=%d",
          i, p_data->doorbell_slots[i].age_s, p_data->doorbell_slots[i].online);
   }
}
