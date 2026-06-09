#ifndef INCLUDE_SHM_UPDATER_H_
#define INCLUDE_SHM_UPDATER_H_

#include "../controller_internal.h"

void shm_update_frame         (const struct LatestData *p_snapshot,
                                const struct SensorData *p_data);
void handle_get_latest        (const struct LatestData *p_snapshot);
void handle_get_device_status (struct CommandMsg       *p_cmd);
void handle_get_room_status   (struct CommandMsg       *p_cmd);

#endif /* INCLUDE_SHM_UPDATER_H_ */
