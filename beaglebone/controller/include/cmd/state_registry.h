/******************************************************************************
 * \file state_registry.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-09
 *
 * \brief Canonical sensor state — single writer authority.
 *
 * \details Owns central_ledger (LatestData) and state_mutex. No other
 *          translation unit holds or modifies these directly. All writes
 *          go through update_snapshot(), all reads through get_snapshot().
 *          Consumers receive a value copy — never a pointer.
 ******************************************************************************/
#ifndef INCLUDE_CMD_STATE_REGISTRY_H_
#define INCLUDE_CMD_STATE_REGISTRY_H_

#include "../sensor_types.h"

void update_snapshot(const struct SensorData *p_data);
void get_snapshot   (struct LatestData *p_out);

#endif
