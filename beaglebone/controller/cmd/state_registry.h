/******************************************************************************
 * \file state_registry.h
 * \brief Canonical sensor state — single writer authority.
 *
 * \details Owns central_ledger and state_mutex. No other translation unit
 *          holds or references these directly. All reads go through
 *          get_snapshot(), all writes go through update_snapshot().
 ******************************************************************************/

#ifndef INCLUDE_STATE_REGISTRY_H_
#define INCLUDE_STATE_REGISTRY_H_

#include "../controller_internal.h"

void update_snapshot (const struct SensorData *p_data);
void get_snapshot    (struct LatestData *p_out);

#endif /* INCLUDE_STATE_REGISTRY_H_ */
