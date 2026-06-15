/******************************************************************************
 * \file doorbell_result_reader.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief Accessor module for the /doorbell_result shared memory segment.
 *
 * \details See doorbell_result_reader.c and doorbell_result_shm.h for the
 *          full publish/consume contract and event_id tracing notes.
 ******************************************************************************/
#ifndef DOORBELL_RESULT_READER_H
#define DOORBELL_RESULT_READER_H

#include <stdint.h>
#include <stddef.h>

/**
 * \brief Attach (read-write) to /doorbell_result if not already attached.
 *
 * \details Safe to call repeatedly; no-op if already attached. Does not
 *          create the segment (O_CREAT intentionally absent) — only
 *          doorbell_daemon creates it. If the segment doesn't exist yet,
 *          returns -1 and leaves the reader unattached; a later call (or
 *          a later doorbell_result_reader_poll(), which calls this
 *          lazily) will retry.
 *
 * \return 0 on success (attached), -1 on failure (not attached).
 */
int doorbell_result_reader_init(void);

/**
 * \brief Poll /doorbell_result for a new (not-yet-consumed) inference
 *        result.
 *
 * \details See doorbell_result_reader.c for full details, including the
 *          "[SHM] -> [CONTROLLER] consume ..." log emitted on a
 *          successful (return 1) poll.
 *
 * \param[out] p_event_id  Event ID of the consumed result (may be NULL).
 * \param[out] p_person    1 if a person was detected, else 0 (may be NULL).
 * \param[out] p_conf_pct  Confidence 0-100 (may be NULL).
 * \param[out] p_asset     Buffer for the asset timestamp string (may be
 *                          NULL if asset_len == 0).
 * \param[in]  asset_len   Size of p_asset buffer, including space for the
 *                          terminating nul.
 *
 * \return 1 if a new result was consumed and copied out, 0 if no new
 *         result is available (or the segment isn't attached yet).
 */
int doorbell_result_reader_poll(uint64_t *p_event_id,
                                 uint8_t *p_person, uint8_t *p_conf_pct,
                                 char *p_asset, size_t asset_len);

#endif /* DOORBELL_RESULT_READER_H */
