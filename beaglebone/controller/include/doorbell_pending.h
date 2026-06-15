/******************************************************************************
 * \file doorbell_pending.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 2026-06-15
 *
 * \brief Doorbell inference "pending result" correlation layer.
 *
 * \details Extracted from uart_controller.c as part of the
 *          transport/protocol/correlation/lock-adapter split (no behavior
 *          change). This is the sole owner of the pending press -> inference
 *          result correlation state.
 *
 *          Lane A (UDP press event -> shm_data->doorbell_pressed) arrives
 *          almost immediately, but Lane B (TCP JPEG -> doorbell_daemon ->
 *          inference -> /doorbell_result publish) can take several seconds.
 *          When a press is seen but no result is available yet,
 *          doorbell_pending_mark() records it. Every subsequent push cycle
 *          calls doorbell_pending_check(), which polls the result shm. When
 *          the result arrives it is stored internally.
 *          doorbell_inject_pending() is called by both push paths
 *          (uart_protocol.c) before their DOORBELL snprintf block — if a
 *          result is ready and no new press arrived this cycle, it
 *          overwrites the caller's local doorbell vars with pressed=1 so
 *          the five-field frame goes out inside the normal complete bundle.
 *
 *          If no result arrives within DOORBELL_PENDING_TIMEOUT_SEC the
 *          pending slot is cleared and a warning is logged.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
#ifndef INCLUDE_DOORBELL_PENDING_H_
#define INCLUDE_DOORBELL_PENDING_H_

#include <stdint.h>

#define DOORBELL_ASSET_LEN  20   /* matches DoorbellResult.asset[20] in doorbell_result_shm.h */

/******************************************************************************
 * \brief Mark a doorbell press as pending an inference result.
 *
 * \param device_id - Doorbell camera device ID that was pressed.
 *
 * \return void
 *
 * \details Called by uart_protocol.c (both push paths) when a press is
 *          seen but doorbell_result_reader_poll() did not return a result
 *          in the same cycle. Records the device_id and current time so
 *          doorbell_pending_check() can poll on subsequent cycles and
 *          enforce DOORBELL_PENDING_TIMEOUT_SEC.
 ******************************************************************************/
void doorbell_pending_mark(int device_id);

/******************************************************************************
 * \brief Poll for late-arriving doorbell inference result and store it.
 *
 * \return void
 *
 * \details If a press is pending and no result is stored yet, polls
 *          doorbell_result_reader_poll(). On a hit, stores the result
 *          fields internally for the next doorbell_inject_pending() call.
 *          On timeout (DOORBELL_PENDING_TIMEOUT_SEC), clears the pending
 *          slot and logs a warning.
 ******************************************************************************/
void doorbell_pending_check(void);

/******************************************************************************
 * \brief Inject a stored pending inference result into the current push cycle.
 *
 * \param p_pressed    - In/out: 1 if a new press occurred this cycle.
 * \param p_device_id  - In/out: doorbell device ID.
 * \param p_valid      - In/out: 1 if db_* fields below are valid.
 * \param p_event_id   - Out: event_id of the injected result.
 * \param p_person     - Out: person detection result.
 * \param p_conf_pct   - Out: confidence percent.
 * \param p_asset      - Out: asset timestamp string buffer.
 * \param asset_len    - Size of p_asset buffer.
 *
 * \return void
 *
 * \details If a result is ready and *p_pressed is 0 (no new press this
 *          cycle), overwrites the caller's local doorbell variables with
 *          the stored result fields and sets *p_pressed = 1 so the
 *          five-field DOORBELL frame goes out in the normal bundle.
 *          Clears the pending slot atomically. Must be called after the
 *          shm doorbell read block and before the DOORBELL snprintf block
 *          in each push path.
 ******************************************************************************/
void doorbell_inject_pending(int      *p_pressed,
                              int      *p_device_id,
                              int      *p_valid,
                              uint64_t *p_event_id,
                              uint8_t  *p_person,
                              uint8_t  *p_conf_pct,
                              char     *p_asset,
                              int       asset_len);

#endif /* INCLUDE_DOORBELL_PENDING_H_ */
