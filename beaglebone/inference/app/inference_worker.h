/******************************************************************************
 * \file inference_worker.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief Shared TFLite inference worker — used by inference_daemon (PIR)
 *        and doorbell_daemon (doorbell).
 *
 * \details Owns the TFLite model, interpreter, and label table.
 *          Provides init/shutdown, run_inference(), and save_jpeg().
 *          Transport parsing (TCP header, framing) stays in each daemon.
 *
 * \note    Storage paths are passed in by the caller so each daemon
 *          can write to its own directory (/data/pir, /data/doorbell).
 ******************************************************************************/

#ifndef INFERENCE_WORKER_H
#define INFERENCE_WORKER_H

#include <stdint.h>
#include <stddef.h>

/**
 * \brief Initialize TFLite model, interpreter, and label table.
 *
 * \param model_path  Path to .tflite model file.
 * \param label_path  Path to labelmap.txt.
 * \return            0 on success, -1 on failure.
 */
int inference_worker_init(const char *model_path, const char *label_path);

/**
 * \brief Shut down interpreter and free model resources.
 *
 * \return void
 */
void inference_worker_shutdown(void);

/**
 * \brief Run person detection on a JPEG buffer.
 *
 * \param jpeg            JPEG payload bytes.
 * \param jpeg_len        JPEG payload length.
 * \param person_detected Output: 1 if person detected, 0 otherwise.
 * \param confidence      Output: detection confidence score.
 * \return                0 on success, -1 on inference failure.
 */
int inference_worker_run(const uint8_t *jpeg, size_t jpeg_len,
                         int *person_detected, float *confidence);

/**
 * \brief Save a JPEG to disk with timestamped filename.
 *
 * \param jpeg       JPEG payload bytes.
 * \param len        JPEG payload length.
 * \param dir        Output directory (e.g. "/data/doorbell").
 * \param detected   1 if person detected, 0 otherwise.
 * \param confidence Detection confidence score.
 * \param tag        Extra tag string embedded in filename (e.g. "dev0"),
 *                   or NULL for no tag.
 * \return           void
 */
void inference_worker_save(const uint8_t *jpeg, size_t len,
                           const char *dir,
                           int detected, float confidence,
                           const char *tag);

#endif /* INFERENCE_WORKER_H */
