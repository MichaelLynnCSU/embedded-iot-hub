/******************************************************************************
 * \file inference_core.h
 * \brief Pure logic for inference_daemon -- no TFLite, stb, or socket deps.
 *
 * \details Extracted from inference_daemon.c to enable host-side unit testing.
 *          All functions are static inline; no .c file required.
 *
 * \note    JPEG framing: 4-byte big-endian length prefix sent by ESP32-CAM.
 *          BBB reads 4 bytes, then reads that many bytes for the JPEG payload.
 *          Any change to byte order or header size breaks the framing silently.
 ******************************************************************************/

#ifndef INFERENCE_CORE_H
#define INFERENCE_CORE_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
/* Config constants                                                            */
/*---------------------------------------------------------------------------*/

#define MAX_JPEG_BYTES      500000
#define PERSON_LABEL        "person"
#define CONFIDENCE_THRESH   0.5f
#define MAX_LABELS          100
#define LABEL_LEN           64
#define JPEG_HDR_SIZE       4     /**< 4-byte big-endian length header        */

/*---------------------------------------------------------------------------*/
/* Pure logic: JPEG length header unpacking                                   */
/*---------------------------------------------------------------------------*/

/**
 * \brief Unpack a 4-byte big-endian header into a 32-bit JPEG length.
 *
 * \details ESP32-CAM sends MSB first. BBB reads these 4 bytes to know
 *          how many bytes follow. Byte order must match cam_pack_jpeg_header()
 *          on the ESP32-CAM side exactly.
 *
 * \param hdr  4-byte buffer received from socket.
 * \return     32-bit JPEG payload length.
 */
static inline uint32_t infer_unpack_jpeg_len(const uint8_t hdr[4])
{
    return ((uint32_t)hdr[0] << 24) |
           ((uint32_t)hdr[1] << 16) |
           ((uint32_t)hdr[2] <<  8) |
            (uint32_t)hdr[3];
}

/*---------------------------------------------------------------------------*/
/* Pure logic: JPEG length validation                                         */
/*---------------------------------------------------------------------------*/

/**
 * \brief Validate a received JPEG length against protocol bounds.
 *
 * \details Rejects zero-length and oversized payloads before malloc.
 *          MAX_JPEG_BYTES guards against memory exhaustion from corrupt
 *          or malicious framing.
 *
 * \param len  Unpacked JPEG length from header.
 * \return     1 if valid, 0 if rejected.
 */
static inline int infer_jpeg_len_valid(uint32_t len)
{
    return (len > 0 && len <= MAX_JPEG_BYTES);
}

/*---------------------------------------------------------------------------*/
/* Pure logic: confidence threshold                                           */
/*---------------------------------------------------------------------------*/

/**
 * \brief Check whether a detection score meets the confidence threshold.
 *
 * \param score  TFLite output score for a detection.
 * \return       1 if above threshold, 0 otherwise.
 */
static inline int infer_above_threshold(float score)
{
    return score >= CONFIDENCE_THRESH;
}

/*---------------------------------------------------------------------------*/
/* Pure logic: person label match                                             */
/*---------------------------------------------------------------------------*/

/**
 * \brief Check whether a label string matches PERSON_LABEL.
 *
 * \param label  Null-terminated label string from labelmap.
 * \return       1 if person, 0 otherwise.
 */
static inline int infer_is_person(const char *label)
{
    if (!label) return 0;
    return strcmp(label, PERSON_LABEL) == 0;
}

/*---------------------------------------------------------------------------*/
/* Pure logic: bounds-checked label lookup                                   */
/*---------------------------------------------------------------------------*/

/**
 * \brief Return label string for a given index, with bounds check.
 *
 * \param labels      2D label array [MAX_LABELS][LABEL_LEN].
 * \param label_count Number of loaded labels.
 * \param idx         Label index (0-based).
 * \return            Label string, or "" if out of bounds.
 */
static inline const char *infer_label_for(
    const char labels[][LABEL_LEN], int label_count, int idx)
{
    if (idx < 0 || idx >= label_count) return "";
    return labels[idx];
}

/*---------------------------------------------------------------------------*/
/* Pure logic: save path filename classification                              */
/*---------------------------------------------------------------------------*/

/**
 * \brief Return the filename classification token based on detection result.
 *
 * \param detected  1 if person detected, 0 otherwise.
 * \return          "person" or "noperson".
 */
static inline const char *infer_detection_token(int detected)
{
    return detected ? "person" : "noperson";
}

#endif /* INFERENCE_CORE_H */
