/******************************************************************************
 * \file inference_core.h
 * \brief Pure logic for inference_daemon -- no TFLite, stb, or socket deps.
 *
 * \details Extracted from inference_daemon.c to enable host-side unit testing.
 *          All functions are static inline; no .c file required.
 *
 * \note    JPEG framing (legacy): 4-byte big-endian length prefix. Retained
 *          below for reference; superseded by indoor_header_t (2026-07-02),
 *          see note further down.
 *
 * \note    TCP header event_id (2026-07-02):
 *          Wire header upgraded to a 20-byte indoor_header_t --
 *          [magic:4][version:1][device_id:1][reserved:2][event_id:8][jpeg_size:4],
 *          network byte order -- matching cam_header_t in
 *          esp32-cam/main/cam_logic.h. Duplicated locally rather than
 *          shared across the ESP32/BBB build boundary, per this
 *          project's existing convention (see doorbell_daemon.c, which
 *          does the same for its own header).
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
#define JPEG_HDR_SIZE       4     /**< 4-byte big-endian length header (legacy) */

/*---------------------------------------------------------------------------*/
/* TCP frame header -- mirrors cam_header_t in esp32-cam/main/cam_logic.h    */
/*---------------------------------------------------------------------------*/

#define CAM_MAGIC           0xCAFEBABEu
#define CAM_HEADER_VERSION  1
#define CAM_HEADER_SIZE     20    /**< packed indoor_header_t size, bytes    */

/**
 * \brief Packed TCP header read before each JPEG frame.
 *
 * \details Same layout as cam_header_t (esp32-cam/main/cam_logic.h).
 *          event_id correlates a received clip back to the UDP CAPTURE
 *          trigger that started it; see inference_daemon.c's receive_clip().
 */
typedef struct {
    uint32_t magic;
    uint8_t  version;
    uint8_t  device_id;
    uint16_t reserved;
    uint64_t event_id;
    uint32_t jpeg_size;
} indoor_header_t;

/**
 * \brief Unpack a 20-byte network-order buffer into an indoor_header_t.
 *
 * \details Byte layout must match cam_pack_header() on the ESP32-CAM side
 *          exactly (esp32-cam/main/cam_logic.h).
 *
 * \param[in]  buf  CAM_HEADER_SIZE input buffer.
 * \param[out] out  Pointer to indoor_header_t to fill.
 */
static inline void infer_unpack_header(const uint8_t *buf, indoor_header_t *out)
{
    out->magic     = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                      ((uint32_t)buf[2] <<  8) |  (uint32_t)buf[3];
    out->version   = buf[4];
    out->device_id = buf[5];
    out->reserved  = 0;
    out->event_id  = ((uint64_t)buf[8]  << 56) | ((uint64_t)buf[9]  << 48) |
                      ((uint64_t)buf[10] << 40) | ((uint64_t)buf[11] << 32) |
                      ((uint64_t)buf[12] << 24) | ((uint64_t)buf[13] << 16) |
                      ((uint64_t)buf[14] <<  8) |  (uint64_t)buf[15];
    out->jpeg_size = ((uint32_t)buf[16] << 24) | ((uint32_t)buf[17] << 16) |
                      ((uint32_t)buf[18] <<  8) |  (uint32_t)buf[19];
}

/**
 * \brief Validate an indoor_header_t magic number.
 *
 * \param[in] h  Pointer to indoor_header_t.
 * \return       1 if valid, 0 if not.
 */
static inline int infer_header_valid(const indoor_header_t *h)
{
    return (h->magic == CAM_MAGIC) ? 1 : 0;
}

/*---------------------------------------------------------------------------*/
/* event_id logging convention -- duplicated from doorbell_result_shm.h so   */
/* this pure-logic header doesn't pull in doorbell-specific shm structs.     */
/*---------------------------------------------------------------------------*/

#define EVENT_ID_FMT "%08lx%08lx"
#define EVENT_ID_ARG(id) \
   (unsigned long)((uint64_t)(id) >> 32), \
   (unsigned long)((uint64_t)(id) & 0xFFFFFFFFUL)

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
