/******************************************************************************
 * \file cam_logic.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief Pure logic for ESP32-S3-CAM node — no hardware dependencies.
 *
 * \details Extracted from main.c to enable host-side unit testing.
 *          All functions are static inline; no .c file required.
 *
 * \note    Header packing: 4-byte big-endian length prefix sent before
 *          JPEG payload. BBB receiver reads 4 bytes, then reads that many
 *          bytes for the image. Any change breaks the BBB parser silently.
 *
 * \note    Trigger identity model (2026-06-18):
 *          cam_is_trigger() replaced by cam_trigger_t + cam_parse_trigger().
 *          The UDP payload now carries event_id, cam_tx_id, and zone so
 *          the camera lifecycle is traceable back to the originating PIR
 *          event. cam_is_trigger() removed — no longer called anywhere.
 *
 *          Wire format (named-field, self-describing):
 *            CAPTURE:event_id=101,cam_tx_id=42,zone=0
 ******************************************************************************/

#ifndef CAM_LOGIC_H
#define CAM_LOGIC_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

/*---------------------------------------------------------------------------*/
/* Network config -- hub/BBB alignment regression guards                       */
/*---------------------------------------------------------------------------*/

#define BBB_HOST            "10.0.1.1"  /**< BeagleBone Wifi USB IP address */
#define BBB_PORT            9090          /**< BBB TCP listen port            */
#define UDP_TRIGGER_PORT    9091          /**< UDP trigger listen port        */
#define RECONNECT_MS        3000          /**< TCP reconnect delay ms         */

/*---------------------------------------------------------------------------*/
/* Camera pin assignments (from board silkscreen)                              */
/*---------------------------------------------------------------------------*/

#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     15
#define CAM_PIN_SIOD      4
#define CAM_PIN_SIOC      5
#define CAM_PIN_D7       16
#define CAM_PIN_D6       17
#define CAM_PIN_D5       18
#define CAM_PIN_D4       12
#define CAM_PIN_D3       10
#define CAM_PIN_D2        8
#define CAM_PIN_D1        9
#define CAM_PIN_D0       11
#define CAM_PIN_VSYNC     6
#define CAM_PIN_HREF      7
#define CAM_PIN_PCLK     13

/*---------------------------------------------------------------------------*/
/* Camera config                                                               */
/*---------------------------------------------------------------------------*/

#define CAM_XCLK_HZ          20000000  /**< XCLK frequency Hz                */
#define CAM_JPEG_QUALITY     12        /**< JPEG quality 0-63, lower=better  */
#define CAM_FB_COUNT         1         /**< Frame buffer count               */
#define CAM_CAPTURE_RETRIES  5         /**< Max capture retries on failure   */
#define CAM_CAPTURE_RETRY_MS 200       /**< Delay between capture retries ms */
#define CAM_WARMUP_MS        500       /**< Camera warmup delay ms           */
#define CAM_CLIP_DURATION_MS 10000     /**< total clip duration ms           */
#define CAM_CLIP_FRAME_MS    500       /**< interval between frames ms       */

/*---------------------------------------------------------------------------*/
/* UDP trigger                                                                 */
/*---------------------------------------------------------------------------*/

#define CAM_TRIGGER_PREFIX  "CAPTURE:"  /**< Named-field trigger prefix       */
#define CAM_HDR_SIZE        4           /**< JPEG length header bytes         */

/*---------------------------------------------------------------------------*/
/* Heartbeat                                                                   */
/*---------------------------------------------------------------------------*/

#define CAM_SLOT             0             /**< this camera's slot index 0-2  */
#define HUB_HOST             "10.0.0.190"  /**< Hub static IP                 */
#define HUB_HEARTBEAT_PORT   9092          /**< UDP heartbeat port on hub     */
#define CAM_HEARTBEAT_MS     30000         /**< heartbeat interval ms         */
#define CAM_HEARTBEAT_JITTER 5000          /**< max jitter ms                 */

/*---------------------------------------------------------------------------*/
/* Trigger identity                                                            */
/*---------------------------------------------------------------------------*/

/**
 * \brief Parsed UDP trigger context.
 *
 * \details Populated by cam_parse_trigger() from the named-field payload:
 *
 *            CAPTURE:event_id=101,cam_tx_id=42,zone=0
 *
 *          Enqueued as a message (not a bare signal) so event_id, cam_tx_id,
 *          and zone survive through the capture pipeline to the BBB send.
 */
typedef struct
{
    uint64_t event_id;   /**< PIR VROOM event_id — correlation key         */
    uint32_t cam_tx_id;  /**< hub-side UDP transport counter               */
    uint8_t  zone;       /**< PIR slot index (0-based)                     */
} cam_trigger_t;

/*---------------------------------------------------------------------------*/
/* Pure logic: JPEG length header packing                                     */
/*---------------------------------------------------------------------------*/

/**
 * \brief Pack a 32-bit JPEG length into a 4-byte big-endian header.
 *
 * \param hdr[4]  Output buffer — must be at least 4 bytes.
 * \param len     JPEG payload length in bytes.
 */
static inline void cam_pack_jpeg_header(uint8_t hdr[4], uint32_t len)
{
    hdr[0] = (len >> 24) & 0xFF;
    hdr[1] = (len >> 16) & 0xFF;
    hdr[2] = (len >>  8) & 0xFF;
    hdr[3] = (len      ) & 0xFF;
}

/**
 * \brief Unpack a 4-byte big-endian header back to a 32-bit length.
 *
 * \param hdr[4]  4-byte big-endian header buffer.
 * \return        32-bit JPEG length.
 */
static inline uint32_t cam_unpack_jpeg_header(const uint8_t hdr[4])
{
    return ((uint32_t)hdr[0] << 24) |
           ((uint32_t)hdr[1] << 16) |
           ((uint32_t)hdr[2] <<  8) |
            (uint32_t)hdr[3];
}

/*---------------------------------------------------------------------------*/
/* Pure logic: UDP trigger parse                                               */
/*---------------------------------------------------------------------------*/

/**
 * \brief Parse a named-field UDP trigger payload into a cam_trigger_t.
 *
 * \details Expects the wire format produced by cam_trigger.c on the hub:
 *
 *            CAPTURE:event_id=<N>,cam_tx_id=<N>,zone=<N>
 *
 *          Fields default to 0 if absent — forward compatible with future
 *          additions. Rejects payloads that don't start with "CAPTURE:".
 *
 * \param buf    Null-terminated receive buffer.
 * \param p_out  Output trigger struct — populated on success.
 * \return       1 if payload is a valid CAPTURE trigger, 0 otherwise.
 */
static inline int cam_parse_trigger(const char *buf, cam_trigger_t *p_out)
{
    unsigned long long event_id  = 0;
    unsigned int       cam_tx_id = 0;
    int                zone      = 0;

    if (NULL == buf || NULL == p_out)                        { return 0; }
    if (strncmp(buf, CAM_TRIGGER_PREFIX,
                strlen(CAM_TRIGGER_PREFIX)) != 0)            { return 0; }

    /* Fields are optional — sscanf fills what it finds, rest stay 0. */
    sscanf(buf + strlen(CAM_TRIGGER_PREFIX),
           "event_id=%llu,cam_tx_id=%u,zone=%d",
           &event_id, &cam_tx_id, &zone);

    p_out->event_id  = (uint64_t)event_id;
    p_out->cam_tx_id = (uint32_t)cam_tx_id;
    p_out->zone      = (uint8_t)zone;

    return 1;
}

#endif /* CAM_LOGIC_H */
