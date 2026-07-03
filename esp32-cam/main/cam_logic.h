/******************************************************************************
 * \file cam_logic.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief Pure logic for ESP32-S3-CAM node — no hardware dependencies.
 *
 * \details Extracted from main.c to enable host-side unit testing.
 *          All functions are static inline; no .c file required.
 *
 * \note    Trigger identity model (2026-06-18):
 *          cam_is_trigger() replaced by cam_trigger_t + cam_parse_trigger().
 *          The UDP payload now carries event_id, cam_tx_id, and zone so
 *          the camera lifecycle is traceable back to the originating PIR
 *          event. cam_is_trigger() removed — no longer called anywhere.
 *
 *          Wire format (named-field, self-describing):
 *            CAPTURE:event_id=101,cam_tx_id=42,zone=0
 *
 * \note    TCP header event_id (2026-07-02):
 *          Legacy wire header (bare 4-byte big-endian JPEG length prefix,
 *          no correlation key) replaced with cam_header_t — the same
 *          struct already used by the doorbell path
 *          (esp32-doorbell/main/cam_logic.h):
 *            [magic:4][version:1][device_id:1][reserved:2][event_id:8][jpeg_size:4]
 *          (20 bytes, packed, network byte order). Sent once per frame,
 *          same as before, just with 16 more bytes ahead of jpeg_size.
 *
 *          event_id is carried unmodified from the UDP CAPTURE trigger
 *          (cam_trigger_t.event_id) into every frame header sent to the
 *          BBB, so inference_daemon can log it on clip_start/rx/inference
 *          lines the same way doorbell_daemon does — one ID, greppable
 *          end to end: PIR -> [TRIGGER] sent -> [UDP_CAM_RX] -> [CAM] ->
 *          [INDOOR] rx / infer_done.
 *
 *          magic+version let inference_daemon reject a peer still running
 *          the old 4-byte-length-only firmware instead of silently
 *          misparsing its first 4 bytes as a bogus magic/event_id. Any
 *          future wire change bumps CAM_HEADER_VERSION the same way.
 *          device_id carries CAM_SLOT so multi-cam deployments can tell
 *          frames apart, mirroring doorbell's device_id semantics.
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

/*---------------------------------------------------------------------------*/
/* TCP frame header -- shared layout with esp32-doorbell/main/cam_logic.h     */
/*---------------------------------------------------------------------------*/

#define CAM_MAGIC           0xCAFEBABEu   /**< Packet magic number            */
#define CAM_HEADER_VERSION  1             /**< TCP header format version      */
#define CAM_HEADER_SIZE     20            /**< TCP header size in bytes       */
#define CAM_MAX_SLOTS       3             /**< indoor/front/back — must match
                                            *   MAX_CAMS (beaglebone/include/
                                            *   ipc_proto.h and friends)       */

/*---------------------------------------------------------------------------*/
/* Heartbeat                                                                   */
/*---------------------------------------------------------------------------*/

#ifndef CAM_SLOT
#define CAM_SLOT             0             /**< override at build time: idf.py -DCAM_SLOT=N build  */
#endif
#define CAMERA_MANAGER_HOST  "10.0.1.1"    /**< BBB camera manager (wlu1 AP)  */
#define CAMERA_MANAGER_PORT  9094          /**< BBB camera manager UDP port   */
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
    uint64_t event_id;   /**< PIR WROOM event_id — correlation key         */
    uint32_t cam_tx_id;  /**< hub-side UDP transport counter               */
    uint8_t  zone;       /**< PIR slot index (0-based)                     */
} cam_trigger_t;

/**
 * \brief Packed TCP header sent before each JPEG frame.
 *
 * \details Identical layout to esp32-doorbell/main/cam_logic.h's
 *          cam_header_t — BBB uses magic to validate, version to handle
 *          future formats, device_id to identify the sending cam slot,
 *          event_id to correlate with the UDP CAPTURE trigger that
 *          started this clip, jpeg_size to recv exactly the right number
 *          of bytes.
 */
typedef struct {
    uint32_t magic;      /**< CAM_MAGIC — validates packet start             */
    uint8_t  version;    /**< CAM_HEADER_VERSION — protocol version          */
    uint8_t  device_id;  /**< CAM_SLOT, identifies which security cam        */
    uint16_t reserved;   /**< alignment padding, set to 0                    */
    uint64_t event_id;   /**< correlation key — matches UDP CAPTURE trigger  */
    uint32_t jpeg_size;  /**< JPEG payload size in bytes                     */
} __attribute__((packed)) cam_header_t;

/*---------------------------------------------------------------------------*/
/* Pure logic: TCP frame header pack / unpack — testable on host              */
/*---------------------------------------------------------------------------*/

/**
 * \brief Pack a cam_header_t into a 20-byte buffer (network byte order).
 *
 * \param[out] buf       CAM_HEADER_SIZE output buffer.
 * \param[in]  device_id Sending cam slot (CAM_SLOT).
 * \param[in]  event_id  Correlation key from the UDP CAPTURE trigger.
 * \param[in]  jpeg_size JPEG payload length in bytes.
 */
static inline void cam_pack_header(uint8_t *buf,
                                   uint8_t  device_id,
                                   uint64_t event_id,
                                   uint32_t jpeg_size)
{
    uint32_t magic = CAM_MAGIC;
    buf[0]  = (uint8_t)((magic     >> 24) & 0xFF);
    buf[1]  = (uint8_t)((magic     >> 16) & 0xFF);
    buf[2]  = (uint8_t)((magic     >>  8) & 0xFF);
    buf[3]  = (uint8_t)((magic          ) & 0xFF);
    buf[4]  = CAM_HEADER_VERSION;
    buf[5]  = device_id;
    buf[6]  = 0;
    buf[7]  = 0;
    buf[8]  = (uint8_t)((event_id  >> 56) & 0xFF);
    buf[9]  = (uint8_t)((event_id  >> 48) & 0xFF);
    buf[10] = (uint8_t)((event_id  >> 40) & 0xFF);
    buf[11] = (uint8_t)((event_id  >> 32) & 0xFF);
    buf[12] = (uint8_t)((event_id  >> 24) & 0xFF);
    buf[13] = (uint8_t)((event_id  >> 16) & 0xFF);
    buf[14] = (uint8_t)((event_id  >>  8) & 0xFF);
    buf[15] = (uint8_t)((event_id       ) & 0xFF);
    buf[16] = (uint8_t)((jpeg_size >> 24) & 0xFF);
    buf[17] = (uint8_t)((jpeg_size >> 16) & 0xFF);
    buf[18] = (uint8_t)((jpeg_size >>  8) & 0xFF);
    buf[19] = (uint8_t)((jpeg_size      ) & 0xFF);
}

/**
 * \brief Unpack a cam_header_t from a 20-byte buffer (network byte order).
 *
 * \param[in]  buf  CAM_HEADER_SIZE input buffer.
 * \param[out] out  Pointer to cam_header_t to fill.
 */
static inline void cam_unpack_header(const uint8_t *buf, cam_header_t *out)
{
    out->magic     = ((uint32_t)buf[0]  << 24) | ((uint32_t)buf[1]  << 16) |
                     ((uint32_t)buf[2]  <<  8) | ((uint32_t)buf[3]);
    out->version   = buf[4];
    out->device_id = buf[5];
    out->reserved  = 0;
    out->event_id  = ((uint64_t)buf[8]  << 56) | ((uint64_t)buf[9]  << 48) |
                     ((uint64_t)buf[10] << 40) | ((uint64_t)buf[11] << 32) |
                     ((uint64_t)buf[12] << 24) | ((uint64_t)buf[13] << 16) |
                     ((uint64_t)buf[14] <<  8) | ((uint64_t)buf[15]);
    out->jpeg_size = ((uint32_t)buf[16] << 24) | ((uint32_t)buf[17] << 16) |
                     ((uint32_t)buf[18] <<  8) | ((uint32_t)buf[19]);
}

/**
 * \brief Validate a cam_header_t magic number.
 *
 * \param[in] h  Pointer to cam_header_t.
 * \return       1 if valid, 0 if not.
 */
static inline int cam_header_valid(const cam_header_t *h)
{
    return (h->magic == CAM_MAGIC) ? 1 : 0;
}

/**
 * \brief Validate a device ID (cam slot) is in range.
 *
 * \param[in] device_id  Device ID to check.
 * \return               1 if valid, 0 if not.
 */
static inline int cam_device_id_valid(uint8_t device_id)
{
    return (device_id < CAM_MAX_SLOTS) ? 1 : 0;
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
