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
 ******************************************************************************/

#ifndef CAM_LOGIC_H
#define CAM_LOGIC_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
/* Network config -- hub/BBB alignment regression guards                       */
/*---------------------------------------------------------------------------*/

#define BBB_HOST            "10.0.0.206"  /**< BeagleBone IP address         */
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

#define CAM_XCLK_HZ         20000000  /**< XCLK frequency Hz                 */
#define CAM_JPEG_QUALITY    12        /**< JPEG quality 0-63, lower=better   */
#define CAM_FB_COUNT        1         /**< Frame buffer count                 */
#define CAM_CAPTURE_RETRIES 5         /**< Max capture retries on failure     */
#define CAM_CAPTURE_RETRY_MS 200      /**< Delay between capture retries ms  */
#define CAM_WARMUP_MS       500       /**< Camera warmup delay ms            */

/*---------------------------------------------------------------------------*/
/* UDP trigger                                                                 */
/*---------------------------------------------------------------------------*/

#define CAM_TRIGGER_STR     "CAPTURE"  /**< UDP trigger string               */
#define CAM_TRIGGER_LEN     7          /**< Length of trigger string         */
#define CAM_HDR_SIZE        4          /**< JPEG length header bytes         */

/*---------------------------------------------------------------------------*/
/* Heartbeat                                                                   */
/*---------------------------------------------------------------------------*/

#define CAM_SLOT              0             /**< this camera's slot index 0-2   */
#define HUB_HOST              "10.0.0.190"  /**< Hub static IP                  */
#define HUB_HEARTBEAT_PORT    9092          /**< UDP heartbeat receive port on hub */
#define CAM_HEARTBEAT_MS      30000         /**< heartbeat interval ms, 30secs         */
#define CAM_HEARTBEAT_JITTER  5000          /**< max jitter ms to avoid sync    */

/*---------------------------------------------------------------------------*/
/* Pure logic: JPEG length header packing                                     */
/*---------------------------------------------------------------------------*/

/**
 * \brief Pack a 32-bit JPEG length into a 4-byte big-endian header.
 *
 * \details BBB receiver reads these 4 bytes first to know how many
 *          bytes follow. Byte order is MSB first (network order).
 *          Any change breaks the BBB parser silently.
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
 * \details Inverse of cam_pack_jpeg_header(). Used for roundtrip testing.
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
/* Pure logic: UDP trigger match                                               */
/*---------------------------------------------------------------------------*/

/**
 * \brief Check whether a received UDP payload matches the trigger string.
 *
 * \param buf  Null-terminated receive buffer.
 * \return     1 if trigger matches, 0 otherwise.
 */
static inline int cam_is_trigger(const char *buf)
{
    return strncmp(buf, CAM_TRIGGER_STR, CAM_TRIGGER_LEN) == 0;
}

#endif /* CAM_LOGIC_H */
