/******************************************************************************
 * \file cam_logic.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief Pure logic for ESP32 doorbell CAM node — no hardware dependencies.
 *
 * \details Extracted from main.c to enable host-side unit testing.
 *          All functions are static inline; no .c file required.
 *
 * \note    Packet header: magic + device_id + jpeg_size (packed).
 *          BBB port 9091 shared by all doorbell cams.
 *          Device ID set at build time via -DDOORBELL_ID=N (0-3).
 *          Max 4 doorbell cams (MAX_DOORBELL_CAMS).
 *
 * \note    Board: AI-Thinker ESP32-CAM (OV2640).
 *          Trigger: GPIO13 button press (active low, internal pull-up disabled
 *          — LED+resistor circuit acts as external pullup).
 *          Destination: BBB port 9091 (separate from S3 PIR cam on 9090).
 *
 * \note    Internal pullup disabled on GPIO13 (GPIO_PULLUP_DISABLE).
 *          External LED+resistor circuit acts as pullup. Internal pullup
 *          caused LED to glow dimly when button not pressed due to weak
 *          leakage current through internal ~45kΩ pullup resistor.
 ******************************************************************************/

#ifndef CAM_LOGIC_H
#define CAM_LOGIC_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
/* Network config                                                               */
/*---------------------------------------------------------------------------*/
#define BBB_HOST            "10.0.0.208"  /**< BeagleBone IP address          */
#define BBB_PORT            9091          /**< BBB TCP listen port (doorbell) */
#define RECONNECT_MS        3000          /**< TCP reconnect delay ms         */

/*---------------------------------------------------------------------------*/
/* Multi-device config                                                         */
/*---------------------------------------------------------------------------*/
#define MAX_DOORBELL_CAMS   4             /**< Max doorbell cams in system    */
#define CAM_MAGIC           0xCAFEBABEu   /**< Packet magic number            */

#ifndef DOORBELL_ID
#define DOORBELL_ID         0             /**< Default device ID (override    */
#endif                                    /**< at build: -DDOORBELL_ID=N)     */

/**
 * \brief Packed packet header sent before each JPEG payload.
 *        BBB uses magic to validate, device_id to route, jpeg_size to recv.
 */
typedef struct {
    uint32_t magic;      /**< CAM_MAGIC — validates packet start             */
    uint16_t device_id;  /**< 0-3, identifies which doorbell cam             */
    uint32_t jpeg_size;  /**< JPEG payload size in bytes                     */
} __attribute__((packed)) cam_header_t;

/*---------------------------------------------------------------------------*/
/* Camera pin assignments (AI-Thinker ESP32-CAM OV2640)                       */
/*---------------------------------------------------------------------------*/
#define CAM_PIN_PWDN     32
#define CAM_PIN_RESET    -1
#define CAM_PIN_XCLK      0
#define CAM_PIN_SIOD     26
#define CAM_PIN_SIOC     27
#define CAM_PIN_D7       35
#define CAM_PIN_D6       34
#define CAM_PIN_D5       39
#define CAM_PIN_D4       36
#define CAM_PIN_D3       21
#define CAM_PIN_D2       19
#define CAM_PIN_D1       18
#define CAM_PIN_D0        5
#define CAM_PIN_VSYNC    25
#define CAM_PIN_HREF     23
#define CAM_PIN_PCLK     22

/*---------------------------------------------------------------------------*/
/* Button config                                                                */
/*---------------------------------------------------------------------------*/
#define BUTTON_GPIO      13   /**< Doorbell button GPIO (active low)         */
#define DEBOUNCE_MS     200   /**< Button debounce delay ms                  */

/*---------------------------------------------------------------------------*/
/* Header pack / unpack — testable on host                                     */
/*---------------------------------------------------------------------------*/

/**
 * \brief Pack a cam_header_t into a byte buffer (network byte order).
 *
 * \param[out] buf       10-byte output buffer.
 * \param[in]  device_id Doorbell device ID (0-3).
 * \param[in]  jpeg_size JPEG payload length in bytes.
 */
static inline void cam_pack_header(uint8_t *buf,
                                   uint16_t device_id,
                                   uint32_t jpeg_size)
{
    uint32_t magic = CAM_MAGIC;
    buf[0] = (uint8_t)((magic     >> 24) & 0xFF);
    buf[1] = (uint8_t)((magic     >> 16) & 0xFF);
    buf[2] = (uint8_t)((magic     >>  8) & 0xFF);
    buf[3] = (uint8_t)((magic          ) & 0xFF);
    buf[4] = (uint8_t)((device_id >>  8) & 0xFF);
    buf[5] = (uint8_t)((device_id      ) & 0xFF);
    buf[6] = (uint8_t)((jpeg_size >> 24) & 0xFF);
    buf[7] = (uint8_t)((jpeg_size >> 16) & 0xFF);
    buf[8] = (uint8_t)((jpeg_size >>  8) & 0xFF);
    buf[9] = (uint8_t)((jpeg_size      ) & 0xFF);
}

/**
 * \brief Unpack a cam_header_t from a byte buffer (network byte order).
 *
 * \param[in]  buf       10-byte input buffer.
 * \param[out] out       Pointer to cam_header_t to fill.
 */
static inline void cam_unpack_header(const uint8_t *buf, cam_header_t *out)
{
    out->magic     = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
                     ((uint32_t)buf[2] <<  8) | ((uint32_t)buf[3]);
    out->device_id = ((uint16_t)buf[4] <<  8) | ((uint16_t)buf[5]);
    out->jpeg_size = ((uint32_t)buf[6] << 24) | ((uint32_t)buf[7] << 16) |
                     ((uint32_t)buf[8] <<  8) | ((uint32_t)buf[9]);
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
 * \brief Validate a device ID is in range.
 *
 * \param[in] device_id  Device ID to check.
 * \return               1 if valid, 0 if not.
 */
static inline int cam_device_id_valid(uint16_t device_id)
{
    return (device_id < MAX_DOORBELL_CAMS) ? 1 : 0;
}

#endif /* CAM_LOGIC_H */
