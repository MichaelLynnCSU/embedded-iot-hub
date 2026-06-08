/******************************************************************************
 * \file cam_logic.h
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 *
 * \brief Pure logic for ESP32 doorbell CAM node — no hardware dependencies.
 *
 * \details Extracted from main.c to enable host-side unit testing.
 *          All functions are static inline; no .c file required.
 *
 * \note    Packet header: magic + version + device_id + reserved +
 *          event_id + jpeg_size (20 bytes, packed).
 *          BBB port 9091 shared by all doorbell cams.
 *          Device ID set at build time via -DDOORBELL_ID=N (0-3).
 *          Max 4 doorbell cams (MAX_DOORBELL_CAMS).
 *
 * \note    Board: AI-Thinker ESP32-CAM (OV2640).
 *          Trigger: GPIO13 button press (active low, internal pull-up disabled
 *          — LED+resistor circuit acts as external pullup).
 *          Destination: BBB port 9091 (separate from S3 PIR cam on 9090).
 *          Hub UDP event: HUB_HOST:HUB_UDP_PORT on each button press.
 *
 * \note    Internal pullup disabled on GPIO13 (GPIO_PULLUP_DISABLE).
 *          External LED+resistor circuit acts as pullup. Internal pullup
 *          caused LED to glow dimly when button not pressed due to weak
 *          leakage current through internal ~45kΩ pullup resistor.
 *
 * \note    Dual-lane event architecture (2026-06-07):
 *          Lane A: UDP event  → hub   (intent, authoritative)
 *          Lane B: TCP JPEG   → BBB   (payload, best-effort)
 *          Neither lane depends on the other for correctness.
 *          event_id is the join key — BBB correlates JPEG to event
 *          without time-window heuristics.
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
#define HUB_HOST            "10.0.0.190"  /**< Hub static IP                  */
#define HUB_UDP_PORT        9092          /**< Hub UDP doorbell event port    */

/*---------------------------------------------------------------------------*/
/* Multi-device config                                                         */
/*---------------------------------------------------------------------------*/
#define MAX_DOORBELL_CAMS   4             /**< Max doorbell cams in system    */
#define CAM_MAGIC           0xCAFEBABEu   /**< Packet magic number            */
#define CAM_HEADER_VERSION  1             /**< TCP header format version      */
#define CAM_HEADER_SIZE     20            /**< TCP header size in bytes       */

#ifndef DOORBELL_ID
#define DOORBELL_ID         0             /**< Default device ID (override    */
#endif                                    /**< at build: -DDOORBELL_ID=N)     */

/**
 * \brief Packed TCP header sent before each JPEG payload.
 *
 * \details BBB uses magic to validate, version to handle future formats,
 *          device_id to route, event_id to correlate with UDP event,
 *          jpeg_size to recv exactly the right number of bytes.
 */
typedef struct {
    uint32_t magic;      /**< CAM_MAGIC — validates packet start             */
    uint8_t  version;    /**< CAM_HEADER_VERSION — protocol version          */
    uint8_t  device_id;  /**< 0-3, identifies which doorbell cam             */
    uint16_t reserved;   /**< alignment padding, set to 0                    */
    uint64_t event_id;   /**< correlation key — matches UDP envelope         */
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
 * \brief Pack a cam_header_t into a 20-byte buffer (network byte order).
 *
 * \param[out] buf       CAM_HEADER_SIZE output buffer.
 * \param[in]  device_id Doorbell device ID (0-3).
 * \param[in]  event_id  Correlation key matching UDP envelope.
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
 * \brief Validate a device ID is in range.
 *
 * \param[in] device_id  Device ID to check.
 * \return               1 if valid, 0 if not.
 */
static inline int cam_device_id_valid(uint8_t device_id)
{
    return (device_id < MAX_DOORBELL_CAMS) ? 1 : 0;
}

/**
 * \brief Generate a correlation-safe event_id from mac tail, boot ms, seq.
 *
 * \details Layout: mac_tail(24b) | boot_ms(24b) | seq(16b) = 64 bits.
 *          Unique across reboots and devices without a UUID library.
 *          mac_tail = last 3 bytes of WiFi STA MAC (set once after WiFi up).
 *          boot_ms  = esp_timer_get_time()/1000 captured at boot.
 *          seq      = monotonic counter, wraps at 65535.
 *
 * \param[in] mac_tail  Lower 24 bits of device MAC address.
 * \param[in] boot_ms   Milliseconds since boot (lower 24 bits used).
 * \param[in] seq       Monotonic sequence counter.
 * \return              64-bit event_id.
 */
static inline uint64_t cam_make_event_id(uint32_t mac_tail,
                                          uint32_t boot_ms,
                                          uint16_t seq)
{
    return ((uint64_t)(mac_tail & 0xFFFFFFUL) << 40) |
           ((uint64_t)(boot_ms  & 0xFFFFFFUL) << 16) |
           ((uint64_t)seq);
}

#endif /* CAM_LOGIC_H */
