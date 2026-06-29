/******************************************************************************
 * \file cam_trigger.c
 * \brief ESP32-CAM UDP trigger implementation.
 *
 * \note  ESP32-CAM UDP trigger (2026-05-31):
 *        On PIR occupied 0->1 transition, send_cam_trigger() fires a UDP
 *        CAPTURE packet to CAM_HOST:CAM_UDP_PORT. ESP32-CAM captures JPEG
 *        and pushes to BeagleBone inference_daemon over persistent TCP.
 *
 * \note  Identity model (2026-06-18):
 *        The camera trigger is a side effect of a PIR event, not a new
 *        event. send_cam_trigger() takes event_id (PIR WROOM event_id)
 *        and zone (PIR slot index) and stamps both into the UDP payload
 *        alongside cam_tx_id so the full camera lifecycle is traceable
 *        back to the originating PIR event with a single grep on event_id.
 *
 *        Wire format (named-field, self-describing):
 *
 *          CAPTURE:event_id=101,cam_tx_id=42,zone=0
 *
 *        Named fields chosen over positional for forward compatibility —
 *        adding a field does not shift existing positions or break parsers
 *        that default missing fields to 0. Packet is ~40 bytes over UDP.
 *
 *        cam_tx_id is a local UDP transport counter — increments on every
 *        trigger attempt so dropped or duplicated packets are visible
 *        independently of the business event identity.
 *
 *        Full trace chain:
 *
 *          [BLE_PIR]    tx_id=144 event_id=101 slot=0
 *          [WROOM]      event_id=101 ingest type=BLE_PIR
 *          [UDP_CAM_TX] cam_tx_id=42 event_id=101 zone=0
 *          [UDP_CAM_RX] cam_tx_id=42 event_id=101 zone=0
 *          [CAM]        event_id=101 capture_start
 *          [CAM]        event_id=101 jpeg_send bytes=9800
 *          [CAM]        event_id=101 clip_done elapsed_ms=10000
 *
 * \note  Host/port moved to network_config.h (2026-06-18):
 *        CAM_HOST and CAM_UDP_PORT were hardcoded here, exposing a network
 *        address in tracked source. Moved to the gitignored network_config.h
 *        alongside all other network topology defines.
 ******************************************************************************/
#include "cam_trigger.h"
#include "network_config.h"
#include "trinity_log.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include <stdio.h>
#include <string.h>

#define CAM_TRIGGER_BUF 64  /**< max trigger payload size in bytes */

static const char *TAG         = "CAM_TRIG";
static uint32_t    g_cam_tx_id = 0; /**< UDP transport counter — increments on
                                      *   every trigger attempt so dropped or
                                      *   duplicated packets are visible       */

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Fire a UDP CAPTURE packet to the ESP32-CAM.
 *
 * \param event_id  PIR WROOM event_id that caused this trigger. Stamped into
 *                  the UDP payload and logged at [UDP_CAM_TX] so the camera
 *                  lifecycle is traceable back to the originating PIR event
 *                  with a single grep on event_id.
 * \param zone      PIR slot index (0-based) that triggered the capture.
 *                  Stamped into payload and logged for multi-zone
 *                  disambiguation.
 *
 * \return void
 *
 * \details Builds named-field payload:
 *
 *            CAPTURE:event_id=<N>,cam_tx_id=<N>,zone=<N>
 *
 *          Opens a SOCK_DGRAM socket, sends the payload to
 *          CAM_HOST:CAM_UDP_PORT, and closes immediately. Logs an error on
 *          socket failure but does not block or retry. cam_tx_id increments
 *          before the socket call so it reflects trigger attempts, not
 *          successes — a gap in the cam_tx_id sequence in the log indicates
 *          a failed send.
 *
 *          CAM_HOST and CAM_UDP_PORT are defined in network_config.h, which
 *          is gitignored. No network addresses appear in tracked source.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void send_cam_trigger(uint64_t event_id, uint8_t zone)
{
    struct sockaddr_in addr    = {0};
    char               buf[CAM_TRIGGER_BUF];
    int                sock    = -1;
    int                buf_len = 0;

    g_cam_tx_id++;

    buf_len = snprintf(buf, sizeof(buf),
                       "CAPTURE:event_id=%llu,cam_tx_id=%u,zone=%d",
                       (unsigned long long)event_id,
                       (unsigned)g_cam_tx_id,
                       (int)zone);

    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(CAM_UDP_PORT);
    inet_pton(AF_INET, CAM_HOST, &addr.sin_addr);

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "[UDP_CAM_TX] cam_tx_id=%u event_id=%llu zone=%d "
                      "— socket() failed",
                 (unsigned)g_cam_tx_id,
                 (unsigned long long)event_id,
                 (int)zone);
        return;
    }

    sendto(sock, buf, (size_t)buf_len, 0,
           (struct sockaddr *)&addr, sizeof(addr));
    close(sock);

    ESP_LOGI(TAG, "[UDP_CAM_TX] cam_tx_id=%u event_id=%llu zone=%d host=%s port=%d",
             (unsigned)g_cam_tx_id,
             (unsigned long long)event_id,
             (int)zone,
             CAM_HOST, CAM_UDP_PORT);

    trinity_log_event("EVENT: CAM_TRIGGER_SENT\n");
}
