/******************************************************************************
 * \file cam_trigger.c
 * \brief ESP32-CAM UDP trigger implementation.
 *
 * \note  ESP32-CAM UDP trigger (2026-05-31):
 *        On PIR occupied 0->1 transition, send_cam_trigger() fires a UDP
 *        CAPTURE packet to CAM_HOST:CAM_UDP_PORT.  ESP32-CAM captures JPEG
 *        and pushes to BeagleBone inference_daemon over persistent TCP.
 ******************************************************************************/

#include "cam_trigger.h"
#include "trinity_log.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include <string.h>

#define CAM_HOST     "10.0.0.222"
#define CAM_UDP_PORT 9091

static const char *TAG = "CAM_TRIG";

/*----------------------------------------------------------------------------*/

void send_cam_trigger(void)
{
    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(CAM_UDP_PORT);
    inet_pton(AF_INET, CAM_HOST, &addr.sin_addr);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "CAM trigger: socket() failed");
        return;
    }
    sendto(sock, "CAPTURE", 7, 0,
           (struct sockaddr *)&addr, sizeof(addr));
    close(sock);
    ESP_LOGI(TAG, "CAM trigger sent to %s:%d", CAM_HOST, CAM_UDP_PORT);
    trinity_log_event("EVENT: CAM_TRIGGER_SENT\n");
}
