/******************************************************************************
 * \file motor_server.c
 * \brief Hub-side TCP server for the ESP32-C3 motor controller.
 *
 * \note  Motor server flip (2026-05-XX): see motor_server.h for full history.
 *
 * \note  Structured event tracing -- tx_id (2026-06-16):
 *        tx_id extracted from motor batt reply JSON. Logged alongside
 *        batt_motor. Defaults to 0 if field absent (old firmware).
 *        tx_id is a per-wake-session counter (RAM only on device, resets
 *        each boot). Correlation key is (IP + tx_id) within a bounded
 *        time window. It is NOT a global unique message ID.
 ******************************************************************************/

#include "motor_server.h"
#include "network_config.h"
#include "config.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "cJSON.h"
#include "trinity_log.h"
#include "wroom_bus.h"
#include "motor_sm.h"
#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <errno.h>
#include <string.h>

#define SOCK_INVALID                -1
#define MOTOR_ACCEPT_TIMEOUT_MS     50
#define MOTOR_BATT_RECV_TIMEOUT_MS  500

static const char *TAG = "MOTOR_SRV";

/*----------------------------------------------------------------------------*/

int open_motor_listen_socket(void)
{
    struct sockaddr_in addr =
    {
        .sin_family      = AF_INET,
        .sin_port        = htons(HUB_MOTOR_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    int sock  = SOCK_INVALID;
    int opt   = 1;
    int flags = 0;

    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (0 > sock)
    {
        ESP_LOGE(TAG, "socket() failed (errno=%d)", errno);
        return SOCK_INVALID;
    }

    (void)setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (0 > bind(sock, (struct sockaddr *)&addr, sizeof(addr)))
    {
        ESP_LOGE(TAG, "bind() failed (errno=%d)", errno);
        close(sock);
        return SOCK_INVALID;
    }

    if (0 > listen(sock, 1))
    {
        ESP_LOGE(TAG, "listen() failed (errno=%d)", errno);
        close(sock);
        return SOCK_INVALID;
    }

    flags = fcntl(sock, F_GETFL, 0);
    (void)fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI(TAG, "Listening on port %d", HUB_MOTOR_PORT);
    trinity_log_event("EVENT: MOTOR_SRV_LISTENING\n");
    return sock;
}

/*----------------------------------------------------------------------------*/

int accept_motor_connection(int listen_sock)
{
    int client = SOCK_INVALID;

    if (SOCK_INVALID == listen_sock) { return SOCK_INVALID; }

    client = accept(listen_sock, NULL, NULL);
    if (0 > client)
    {
        if ((EAGAIN == errno) || (EWOULDBLOCK == errno)) { return SOCK_INVALID; }
        ESP_LOGW(TAG, "accept() error (errno=%d)", errno);
        return SOCK_INVALID;
    }

    ESP_LOGI(TAG, "Motor connected fd=%d", client);
    trinity_log_event("EVENT: TCP_MOTOR_ACCEPTED\n");
    return client;
}

/*----------------------------------------------------------------------------*/

void send_pwm_to_c3(int client_sock, float pwm_pct,
                    uint8_t *p_motor_online, int *p_motor_batt)
{
    cJSON      *p_root    = NULL;
    cJSON      *p_rx_json = NULL;
    cJSON      *p_batt    = NULL;
    cJSON      *p_tx_id   = NULL;
    char       *p_msg     = NULL;
    char        rx[64]    = {0};
    int         duty      = 0;
    int         sent      = 0;
    int         rlen      = 0;
    uint16_t    tx_id     = 0u; /**< device-stamped sequence counter;
                                 *   0 = old firmware, no tx_id in payload */
    fd_set      fds;
    struct timeval tv;

    if (SOCK_INVALID == client_sock) { return; }

    duty   = (int)((pwm_pct / PWM_OUT_MAX) * (float)PWM_DUTY_MAX);
    p_root = cJSON_CreateObject();
    if (NULL == p_root)
    {
        ESP_LOGE(TAG, "cJSON root alloc failed");
        close(client_sock);
        return;
    }

    (void)cJSON_AddNumberToObject(p_root, "pwm", duty);
    p_msg = cJSON_PrintUnformatted(p_root);
    cJSON_Delete(p_root);
    if (NULL == p_msg)
    {
        ESP_LOGE(TAG, "cJSON serialize failed");
        close(client_sock);
        return;
    }

    sent = send(client_sock, p_msg, strlen(p_msg), 0);
    cJSON_free(p_msg);

    if (0 > sent)
    {
        ESP_LOGW(TAG, "send() failed (errno=%d) -- motor offline", errno);
        trinity_log_event("EVENT: TCP_MOTOR_SEND_FAIL\n");
        *p_motor_online = 0;
        bus_publish_motor(0, -1);
        close(client_sock);
        return;
    }

    ESP_LOGI(TAG, "Sent pwm=%d (%.1f%%)", duty, pwm_pct);

    FD_ZERO(&fds);
    FD_SET(client_sock, &fds);
    tv.tv_sec  = MOTOR_BATT_RECV_TIMEOUT_MS / 1000;
    tv.tv_usec = (MOTOR_BATT_RECV_TIMEOUT_MS % 1000) * 1000;

    if (0 < select(client_sock + 1, &fds, NULL, NULL, &tv))
    {
        rlen = recv(client_sock, rx, sizeof(rx) - 1, 0);
        if (rlen > 0)
        {
            rx[rlen]  = '\0';
            p_rx_json = cJSON_Parse(rx);
            if (NULL != p_rx_json)
            {
                p_batt  = cJSON_GetObjectItem(p_rx_json, "batt_motor");
                p_tx_id = cJSON_GetObjectItem(p_rx_json, "tx_id");

                tx_id = (NULL != p_tx_id) ?
                        (uint16_t)p_tx_id->valueint : 0u;

                if ((NULL != p_batt) && (p_batt->valueint >= 0))
                {
                    *p_motor_batt = p_batt->valueint;
                    ESP_LOGI(TAG, "batt_motor=%d%% tx_id=%u",
                             *p_motor_batt, (unsigned)tx_id);
                }
                cJSON_Delete(p_rx_json);
            }
            else
            {
                ESP_LOGW(TAG, "JSON parse failed (rlen=%d) rx='%.*s'",
                         rlen, rlen, rx);
                trinity_log_event("EVENT: MOTOR_BATT_PARSE_FAIL\n");
            }
        }
        else if (0 == rlen)
        {
            ESP_LOGW(TAG, "Motor closed before sending batt reply");
            trinity_log_event("EVENT: MOTOR_BATT_EMPTY_RECV\n");
        }
        else
        {
            ESP_LOGW(TAG, "recv() error (errno=%d)", errno);
            trinity_log_event("EVENT: MOTOR_BATT_RECV_ERR\n");
        }
    }
    else
    {
        ESP_LOGW(TAG, "batt reply timed out after %d ms",
                 MOTOR_BATT_RECV_TIMEOUT_MS);
        trinity_log_event("EVENT: MOTOR_BATT_TIMEOUT\n");
    }

    *p_motor_online = 1;
    bus_publish_motor(1, *p_motor_batt);
    close(client_sock);
}
