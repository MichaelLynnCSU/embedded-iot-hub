/******************************************************************************
 * \file    tcp_server.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   TCP server task for ESP32-C3 motor controller node.
 *
 * \details Listens on TCP_PORT, accepts one client at a time, accumulates
 *          framed JSON objects and dispatches them to parse_tcp_json() in
 *          motor_control.c. Exposes g_client_sock for motor_task battery
 *          send-back.
 ******************************************************************************/

#include "tcp_server.h"
#include "motor_control.h"
#include "wifi.h"
#include "main.h"
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "esp_log.h"
#include "trinity_log.h"

#define RX_BUF_SIZE          512
#define ACCUM_BUF_SIZE       2048
#define TCP_BACKLOG          1
#define TCP_READY_MSG        "READY\n"
#define TCP_READY_LEN        6
#define RECV_IDLE_DELAY_MS   10

volatile int  g_client_sock       = -1;
volatile bool g_client_sock_valid = false;

/*----------------------------------------------------------------------------*/

static char *find_json_object(char *p_buf, int p_len, char **p_end)
{
    char *p_start = NULL;
    char *p       = NULL;
    int   depth   = 0;

    *p_end = NULL;

    for (p = p_buf; p < (p_buf + p_len); p++)
    {
        if ('{' == *p) { p_start = p; break; }
    }

    if (NULL == p_start) { return NULL; }

    for (p = p_start; p < (p_buf + p_len); p++)
    {
        if      ('{' == *p) { depth++; }
        else if ('}' == *p)
        {
            depth--;
            if (0 == depth) { *p_end = p + 1; return p_start; }
        }
    }

    return NULL;
}

/*----------------------------------------------------------------------------*/

static void handle_client(int sock)
{
    char     rx_buf[RX_BUF_SIZE]      = {0};
    char     accum[ACCUM_BUF_SIZE]    = {0};
    int      accum_len                = 0;
    int      len                      = 0;
    int      flags                    = 0;
    int      yes                      = 1;
    bool     motor_enabled            = false;
    bool     socket_dead              = false;
    uint32_t last_rx_tick             = 0;
    uint32_t now                      = 0;
    char    *p_obj_start              = NULL;
    char    *p_obj_end                = NULL;
    int      obj_len                  = 0;
    char     json_tmp[ACCUM_BUF_SIZE] = {0};

    trinity_log_event("EVENT: TCP_CONNECT\n");

    (void)setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes));
    flags = fcntl(sock, F_GETFL, 0);
    (void)fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    (void)send(sock, TCP_READY_MSG, TCP_READY_LEN, 0);

    g_client_sock       = sock;
    g_client_sock_valid = true;

    last_rx_tick = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while (!socket_dead)
    {
        trinity_wdt_kick();

        while (1)
        {
            len = recv(sock, rx_buf, sizeof(rx_buf) - 1, 0);
            if (0 == len) { socket_dead = true; break; }
            if (0 > len)
            {
                if ((EAGAIN == errno) || (EWOULDBLOCK == errno)) { break; }
                socket_dead = true; break;
            }

            last_rx_tick = xTaskGetTickCount() * portTICK_PERIOD_MS;

            if (!motor_enabled)
            {
                motor_enable();
                motor_enabled = true;
            }

            if ((accum_len + len) < (int)(ACCUM_BUF_SIZE - 1))
            {
                (void)memcpy(accum + accum_len, rx_buf, len);
                accum_len += len;
                accum[accum_len] = '\0';
            }
            else
            {
                trinity_log_event("EVENT: ACCUM_OVERFLOW\n");
                (void)memset(accum, 0, sizeof(accum));
                accum_len = 0;
            }
        }

        if (socket_dead) { break; }

        while (accum_len > 0)
        {
            p_obj_start = find_json_object(accum, accum_len, &p_obj_end);
            if (NULL == p_obj_start)
            {
                (void)memset(accum, 0, sizeof(accum));
                accum_len = 0;
                break;
            }
            if (NULL == p_obj_end)
            {
                if (p_obj_start > accum)
                {
                    accum_len -= (int)(p_obj_start - accum);
                    (void)memmove(accum, p_obj_start, accum_len);
                    accum[accum_len] = '\0';
                }
                break;
            }

            obj_len = (int)(p_obj_end - p_obj_start);
            if (obj_len < (int)(ACCUM_BUF_SIZE - 1))
            {
                (void)memcpy(json_tmp, p_obj_start, obj_len);
                json_tmp[obj_len] = '\0';
                parse_tcp_json(json_tmp);
            }

            accum_len -= (int)(p_obj_end - accum);
            if (accum_len > 0) { (void)memmove(accum, p_obj_end, accum_len); }
            else               { accum_len = 0; }
            accum[accum_len] = '\0';
        }

        now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if ((now - last_rx_tick) > RECV_TIMEOUT_MS)
        {
            trinity_log_event("EVENT: TCP_CLIENT_TIMEOUT\n");
            socket_dead = true;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(RECV_IDLE_DELAY_MS));
    }

    g_client_sock_valid = false;
    g_client_sock       = -1;

    trinity_log_event("EVENT: TCP_DISCONNECT\n");
}

/*----------------------------------------------------------------------------*/

void tcp_rx_task(void *p_arg)
{
    struct sockaddr_in addr;
    int listen_sock = -1;
    int sock        = -1;
    int opt         = 1;

    (void)p_arg;

    trinity_wdt_add();

    (void)xEventGroupWaitBits(g_wifi_eg, WIFI_CONNECTED_BIT,
                               pdFALSE, pdTRUE, portMAX_DELAY);

    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(TCP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (0 > listen_sock)
    {
        trinity_log_event("EVENT: TCP_SOCKET_FAIL\n");
        vTaskDelete(NULL);
        return;
    }

    (void)setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (0 > bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)))
    {
        trinity_log_event("EVENT: TCP_BIND_FAIL\n");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    if (0 > listen(listen_sock, TCP_BACKLOG))
    {
        trinity_log_event("EVENT: TCP_LISTEN_FAIL\n");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    trinity_log_event("EVENT: TCP_LISTENING\n");

    {
        struct timeval accept_tv = { .tv_sec = ACCEPT_TIMEOUT_SEC, .tv_usec = 0 };
        (void)setsockopt(listen_sock, SOL_SOCKET, SO_RCVTIMEO,
                         &accept_tv, sizeof(accept_tv));
    }

    while (1)
    {
        trinity_wdt_kick();

        sock = accept(listen_sock, NULL, NULL);
        if (0 > sock)
        {
            if ((EAGAIN == errno) || (EWOULDBLOCK == errno)) { continue; }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        handle_client(sock);
        close(sock);
        sock = -1;
    }
}
