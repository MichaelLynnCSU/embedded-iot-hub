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
 *
 * \note    WDT fix (2026-05-04):
 *          portMAX_DELAY wait on WIFI_CONNECTED_BIT replaced with a
 *          1-second timed loop that kicks the WDT each iteration.
 *          The WDT is armed at 5s; WiFi reconnect after beacon drop takes
 *          ~9s, so the blocking wait starved the watchdog and caused the
 *          crash loop. Same fix applied to motor_task in motor_control.c.
 *
 * \note    TX sweep fix (2026-05-04):
 *          wifi_service_tx_sweep() called from task context in both the
 *          pre-loop WiFi wait and the accept loop. The sweep previously
 *          called esp_wifi_set_max_tx_power() from the WiFi event handler
 *          (interrupt context), which exceeded the 300ms INT WDT timeout
 *          and caused TG1WDT_SYS_RST. Now the event handler sets a flag
 *          only; this task applies the RF power change safely.
 *
 * \note    Reconnect fix (2026-05-04):
 *          wifi_service_reconnect() added alongside wifi_service_tx_sweep()
 *          in every service loop. The event handler sets g_reconnect_pending;
 *          this task calls esp_wifi_connect() from safe task context.
 *          Previously g_reconnect_pending was set but never consumed here,
 *          so the C3 never attempted to reconnect after a beacon drop.
 *
 * \note    Listen socket lifecycle fix (2026-05-04):
 *          On WiFi disconnect WIFI_CONNECTED_BIT is now cleared by the event
 *          handler. tcp_rx_task detects this, closes the listen socket, waits
 *          for reconnect, then re-creates the listen socket. Previously the
 *          task looped forever on accept() against a dead interface.
 *
 * \note    motor_enable() fix (2026-05-05):
 *          motor_enable() was called on the first recv() byte, before JSON
 *          parsing, meaning a {"ping":1} from the hub would incorrectly
 *          enable the motor. Moved into the else branch of command dispatch
 *          so it only fires on real PWM commands.
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
#include "esp_wifi.h"
#include "trinity_log.h"
#include "cJSON.h"
#include "battery.h"

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

        /* If WiFi dropped mid-session, bail immediately so tcp_rx_task
         * can close the listen socket and wait for reconnect. */
        if (!(xEventGroupGetBits(g_wifi_eg) & WIFI_CONNECTED_BIT))
        {
            trinity_log_event("EVENT: TCP_WIFI_LOST\n");
            socket_dead = true;
            break;
        }

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

            /* NOTE: motor_enable() intentionally NOT called here.
             * It is called only when a real PWM command is dispatched,
             * not on every recv -- a ping must not enable the motor. */

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
                cJSON *p_cmd = cJSON_Parse(json_tmp);
                if (NULL != p_cmd)
                {
                   if (NULL != cJSON_GetObjectItem(p_cmd, "ping"))
                   {
                      /* Ping: respond with battery only -- do NOT enable motor */
                      int  batt_mv  = battery_read_mv();
                      int  batt_pct = (batt_mv >= 0) ? (int)mv_to_soc(batt_mv) : -1;
                      char pong[32] = {0};
                      (void)snprintf(pong, sizeof(pong), "{\"batt_motor\":%d}", batt_pct);
                      (void)send(sock, pong, strlen(pong), 0);
                      ESP_LOGI("TCP", "[PING] batt_motor=%d%%", batt_pct);
                   }
                   else
                   {
                      /* Real PWM command -- enable motor before applying */
                      if (!motor_enabled)
                      {
                          motor_enable();
                          motor_enabled = true;
                      }
                      parse_tcp_json(json_tmp);
                   }
                   cJSON_Delete(p_cmd);
                }
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

/* Wait for WIFI_CONNECTED_BIT, kicking WDT and servicing reconnect + sweep
 * each second. Returns only once the bit is set. */
static void wait_for_wifi(void)
{
    static uint32_t s_wait_ticks = 0;

    ESP_LOGI("TCP", "[WAIT_WIFI] entering wait loop");
    s_wait_ticks = 0;

    while (!(xEventGroupWaitBits(g_wifi_eg, WIFI_CONNECTED_BIT,
                                  pdFALSE, pdTRUE,
                                  pdMS_TO_TICKS(1000)) & WIFI_CONNECTED_BIT))
    {
        s_wait_ticks++;
        ESP_LOGI("TCP", "[WAIT_WIFI] tick %lu -- calling reconnect+sweep",
                 (unsigned long)s_wait_ticks);
        trinity_wdt_kick();
        wifi_service_reconnect();
        wifi_service_tx_sweep();
    }

    ESP_LOGI("TCP", "[WAIT_WIFI] WIFI_CONNECTED_BIT set after %lu ticks",
             (unsigned long)s_wait_ticks);

    /* Service any flags that fired exactly as the bit was set */
    wifi_service_reconnect();
    wifi_service_tx_sweep();
}

/*----------------------------------------------------------------------------*/

static int open_listen_socket(void)
{
    struct sockaddr_in addr;
    int listen_sock = -1;
    int opt         = 1;
    struct timeval accept_tv = { .tv_sec = ACCEPT_TIMEOUT_SEC, .tv_usec = 0 };

    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(TCP_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (0 > listen_sock)
    {
        trinity_log_event("EVENT: TCP_SOCKET_FAIL\n");
        return -1;
    }

    (void)setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    (void)setsockopt(listen_sock, SOL_SOCKET, SO_RCVTIMEO,
                     &accept_tv, sizeof(accept_tv));

    if (0 > bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)))
    {
        trinity_log_event("EVENT: TCP_BIND_FAIL\n");
        close(listen_sock);
        return -1;
    }

    if (0 > listen(listen_sock, TCP_BACKLOG))
    {
        trinity_log_event("EVENT: TCP_LISTEN_FAIL\n");
        close(listen_sock);
        return -1;
    }

    ESP_LOGI("TCP", "[LISTEN] socket open, fd=%d", listen_sock);
    trinity_log_event("EVENT: TCP_LISTENING\n");
    return listen_sock;
}

/*----------------------------------------------------------------------------*/

void tcp_rx_task(void *p_arg)
{
    int listen_sock = -1;
    int sock        = -1;

    (void)p_arg;

    trinity_wdt_add();

    while (1)
    {
        /* Block here (with WDT kicks) until WiFi is up */
        wait_for_wifi();

        /* Open a fresh listen socket every time WiFi (re)connects */
        listen_sock = open_listen_socket();
        if (0 > listen_sock)
        {
            /* Socket setup failed -- wait a moment and retry */
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        /* Accept loop -- exits when WiFi drops or a fatal socket error */
        while (1)
        {
            trinity_wdt_kick();
            wifi_service_reconnect();
            wifi_service_tx_sweep();

            /* If WiFi dropped, close listen socket and go back to wait */
            if (!(xEventGroupGetBits(g_wifi_eg) & WIFI_CONNECTED_BIT))
            {
                ESP_LOGW("TCP", "[ACCEPT_LOOP] WIFI_CONNECTED_BIT lost -- closing listen sock");
                trinity_log_event("EVENT: TCP_WIFI_LOST\n");
                close(listen_sock);
                listen_sock = -1;
                break;
            }

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
}
