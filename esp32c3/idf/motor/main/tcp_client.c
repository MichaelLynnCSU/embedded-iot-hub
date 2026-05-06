/******************************************************************************
 * \file    tcp_client.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   TCP client task for ESP32-C3 motor controller node.
 *
 * \details Replaces tcp_server.c. The motor node is now the TCP *client* --
 *          it connects outbound to the hub's motor listen port on each wake
 *          cycle, receives a {"pwm": X} command, sends back {"batt_motor":Y},
 *          then returns so motor_task can apply the duty and deep sleep.
 *
 *          There is no persistent accept loop and no g_client_sock global.
 *          The connection is opened and closed entirely within
 *          tcp_client_exchange(), which motor_task calls once per wake.
 *
 * \note    Arch change (2026-05-XX):
 *          Hub flipped to TCP server for motor (tcp_manager.c). Motor wakes
 *          from deep sleep, connects, exchanges one frame, disconnects.
 *          listen socket, tcp_rx_task, READY banner, and handle_client()
 *          are all removed. ping path also gone -- hub no longer needs to
 *          initiate contact; motor self-reports on every wake.
 *
 * \note    WDT / WiFi wait:
 *          Same kicked-loop pattern as old tcp_server.c wait_for_wifi().
 *          Reconnect + TX sweep serviced each second while waiting.
 *
 * \note    TX sweep / reconnect fix (2026-05-04, carried forward):
 *          wifi_service_reconnect() and wifi_service_tx_sweep() called from
 *          task context only -- never from ISR / event handler.
 ******************************************************************************/

#include "tcp_client.h"
#include "motor_control.h"
#include "wifi.h"
#include "battery.h"
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
#include "cJSON.h"
#include "network_config.h"

static const char *TAG = "TCP_CLIENT";

#define CONNECT_TIMEOUT_MS   5000
/* RECV_TIMEOUT_MS defined in main.h (30000u) -- used as-is for hub reply wait */
#define RX_BUF_SIZE          256

/*----------------------------------------------------------------------------*/

/** \brief Block until WIFI_CONNECTED_BIT is set, kicking WDT each second. */
static void wait_for_wifi(void)
{
    ESP_LOGI(TAG, "[WAIT_WIFI] waiting for connection");

    while (!(xEventGroupWaitBits(g_wifi_eg, WIFI_CONNECTED_BIT,
                                  pdFALSE, pdTRUE,
                                  pdMS_TO_TICKS(1000)) & WIFI_CONNECTED_BIT))
    {
        trinity_wdt_kick();
        wifi_service_reconnect();
        wifi_service_tx_sweep();
    }

    /* Service any flags that fired exactly as the bit was set */
    wifi_service_reconnect();
    wifi_service_tx_sweep();
    ESP_LOGI(TAG, "[WAIT_WIFI] connected");
}

/*----------------------------------------------------------------------------*/

/**
 * \brief  Open a blocking TCP connection to the hub's motor server port.
 *
 * \return Connected socket fd on success, -1 on failure.
 *
 * \details Uses a non-blocking connect + select() so the WDT can be kicked
 *          while waiting for the handshake to complete.
 */
static int open_connection(void)
{
    struct sockaddr_in addr =
    {
        .sin_family      = AF_INET,
        .sin_port        = htons(HUB_MOTOR_PORT),
        .sin_addr.s_addr = inet_addr(HUB_MOTOR_IP),
    };
    int       sock  = -1;
    int       flags = 0;
    int       ret   = 0;
    int       err   = 0;
    socklen_t el    = sizeof(err);
    fd_set    fds;
    struct timeval tv;

    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (0 > sock)
    {
        ESP_LOGW(TAG, "socket() failed (errno=%d)", errno);
        return -1;
    }

    flags = fcntl(sock, F_GETFL, 0);
    (void)fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    ESP_LOGI(TAG, "Connecting to hub %s:%d", HUB_MOTOR_IP, HUB_MOTOR_PORT);
    ret = connect(sock, (struct sockaddr *)&addr, sizeof(addr));

    if (0 == ret)
    {
        /* Immediate connect (loopback / same subnet) */
        ESP_LOGI(TAG, "Connected (immediate)");
        trinity_log_event("EVENT: TCP_MOTOR_CONNECTED\n");
        return sock;
    }

    if (EINPROGRESS != errno)
    {
        ESP_LOGW(TAG, "connect() failed (errno=%d)", errno);
        close(sock);
        return -1;
    }

    FD_ZERO(&fds);
    FD_SET(sock, &fds);
    tv.tv_sec  = CONNECT_TIMEOUT_MS / 1000;
    tv.tv_usec = (CONNECT_TIMEOUT_MS % 1000) * 1000;

    if (0 >= select(sock + 1, NULL, &fds, NULL, &tv))
    {
        ESP_LOGW(TAG, "connect timed out after %d ms", CONNECT_TIMEOUT_MS);
        close(sock);
        return -1;
    }

    (void)getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, &el);
    if (0 != err)
    {
        ESP_LOGW(TAG, "connect SO_ERROR=%d", err);
        close(sock);
        return -1;
    }

    ESP_LOGI(TAG, "Connected via select()");
    trinity_log_event("EVENT: TCP_MOTOR_CONNECTED\n");
    return sock;
}

/*----------------------------------------------------------------------------*/

/**
 * \brief  Receive one JSON frame from the hub with a timeout.
 *
 * \param  sock      Connected socket fd.
 * \param  p_buf     Caller-supplied buffer.
 * \param  buf_size  Size of p_buf.
 *
 * \return Number of bytes placed in p_buf (NUL-terminated), or -1 on error.
 */
static int recv_frame(int sock, char *p_buf, int buf_size)
{
    fd_set fds;
    struct timeval tv;
    int rlen = 0;

    FD_ZERO(&fds);
    FD_SET(sock, &fds);
    tv.tv_sec  = RECV_TIMEOUT_MS / 1000;
    tv.tv_usec = (RECV_TIMEOUT_MS % 1000) * 1000;

    if (0 >= select(sock + 1, &fds, NULL, NULL, &tv))
    {
        ESP_LOGW(TAG, "recv timed out after %d ms", RECV_TIMEOUT_MS);
        return -1;
    }

    rlen = recv(sock, p_buf, buf_size - 1, 0);
    if (rlen <= 0)
    {
        ESP_LOGW(TAG, "recv() returned %d (errno=%d)", rlen, errno);
        return -1;
    }

    p_buf[rlen] = '\0';
    return rlen;
}

/*----------------------------------------------------------------------------*/

/**
 * \brief  Single wake-cycle exchange with the hub.
 *
 * \details Connects, receives {"pwm": X}, applies duty via parse_tcp_json(),
 *          sends back current battery SOC, then closes the socket.
 *          Called once from motor_task() after WiFi is up.
 *
 * \return Duty value applied (0 if connection failed or pwm=0 received).
 */
uint32_t tcp_client_exchange(void)
{
    int   sock     = -1;
    char  rx[RX_BUF_SIZE] = {0};
    char  tx[32]   = {0};
    int   rlen     = 0;
    int   batt_mv  = -1;
    int   batt_pct = -1;
    uint32_t duty  = 0u;

    sock = open_connection();
    if (0 > sock)
    {
        ESP_LOGW(TAG, "Hub unreachable -- skipping exchange");
        return 0u;
    }

    /* Receive PWM command from hub */
    rlen = recv_frame(sock, rx, sizeof(rx));
    if (rlen <= 0)
    {
        trinity_log_event("EVENT: TCP_MOTOR_RECV_FAIL\n");
        close(sock);
        return 0u;
    }

    ESP_LOGI(TAG, "RX: %s", rx);

    /* Parse and apply -- parse_tcp_json() updates g_pwm_duty internally */
    parse_tcp_json(rx);
    duty = get_pwm_duty();

    if (duty > 0u)
    {
        motor_enable();
    }

    /* Report battery SOC back to hub */
    batt_mv  = battery_read_mv();
    batt_pct = (batt_mv > 0) ? (int)mv_to_soc(batt_mv) : -1;
    (void)snprintf(tx, sizeof(tx), "{\"batt_motor\":%d}", batt_pct);
    (void)send(sock, tx, strlen(tx), 0);
    ESP_LOGI(TAG, "TX: %s (duty=%lu)", tx, (unsigned long)duty);

    close(sock);
    trinity_log_event("EVENT: TCP_MOTOR_DONE\n");
    return duty;
}

/*----------------------------------------------------------------------------*/

/**
 * \brief  Wait for WiFi, run one exchange, return.
 *
 * \details Called from motor_task() in the wake-up path.
 *          Does NOT loop -- motor_task owns the sleep/wake cycle.
 */
void tcp_client_run_once(void)
{
    wait_for_wifi();
    (void)tcp_client_exchange();
}
