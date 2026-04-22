/******************************************************************************
 * \file motor_control.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief ESP32-C3 motor controller application entry point.
 *
 * \details Trinity additions:
 *          - trinity_wdt_init()  called in app_main after trinity_log_init()
 *          - trinity_wdt_add()   called at start of tcp_rx_task and motor_task
 *          - trinity_wdt_kick()  called in each task's main loop
 *          - trinity_log_heap_stats() / trinity_log_task_stats() called
 *            from motor_task every STATS_INTERVAL_MS
 *
 * \note    Battery reporting added (2026-04-08):
 *          battery_init() called in app_main after adc_init() -- shares
 *          g_adc1_handle (ADC_UNIT_1). Battery uses ADC1_CH1 (GPIO1),
 *          knob uses ADC1_CH0 (GPIO0) -- no conflict.
 *          motor_task reads battery every BATT_INTERVAL_MS and sends
 *          JSON back to hub on the client socket:
 *            {"batt_motor": <mV>}
 *          Hub receives this in handle_client() recv path and parses
 *          "batt_motor" key to update its TCP state for BeagleBone relay.
 *          g_client_sock exposed as global so motor_task can write to it.
 *          Protected by g_client_sock_valid flag -- motor_task only sends
 *          when a client is connected.
 *
 *          Divider: Vbat (9V) → 100k → 100k → ┬ → GPIO1 (ADC1_CH1)
 *                                              └ → 100k → GND (always)
 *          Bottom resistor is permanently tied to GND rail -- no switching
 *          GPIO. Quiescent draw is ~27uA, negligible vs WiFi and MCU load.
 *          See battery.c for full design rationale.
 *
 *          WiFi TX burst sag rejection:
 *          WiFi TX pulls 100-300mA causing real transient voltage sag on the
 *          9V rail. Readings more than BATT_SAG_REJECT_MV (200mV) below the
 *          last accepted value are discarded and not sent to the hub. A real
 *          battery cannot decline 200mV in 30 seconds under this load so the
 *          threshold safely separates sag artifacts from genuine discharge.
 *          Rejected readings are logged as warnings for visibility.
 ******************************************************************************/

#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "wifi_secrets.h"
#include "network_config.h"
#include "cJSON.h"
#include "wifi_tx_sweep.h"
#include "trinity_log.h"
#include "battery.h"

#define TCP_PORT             3333
#define RX_BUF_SIZE          512
#define ACCUM_BUF_SIZE       2048
#define TCP_BACKLOG          1
#define TCP_READY_MSG        "READY\n"
#define TCP_READY_LEN        6
#define KNOB_ADC_CHANNEL     ADC_CHANNEL_0
#define MOTOR_PWM_PIN        GPIO_NUM_2
#define MOTOR_IN1_PIN        GPIO_NUM_3
#define MOTOR_IN2_PIN        GPIO_NUM_4
#define PWM_TIMER            LEDC_TIMER_0
#define PWM_MODE             LEDC_LOW_SPEED_MODE
#define PWM_CHANNEL          LEDC_CHANNEL_0
#define PWM_DUTY_RES         LEDC_TIMER_13_BIT
#define PWM_FREQUENCY        2000
#define ADC_MAX_RAW          4095
#define PWM_DUTY_MAX         ((1 << PWM_DUTY_RES) - 1)
#define MOTOR_LOOP_MS        100
#define WIFI_CONNECTED_BIT   BIT0
#define MOTOR_EVENT_BUF_SIZE 48
#define DEFAULT_AWS_LOW      20
#define DEFAULT_AWS_HIGH     35
#define DEFAULT_AVG_TEMP     25
#define TCP_TASK_STACK       6144
#define MOTOR_TASK_STACK     4096
#define TASK_PRIORITY        5
#define TX_SWEEP_MIN         34
#define TX_SWEEP_STEP        4
#define TX_SWEEP_MAX         84
#define RECV_IDLE_DELAY_MS   10
#define RECV_TIMEOUT_MS      30000
#define STATS_INTERVAL_MS    60000u  /**< heap/task stats log interval */
#define ACCEPT_TIMEOUT_SEC   2       /**< accept() wakeup interval for WDT kicks */
#define BATT_INTERVAL_MS     30000u  /**< battery read and report interval */
#define BATT_JSON_BUF_SIZE   48      /**< {"batt_motor": 5000} + null */
#define BATT_SAG_REJECT_MV   200     /**< reject readings more than 200mV below
                                      *   last good value -- WiFi TX burst sag.
                                      *   Real battery decline between 30s reads
                                      *   will never exceed this threshold.      */

static const char *TAG = "MOTOR_CTRL";

static WIFI_TX_SWEEP_T           g_tx_sweep;
static EventGroupHandle_t        g_wifi_eg;
static adc_oneshot_unit_handle_t g_adc1_handle;

static int     g_aws_motor = 0;
static int     g_aws_low   = DEFAULT_AWS_LOW;
static int     g_aws_high  = DEFAULT_AWS_HIGH;
static uint8_t g_avg_temp  = DEFAULT_AVG_TEMP;
static uint16_t g_knob_adc = 0;

/* Shared client socket for motor_task battery send-back.
 * Written by tcp_rx_task under g_client_sock_valid flag.
 * Read by motor_task -- send() is safe from multiple tasks on lwIP. */
static volatile int  g_client_sock       = -1;
static volatile bool g_client_sock_valid = false;

/* Last accepted battery reading in mV. Used to detect and reject WiFi TX
 * burst sag artifacts. A reading more than BATT_SAG_REJECT_MV below this
 * value is discarded and not sent to the hub. Initialised to 0 (no reading
 * yet) -- first reading is always accepted regardless of value.           */
static int g_batt_last_good_mv = 0;

static void wifi_event_handler(void *p_arg,
                                esp_event_base_t base,
                                int32_t id,
                                void *p_data)
{
   ip_event_got_ip_t *p_event = NULL;

   (void)p_arg;

   if ((WIFI_EVENT == base) && (WIFI_EVENT_STA_START == id))
   {
      (void)esp_wifi_connect();
   }
   else if ((WIFI_EVENT == base) && (WIFI_EVENT_STA_DISCONNECTED == id))
   {
      wifi_tx_sweep_on_disconnect(&g_tx_sweep);
      (void)esp_wifi_connect();
   }
   else if ((IP_EVENT == base) && (IP_EVENT_STA_GOT_IP == id))
   {
      p_event = (ip_event_got_ip_t *)p_data;
      ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&p_event->ip_info.ip));
      (void)xEventGroupSetBits(g_wifi_eg, WIFI_CONNECTED_BIT);
      trinity_log_event("EVENT: WIFI_CONNECTED\n");
   }
}

static void wifi_init(void)
{
   esp_netif_t        *p_netif  = NULL;
   esp_netif_ip_info_t ip_info;
   wifi_init_config_t  cfg      = WIFI_INIT_CONFIG_DEFAULT();
   wifi_config_t       wifi_cfg = {0};

   g_wifi_eg = xEventGroupCreate();

   (void)esp_netif_init();
   (void)esp_event_loop_create_default();

   p_netif = esp_netif_create_default_wifi_sta();
   (void)esp_netif_dhcpc_stop(p_netif);

   (void)memset(&ip_info, 0, sizeof(ip_info));
   ip_info.ip.addr      = ipaddr_addr(STATIC_IP);
   ip_info.gw.addr      = ipaddr_addr(STATIC_GW);
   ip_info.netmask.addr = ipaddr_addr(STATIC_NETMASK);
   (void)esp_netif_set_ip_info(p_netif, &ip_info);

   (void)esp_wifi_init(&cfg);
   (void)esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
   (void)esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

   strncpy((char *)wifi_cfg.sta.ssid, WIFI_SSID, sizeof(wifi_cfg.sta.ssid) - 1);
   wifi_cfg.sta.ssid[sizeof(wifi_cfg.sta.ssid) - 1] = '\0';
   strncpy((char *)wifi_cfg.sta.password, WIFI_PASS, sizeof(wifi_cfg.sta.password) - 1);
   wifi_cfg.sta.password[sizeof(wifi_cfg.sta.password) - 1] = '\0';

   (void)esp_wifi_set_mode(WIFI_MODE_STA);
   (void)esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
   (void)esp_wifi_start();
}

static void parse_tcp_json(const char *p_buf)
{
   static uint32_t update_log_count = 0;
   cJSON  *p_json  = NULL;
   cJSON  *p_low   = NULL;
   cJSON  *p_high  = NULL;
   cJSON  *p_motor = NULL;
   cJSON  *p_temp  = NULL;
   char    buf[MOTOR_EVENT_BUF_SIZE] = {0};

   p_json = cJSON_Parse(p_buf);
   if (NULL == p_json) { return; }

   p_low   = cJSON_GetObjectItem(p_json, "low");
   p_high  = cJSON_GetObjectItem(p_json, "high");
   p_motor = cJSON_GetObjectItem(p_json, "motor");
   p_temp  = cJSON_GetObjectItem(p_json, "avg_temp");

   if (NULL != p_low)   { g_aws_low  = p_low->valueint;  }
   if (NULL != p_high)  { g_aws_high = p_high->valueint; }
   if (NULL != p_motor)
   {
      g_aws_motor = p_motor->valueint;
      (void)snprintf(buf, sizeof(buf), "EVENT: MOTOR=%d\n", g_aws_motor);
      trinity_log_event(buf);
   }
   if (NULL != p_temp)  { g_avg_temp = (uint8_t)p_temp->valueint; }

   if (++update_log_count % 20u == 0) {
   ESP_LOGI(TAG, "UPDATED: motor=%d low=%d high=%d temp=%d",
            g_aws_motor, g_aws_low, g_aws_high, g_avg_temp);
   }
   cJSON_Delete(p_json);
}

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
      else if ('}' == *p) { depth--; if (0 == depth) { *p_end = p + 1; return p_start; } }
   }

   return NULL;
}

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

   /* Expose socket to motor_task for battery send-back */
   g_client_sock       = sock;
   g_client_sock_valid = true;

   last_rx_tick = xTaskGetTickCount() * portTICK_PERIOD_MS;

   while (!socket_dead)
   {
      /* ---- Trinity: kick WDT while serving client ---- */
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
            (void)gpio_set_level(MOTOR_IN1_PIN, 1);
            (void)gpio_set_level(MOTOR_IN2_PIN, 0);
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
         if (NULL == p_obj_start) { (void)memset(accum, 0, sizeof(accum)); accum_len = 0; break; }
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

   /* Invalidate shared socket before closing */
   g_client_sock_valid = false;
   g_client_sock       = -1;

   trinity_log_event("EVENT: TCP_DISCONNECT\n");
}

static void tcp_rx_task(void *p_arg)
{
   struct sockaddr_in addr;
   int listen_sock = -1;
   int sock        = -1;
   int opt         = 1;

   (void)p_arg;

   /* ---- Trinity: register with task WDT ---- */
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
      /* ---- Trinity: kick WDT on every accept() timeout (~2 s) ---- */
      trinity_wdt_kick();

      sock = accept(listen_sock, NULL, NULL);
      if (0 > sock)
      {
         if ((EAGAIN == errno) || (EWOULDBLOCK == errno))
         {
            continue;
         }
         vTaskDelay(pdMS_TO_TICKS(100));
         continue;
      }

      handle_client(sock);
      close(sock);
      sock = -1;
   }
}

static void motor_task(void *p_arg)
{
   uint32_t pwm_duty      = 0;
   int      adc_raw       = 0;
   int      t             = 0;
   uint32_t last_stats_ms = 0u;
   uint32_t last_batt_ms  = 0u;
   uint32_t now_ms        = 0u;
   int      batt_mv       = -1;
   char     batt_json[BATT_JSON_BUF_SIZE] = {0};
   int      sock          = -1;

   (void)p_arg;

   /* ---- Trinity: register with task WDT ---- */
   trinity_wdt_add();

   (void)xEventGroupWaitBits(g_wifi_eg, WIFI_CONNECTED_BIT,
                               pdFALSE, pdTRUE, portMAX_DELAY);

   last_stats_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
   last_batt_ms  = xTaskGetTickCount() * portTICK_PERIOD_MS;

   while (1)
   {
      /* ---- Trinity: kick WDT every motor loop iteration ---- */
      trinity_wdt_kick();

      pwm_duty = 0;

      if (0 == g_aws_motor)
      {
         (void)adc_oneshot_read(g_adc1_handle, KNOB_ADC_CHANNEL, &adc_raw);
         g_knob_adc = (uint16_t)(ADC_MAX_RAW - adc_raw);
         pwm_duty   = (g_knob_adc * PWM_DUTY_MAX) / ADC_MAX_RAW;
      }
      else
      {
         t = (int)g_avg_temp;
         if (t < g_aws_low)  { t = g_aws_low;  }
         if (t > g_aws_high) { t = g_aws_high; }
         if (g_aws_high > g_aws_low)
         {
            pwm_duty = PWM_DUTY_MAX *
                       (uint32_t)(t - g_aws_low) /
                       (uint32_t)(g_aws_high - g_aws_low);
         }
      }

      (void)ledc_set_duty(PWM_MODE, PWM_CHANNEL, pwm_duty);
      (void)ledc_update_duty(PWM_MODE, PWM_CHANNEL);

      now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

      /* ---- Battery read and send-back every BATT_INTERVAL_MS ---- */
      if ((now_ms - last_batt_ms) >= BATT_INTERVAL_MS)
      {
         last_batt_ms = now_ms;

         batt_mv = battery_read_mv();
         if (0 < batt_mv)
         {
            /* Reject WiFi TX burst sag artifacts. WiFi TX pulls 100-300mA
             * causing real but transient voltage sag on the 9V rail. A
             * genuine battery cannot drop more than BATT_SAG_REJECT_MV in
             * 30 seconds under this load -- any reading that does is a sag
             * artifact and is discarded. First reading (last_good == 0) is
             * always accepted to establish the baseline.                   */
            if ((g_batt_last_good_mv > 0) &&
                (batt_mv < (g_batt_last_good_mv - BATT_SAG_REJECT_MV)))
            {
               ESP_LOGW(TAG, "[BATT] Rejected sag reading %d mV (last good %d mV)",
                        batt_mv, g_batt_last_good_mv);
            }
            else
            {
               g_batt_last_good_mv = batt_mv;
               ESP_LOGI(TAG, "[BATT] %d mV", batt_mv);

               /* Send battery reading back to hub as JSON if connected */
               if (g_client_sock_valid)
               {
                  sock = g_client_sock;
                  (void)snprintf(batt_json, sizeof(batt_json),
                                 "{\"batt_motor\":%d}", batt_mv);
                  (void)send(sock, batt_json, strlen(batt_json), 0);
                  ESP_LOGI(TAG, "[BATT] Sent to hub: %s", batt_json);
               }
            }
         }
         else
         {
            ESP_LOGW(TAG, "[BATT] Read failed");
         }
      }

      /* ---- Trinity: heap and task stats every STATS_INTERVAL_MS ---- */
      if ((now_ms - last_stats_ms) >= STATS_INTERVAL_MS)
      {
         last_stats_ms = now_ms;
         trinity_log_heap_stats();
         trinity_log_task_stats();
      }

      vTaskDelay(pdMS_TO_TICKS(MOTOR_LOOP_MS));
   }
}

static void pwm_init(void)
{
   ledc_timer_config_t timer =
   {
      .speed_mode = PWM_MODE, .timer_num = PWM_TIMER,
      .duty_resolution = PWM_DUTY_RES, .freq_hz = PWM_FREQUENCY,
      .clk_cfg = LEDC_AUTO_CLK,
   };
   ledc_channel_config_t channel =
   {
      .speed_mode = PWM_MODE, .channel = PWM_CHANNEL,
      .timer_sel = PWM_TIMER, .gpio_num = MOTOR_PWM_PIN, .duty = 0,
   };
   (void)ledc_timer_config(&timer);
   (void)ledc_channel_config(&channel);
}

static void adc_init(void)
{
   adc_oneshot_unit_init_cfg_t adc_cfg = { .unit_id = ADC_UNIT_1 };
   adc_oneshot_chan_cfg_t chan_cfg = { .bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_12 };
   (void)adc_oneshot_new_unit(&adc_cfg, &g_adc1_handle);
   (void)adc_oneshot_config_channel(g_adc1_handle, KNOB_ADC_CHANNEL, &chan_cfg);
}

void app_main(void)
{
   (void)nvs_flash_init();
   trinity_log_dump_previous();
   trinity_log_init();

   ESP_LOGI(TAG, "ESP32-C3 Motor Controller Starting");

   /* ---- Trinity: arm task watchdog ---- */
   trinity_wdt_init();

   wifi_tx_sweep_init(&g_tx_sweep, TX_SWEEP_MIN, TX_SWEEP_STEP, TX_SWEEP_MAX);
   wifi_init();

   (void)gpio_set_direction(MOTOR_IN1_PIN, GPIO_MODE_OUTPUT);
   (void)gpio_set_direction(MOTOR_IN2_PIN, GPIO_MODE_OUTPUT);
   (void)gpio_set_level(MOTOR_IN1_PIN, 0);
   (void)gpio_set_level(MOTOR_IN2_PIN, 0);

   pwm_init();
   adc_init();

   /* Battery init -- shares g_adc1_handle, configures ADC1_CH1 (GPIO1) */
   if (0 != battery_init(g_adc1_handle))
   {
      ESP_LOGW(TAG, "Battery init failed -- batt_motor will not report");
   }

   trinity_log_event("EVENT: HARDWARE_READY\n");

   xTaskCreate(tcp_rx_task, "tcp_rx", TCP_TASK_STACK, NULL, TASK_PRIORITY, NULL);
   xTaskCreate(motor_task,  "motor",  MOTOR_TASK_STACK, NULL, TASK_PRIORITY, NULL);
}
