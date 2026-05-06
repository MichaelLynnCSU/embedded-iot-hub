/******************************************************************************
 * \file tcp_manager.c
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date 01-01-2025
 *
 * \brief TCP manager for ESP32 hub node.
 *
 * \details Maintains non-blocking TCP connections to BeagleBone and
 *          ESP32-C3 motor controller. Drains vroom bus queues and sends
 *          consolidated JSON payloads every TCP_SEND_INTERVAL_MS.
 *
 *          BeagleBone connection (outbound client, unchanged):
 *          State 0 — disconnected, create socket and initiate connect
 *          State 1 — connect in progress, poll with select()
 *          State 2 — connected, ready to send
 *
 *          Motor connection (inbound server, NEW):
 *          Hub listens on HUB_MOTOR_PORT. Motor wakes from deep sleep,
 *          connects in, receives {"pwm": X}, sends back {"batt_motor": Y},
 *          closes. Hub accept()s each wake-cycle connection, dispatches
 *          send_pwm_to_c3(), reads battery reply, closes client socket.
 *          No persistent C3 client socket. No connect state machine for C3.
 *
 *          Backpressure handling (BeagleBone only, unchanged):
 *          - EAGAIN/EWOULDBLOCK increments block counter
 *          - 5 consecutive blocks triggers force reconnect
 *
 * \note    Phantom widget fix (2026-04-21): (unchanged)
 * \note    WDT fix (2026-03-21): (unchanged)
 * \note    Motor battery (2026-04-08, updated 2026-04-27): (unchanged)
 * \note    PI controller + motor state machine (2026-05-04): (unchanged)
 *
 * \note    Motor server flip (2026-05-XX):
 *          Hub is now TCP server for motor. run_c3_state_machine() and
 *          disconnect_c3() removed. open_motor_listen_socket() and
 *          accept_motor_connection() replace them.
 *          send_pwm_to_c3() updated: takes the accepted client fd directly,
 *          no longer owns a persistent socket.
 *          Ping path removed: ping_motor_for_health() / motor_sm_ping_due()
 *          no longer called. Motor self-reports batt on every wake connection.
 *          motor_online set to 1 on successful accept+send, 0 on listen fail.
 *
 * \note    motor_online ownership (2026-05-05, updated 2026-05-XX):
 *          motor_online owned by:
 *            - successful send_pwm_to_c3()  -> motor_online = 1
 *            - accept() / listen failure    -> motor_online = 0
 *          No longer touched by disconnect_c3() (removed) or ping path
 *          (removed).
 ******************************************************************************/

#include "config.h"
#include "network_config.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <fcntl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <errno.h>
#include <string.h>
#include "ble_manager.h"
#include "uart_manager.h"
#include "tcp_manager.h"
#include "aws_manager.h"
#include "vroom_bus.h"
#include "trinity_log.h"
#include "pi_controller.h"
#include "motor_sm.h"

#define MAX_REEDS               6
#define REED_OFFLINE_S          150
#define BB_CONNECT_TIMEOUT_MS   2000
#define BLOCK_COUNT_MAX         5
#define SOCK_POLL_DELAY_MS      100
#define SOCK_RETRY_DELAY_MS     1000
#define REED_NAME_BUF_SIZE      32
#define SOCK_INVALID            -1
#define TCP_STATE_DISCONNECTED  0
#define TCP_STATE_CONNECTING    1
#define TCP_STATE_CONNECTED     2

/* Motor server accept() timeout -- non-blocking so the main loop continues
 * even when the motor is idle and no connection arrives this cycle.       */
#define MOTOR_ACCEPT_TIMEOUT_MS  50
/* Battery reply read timeout after PWM frame sent */
#define MOTOR_BATT_RECV_TIMEOUT_MS  500

static const char *TAG = "TCP_MGR";

typedef struct
{
   int      batt;
   uint16_t age;
   bool     active;
   uint8_t  state;
   uint8_t  offline;
   uint16_t gen;
} REED_SLOT_STATE_T;

typedef struct
{
   int               avg_temp;
   uint32_t          motion_count;
   int               pir_batt;
   int               pir_occupied;
   uint8_t           light_state;
   uint8_t           lock_state;
   int               lock_batt;
   uint8_t           motor_online;
   int               motor_batt;
   REED_SLOT_STATE_T reed_slots[MAX_REEDS];
   uint16_t          age_pir;
   uint16_t          age_lgt;
   uint16_t          age_lck;
   int               reed_count;
} TCP_STATE_T;

static TCP_STATE_T g_state =
{
   .avg_temp   = DEFAULT_AVG_TEMP,
   .pir_batt   = -1,
   .lock_batt  = -1,
   .motor_batt = -1,
   .age_pir    = 0xFFFF,
   .age_lgt    = 0xFFFF,
   .age_lck    = 0xFFFF,
};

/*----------------------------------------------------------------------------*/

static void drain_queues(BUS_SUBSCRIBER_T sub)
{
   PIR_PAYLOAD_T   p_pir;
   REED_PAYLOAD_T  p_reed;
   LOCK_PAYLOAD_T  p_lock;
   LIGHT_PAYLOAD_T p_light;
   TEMP_PAYLOAD_T  p_temp;
   MOTOR_PAYLOAD_T p_motor;
   int             slot        = 0;
   const char     *p_state_str = NULL;

   /* ---- LOCK first: latency sensitive ---- */
   if (pdTRUE == xQueueReceive(sub.mb_lock, &p_lock, 0))
   {
      g_state.lock_state = p_lock.state;
      g_state.lock_batt  = p_lock.batt;
   }

   /* ---- REED ---- */
   if (pdTRUE == xQueueReceive(sub.mb_reed, &p_reed, 0))
   {
      slot = (int)p_reed.id - 1;
      if ((0 <= slot) && (slot < MAX_REEDS))
      {
         g_state.reed_slots[slot].batt   = p_reed.batt;
         g_state.reed_slots[slot].active = true;
         g_state.reed_slots[slot].state  = p_reed.state;
      }

      if (0 == p_reed.state)      { p_state_str = "closed";  }
      else if (1 == p_reed.state) { p_state_str = "open";    }
      else                        { p_state_str = "unknown"; }

      if (1 == p_reed.id)      { ble_update_room_sensor(1, p_state_str); }
      else if (2 == p_reed.id) { ble_update_room_sensor(7, p_state_str); }
   }

   /* ---- PIR ---- */
   if (pdTRUE == xQueueReceive(sub.mb_pir, &p_pir, 0))
   {
      g_state.motion_count = p_pir.count;
      g_state.pir_batt     = p_pir.batt;
   }

   /* ---- LIGHT ---- */
   if (pdTRUE == xQueueReceive(sub.mb_light, &p_light, 0))
   {
      g_state.light_state = p_light.state;
   }

   /* ---- TEMP ---- */
   if (pdTRUE == xQueueReceive(sub.mb_temp, &p_temp, 0))
   {
      g_state.avg_temp = p_temp.avg_temp;
   }

   /* ---- MOTOR: drain only, motor_online owned by accept/send paths ---- */
   if (pdTRUE == xQueueReceive(sub.mb_motor, &p_motor, 0))
   {
      if (p_motor.batt >= 0)
      {
         g_state.motor_batt = p_motor.batt;
      }
   }

   /* ---- snapshot BLE state after mailboxes drained ---- */
   g_state.pir_occupied = ble_get_pir_occupied();
   g_state.age_pir      = ble_get_device_age_s(BLE_DEV_PIR);
   g_state.age_lgt      = ble_get_device_age_s(BLE_DEV_LIGHT);
   g_state.age_lck      = ble_get_device_age_s(BLE_DEV_LOCK);

   int      count = ble_get_reed_count();
   int      i     = 0;
   uint16_t age   = 0xFFFF;
   uint8_t  slot_state = 0xFF;
   uint16_t gen   = 0;

   for (i = 0; (i < count) && (i < MAX_REEDS); i++)
   {
      age = 0xFFFF; slot_state = 0xFF; gen = 0;
      (void)ble_get_reed_slot_info(i, NULL, NULL, &age, &slot_state, &gen);
      g_state.reed_slots[i].age     = age;
      g_state.reed_slots[i].active  = true;
      g_state.reed_slots[i].offline = (age > REED_OFFLINE_S) ? 1 : 0;
      g_state.reed_slots[i].gen     = gen;
   }

   for (i = 0; i < MAX_REEDS; i++)
   {
      if (g_state.reed_slots[i].active)
      {
         ESP_LOGI(TAG, "[REED] slot=%d state=%d batt=%d age=%d offline=%d gen=%d",
                  i + 1, g_state.reed_slots[i].state, g_state.reed_slots[i].batt,
                  g_state.reed_slots[i].age, g_state.reed_slots[i].offline,
                  g_state.reed_slots[i].gen);
      }
   }
}

/*----------------------------------------------------------------------------*/

static void handle_bb_send_error(int *p_sock, int *p_block_count, int *p_state)
{
   if ((EAGAIN == errno) || (EWOULDBLOCK == errno))
   {
      (*p_block_count)++;
      ESP_LOGW(TAG, "[BEAGLEBONE] Send would block (%d)", *p_block_count);
      if (*p_block_count >= BLOCK_COUNT_MAX)
      {
         trinity_log_event("EVENT: TCP_BB_FORCE_RECONNECT\n");
         close(*p_sock);
         *p_sock = SOCK_INVALID; *p_state = TCP_STATE_DISCONNECTED; *p_block_count = 0;
      }
   }
   else
   {
      trinity_log_event("EVENT: TCP_BB_DISCONNECTED\n");
      close(*p_sock);
      *p_sock = SOCK_INVALID; *p_state = TCP_STATE_DISCONNECTED; *p_block_count = 0;
   }
}

/*----------------------------------------------------------------------------*/

static void send_to_bb(int *p_bb_sock, int *p_bb_block_count, int *p_bb_state)
{
   cJSON   *p_root  = NULL;
   cJSON   *p_reeds = NULL;
   cJSON   *p_rooms = NULL;
   cJSON   *p_entry = NULL;
   char    *p_msg   = NULL;
   char     name[REED_NAME_BUF_SIZE] = {0};
   uint8_t  door_state = 0xFF;
   uint16_t gen        = 0;
   int      i          = 0;
   int      sent       = 0;

   if ((TCP_STATE_CONNECTED != *p_bb_state) || (SOCK_INVALID == *p_bb_sock)) { return; }

   p_root = cJSON_CreateObject();
   if (NULL == p_root) { ESP_LOGE(TAG, "cJSON root alloc failed (BB)"); return; }

   (void)cJSON_AddNumberToObject(p_root, "avg_temp",     g_state.avg_temp);
   (void)cJSON_AddNumberToObject(p_root, "motion_count", g_state.motion_count);
   (void)cJSON_AddNumberToObject(p_root, "light_state",  g_state.light_state);
   (void)cJSON_AddNumberToObject(p_root, "lock_state",   g_state.lock_state);
   (void)cJSON_AddNumberToObject(p_root, "age_pir",      g_state.age_pir);
   (void)cJSON_AddNumberToObject(p_root, "age_lgt",      g_state.age_lgt);
   (void)cJSON_AddNumberToObject(p_root, "age_lck",      g_state.age_lck);
   (void)cJSON_AddNumberToObject(p_root, "batt_pir",     g_state.pir_batt);
   (void)cJSON_AddNumberToObject(p_root, "pir_occupied", g_state.pir_occupied);
   (void)cJSON_AddNumberToObject(p_root, "batt_lck",     g_state.lock_batt);
   (void)cJSON_AddNumberToObject(p_root, "batt_motor",   g_state.motor_batt);
   (void)cJSON_AddNumberToObject(p_root, "motor_online", g_state.motor_online);

   g_state.reed_count = ble_get_reed_count();
   p_reeds = cJSON_CreateArray();
   if (NULL != p_reeds)
   {
      for (i = 0; (i < g_state.reed_count) && (i < MAX_REEDS); i++)
      {
         (void)memset(name, 0, sizeof(name));
         door_state = 0xFF; gen = 0;
         (void)ble_get_reed_slot_info(i, name, NULL, NULL, &door_state, &gen);
         p_entry = cJSON_CreateObject();
         if (NULL != p_entry)
         {
            (void)cJSON_AddNumberToObject(p_entry, "id",      i + 1);
            (void)cJSON_AddNumberToObject(p_entry, "batt",    g_state.reed_slots[i].batt);
            (void)cJSON_AddNumberToObject(p_entry, "age",     g_state.reed_slots[i].age);
            (void)cJSON_AddNumberToObject(p_entry, "state",   door_state);
            (void)cJSON_AddNumberToObject(p_entry, "offline", g_state.reed_slots[i].offline);
            (void)cJSON_AddNumberToObject(p_entry, "gen",     gen);
            (void)cJSON_AddStringToObject(p_entry, "name",    name);
            (void)cJSON_AddItemToArray(p_reeds, p_entry);
         }
      }
      (void)cJSON_AddItemToObject(p_root, "reeds", p_reeds);
   }

   p_rooms = cJSON_CreateArray();
   if (NULL != p_rooms)
   {
      for (i = 0; i < ROOM_COUNT; i++)
      {
         p_entry = cJSON_CreateObject();
         if (NULL != p_entry)
         {
            (void)cJSON_AddNumberToObject(p_entry, "sensor_id", rooms[i].sensor_id);
            (void)cJSON_AddStringToObject(p_entry, "room",      rooms[i].room);
            (void)cJSON_AddStringToObject(p_entry, "state",     rooms[i].state);
            (void)cJSON_AddStringToObject(p_entry, "location",  rooms[i].location);
            (void)cJSON_AddItemToArray(p_rooms, p_entry);
         }
      }
      (void)cJSON_AddItemToObject(p_root, "rooms", p_rooms);
   }

   p_msg = cJSON_PrintUnformatted(p_root);
   cJSON_Delete(p_root);
   if (NULL == p_msg) { ESP_LOGE(TAG, "cJSON serialize failed (BB)"); return; }

   sent = send(*p_bb_sock, p_msg, strlen(p_msg), 0);
   if (0 > sent)
   {
      handle_bb_send_error(p_bb_sock, p_bb_block_count, p_bb_state);
   }
   else
   {
      *p_bb_block_count = 0;
      ESP_LOGI(TAG, "[BEAGLEBONE] tmp=%d pir=%u occ=%d lgt=%d lck=%d reeds=%d mtr=%d batt_mtr=%d batt_pir=%d batt_lck=%d",
               g_state.avg_temp, (unsigned)g_state.motion_count, g_state.pir_occupied,
               g_state.light_state, g_state.lock_state, g_state.reed_count,
               (int)g_state.motor_online, g_state.motor_batt,
               g_state.pir_batt, g_state.lock_batt);
   }
   cJSON_free(p_msg);
}

/*----------------------------------------------------------------------------*/

/**
 * \brief  Open the hub-side motor listen socket.
 *
 * \return Listening socket fd on success, SOCK_INVALID on failure.
 *
 * \details Called once at task startup (and again if the socket is lost).
 *          Bound to HUB_MOTOR_PORT on INADDR_ANY. Non-blocking so the
 *          accept() loop can return immediately when no motor is connecting.
 */
static int open_motor_listen_socket(void)
{
   struct sockaddr_in addr =
   {
      .sin_family      = AF_INET,
      .sin_port        = htons(HUB_MOTOR_PORT),
      .sin_addr.s_addr = htonl(INADDR_ANY),
   };
   int sock = SOCK_INVALID;
   int opt  = 1;
   int flags = 0;

   sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
   if (0 > sock)
   {
      ESP_LOGE(TAG, "[MOTOR_SRV] socket() failed (errno=%d)", errno);
      return SOCK_INVALID;
   }

   (void)setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

   if (0 > bind(sock, (struct sockaddr *)&addr, sizeof(addr)))
   {
      ESP_LOGE(TAG, "[MOTOR_SRV] bind() failed (errno=%d)", errno);
      close(sock);
      return SOCK_INVALID;
   }

   if (0 > listen(sock, 1))
   {
      ESP_LOGE(TAG, "[MOTOR_SRV] listen() failed (errno=%d)", errno);
      close(sock);
      return SOCK_INVALID;
   }

   flags = fcntl(sock, F_GETFL, 0);
   (void)fcntl(sock, F_SETFL, flags | O_NONBLOCK);

   ESP_LOGI(TAG, "[MOTOR_SRV] Listening on port %d", HUB_MOTOR_PORT);
   trinity_log_event("EVENT: MOTOR_SRV_LISTENING\n");
   return sock;
}

/*----------------------------------------------------------------------------*/

/**
 * \brief  Non-blocking accept on the motor listen socket.
 *
 * \param  listen_sock  fd returned by open_motor_listen_socket().
 *
 * \return Accepted client fd, or SOCK_INVALID if no connection pending.
 */
static int accept_motor_connection(int listen_sock)
{
   int client = SOCK_INVALID;

   if (SOCK_INVALID == listen_sock) { return SOCK_INVALID; }

   client = accept(listen_sock, NULL, NULL);
   if (0 > client)
   {
      if ((EAGAIN == errno) || (EWOULDBLOCK == errno)) { return SOCK_INVALID; }
      ESP_LOGW(TAG, "[MOTOR_SRV] accept() error (errno=%d)", errno);
      return SOCK_INVALID;
   }

   ESP_LOGI(TAG, "[MOTOR_SRV] Motor connected fd=%d", client);
   trinity_log_event("EVENT: TCP_MOTOR_ACCEPTED\n");
   return client;
}

/*----------------------------------------------------------------------------*/

/**
 * \brief  Send PWM command to a connected motor client and read batt reply.
 *
 * \param  client_sock  fd from accept_motor_connection() -- closed here.
 * \param  pwm_pct      Effective PWM percent from motor state machine.
 *
 * \details Sends {"pwm": X}, waits up to MOTOR_BATT_RECV_TIMEOUT_MS for
 *          {"batt_motor": Y}, updates g_state, closes client_sock.
 *          motor_online set to 1 on success.
 */
static void send_pwm_to_c3(int client_sock, float pwm_pct)
{
   cJSON      *p_root    = NULL;
   cJSON      *p_rx_json = NULL;
   cJSON      *p_batt    = NULL;
   char       *p_msg     = NULL;
   char        rx[64]    = {0};
   int         duty      = 0;
   int         sent      = 0;
   int         rlen      = 0;
   fd_set      fds;
   struct timeval tv;

   if (SOCK_INVALID == client_sock) { return; }

   duty   = (int)((pwm_pct / PWM_OUT_MAX) * (float)PWM_DUTY_MAX);
   p_root = cJSON_CreateObject();
   if (NULL == p_root) { ESP_LOGE(TAG, "cJSON root alloc failed (C3)"); close(client_sock); return; }

   (void)cJSON_AddNumberToObject(p_root, "pwm", duty);
   p_msg = cJSON_PrintUnformatted(p_root);
   cJSON_Delete(p_root);
   if (NULL == p_msg) { ESP_LOGE(TAG, "cJSON serialize failed (C3)"); close(client_sock); return; }

   sent = send(client_sock, p_msg, strlen(p_msg), 0);
   cJSON_free(p_msg);

   if (0 > sent)
   {
      ESP_LOGW(TAG, "[MOTOR_SRV] send() failed (errno=%d) -- motor offline", errno);
      trinity_log_event("EVENT: TCP_MOTOR_SEND_FAIL\n");
      g_state.motor_online = 0;
      bus_publish_motor(0, -1);
      close(client_sock);
      return;
   }

   ESP_LOGI(TAG, "[MOTOR_SRV] Sent pwm=%d (%.1f%%)", duty, pwm_pct);

   /* Read battery reply with timeout */
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
            p_batt = cJSON_GetObjectItem(p_rx_json, "batt_motor");
            if ((NULL != p_batt) && (p_batt->valueint >= 0))
            {
               g_state.motor_batt = p_batt->valueint;
               ESP_LOGI(TAG, "[MOTOR_SRV] batt_motor=%d%%", g_state.motor_batt);
            }
            cJSON_Delete(p_rx_json);
         }
      }
   }
   else
   {
      ESP_LOGW(TAG, "[MOTOR_SRV] batt reply timed out after %d ms",
               MOTOR_BATT_RECV_TIMEOUT_MS);
   }

   g_state.motor_online = 1;
   bus_publish_motor(1, g_state.motor_batt);
   close(client_sock);
}

/*----------------------------------------------------------------------------*/

static void run_bb_state_machine(int *p_sock, int *p_state, int *p_block_count,
                                  uint32_t *p_connect_start,
                                  struct sockaddr_in *p_addr, uint32_t now)
{
   int flags = 0, ret = 0, err = 0;
   socklen_t el = sizeof(err);
   fd_set writefds, errorfds;
   struct timeval tv;

   switch (*p_state)
   {
      case TCP_STATE_DISCONNECTED:
      {
         if (SOCK_INVALID != *p_sock) { close(*p_sock); *p_sock = SOCK_INVALID; }
         *p_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
         if (0 > *p_sock) { vTaskDelay(pdMS_TO_TICKS(SOCK_RETRY_DELAY_MS)); break; }
         flags = fcntl(*p_sock, F_GETFL, 0);
         ESP_LOGI(TAG, "[BEAGLEBONE] Connecting to %s:%d", BEAGLEBONE_IP, BEAGLEBONE_PORT);
         (void)fcntl(*p_sock, F_SETFL, flags | O_NONBLOCK);
         ret = connect(*p_sock, (struct sockaddr *)p_addr, sizeof(*p_addr));
         if (0 == ret)
         {
            trinity_log_event("EVENT: TCP_BB_CONNECTED\n");
            *p_state = TCP_STATE_CONNECTED; *p_block_count = 0;
            ESP_LOGI(TAG, "[BEAGLEBONE] Connected (immediate)");
         }
         else if (EINPROGRESS == errno)
         {
            ESP_LOGI(TAG, "[BEAGLEBONE] Connect in progress...");
            *p_connect_start = now; *p_state = TCP_STATE_CONNECTING;
         }
         else
         {
            ESP_LOGW(TAG, "[BEAGLEBONE] Connect failed (errno=%d), retrying", errno);
            close(*p_sock); *p_sock = SOCK_INVALID;
            vTaskDelay(pdMS_TO_TICKS(CONNECTION_RETRY_DELAY_MS));
         }
         break;
      }
      case TCP_STATE_CONNECTING:
      {
         if ((now - *p_connect_start) > BB_CONNECT_TIMEOUT_MS)
         {
            ESP_LOGW(TAG, "[BEAGLEBONE] Connect timed out after %d ms", BB_CONNECT_TIMEOUT_MS);
            close(*p_sock); *p_sock = SOCK_INVALID; *p_state = TCP_STATE_DISCONNECTED;
            vTaskDelay(pdMS_TO_TICKS(CONNECTION_RETRY_DELAY_MS)); break;
         }
         FD_ZERO(&writefds); FD_ZERO(&errorfds);
         FD_SET(*p_sock, &writefds); FD_SET(*p_sock, &errorfds);
         tv.tv_sec = 0; tv.tv_usec = 10000;
         if (0 < select(*p_sock + 1, NULL, &writefds, &errorfds, &tv))
         {
            (void)getsockopt(*p_sock, SOL_SOCKET, SO_ERROR, &err, &el);
            if (FD_ISSET(*p_sock, &errorfds) || (0 != err))
            {
               ESP_LOGW(TAG, "[BEAGLEBONE] Connect error (SO_ERROR=%d)", err);
               close(*p_sock); *p_sock = SOCK_INVALID; *p_state = TCP_STATE_DISCONNECTED;
               vTaskDelay(pdMS_TO_TICKS(CONNECTION_RETRY_DELAY_MS));
            }
            else if (FD_ISSET(*p_sock, &writefds))
            {
               trinity_log_event("EVENT: TCP_BB_CONNECTED\n");
               ESP_LOGI(TAG, "[BEAGLEBONE] Connected");
               *p_state = TCP_STATE_CONNECTED; *p_block_count = 0;
            }
         }
         break;
      }
      case TCP_STATE_CONNECTED: break;
      default: *p_state = TCP_STATE_DISCONNECTED; break;
   }
}

/*----------------------------------------------------------------------------*/

void tcp_manager_init(void)
{
   ESP_LOGI(TAG, "TCP manager initialized");
}

void tcp_manager_task(EventGroupHandle_t p_system_eg,
                      EventGroupHandle_t p_wifi_eg,
                      BUS_SUBSCRIBER_T   sub)
{
   int         motor_listen     = SOCK_INVALID;
   int         motor_client     = SOCK_INVALID;
   int         bb_sock          = SOCK_INVALID;
   int         bb_state         = TCP_STATE_DISCONNECTED;
   int         bb_block_count   = 0;
   uint32_t    bb_connect_start = 0;
   uint32_t    now              = 0;
   float       pi_out           = 0.0f;
   float       eff_pwm          = 0.0f;
   bool        do_connect       = false;   /* unused, kept for run_motor_sm signature */
   bool        do_disconnect    = false;

   (void)p_system_eg;
   (void)do_connect;

   struct sockaddr_in bb_addr =
   {
      .sin_family      = AF_INET,
      .sin_port        = htons(BEAGLEBONE_PORT),
      .sin_addr.s_addr = inet_addr(BEAGLEBONE_IP),
   };

   (void)xEventGroupWaitBits(p_wifi_eg, WIFI_CONNECTED_BIT,
                               pdFALSE, pdTRUE, portMAX_DELAY);
   ESP_LOGI(TAG, "TCP task running");

   /* Open motor listen socket once -- kept open for the task lifetime. */
   motor_listen = open_motor_listen_socket();
   if (SOCK_INVALID == motor_listen)
   {
      ESP_LOGE(TAG, "[MOTOR_SRV] Failed to open listen socket -- motor offline");
      g_state.motor_online = 0;
   }

   while (1)
   {
      trinity_wdt_kick();
      now = xTaskGetTickCount() * portTICK_PERIOD_MS;

      pi_out  = run_pi_controller(now);
      eff_pwm = run_motor_sm(pi_out, now, &do_connect, &do_disconnect);

      /* Motor server: non-blocking accept each loop iteration.
       * If the motor woke and connected, send current PWM and read batt.
       * do_disconnect from SM just drives eff_pwm=0 -- listen stays open. */
      if (SOCK_INVALID != motor_listen)
      {
         motor_client = accept_motor_connection(motor_listen);
         if (SOCK_INVALID != motor_client)
         {
            /* send_pwm_to_c3() closes motor_client internally */
            send_pwm_to_c3(motor_client, eff_pwm);
            motor_client = SOCK_INVALID;
         }
      }
      else
      {
         /* Attempt to re-open listen socket if it was lost */
         motor_listen = open_motor_listen_socket();
         if (SOCK_INVALID == motor_listen)
         {
            g_state.motor_online = 0;
         }
      }

      run_bb_state_machine(&bb_sock, &bb_state, &bb_block_count,
                            &bb_connect_start, &bb_addr, now);

      if (TCP_STATE_CONNECTED != bb_state)
      {
         vTaskDelay(pdMS_TO_TICKS(SOCK_POLL_DELAY_MS));
         continue;
      }

      (void)xEventGroupWaitBits(sub.events, sub.mask, pdTRUE, pdFALSE,
                           pdMS_TO_TICKS(TCP_SEND_INTERVAL_MS));
      drain_queues(sub);

      send_to_bb(&bb_sock, &bb_block_count, &bb_state);
   }
}
