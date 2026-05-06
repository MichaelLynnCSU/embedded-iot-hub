/******************************************************************************
 * \file    motor_sm.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Motor state machine for ESP32 hub.
 *
 * \details Implements the three-state motor lifecycle (IDLE / RUNNING /
 *          COOLDOWN). The hub drives the C3 on-demand: when the PI
 *          controller outputs > 0 the hub's TCP server accepts the motor's
 *          inbound connection and sends {"pwm": X}.
 *
 * \note    Motor state machine (2026-05-04, unchanged):
 *          IDLE:     PWM=0, hub listen socket open but motor not connected.
 *          RUNNING:  Temp >= low threshold; PI output > 0. Hub sends
 *                    {"pwm": X} on each motor wake connection. Motor must
 *                    run for MIN_RUN_MS (10 min) regardless of temp drop.
 *          COOLDOWN: PWM=0 sent; hub keeps listen socket open until
 *                    COOLDOWN_MS (5 min) elapses, then state -> IDLE.
 *
 * \note    Ping removed (2026-05-XX):
 *          ping_motor_for_health(), motor_sm_ping_due(), and g_last_ping_ms
 *          are all removed. The motor node now initiates contact on every
 *          deep-sleep wake cycle, so the hub always receives a fresh batt
 *          report without needing to probe. tcp_manager.c no longer calls
 *          ping_motor_for_health() or motor_sm_ping_due().
 *          motor_sm.h updated to remove those declarations.
 *          Network config constants MOTOR_PING_INTERVAL_MS,
 *          MOTOR_PING_TIMEOUT_MS, and MOTOR_PING_RX_BUF no longer
 *          referenced here (may be removed from network_config.h).
 ******************************************************************************/

#include "motor_sm.h"
#include "network_config.h"
#include "esp_log.h"
#include "trinity_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TCP_MGR";

/** \brief Motor state machine persistent state -- private to this file. */
typedef struct
{
   uint8_t  state;         /*!< MOTOR_IDLE / MOTOR_RUNNING / MOTOR_COOLDOWN */
   uint32_t run_start_ms;  /*!< tick when motor entered MOTOR_RUNNING */
   uint32_t stop_start_ms; /*!< tick when motor entered MOTOR_COOLDOWN */
} MOTOR_SM_T;

static MOTOR_SM_T g_motor_sm = { MOTOR_IDLE, 0u, 0u };

/*----------------------------------------------------------------------------*/

uint8_t motor_sm_get_state(void)
{
   return g_motor_sm.state;
}

/*----------------------------------------------------------------------------*/

float run_motor_sm(float      pi_out_pct,
                   uint32_t   now_ms,
                   bool      *p_connect,
                   bool      *p_disconnect)
{
   float effective_pwm = 0.0f;

   *p_connect    = false;
   *p_disconnect = false;

   switch (g_motor_sm.state)
   {
      case MOTOR_IDLE:
      {
         if (pi_out_pct > 0.0f)
         {
            g_motor_sm.state        = MOTOR_RUNNING;
            g_motor_sm.run_start_ms = now_ms;
            effective_pwm           = pi_out_pct;
            *p_connect              = true;
            trinity_log_event("EVENT: MOTOR_START\n");
            ESP_LOGI(TAG, "[MOTOR_SM] IDLE->RUNNING pwm=%.1f%%", effective_pwm);
         }
         /* else stay idle, pwm=0, listen socket stays open for motor wakes */
         break;
      }

      case MOTOR_RUNNING:
      {
         uint32_t run_elapsed = now_ms - g_motor_sm.run_start_ms;

         if (pi_out_pct > 0.0f)
         {
            /* PI still wants motor on -- normal running */
            effective_pwm = pi_out_pct;
         }
         else if (run_elapsed < MIN_RUN_MS)
         {
            /* PI wants off but minimum run not met -- hold at 1% */
            effective_pwm = 1.0f;
            ESP_LOGW(TAG, "[MOTOR_SM] Min-run enforced (%lu/%lu ms)",
                     run_elapsed, MIN_RUN_MS);
         }
         else
         {
            /* PI wants off and min-run satisfied -- enter cooldown */
            g_motor_sm.state         = MOTOR_COOLDOWN;
            g_motor_sm.stop_start_ms = now_ms;
            effective_pwm            = 0.0f;
            trinity_log_event("EVENT: MOTOR_STOP\n");
            ESP_LOGI(TAG, "[MOTOR_SM] RUNNING->COOLDOWN after %lu ms", run_elapsed);
         }
         break;
      }

      case MOTOR_COOLDOWN:
      {
         uint32_t cool_elapsed = now_ms - g_motor_sm.stop_start_ms;
         effective_pwm = 0.0f;

         if (cool_elapsed >= COOLDOWN_MS)
         {
            g_motor_sm.state = MOTOR_IDLE;
            *p_disconnect    = true;
            trinity_log_event("EVENT: MOTOR_COOLDOWN_DONE\n");
            ESP_LOGI(TAG, "[MOTOR_SM] COOLDOWN->IDLE after %lu ms", cool_elapsed);
         }
         else
         {
            ESP_LOGI(TAG, "[MOTOR_SM] Cooldown %lu/%lu ms",
                     cool_elapsed, COOLDOWN_MS);
         }
         break;
      }

      default:
         g_motor_sm.state = MOTOR_IDLE;
         break;
   }

   return effective_pwm;
}
