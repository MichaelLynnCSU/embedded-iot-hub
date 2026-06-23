/******************************************************************************
 * \file    pi_controller.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   PI controller implementation for ESP32 hub TCP manager.
 *
 * \details Runs a PI loop each TCP cycle. Gains (kp, ki, kd) and setpoint
 *          are consumed from aws_manager getters -- updated by Lambda every
 *          24 hours. Integral is clamped to INTEGRAL_CLAMP to prevent
 *          windup. Derivative term is included for completeness but kd
 *          defaults to 0.0 from Lambda config.
 *
 *          dt is derived from elapsed wall-clock time between calls rather
 *          than a fixed period so accuracy is maintained if the TCP loop
 *          slows down under load.
 ******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uart_manager.h"
#include "pi_controller.h"
#include "config.h"
#include "device_gateway.h"
#include "esp_log.h"

static const char *TAG = "TCP_MGR";

#define INTEGRAL_CLAMP   100.0f   /**< anti-windup clamp (±) */

/** \brief PI controller persistent state -- private to this file. */
typedef struct
{
   float    integral;     /*!< accumulated integral term */
   float    last_error;   /*!< previous error for derivative */
   uint32_t last_run_ms;  /*!< timestamp of last PI execution (ms) */
} PI_STATE_T;

static PI_STATE_T g_pi = { 0.0f, 0.0f, 0u };

/*----------------------------------------------------------------------------*/

float run_pi_controller(uint32_t now_ms)
{
   float kp       = gateway_get_kp();
   float ki       = gateway_get_ki();
   float kd       = gateway_get_kd();
   int   setpoint = gateway_get_setpoint();
   int   avg_temp = uart_get_avg_temp();
   float error    = 0.0f;
   float dt       = 0.0f;
   float p_term   = 0.0f;
   float i_term   = 0.0f;
   float d_term   = 0.0f;
   float output   = 0.0f;

   /* dt in seconds -- guard against zero on first call */
   if (g_pi.last_run_ms != 0u)
   {
      dt = (float)(now_ms - g_pi.last_run_ms) / 1000.0f;
   }
   else
   {
      dt = (float)TCP_SEND_INTERVAL_MS / 1000.0f;
   }

   g_pi.last_run_ms = now_ms;

   error  = (float)(setpoint - avg_temp);
   p_term = kp * error;

   /* Integral with anti-windup clamp */
   g_pi.integral += error * dt;
   if      (g_pi.integral >  INTEGRAL_CLAMP) { g_pi.integral =  INTEGRAL_CLAMP; }
   else if (g_pi.integral < -INTEGRAL_CLAMP) { g_pi.integral = -INTEGRAL_CLAMP; }
   i_term = ki * g_pi.integral;

   /* Derivative on error (not measurement) */
   d_term          = (dt > 0.0f) ? (kd * (error - g_pi.last_error) / dt) : 0.0f;
   g_pi.last_error = error;

   output = p_term + i_term + d_term;

   if      (output > PWM_OUT_MAX) { output = PWM_OUT_MAX; }
   else if (output < PWM_OUT_MIN) { output = PWM_OUT_MIN; }

   ESP_LOGI(TAG, "[PI] sp=%d temp=%d err=%.1f P=%.2f I=%.2f D=%.2f out=%.1f%%",
            setpoint, avg_temp, error, p_term, i_term, d_term, output);

   return output;
}
