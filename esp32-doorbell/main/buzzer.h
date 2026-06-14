/******************************************************************************
 * \file buzzer.h
 * \brief Passive buzzer driver — ESP32 doorbell.
 *
 * \details Drives a passive buzzer via LEDC PWM on GPIO14 through a
 *          2N7000 N-channel MOSFET (gate → GPIO14, drain → buzzer-,
 *          source → GND). Uses LEDC_TIMER_1 / LEDC_CHANNEL_1 to avoid
 *          conflicting with the camera XCLK (LEDC_TIMER_0 / LEDC_CHANNEL_0).
 *
 *          buzzer_init() must be called once from app_main() before any
 *          buzzer_beep() calls.
 ******************************************************************************/

#ifndef BUZZER_H
#define BUZZER_H

/**
 * \brief Initialise LEDC timer and channel for buzzer output.
 *        Call once from app_main() before buzzer_beep().
 */
void buzzer_init(void);

/**
 * \brief Emit a single beep on the buzzer.
 *        Blocking — returns after BUZZER_DURATION_MS ms.
 *        Safe to call from any task.
 */
void buzzer_beep(void);

#endif /* BUZZER_H */
