/******************************************************************************
 * \file    motor.h
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Motor driver API for nRF52840 smart lock node.
 ******************************************************************************/

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/**
 * \brief  Initialise motor GPIO (AIN1, AIN2, STBY) and PWM.
 * \return 0 on success, negative errno on failure.
 */
int motor_init(void);

/**
 * \brief  Drive motor in lock (1) or unlock (0) direction for 500ms
 *         then cut power via STBY. Submitted as work item from BLE RX WQ
 *         context -- never called directly from interrupt or BT RX WQ.
 *
 * \param  cmd  1=lock, 0=unlock.
 */
void motor_drive(uint8_t cmd);

#endif /* MOTOR_H */
