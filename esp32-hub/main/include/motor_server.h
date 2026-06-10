#ifndef MOTOR_SERVER_H
#define MOTOR_SERVER_H

/******************************************************************************
 * \file motor_server.h
 * \brief Hub-side TCP server for the ESP32-C3 motor controller.
 *
 * \details The hub listens on HUB_MOTOR_PORT.  The motor wakes from deep
 *          sleep, connects in, receives {"pwm": X}, replies {"batt_motor": Y},
 *          and closes.  Each wake cycle is a fresh connection; there is no
 *          persistent C3 client socket.
 *
 * \note    Motor server flip (2026-05-XX):
 *          Hub is now TCP server for motor.  open_motor_listen_socket() and
 *          accept_motor_connection() replace the former run_c3_state_machine()
 *          / disconnect_c3() client approach.
 *          send_pwm_to_c3() takes the accepted client fd directly and owns
 *          its lifetime (closes on exit).
 *
 * \note    motor_online ownership (2026-05-05, updated 2026-05-XX):
 *          motor_online is set to 1 by a successful send_pwm_to_c3() and to
 *          0 on send failure.  accept() / listen failures leave it unchanged;
 *          the caller should set it to 0 when open_motor_listen_socket() fails.
 ******************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/**
 * \brief  Create and bind the motor listen socket.
 * \return Valid fd on success, -1 (SOCK_INVALID) on any failure.
 *
 * Socket is set non-blocking after listen().  Caller stores the fd and
 * passes it to accept_motor_connection() each loop iteration.
 */
int open_motor_listen_socket(void);

/**
 * \brief  Non-blocking accept on the motor listen socket.
 * \param  listen_sock  fd returned by open_motor_listen_socket().
 * \return Accepted client fd, or -1 if no connection is pending or on error.
 *
 * EAGAIN / EWOULDBLOCK are silent (expected each loop with no inbound).
 * Other errors are logged as warnings.
 */
int accept_motor_connection(int listen_sock);

/**
 * \brief  Send PWM duty to the motor and read back its battery level.
 * \param  client_sock  Accepted client fd from accept_motor_connection().
 * \param  pwm_pct      Effective PWM percentage [0, PWM_OUT_MAX].
 * \param  p_motor_online  Out: set to 1 on success, 0 on send failure.
 * \param  p_motor_batt    Out: updated with received batt_motor value (≥ 0).
 *                         Left unchanged on timeout or parse failure.
 *
 * Closes client_sock before returning regardless of outcome.
 * Publishes motor state to the vroom bus via bus_publish_motor().
 */
void send_pwm_to_c3(int client_sock, float pwm_pct,
                    uint8_t *p_motor_online, int *p_motor_batt);

#endif /* MOTOR_SERVER_H */
