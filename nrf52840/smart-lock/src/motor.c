/******************************************************************************
 * \file    motor.c
 * \author  MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 * \date    01-01-2025
 *
 * \brief   Motor driver for nRF52840 smart lock node.
 *
 * \details TB6612FNG H-bridge driver. AIN1/AIN2 set direction, STBY
 *          enables the bridge. PWM duty fixed at 50% (10ms/20ms).
 *          Motor runs for 500ms then STBY is pulled low via timer.
 *
 *          When motor_off_timer fires, motor_settle_handler() calls
 *          ble_lock_settle() to advance the lock state machine from
 *          UNLOCKING->UNLOCKED or LOCKING->LOCKED. ble_lock_settle()
 *          owns the BLE notify, adv update, and settings save.
 *
 * \note    Command pattern (2026-04-29):
 *          g_motor_cmd is a MOTOR_CMD_T -- the command value travels inside
 *          the work item via CONTAINER_OF. Eliminates the bare uint8_t global
 *          that was shared between motor_drive() and motor_work_handler().
 *
 * \note    BT RX WQ stack fix (2026-03-24):
 *          motor_drive() submits work to sysworkq -- never blocks BT RX WQ.
 *          k_sleep(K_MSEC(500)) was previously in lock_write() on BT RX WQ,
 *          driving stack to 768/1216. Moved here eliminates that spike.
 ******************************************************************************/

#include "motor.h"
#include "ble_gatt.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(motor, LOG_LEVEL_INF);

static const struct gpio_dt_spec g_ain1 = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)), .pin = 11, .dt_flags = GPIO_ACTIVE_HIGH
};
static const struct gpio_dt_spec g_ain2 = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)), .pin = 15, .dt_flags = GPIO_ACTIVE_HIGH
};
static const struct gpio_dt_spec g_stby = {
    .port = DEVICE_DT_GET(DT_NODELABEL(gpio1)), .pin = 13, .dt_flags = GPIO_ACTIVE_HIGH
};

static struct pwm_dt_spec g_motor_pwm = {
    .dev     = DEVICE_DT_GET(DT_NODELABEL(pwm0)),
    .channel = 0,
    .period  = PWM_MSEC(20),
    .flags   = PWM_POLARITY_NORMAL
};

/*----------------------------------------------------------------------------*/
/* Command object — command data travels with the work item                   */

typedef struct
{
    struct k_work work; /**< must be first — CONTAINER_OF base */
    uint8_t       cmd;  /**< lock direction: 1 = lock, 0 = unlock */
} MOTOR_CMD_T;

static MOTOR_CMD_T        g_motor_cmd;
static struct k_work      g_motor_settle_work; /**< settle work — safe context for BLE */

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Work handler — calls ble_lock_settle() in safe sysworkq context.
 *
 * \details Timer ISR cannot call ble_lock_settle() directly because
 *          settings_save_one() and bt_gatt_notify() require task context.
 *          Submitted from motor_off_handler() ISR via k_work_submit().
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void motor_settle_handler(struct k_work *work)
{
    ble_lock_settle();
}

/******************************************************************************
 * \brief Timer callback — cuts motor power and submits settle work.
 *
 * \details Runs in ISR context. Pulls STBY low to cut H-bridge power,
 *          then submits g_motor_settle_work to sysworkq so ble_lock_settle()
 *          runs in safe task context.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void motor_off_handler(struct k_timer *timer)
{
    gpio_pin_set_dt(&g_stby, 0);
    k_work_submit(&g_motor_settle_work);
}

K_TIMER_DEFINE(g_motor_off_timer, motor_off_handler, NULL);

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Work handler — drives H-bridge using command from MOTOR_CMD_T.
 *
 * \details Extracts cmd from the enclosing MOTOR_CMD_T via CONTAINER_OF.
 *          Command data is self-contained in the work item — no shared
 *          global read required.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
static void motor_work_handler(struct k_work *work)
{
    MOTOR_CMD_T *p_cmd = CONTAINER_OF(work, MOTOR_CMD_T, work);

    gpio_pin_set_dt(&g_stby, 1);
    gpio_pin_set_dt(&g_ain1, p_cmd->cmd ? 1 : 0);
    gpio_pin_set_dt(&g_ain2, p_cmd->cmd ? 0 : 1);
    pwm_set_dt(&g_motor_pwm, PWM_MSEC(20), PWM_MSEC(10));
    k_timer_start(&g_motor_off_timer, K_MSEC(500), K_NO_WAIT);
}

/*----------------------------------------------------------------------------*/

int motor_init(void)
{
    if (!device_is_ready(g_ain1.port) ||
        !device_is_ready(g_ain2.port) ||
        !device_is_ready(g_stby.port))
    {
        LOG_ERR("Motor GPIO not ready");
        return -ENODEV;
    }

    gpio_pin_configure_dt(&g_ain1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&g_ain2, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&g_stby, GPIO_OUTPUT_INACTIVE);

    if (!device_is_ready(g_motor_pwm.dev))
    {
        LOG_ERR("PWM not ready");
        return -ENODEV;
    }

    k_work_init(&g_motor_cmd.work,    motor_work_handler);
    k_work_init(&g_motor_settle_work, motor_settle_handler);

    LOG_INF("Motor init OK");
    return 0;
}

/*----------------------------------------------------------------------------*/

/******************************************************************************
 * \brief Drive the motor in the requested direction.
 *
 * \details Stores cmd in the MOTOR_CMD_T command object and submits it
 *          to sysworkq. The handler extracts cmd via CONTAINER_OF —
 *          no separate global needed.
 *
 * \param cmd  1 = lock direction, 0 = unlock direction.
 *
 * \author MichaelLynnCSU (https://github.com/MichaelLynnCSU)
 ******************************************************************************/
void motor_drive(uint8_t cmd)
{
    g_motor_cmd.cmd = cmd;
    k_work_submit(&g_motor_cmd.work);
}
