#ifndef REED_HW_H
#define REED_HW_H
/**
 * \brief  Initialise GPIO for reed switch and debug LED.
 * \return 0 on success, negative errno on failure.
 */
int reed_hw_init(void);

/**
 * \brief  Read current reed switch state.
 * \return 1=open, 0=closed, negative errno on failure.
 */
int reed_hw_read(void);

/**
 * \brief  Set debug LED state.
 * \param  state  1=on, 0=off.
 */
void reed_hw_set_led(int state);
#endif /* REED_HW_H */
