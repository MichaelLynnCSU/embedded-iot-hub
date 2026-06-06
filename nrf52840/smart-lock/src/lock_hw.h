#ifndef LOCK_HW_H
#define LOCK_HW_H
/**
 * \brief  Initialise lock status LED GPIO.
 * \return 0 on success, negative errno on failure.
 */
int lock_hw_init(void);

/**
 * \brief  Set lock status LED.
 * \param  state  1=on, 0=off.
 */
void lock_hw_set_led(int state);
#endif /* LOCK_HW_H */
