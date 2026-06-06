#ifndef PIR_HW_H
#define PIR_HW_H
/**
 * \brief  Initialise PIR GPIO pin as input.
 * \return 0 on success, negative errno on failure.
 */
int pir_hw_init(void);

/**
 * \brief  Read current PIR pin state.
 * \return 1=HIGH (motion), 0=LOW (idle), negative errno on failure.
 */
int pir_hw_read(void);
#endif /* PIR_HW_H */
