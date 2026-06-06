#ifndef LIGHT_HW_H
#define LIGHT_HW_H

#include <stdint.h>

/**
 * \brief  Initialise relay GPIO and status LED GPIO.
 * \return 0 on success, negative errno on failure.
 */
int light_hw_init(void);

/**
 * \brief  Drive relay and status LED together.
 * \param  state  1=ON, 0=OFF.
 */
void light_hw_set(uint8_t state);

#endif /* LIGHT_HW_H */
