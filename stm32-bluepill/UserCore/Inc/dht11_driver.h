#ifndef DHT11_DRIVER_H
#define DHT11_DRIVER_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

void    DHT11_Init(void);
uint8_t DHT11_ReadData(uint16_t GPIO_Pin, uint8_t *temperature, uint8_t *humidity);

#endif /* DHT11_DRIVER_H */
