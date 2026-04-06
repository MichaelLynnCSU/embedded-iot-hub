#ifndef XPT2046_H
#define XPT2046_H

#include "main.h"
#include <stdint.h>

typedef struct {
    uint16_t x;
    uint16_t y;
} TouchPoint_t;

void     XPT2046_Init(SPI_HandleTypeDef *hspi);
uint8_t  XPT2046_IsTouched(void);
TouchPoint_t XPT2046_GetTouch(void);
void XPT2046_GetRaw(uint16_t *raw_x, uint16_t *raw_y);
#endif /* XPT2046_H */
