#include "xpt2046.h"

#define TOUCH_X_MIN     115
#define TOUCH_X_MAX     1919
#define TOUCH_Y_MIN     159
#define TOUCH_Y_MAX     1952
#define TOUCH_SCREEN_W  320
#define TOUCH_SCREEN_H  240

#define T_CS_LOW()   HAL_GPIO_WritePin(T_CS_GPIO_Port, T_CS_Pin, GPIO_PIN_RESET)
#define T_CS_HIGH()  HAL_GPIO_WritePin(T_CS_GPIO_Port, T_CS_Pin, GPIO_PIN_SET)
#define T_IRQ_READ() HAL_GPIO_ReadPin(T_IRQ_GPIO_Port, T_IRQ_Pin)

static SPI_HandleTypeDef *touch_spi;

void XPT2046_Init(SPI_HandleTypeDef *hspi)
{
    touch_spi = hspi;
    T_CS_HIGH();
}

static uint16_t XPT2046_ReadRaw(uint8_t cmd)
{
    uint8_t tx[3] = { cmd, 0x00, 0x00 };
    uint8_t rx[3] = { 0 };
    T_CS_LOW();
    HAL_SPI_TransmitReceive(touch_spi, tx, rx, 3, 100);
    T_CS_HIGH();
    return ((rx[1] << 8) | rx[2]) >> 4;
}

uint8_t XPT2046_IsTouched(void)
{
    return (T_IRQ_READ() == GPIO_PIN_RESET);
}

void XPT2046_GetRaw(uint16_t *raw_x, uint16_t *raw_y)
{
    uint32_t x = 0, y = 0;
    for (uint8_t i = 0; i < 4; i++) {
        x += XPT2046_ReadRaw(0xD0);
        y += XPT2046_ReadRaw(0x90);
    }
    *raw_x = x / 4;
    *raw_y = y / 4;
}

TouchPoint_t XPT2046_GetTouch(void)
{
    TouchPoint_t tp = {0, 0};
    uint32_t raw_x = 0, raw_y = 0;
    for (uint8_t i = 0; i < 4; i++) {
        raw_x += XPT2046_ReadRaw(0xD0);
        raw_y += XPT2046_ReadRaw(0x90);
    }
    raw_x /= 4;
    raw_y /= 4;

    if (raw_x < TOUCH_X_MIN) raw_x = TOUCH_X_MIN;
    if (raw_x > TOUCH_X_MAX) raw_x = TOUCH_X_MAX;
    if (raw_y < TOUCH_Y_MIN) raw_y = TOUCH_Y_MIN;
    if (raw_y > TOUCH_Y_MAX) raw_y = TOUCH_Y_MAX;

    tp.x = (raw_x - TOUCH_X_MIN) * TOUCH_SCREEN_W / (TOUCH_X_MAX - TOUCH_X_MIN);
    tp.y = (raw_y - TOUCH_Y_MIN) * TOUCH_SCREEN_H / (TOUCH_Y_MAX - TOUCH_Y_MIN);
    return tp;
}
