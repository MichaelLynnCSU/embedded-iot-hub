#ifndef ILI9341_H
#define ILI9341_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "main.h"

/* ===== Pin control macros ===== */
#define ILI9341_CS_LOW()    HAL_GPIO_WritePin(CS_GPIO_Port,    CS_Pin,     GPIO_PIN_RESET)
#define ILI9341_CS_HIGH()   HAL_GPIO_WritePin(CS_GPIO_Port,    CS_Pin,     GPIO_PIN_SET)
#define ILI9341_DC_LOW()    HAL_GPIO_WritePin(DC_RS_GPIO_Port, DC_RS_Pin,  GPIO_PIN_RESET)
#define ILI9341_DC_HIGH()   HAL_GPIO_WritePin(DC_RS_GPIO_Port, DC_RS_Pin,  GPIO_PIN_SET)
#define ILI9341_RST_LOW()   HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin,  GPIO_PIN_RESET)
#define ILI9341_RST_HIGH()  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin,  GPIO_PIN_SET)

/* ===== Display dimensions ===== */
#define ILI9341_WIDTH   249
#define ILI9341_HEIGHT  320

/* ===== Colors (RGB565) ===== */
#define ILI9341_BLACK   0x0000
#define ILI9341_WHITE   0xFFFF
#define ILI9341_RED     0xF800
#define ILI9341_GREEN   0x07E0
#define ILI9341_BLUE    0x001F
#define ILI9341_YELLOW  0xFFE0
#define ILI9341_CYAN    0x07FF
#define ILI9341_MAGENTA 0xF81F

/* ===== Public API ===== */
void ILI9341_Init(SPI_HandleTypeDef *hspi);
void ILI9341_FillScreen(uint16_t color);
void ILI9341_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ILI9341_DrawChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t size);
void ILI9341_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t fg, uint16_t bg, uint8_t size);

void ILI9341_DrawBitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *data);

#endif /* ILI9341_H */
