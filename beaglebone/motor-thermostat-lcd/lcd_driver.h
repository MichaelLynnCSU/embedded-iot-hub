#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

#include <stdint.h>

typedef struct {
    int bank;
    int bit;
    const char *name;
} gpio_t;

extern gpio_t lcd_pins[];
extern const int NUM_PINS;

// Function prototypes
void map_gpio_banks(void);
void gpio_set_output(int bank, int bit);
void gpio_set(int bank, int bit);
void gpio_clear(int bank, int bit);

void lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_print(const char *str);
void lcd_command(uint8_t cmd);
void lcd_data(uint8_t data);

#endif // LCD_DRIVER_H
