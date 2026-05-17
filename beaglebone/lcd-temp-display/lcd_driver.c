#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdint.h>
#include "lcd_driver.h"

// Bank base addresses
#define GPIO0_BASE 0x44E07000
#define GPIO1_BASE 0x4804C000
#define GPIO2_BASE 0x481AC000
#define GPIO3_BASE 0x481AE000
#define GPIO_SIZE  0x1000

// Offsets for GPIO registers
#define GPIO_OE           0x134
#define GPIO_SETDATAOUT   0x194
#define GPIO_CLEARDATAOUT 0x190

// Mapped GPIO banks (global)
static volatile uint32_t *gpio0_map = NULL;
static volatile uint32_t *gpio1_map = NULL;
static volatile uint32_t *gpio2_map = NULL;
static volatile uint32_t *gpio3_map = NULL;

// LCD pin mapping
gpio_t lcd_pins[] = {
    { 3, 14, "D4" },    // GPIO3_14 / P9_31
    { 3, 15, "D5" },    // GPIO3_15 / P9_29
    { 1, 18, "D6" },    // GPIO1_18 / P9_14
    { 1, 16, "D7" },    // GPIO1_16 / P9_15
    { 3, 19, "RS" },    // GPIO3_19 / P9_27
    { 3, 21, "E" },     // GPIO3_21 / P9_25
};

const int NUM_PINS = sizeof(lcd_pins)/sizeof(lcd_pins[0]);

// Pin indexes
#define PIN_D4 0
#define PIN_D5 1
#define PIN_D6 2
#define PIN_D7 3
#define PIN_RS 4
#define PIN_E  5

volatile uint32_t* get_gpio_base(int bank) {
    if (bank == 0) return gpio0_map;
    if (bank == 1) return gpio1_map;
    if (bank == 2) return gpio2_map;
    if (bank == 3) return gpio3_map;
    return NULL;
}

void map_gpio_banks(void) {
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        perror("open /dev/mem");
        exit(1);
    }

    gpio0_map = (volatile uint32_t *)mmap(NULL, GPIO_SIZE,
                    PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, GPIO0_BASE);
    gpio1_map = (volatile uint32_t *)mmap(NULL, GPIO_SIZE,
                    PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, GPIO1_BASE);
    gpio2_map = (volatile uint32_t *)mmap(NULL, GPIO_SIZE,
                    PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, GPIO2_BASE);
    gpio3_map = (volatile uint32_t *)mmap(NULL, GPIO_SIZE,
                    PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, GPIO3_BASE);

    close(mem_fd);

    if (gpio0_map == MAP_FAILED || gpio1_map == MAP_FAILED ||
        gpio2_map == MAP_FAILED || gpio3_map == MAP_FAILED) {
        perror("mmap");
        exit(1);
    }
}

void gpio_set_output(int bank, int bit) {
    volatile uint32_t *base = get_gpio_base(bank);
    if (base) {
        base[GPIO_OE/4] &= ~(1 << bit);
    }
}

void gpio_set(int bank, int bit) {
    volatile uint32_t *base = get_gpio_base(bank);
    if (base) {
        base[GPIO_SETDATAOUT/4] = (1 << bit);
    }
}

void gpio_clear(int bank, int bit) {
    volatile uint32_t *base = get_gpio_base(bank);
    if (base) {
        base[GPIO_CLEARDATAOUT/4] = (1 << bit);
    }
}

void lcd_set_pin(int pin_idx, int value) {
    if (value)
        gpio_set(lcd_pins[pin_idx].bank, lcd_pins[pin_idx].bit);
    else
        gpio_clear(lcd_pins[pin_idx].bank, lcd_pins[pin_idx].bit);
}

void lcd_enable_pulse(void) {
    lcd_set_pin(PIN_E, 1);
    usleep(1);
    lcd_set_pin(PIN_E, 0);
    usleep(50);
}

void lcd_write_nibble(uint8_t nibble) {
    lcd_set_pin(PIN_D4, (nibble >> 0) & 1);
    lcd_set_pin(PIN_D5, (nibble >> 1) & 1);
    lcd_set_pin(PIN_D6, (nibble >> 2) & 1);
    lcd_set_pin(PIN_D7, (nibble >> 3) & 1);
    lcd_enable_pulse();
}

void lcd_write_byte(uint8_t byte, int is_data) {
    lcd_set_pin(PIN_RS, is_data);
    lcd_write_nibble(byte >> 4);
    lcd_write_nibble(byte & 0x0F);
    usleep(50);
}

void lcd_command(uint8_t cmd) {
    lcd_write_byte(cmd, 0);
}

void lcd_data(uint8_t data) {
    lcd_write_byte(data, 1);
}

void lcd_init(void) {
    // Set all pins low
    for (int i = 0; i < NUM_PINS; i++) {
        lcd_set_pin(i, 0);
    }
    usleep(50000);  // Wait 50ms

    // Initialize in 8-bit mode first
    lcd_set_pin(PIN_RS, 0);
    
    lcd_write_nibble(0x03);
    usleep(4500);
    
    lcd_write_nibble(0x03);
    usleep(150);
    
    lcd_write_nibble(0x03);
    usleep(150);

    // Switch to 4-bit mode
    lcd_write_nibble(0x02);
    usleep(150);

    // Function set: 4-bit, 2 lines, 5x8 font
    lcd_command(0x28);
    usleep(50);

    // Display off
    lcd_command(0x08);
    usleep(50);

    // Clear display
    lcd_command(0x01);
    usleep(2000);

    // Entry mode: increment, no shift
    lcd_command(0x06);
    usleep(50);

    // Display on, cursor off, blink off
    lcd_command(0x0C);
    usleep(50);
}

void lcd_clear(void) {
    lcd_command(0x01);
    usleep(2000);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x00 : 0x40;
    addr += col;
    lcd_command(0x80 | addr);
}

void lcd_print(const char *str) {
    while (*str) {
        lcd_data(*str++);
        usleep(100);
    }
}
