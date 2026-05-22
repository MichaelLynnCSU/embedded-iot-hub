/*
 * lcd_driver.c
 * HD44780 LCD driver for BeagleBone Black via direct register access.
 *
 * References:
 *   [TRM]  AM335x ARM Cortex-A8 Microprocessors Technical Reference Manual
 *          Literature number: SPRUH73Q
 *          https://www.ti.com/lit/ug/spruh73q/spruh73q.pdf
 *
 *   [SRM]  BeagleBone Black System Reference Manual
 *          https://github.com/beagleboard/beaglebone-black/blob/master/BBB_SRM.pdf
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdint.h>
#include "lcd_driver.h"

/*
 * GPIO bank base addresses.
 *
 * [TRM] Section 2, Table 2-2 "L4_PER Peripheral Memory Map"
 *   GPIO0: 0x44E07000
 *   GPIO1: 0x4804C000
 *   GPIO2: 0x481AC000
 *   GPIO3: 0x481AE000
 *
 * Each bank controls 32 GPIO pins and occupies 0x1000 bytes of address space.
 */
#define GPIO0_BASE 0x44E07000
#define GPIO1_BASE 0x4804C000
#define GPIO2_BASE 0x481AC000
#define GPIO3_BASE 0x481AE000
#define GPIO_SIZE  0x1000

/*
 * GPIO register offsets within each bank.
 *
 * [TRM] Section 25.4.1, Table 25-5 "GPIO Registers"
 *
 *   GPIO_OE           offset 0x134  — Section 25.4.1.16
 *     Output Enable. Each bit controls direction of the corresponding pin.
 *     0 = output, 1 = input (tristated). Default after reset is all inputs (0xFFFFFFFF).
 *
 *   GPIO_SETDATAOUT   offset 0x194  — Section 25.4.1.26
 *     Writing a 1 to bit N drives pin N high. Writing 0 has no effect.
 *     Use this instead of read-modify-write on GPIO_DATAOUT; it is atomic.
 *
 *   GPIO_CLEARDATAOUT offset 0x190  — Section 25.4.1.25
 *     Writing a 1 to bit N drives pin N low. Writing 0 has no effect.
 *     Atomic, same reasoning as GPIO_SETDATAOUT.
 */
#define GPIO_OE           0x134
#define GPIO_SETDATAOUT   0x194
#define GPIO_CLEARDATAOUT 0x190

/* Mapped GPIO banks (global). Populated by map_gpio_banks(). */
static volatile uint32_t *gpio0_map = NULL;
static volatile uint32_t *gpio1_map = NULL;
static volatile uint32_t *gpio2_map = NULL;
static volatile uint32_t *gpio3_map = NULL;

/*
 * LCD pin mapping — BBB header pin to GPIO bank/bit.
 *
 * [SRM] Section 7.1, Table 12 "Expansion Header P9 Pinout"
 *   Each row gives: header pin, ball, signal name, and mode 0–7.
 *   Mode 7 is always GPIO, expressed as gpio<bank>[<bit>].
 *
 *   P9_31  Ball A13  gpio3[14]  → bank 3, bit 14
 *   P9_29  Ball B13  gpio3[15]  → bank 3, bit 15
 *   P9_14  Ball U14  gpio1[18]  → bank 1, bit 18
 *   P9_15  Ball R14  gpio1[16]  → bank 1, bit 16
 *   P9_27  Ball C13  gpio3[19]  → bank 3, bit 19
 *   P9_25  Ball A14  gpio3[21]  → bank 3, bit 21
 *
 * D4–D7 are the HD44780 data lines (4-bit mode uses upper nibble only).
 * RS = Register Select (0 = command, 1 = data).
 * E  = Enable strobe (data latched on falling edge).
 */
gpio_t lcd_pins[] = {
    { 3, 14, "D4" },    // GPIO3_14 / P9_31  [SRM Table 12]
    { 3, 15, "D5" },    // GPIO3_15 / P9_29  [SRM Table 12]
    { 1, 18, "D6" },    // GPIO1_18 / P9_14  [SRM Table 12]
    { 1, 16, "D7" },    // GPIO1_16 / P9_15  [SRM Table 12]
    { 3, 19, "RS" },    // GPIO3_19 / P9_27  [SRM Table 12]
    { 3, 21, "E"  },    // GPIO3_21 / P9_25  [SRM Table 12]
};

const int NUM_PINS = sizeof(lcd_pins)/sizeof(lcd_pins[0]);

/* Index constants into lcd_pins[]. */
#define PIN_D4 0
#define PIN_D5 1
#define PIN_D6 2
#define PIN_D7 3
#define PIN_RS 4
#define PIN_E  5

/*
 * Return the mmap'd base pointer for a GPIO bank.
 * The returned pointer is uint32_t*, so adding (offset/4) gives the register.
 */
volatile uint32_t* get_gpio_base(int bank) {
    if (bank == 0) return gpio0_map;
    if (bank == 1) return gpio1_map;
    if (bank == 2) return gpio2_map;
    if (bank == 3) return gpio3_map;
    return NULL;
}

/*
 * Map all four GPIO banks into process address space via /dev/mem.
 *
 * /dev/mem exposes physical memory. mmap with MAP_SHARED makes writes go
 * directly to the hardware registers without kernel involvement.
 *
 * [TRM] Section 2, Table 2-2 — base addresses used here.
 * [TRM] Section 25.4.1.16 GPIO_OE — registers accessed after mapping.
 */
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

/*
 * Set a GPIO pin as an output by clearing its bit in GPIO_OE.
 *
 * [TRM] Section 25.4.1.16, Table 25-21 "GPIO_OE Register"
 *   Bits 31-0: OUTPUTEN
 *     0 = The corresponding GPIO pin is configured as an output.
 *     1 = The corresponding GPIO pin is configured as an input (default).
 *
 * base[GPIO_OE/4]  — /4 converts byte offset to uint32_t array index.
 * &= ~(1 << bit)   — clears only the target bit, leaving all others unchanged.
 */
void gpio_set_output(int bank, int bit) {
    volatile uint32_t *base = get_gpio_base(bank);
    if (base) {
        base[GPIO_OE/4] &= ~(1 << bit);
    }
}

/*
 * Drive a GPIO pin high.
 *
 * [TRM] Section 25.4.1.26, Table 25-31 "GPIO_SETDATAOUT Register"
 *   Bits 31-0: SETDATAOUT
 *     Writing 1 to bit N sets the output latch for pin N high.
 *     Writing 0 to bit N has no effect.
 *
 * Because this register is write-only with no side effects on other bits,
 * a plain assignment (not read-modify-write) is safe and atomic.
 */
void gpio_set(int bank, int bit) {
    volatile uint32_t *base = get_gpio_base(bank);
    if (base) {
        base[GPIO_SETDATAOUT/4] = (1 << bit);
    }
}

/*
 * Drive a GPIO pin low.
 *
 * [TRM] Section 25.4.1.25, Table 25-30 "GPIO_CLEARDATAOUT Register"
 *   Bits 31-0: CLEARDATAOUT
 *     Writing 1 to bit N clears the output latch for pin N (drives low).
 *     Writing 0 to bit N has no effect.
 *
 * Same atomic write-only property as GPIO_SETDATAOUT.
 */
void gpio_clear(int bank, int bit) {
    volatile uint32_t *base = get_gpio_base(bank);
    if (base) {
        base[GPIO_CLEARDATAOUT/4] = (1 << bit);
    }
}

/* Set or clear an LCD pin by index into lcd_pins[]. */
void lcd_set_pin(int pin_idx, int value) {
    if (value)
        gpio_set(lcd_pins[pin_idx].bank, lcd_pins[pin_idx].bit);
    else
        gpio_clear(lcd_pins[pin_idx].bank, lcd_pins[pin_idx].bit);
}

/*
 * Pulse the Enable line to latch data into the HD44780.
 * Data is latched on the falling edge of E.
 * Minimum pulse width per HD44780 datasheet: 230ns — usleep(1) is sufficient.
 */
void lcd_enable_pulse(void) {
    lcd_set_pin(PIN_E, 1);
    usleep(1);
    lcd_set_pin(PIN_E, 0);
    usleep(50);     // Minimum 37us execution time per HD44780 datasheet.
}

/*
 * Write the lower 4 bits of 'nibble' to D4-D7 and pulse Enable.
 * In 4-bit mode the HD44780 reads D4-D7 only; D0-D3 are unused.
 */
void lcd_write_nibble(uint8_t nibble) {
    lcd_set_pin(PIN_D4, (nibble >> 0) & 1);
    lcd_set_pin(PIN_D5, (nibble >> 1) & 1);
    lcd_set_pin(PIN_D6, (nibble >> 2) & 1);
    lcd_set_pin(PIN_D7, (nibble >> 3) & 1);
    lcd_enable_pulse();
}

/*
 * Write a full byte to the HD44780 as two nibbles (high then low).
 * is_data: 0 = command (RS low), 1 = character data (RS high).
 */
void lcd_write_byte(uint8_t byte, int is_data) {
    lcd_set_pin(PIN_RS, is_data);
    lcd_write_nibble(byte >> 4);    // High nibble first.
    lcd_write_nibble(byte & 0x0F);  // Low nibble second.
    usleep(50);
}

void lcd_command(uint8_t cmd) {
    lcd_write_byte(cmd, 0);     // RS = 0: instruction register.
}

void lcd_data(uint8_t data) {
    lcd_write_byte(data, 1);    // RS = 1: data register.
}

/*
 * Initialise the HD44780 in 4-bit mode.
 *
 * The sequence below follows the HD44780 datasheet Figure 24
 * "Initializing by Instruction — 4-Bit Interface".
 *
 * The three 0x03 nibbles bring the controller to a known 8-bit state
 * regardless of any previous partial initialisation. 0x02 then switches
 * it to 4-bit mode.
 */
void lcd_init(void) {
    for (int i = 0; i < NUM_PINS; i++)
        lcd_set_pin(i, 0);
    usleep(50000);          // >40ms after VCC rises to 4.5V (HD44780 datasheet).

    lcd_set_pin(PIN_RS, 0);

    lcd_write_nibble(0x03); // Function set (8-bit). Wait >4.1ms.
    usleep(4500);
    lcd_write_nibble(0x03); // Function set (8-bit). Wait >100us.
    usleep(150);
    lcd_write_nibble(0x03); // Function set (8-bit).
    usleep(150);
    lcd_write_nibble(0x02); // Switch to 4-bit mode.
    usleep(150);

    lcd_command(0x28);      // Function set: 4-bit, 2 lines, 5x8 font.
    usleep(50);
    lcd_command(0x08);      // Display off.
    usleep(50);
    lcd_command(0x01);      // Clear display. Requires >1.52ms.
    usleep(2000);
    lcd_command(0x06);      // Entry mode: increment cursor, no display shift.
    usleep(50);
    lcd_command(0x0C);      // Display on, cursor off, blink off.
    usleep(50);
}

/* Clear display and return cursor home. Requires >1.52ms. */
void lcd_clear(void) {
    lcd_command(0x01);
    usleep(2000);
}

/*
 * Move cursor to row/col (zero-indexed).
 * Row 0 starts at DDRAM address 0x00, row 1 at 0x40.
 * Command 0x80 | address sets the DDRAM address.
 */
void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x00 : 0x40;
    addr += col;
    lcd_command(0x80 | addr);
}

/* Write a null-terminated string at the current cursor position. */
void lcd_print(const char *str) {
    while (*str) {
        lcd_data(*str++);
        usleep(100);
    }
}
