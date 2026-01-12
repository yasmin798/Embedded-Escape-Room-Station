#include "lcd_i2c.h"
#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include <hardware/i2c.h>

#define LCD_ADDR 0x27
#define LCD_COLS 16
static i2c_inst_t *lcd_i2c;
static uint8_t lcd_addr;
//static uint8_t backlight = 0x08;
static void lcd_write_cmd(uint8_t cmd);
static void lcd_write_data(uint8_t data);
static uint8_t backlight = 0x08;

// INTERNAL: send 4-phase I2C sequence
static void lcd_write(uint8_t data, uint8_t mode) {
    uint8_t hi = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;

    uint8_t buf[4] = {
        hi | backlight | mode | 0x04,
        hi | backlight | mode,
        lo | backlight | mode | 0x04,
        lo | backlight | mode
    };

    // FIXED: use lcd_addr (NOT hardcoded address)
    i2c_write_blocking(lcd_i2c, lcd_addr, buf, 4, false);
}

static void lcd_write_cmd(uint8_t cmd) {
    lcd_write(cmd, 0x00);
}

static void lcd_write_data(uint8_t data) {
    lcd_write(data, 0x01);
}

void lcd_init(i2c_inst_t *i2c, uint32_t sda, uint32_t scl, uint8_t addr) {
    lcd_i2c = i2c;
    lcd_addr = addr;     // FIXED

    i2c_init(i2c, 100000);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
    sleep_ms(50);

    uint8_t seq[] = {0x33, 0x32, 0x28, 0x0C, 0x06, 0x01};
    for (int i = 0; i < 6; i++) {
        lcd_write_cmd(seq[i]);
        sleep_ms(5);
    }
}

void lcd_clear() {
    lcd_write_cmd(0x01);
    sleep_ms(2);
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t offsets[] = {0x00, 0x40};
    lcd_write_cmd(0x80 | (col + offsets[row]));
}

void lcd_print(const char *str) {
    while (*str) lcd_write_data(*str++);
}

void lcd_backlight(bool on) {
    backlight = on ? 0x08 : 0x00;
    lcd_write_cmd(0x00);
}
