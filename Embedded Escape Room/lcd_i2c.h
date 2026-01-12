#ifndef LCD_I2C_H
#define LCD_I2C_H

#include <stdint.h>
#include <stdbool.h>

// Prefer angled include but fall back safely if SDK headers aren't available
#if defined(__has_include)
#  if __has_include(<hardware/i2c.h>)
#    include <hardware/i2c.h>
#  elif __has_include("hardware/i2c.h")
#    include "hardware/i2c.h"
#  else
#    /* forward-declare minimal type so headers that include this file can still
#       be parsed when Pico SDK include paths are not configured. */
#    typedef struct i2c_inst i2c_inst_t;
#  endif
#else
#  include <hardware/i2c.h>
#endif

// lcd_i2c.h
void lcd_init(i2c_inst_t *i2c, uint32_t sda, uint32_t scl, uint8_t addr);
void lcd_backlight(bool on);
void lcd_clear(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_print(const char *str);

#endif
