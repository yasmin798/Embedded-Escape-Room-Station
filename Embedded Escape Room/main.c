#include <stdio.h>
#include "pico/stdlib.h"
#include "lcd_i2c.h"
#include "state_machine.h"
#include "buzzer.h"

#define BUZZER_GPIO 14

#define I2C_SDA_PIN 12
#define I2C_SCL_PIN 13
#define LCD_ADDRESS 0x27
#define I2C_PORT    i2c0

int main() {
    stdio_init_all();
    sleep_ms(300);

    // Initialize LCD
    lcd_init(I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, LCD_ADDRESS);
    lcd_backlight(true);
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Booting...");
    lcd_set_cursor(0, 1);
    lcd_print("Please wait");
 station_ctx_t ctx;
    station_init(&ctx);
    // Initialize buzzer
    buzzer_init(BUZZER_GPIO);

    // Initialize station context (handles all other hardware init)
   

    // Main loop
    while (1) {
        station_update(&ctx);
        sleep_ms(20);
    }

    return 0;
}