/**
 * @file buzzer.c
 * @brief Implementation for active piezo buzzer driver.
 */

#include <stdint.h>
#include "buzzer.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

static uint32_t buzzer_pin;

void buzzer_init(uint32_t gpio_pin) {
    buzzer_pin = gpio_pin;
    gpio_init(buzzer_pin);
    gpio_set_dir(buzzer_pin, GPIO_OUT);
    gpio_put(buzzer_pin, 0); // off
}

void buzzer_on(void) {
    gpio_put(buzzer_pin, 1);
}

void buzzer_off(void) {
    gpio_put(buzzer_pin, 0);
}

void buzzer_beep_ms(uint32_t duration_ms) {
    buzzer_on();
    sleep_ms(duration_ms);
    buzzer_off();
}
