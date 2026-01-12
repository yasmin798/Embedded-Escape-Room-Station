#include "buttons.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

void arcade_button_init(arcade_button_t *btn) {
    gpio_init(btn->btn_pin);
    gpio_set_dir(btn->btn_pin, GPIO_IN);
    gpio_pull_up(btn->btn_pin);

    gpio_init(btn->led_pin);
    gpio_set_dir(btn->led_pin, GPIO_OUT);
    gpio_put(btn->led_pin, 0);
}

bool arcade_button_read(arcade_button_t *btn) {
    return gpio_get(btn->btn_pin) == 0;
}

void arcade_button_led_on(arcade_button_t *btn) {
    gpio_put(btn->led_pin, 1);
}

void arcade_button_led_off(arcade_button_t *btn) {
    gpio_put(btn->led_pin, 0);
}

bool arcade_button_is_pressed(arcade_button_t *btn) {
    return gpio_get(btn->btn_pin) == 0;
}
