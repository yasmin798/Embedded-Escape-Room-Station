#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t btn_pin;     // button input pin
    uint32_t led_pin;     // LED output pin
} arcade_button_t;

void arcade_button_init(arcade_button_t *btn);
bool arcade_button_read(arcade_button_t *btn);
void arcade_button_led_on(arcade_button_t *btn);
void arcade_button_led_off(arcade_button_t *btn);
bool arcade_button_is_pressed(arcade_button_t *btn);
#endif
