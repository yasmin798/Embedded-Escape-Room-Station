#include "keypad.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

static const char keymap[4][4] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

void keypad_init(keypad_t *k) {
    for (int r = 0; r < 4; r++) {
        gpio_init(k->row_pins[r]);
        gpio_set_dir(k->row_pins[r], GPIO_OUT);
        gpio_put(k->row_pins[r], 1);
    }
    for (int c = 0; c < 4; c++) {
        gpio_init(k->col_pins[c]);
        gpio_set_dir(k->col_pins[c], GPIO_IN);
        gpio_pull_up(k->col_pins[c]);
    }
}

char keypad_get_key(keypad_t *k) {
    for (int r = 0; r < 4; r++) {
        gpio_put(k->row_pins[r], 0);
        sleep_us(30);
        for (int c = 0; c < 4; c++) {
            if (!gpio_get(k->col_pins[c])) {
                sleep_ms(20);
                while (!gpio_get(k->col_pins[c]));
                gpio_put(k->row_pins[r], 1);
                return keymap[r][c];
            }
        }
        gpio_put(k->row_pins[r], 1);
    }
    return '\0';
}
