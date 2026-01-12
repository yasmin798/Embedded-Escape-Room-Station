#ifndef KEYPAD_H
#define KEYPAD_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t row_pins[4];
    uint32_t col_pins[4];
} keypad_t;

void keypad_init(keypad_t *k);
char keypad_get_key(keypad_t *k);

#endif
