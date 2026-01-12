/**

 * @brief Driver for an active piezo buzzer using the Pico SDK.
 */

#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize buzzer control.
 *
 * @param gpio_pin GPIO connected to transistor base (through 1k resistor)
 */
void buzzer_init(uint32_t gpio_pin);

/**
 * @brief Turn the buzzer ON.
 */
void buzzer_on(void);

/**
 * @brief Turn the buzzer OFF.
 */
void buzzer_off(void);

/**
 * @brief Beep the buzzer for a duration.
 *
 * @param duration_ms How long to beep (milliseconds)
 */
void buzzer_beep_ms(uint32_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif // BUZZER_H
