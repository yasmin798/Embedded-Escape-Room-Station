#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// Map function (like Arduino)
long map(long x, long in_min, long in_max, long out_min, long out_max);

// PWM Setup - configures a pin for 50Hz servo control
void servo_pwm_setup(uint servo_pin);

// Servo Control - sets servo pulse width in microseconds
void set_servo_pulse(uint servo_pin, uint16_t pulse_us);

#endif // SERVO_H
