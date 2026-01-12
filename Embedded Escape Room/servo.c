#include "servo.h"

// Map function (like Arduino)
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// --- PWM Setup ---
// Sets up a single PWM pin for 50Hz and 1us tick
void servo_pwm_setup(uint servo_pin) {
    uint slice_num = pwm_gpio_to_slice_num(servo_pin);
    
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0f); 

    pwm_config_set_wrap(&config, 19999); 
    
    pwm_init(slice_num, &config, true);
    gpio_set_function(servo_pin, GPIO_FUNC_PWM);
}

// --- Servo Control ---
void set_servo_pulse(uint servo_pin, uint16_t pulse_us) {
    if(pulse_us < 1000) pulse_us = 1000;
    if(pulse_us > 2000) pulse_us = 2000;
    uint slice_num = pwm_gpio_to_slice_num(servo_pin);
    uint channel = pwm_gpio_to_channel(servo_pin);
    pwm_set_chan_level(slice_num, channel, pulse_us);
}