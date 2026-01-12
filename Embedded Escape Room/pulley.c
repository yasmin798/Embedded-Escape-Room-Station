#include "pulley.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// ================== PIN DEFINITIONS ==================
#define ENB_PIN  6   // PWM pin
#define IN3_PIN  27
#define IN4_PIN  26

// ================== MOTOR SETTINGS ==================
#define MAX_PWM        180   // 0–255 (tune)
#define RAMP_DELAY     15    // ms
#define DOOR_MOVE_TIME 8000  // ms (doubled from 4000)

// ================== SETUP ==================
void pulley_init() {
    // Configure direction pins
    gpio_init(IN3_PIN);
    gpio_set_dir(IN3_PIN, GPIO_OUT);
    gpio_init(IN4_PIN);
    gpio_set_dir(IN4_PIN, GPIO_OUT);

    // Configure PWM pin
    gpio_set_function(ENB_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(ENB_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.0f);
    pwm_init(slice_num, &config, true);

    stopMotor();
}

// ================== MOTOR CONTROL ==================
void stopMotor() {
    uint slice_num = pwm_gpio_to_slice_num(ENB_PIN);
    uint channel = pwm_gpio_to_channel(ENB_PIN);
    pwm_set_chan_level(slice_num, channel, 0);
    gpio_put(IN3_PIN, 0);
    gpio_put(IN4_PIN, 0);
}

static void setPWM(uint8_t duty) {
    uint slice_num = pwm_gpio_to_slice_num(ENB_PIN);
    uint channel = pwm_gpio_to_channel(ENB_PIN);
    // Map 0-255 to PWM level (assuming wrap is default 65535)
    uint16_t level = (duty * 65535) / 255;
    pwm_set_chan_level(slice_num, channel, level);
}

// ================== OPEN DOOR ==================
void openDoor() {
    // Direction: OPEN
    gpio_put(IN3_PIN, 1);
    gpio_put(IN4_PIN, 0);

    // Soft start
    for (int pwm = 0; pwm <= MAX_PWM; pwm++) {
        setPWM(pwm);
        sleep_ms(RAMP_DELAY);
    }

    // Run motor
    sleep_ms(DOOR_MOVE_TIME);

    // Soft stop
    for (int pwm = MAX_PWM; pwm >= 0; pwm--) {
        setPWM(pwm);
        sleep_ms(RAMP_DELAY);
    }

    stopMotor();
}

// ================== CLOSE DOOR ==================
void closeDoor() {
    // Direction: CLOSE
    gpio_put(IN3_PIN, 0);
    gpio_put(IN4_PIN, 1);

    // Soft start
    for (int pwm = 0; pwm <= MAX_PWM; pwm++) {
        setPWM(pwm);
        sleep_ms(RAMP_DELAY);
    }

    // Run motor
    sleep_ms(DOOR_MOVE_TIME);

    // Soft stop
    for (int pwm = MAX_PWM; pwm >= 0; pwm--) {
        setPWM(pwm);
        sleep_ms(RAMP_DELAY);
    }

    stopMotor();
}