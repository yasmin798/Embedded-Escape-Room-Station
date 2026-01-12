/**
 * @file ultrasonic_sensor.c
 * @brief HC-SR04 Ultrasonic Distance Sensor Driver for Pico SDK
 * @author
 * @date 2025
 */

#include "ultrasonic_sensor.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ====================== PRIVATE HELPERS ======================
static uint32_t HCSR04_PulseIn(uint8_t pin, uint8_t state, uint32_t timeout_us);
static float HCSR04_CalculateDistance(uint32_t duration_us);

// =============================================================
//                    INITIALIZATION
// =============================================================
HCSR04_Status_t HCSR04_Init(HCSR04_Config_t* config, uint8_t trigPin, uint8_t echoPin) {

    if (config == NULL)
        return HCSR04_ERROR_INVALID_PIN;

    // Validate GPIO range
    if (trigPin > 29 || echoPin > 29)
        return HCSR04_ERROR_INVALID_PIN;

    config->triggerPin = trigPin;
    config->echoPin = echoPin;
    config->isInitialized = false;
    config->lastMeasurementTime = 0;

    // Init GPIO pins
    gpio_init(trigPin);
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_put(trigPin, 0);

    gpio_init(echoPin);
    gpio_set_dir(echoPin, GPIO_IN);
    gpio_pull_down(echoPin);

    sleep_us(5);

    config->isInitialized = true;
    return HCSR04_OK;
}

// =============================================================
//                    SINGLE DISTANCE MEASUREMENT
// =============================================================
HCSR04_Status_t HCSR04_MeasureDistance(HCSR04_Config_t* config, float* distance) {

    if (!config->isInitialized)
        return HCSR04_ERROR_NOT_INIT;

    if (distance == NULL)
        return HCSR04_ERROR_INVALID_PIN;

    uint32_t now = to_ms_since_boot(get_absolute_time());

    // Enforce minimum delay between measurements
    if (now - config->lastMeasurementTime < HCSR04_MEASUREMENT_DELAY_MS) {
        sleep_ms(HCSR04_MEASUREMENT_DELAY_MS - (now - config->lastMeasurementTime));
    }

    // Trigger pulse
    gpio_put(config->triggerPin, 0);
    sleep_us(2);
    gpio_put(config->triggerPin, 1);
    sleep_us(HCSR04_TRIGGER_PULSE_US);
    gpio_put(config->triggerPin, 0);

    // Read duration
    uint32_t pulse_us = HCSR04_PulseIn(config->echoPin, 1, HCSR04_TIMEOUT_US);

    config->lastMeasurementTime = to_ms_since_boot(get_absolute_time());

    if (pulse_us == 0)
        return HCSR04_ERROR_TIMEOUT;

    float dist = HCSR04_CalculateDistance(pulse_us);
    *distance = dist;

    if (dist < HCSR04_MIN_DISTANCE_CM || dist > HCSR04_MAX_DISTANCE_CM)
        return HCSR04_ERROR_OUT_OF_RANGE;

    return HCSR04_OK;
}

// =============================================================
//                   MULTI-SAMPLE AVERAGING
// =============================================================
HCSR04_Status_t HCSR04_MeasureDistanceAvg(HCSR04_Config_t* config, float* distance, uint8_t samples) {

    if (!config->isInitialized)
        return HCSR04_ERROR_NOT_INIT;

    if (distance == NULL || samples == 0 || samples > 10)
        return HCSR04_ERROR_INVALID_PIN;

    float sum = 0;
    uint8_t valid = 0;

    for (uint8_t i = 0; i < samples; i++) {
        float d;
        HCSR04_Status_t st = HCSR04_MeasureDistance(config, &d);

        if (st == HCSR04_OK) {
            sum += d;
            valid++;
        }
        sleep_ms(10);
    }

    if (valid == 0)
        return HCSR04_ERROR_TIMEOUT;

    *distance = sum / valid;

    return HCSR04_OK;
}

// =============================================================
//                     CHECK IN RANGE
// =============================================================
HCSR04_Status_t HCSR04_CheckInRange(
    HCSR04_Config_t* config,
    float target,
    float tolerance,
    bool* isInRange
) {
    if (isInRange == NULL)
        return HCSR04_ERROR_INVALID_PIN;

    float d;
    HCSR04_Status_t st = HCSR04_MeasureDistance(config, &d);

    if (st != HCSR04_OK) {
        *isInRange = false;
        return st;
    }

    *isInRange = (d >= target - tolerance) && (d <= target + tolerance);
    return HCSR04_OK;
}

HCSR04_Status_t HCSR04_GetDistance(HCSR04_Config_t* config, float* distance) {
    return HCSR04_MeasureDistance(config, distance);
}

// =============================================================
//                   ERROR → TEXT STRING
// =============================================================
const char* HCSR04_GetErrorString(HCSR04_Status_t status) {
    switch (status) {
        case HCSR04_OK:                return "OK";
        case HCSR04_ERROR_TIMEOUT:     return "Echo timeout";
        case HCSR04_ERROR_OUT_OF_RANGE:return "Out of range";
        case HCSR04_ERROR_INVALID_PIN: return "Invalid pin";
        case HCSR04_ERROR_NOT_INIT:    return "Sensor not initialized";
        default:                       return "Unknown error";
    }
}

// =============================================================
//                  DEINITIALIZE SENSOR
// =============================================================
void HCSR04_DeInit(HCSR04_Config_t* config) {
    if (config) {
        gpio_set_dir(config->triggerPin, GPIO_IN);
        gpio_set_dir(config->echoPin, GPIO_IN);
        config->isInitialized = false;
    }
}

// =============================================================
//               PRIVATE: pulseIn() alternative
// =============================================================
static uint32_t HCSR04_PulseIn(uint8_t pin, uint8_t state, uint32_t timeout_us) {

    uint32_t start = time_us_32();

    // wait for signal to go HIGH
    while (gpio_get(pin) != state) {
        if (time_us_32() - start > timeout_us)
            return 0;
    }

    uint32_t pulse_start = time_us_32();

    // measure pulse HIGH duration
    while (gpio_get(pin) == state) {
        if (time_us_32() - pulse_start > timeout_us)
            return 0;
    }

    return time_us_32() - pulse_start;
}

// =============================================================
//               PRIVATE: Distance Calculation
// =============================================================
static float HCSR04_CalculateDistance(uint32_t duration_us) {
    // distance = speed * time / 2
    return (duration_us * HCSR04_SOUND_SPEED_CM_US) / 2.0f;
}
