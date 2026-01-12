#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

// =============== CONSTANTS ===============
#define HCSR04_TRIGGER_PULSE_US       10        // 10 microseconds trigger
#define HCSR04_TIMEOUT_US             30000     // 30ms timeout
#define HCSR04_MEASUREMENT_DELAY_MS   60        // Minimum delay between readings
#define HCSR04_SOUND_SPEED_CM_US      0.0343f   // Speed of sound in cm/us

#define HCSR04_MIN_DISTANCE_CM        2.0f
#define HCSR04_MAX_DISTANCE_CM        400.0f

// =============== STATUS ENUM ===============
typedef enum {
    HCSR04_OK = 0,
    HCSR04_ERROR_TIMEOUT,
    HCSR04_ERROR_OUT_OF_RANGE,
    HCSR04_ERROR_INVALID_PIN,
    HCSR04_ERROR_NOT_INIT
} HCSR04_Status_t;

// =============== SENSOR CONFIG STRUCT ===============
typedef struct {
    uint8_t triggerPin;
    uint8_t echoPin;
    uint32_t lastMeasurementTime;
    bool isInitialized;
} HCSR04_Config_t;

// =============== PUBLIC API ===============
HCSR04_Status_t HCSR04_Init(HCSR04_Config_t* config, uint8_t trigPin, uint8_t echoPin);
HCSR04_Status_t HCSR04_MeasureDistance(HCSR04_Config_t* config, float* distance);
HCSR04_Status_t HCSR04_MeasureDistanceAvg(HCSR04_Config_t* config, float* distance, uint8_t samples);
HCSR04_Status_t HCSR04_CheckInRange(HCSR04_Config_t* config, float targetDistance, float tolerance, bool* isInRange);
HCSR04_Status_t HCSR04_GetDistance(HCSR04_Config_t* config, float* distance);
const char*      HCSR04_GetErrorString(HCSR04_Status_t status);
void             HCSR04_DeInit(HCSR04_Config_t* config);

#endif
