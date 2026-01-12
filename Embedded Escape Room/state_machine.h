#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>

#include "servo.h"
#include "ultrasonic_sensor.h"
#include "keypad.h"
#include "buttons.h"

// -------- Pin definitions (with guards so no conflicts) --------
#ifndef SERVO1_PIN
#define SERVO1_PIN 29 //respon for back and fourth
#endif

#ifndef SERVO2_PIN
#define SERVO2_PIN 28 //up and down
#endif

#ifndef TRIGGER_PIN
#define TRIGGER_PIN 25
#endif

#ifndef ECHO_PIN
#define ECHO_PIN    15
#endif

#ifndef HBRIDGE_IN3_PIN
#define HBRIDGE_IN3_PIN 27
#endif

#ifndef HBRIDGE_IN4_PIN
#define HBRIDGE_IN4_PIN 26
#endif

#ifndef HBRIDGE_ENABLE_PIN
#define HBRIDGE_ENABLE_PIN 6
#endif

// (BUZZER_GPIO is defined in main.c for buzzer_init)

// -------- Station states --------
typedef enum {
    ST_INIT = 0,
    ST_CODE_INPUT,
    ST_FINGER_SCAN_WAIT,      // NEW: Wait for CV finger detection
    ST_FINGER_CONFIRM,        // NEW: Wait for user confirmation (Correct/Wrong)
    ST_DISTANCE_HOLD,
    ST_SERVO_MOVE_AND_VERIFY, // UPDATED: Combined movement + continuous verification
    ST_WAIT_FINAL_CONFIRM,    // Renamed from ST_WAIT_CONFIRM for clarity
    ST_NEXT_STAGE,
    ST_FINAL_REWARD,
    ST_ERROR
} station_state_t;

// -------- Context struct --------
typedef struct {
    station_state_t state;
    uint8_t  stage;          // 1 or 2
    uint8_t  targetCount;    // finger count from code digit

    float targetDistance;    // e.g. 80cm
    float tolerance;         // e.g. ±5cm
    float angleTolerance;    // e.g. ±3°

    HCSR04_Config_t ultrasonic;
    keypad_t        keypad;
    arcade_button_t correctBtn;
    arcade_button_t wrongBtn;    // NEW: Wrong button

    uint16_t servo1_target_angle;
    uint16_t servo2_target_angle;

    // NEW: Verification tracking
    uint32_t verification_start_time;  
    bool     verification_in_progress;
    
    // NEW: CV integration
    uint8_t  detected_finger_count;  
    float    cv_angle1;                
    float    cv_angle2;                
    bool     cv_data_ready;
    bool     cv_started;        // first CV pair received

    // NEW: sequential movement phases
    uint8_t  move_phase;        // 0: wait CV, 1: moving to targets
    uint32_t phase_start_time;  // for stability timing per phase

} station_ctx_t;


// -------- API --------
void station_init(station_ctx_t *ctx);
void station_update(station_ctx_t *ctx);

#endif // STATE_MACHINE_H