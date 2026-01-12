#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include "servo.h"

// Full angle control for continuous rotation servo using camera feedback

typedef struct {
    servo_t *servo;
    float target_angle;
    float current_angle;
    float Kp;            // proportional gain
    float tolerance;     // stop condition, e.g. 2 degrees
} ServoController;

void servo_controller_init(ServoController *ctrl, servo_t *servo);
void servo_goto_angle(ServoController *ctrl, float angle);
void servo_controller_update(ServoController *ctrl, float measured_angle);

#endif
