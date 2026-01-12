#include "servo_controller.h"
#include "servo.h"
#include <math.h>

void servo_controller_init(ServoController *ctrl, servo_t *servo) {
    ctrl->servo = servo;
    ctrl->target_angle = 90;   // start at neutral
    ctrl->current_angle = 90;
    ctrl->Kp = 1.2f;           // Higher gain for continuous rotation servos
    ctrl->tolerance = 2.0f;    // degrees
}

void servo_goto_angle(ServoController *ctrl, float angle) {
    ctrl->target_angle = angle;
}

void servo_controller_update(ServoController *ctrl, float measured_angle) {
    ctrl->current_angle = measured_angle;

    // Compute error
    float error = ctrl->target_angle - ctrl->current_angle;

    // If close enough, hold/stop based on servo type
    if (fabsf(error) < ctrl->tolerance) {
        if (ctrl->servo->type == SERVO_TYPE_POSITIONAL) {
            // Positional: hold at target
            servo_set_angle(ctrl->servo, (uint16_t)ctrl->target_angle);
        } else {
            // Continuous: stop at neutral
            servo_set_angle(ctrl->servo, 90);
        }
        return;
    }

    // Control based on servo type
    if (ctrl->servo->type == SERVO_TYPE_POSITIONAL) {
        // Direct position control for 180° servo
        // Use small incremental steps for smooth CV-driven movement
        float step = ctrl->Kp * error;
        if (step > 3.0f) step = 3.0f;
        if (step < -3.0f) step = -3.0f;
        
        float next_angle = ctrl->current_angle + step;
        if (next_angle < 0) next_angle = 0;
        if (next_angle > 180) next_angle = 180;
        
        servo_set_angle(ctrl->servo, (uint16_t)next_angle);
    } else {
        // Speed-based control for 360° continuous rotation
        int speed = (int)(ctrl->Kp * error);
        if (speed > 100) speed = 100;
        if (speed < -100) speed = -100;
        
        uint16_t angle = 90 + (speed * 90) / 100;
        servo_set_angle(ctrl->servo, angle);
    }
}
