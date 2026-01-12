
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"

#include "state_machine.h"
#include "lcd_i2c.h"
#include "buzzer.h"
#include "servo.h"
#include "pulley.h"

// ...existing code...

// Blocking read for ANGLE:a1,a2 from console, similar to finger count
static bool read_angles_from_console(float *a1, float *a2) {
    static char buffer[64];
    int index = 0;
    while (1) {
        int ch = getchar_timeout_us(0);
        if (ch == PICO_ERROR_TIMEOUT) {
            sleep_ms(10);
            continue;
        }
        char c = (char)ch;
        if (c == '\r') continue;
        if (c == '\n') {
            buffer[index] = '\0';
            if (strncmp(buffer, "ANGLE:", 7) == 0) {
                const char *p = buffer + 7;
                const char *comma = strchr(p, ',');
                if (comma) {
                    char a1_str[24], a2_str[24];
                    int len1 = (int)(comma - p);
                    if (len1 > 0 && len1 < (int)sizeof(a1_str)) {
                        memcpy(a1_str, p, len1);
                        a1_str[len1] = '\0';
                        strncpy(a2_str, comma + 1, sizeof(a2_str) - 1);
                        a2_str[sizeof(a2_str) - 1] = '\0';
                        *a1 = (float)atof(a1_str);
                        *a2 = (float)atof(a2_str);
                        return true;
                    }
                }
            }
            index = 0;
        } else {
            if (index < (int)sizeof(buffer) - 1) buffer[index++] = c;
        }
    }
    return false;
}
#include "state_machine.h"
#include "lcd_i2c.h"
#include "buzzer.h"
#include "servo.h"
#include "pulley.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/timer.h"

#include "hardware/gpio.h"

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#define I2C_SDA_PIN 12
#define I2C_SCL_PIN 13
#define LCD_ADDRESS 0x3F
#define I2C_PORT    i2c0

// Small helper to print two lines on LCD
static void LCDmsg(const char *l1, const char *l2) {
  //  lcd_init(I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, LCD_ADDRESS);
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print(l1);
    lcd_set_cursor(0, 1);
    lcd_print(l2);
}

// Helper to check if both distance and angles are within tolerance
// Lookup table for 9 target angle pairs and their corresponding (c1, c2) values
typedef struct {
    int angle1;
    int angle2;
    float c1;
    float c2;
} target_angle_t;

static const target_angle_t target_angles[9] = {
    {40,  90,  89.0f, 8.0f},
    {50,  99,  89.0f, 8.0f},
    {60, 108,  77.7f, 8.0f},
    {70, 117,  70.0f, 8.0f},
    {80, 126,  62.6f, 8.0f},
    {90, 135,  52.7f, 8.2f},
    {100,144,  43.1f, 8.2f},
    {110,153,  38.3f, 6.0f},
    {120,162,  38.2f, 6.0f}
};

static bool check_targets_met(station_ctx_t *ctx, float current_distance) {
    // Check distance
    bool distance_ok = (current_distance >= ctx->targetDistance - ctx->tolerance &&
                       current_distance <= ctx->targetDistance + ctx->tolerance);

    // For both Stage 1 and Stage 2, use finger count to select target and check c1/c2
    bool angles_ok = false;
    if ((ctx->stage == 1 || ctx->stage == 2) && ctx->detected_finger_count >= 1 && ctx->detected_finger_count <= 9) {
        int idx = ctx->detected_finger_count - 1;
        float expected_c1 = target_angles[idx].c1;
        float expected_c2 = target_angles[idx].c2;
        float tol = 10.0f; // 10 degrees tolerance
        if (ctx->cv_data_ready) {
            float c1_diff = ctx->cv_angle1 - expected_c1;
            float c2_diff = ctx->cv_angle2 - expected_c2;
            if (c1_diff < 0) c1_diff = -c1_diff;
            if (c2_diff < 0) c2_diff = -c2_diff;
            angles_ok = (c1_diff <= tol && c2_diff <= tol);
        }
    } else {
        // For other stages or if finger count invalid, fallback to original logic
        if (ctx->cv_data_ready) {
            float angle1_diff = ctx->cv_angle1 - ctx->servo1_target_angle;
            float angle2_diff = ctx->cv_angle2 - ctx->servo2_target_angle;
            if (angle1_diff < 0) angle1_diff = -angle1_diff;
            if (angle2_diff < 0) angle2_diff = -angle2_diff;
            angles_ok = (angle1_diff <= ctx->angleTolerance && angle2_diff <= ctx->angleTolerance);
        }
    }
    return distance_ok && angles_ok;
}

void station_init(station_ctx_t *ctx) {
    // Basic state
   
    // -------------------- UART FOR CV DATA --------------------
    stdio_init_all();
// CORRECT - matches Python's 115200
uart_init(uart0, 115200);



// UART TX (GPIO 0) and RX (GPIO 1)
gpio_set_function(0, GPIO_FUNC_UART);
gpio_set_function(1, GPIO_FUNC_UART);

// Optional: disable flow control
uart_set_hw_flow(uart0, false, false);

// Optional: set 8N1 (default)
uart_set_format(uart0, 8, 1, UART_PARITY_NONE);

    ctx->state  = ST_FINAL_REWARD;  // Start directly from pulley/final reward state
    ctx->stage  = 1;
    ctx->targetDistance = 80.0f;
    ctx->tolerance      = 5.0f;
    ctx->angleTolerance = 10.0f;
    ctx->targetCount    = 0;
    ctx->servo1_target_angle = 0;
    ctx->servo2_target_angle = 0;
    
    ctx->verification_start_time = 0;
    ctx->verification_in_progress = false;
    ctx->detected_finger_count = 0;
    ctx->cv_angle1 = 0.0f;
    ctx->cv_angle2 = 0.0f;
    ctx->cv_data_ready = false;
    ctx->cv_started = false;
    ctx->move_phase = 0; // wait for CV
    ctx->phase_start_time = 0;

    // ----- Servo init (PWM-based) -----
    servo_pwm_setup(SERVO1_PIN);  // GPIO15 - up/down
    servo_pwm_setup(SERVO2_PIN);  // GPIO25 - back/forth

    // ----- Pulley/Door motor init -----
    pulley_init();

    // ----- Ultrasonic init -----
    HCSR04_Init(&ctx->ultrasonic, TRIGGER_PIN, ECHO_PIN);

    // ----- Keypad pin mapping -----
    ctx->keypad.row_pins[0] = 17;
    ctx->keypad.row_pins[1] = 18;
    ctx->keypad.row_pins[2] = 19;
    ctx->keypad.row_pins[3] = 20;

    ctx->keypad.col_pins[0] = 21;
    ctx->keypad.col_pins[1] = 5;
    ctx->keypad.col_pins[2] = 7;
    ctx->keypad.col_pins[3] = 4;

    keypad_init(&ctx->keypad);

    // ----- Correct button -----
    ctx->correctBtn.btn_pin = 16;
    //ctx->correctBtn.led_pin = 16; //will use for servo
    arcade_button_init(&ctx->correctBtn);

    // ----- Wrong button (NEW) -----
    // Avoid conflict with SERVO1_PIN=15 (PWM). Use a free GPIO (e.g., 8).
    ctx->wrongBtn.btn_pin = 8;
    //ctx->wrongBtn.led_pin = 1; //will use for servo
    arcade_button_init(&ctx->wrongBtn);

    // Start screen - skipping for pulley test
  //  LCDmsg("Station Ready", "Enter Code...");
  //lcd_set_cursor(0, 0);
  //  lcd_print("Station Ready");
  //  lcd_set_cursor(0, 1);
  //  lcd_print( "Enter Code...");
    
    // DON'T override state - we set it to ST_FINAL_REWARD above
    // ctx->state = ST_CODE_INPUT;
}
static int read_finger_count_from_uart() {
    // Read from USB CDC stdio (console), not GPIO UART
    static char buffer[64];
    static int index = 0;

    while (true) {
        int ch = getchar_timeout_us(0); // non-blocking
        if (ch == PICO_ERROR_TIMEOUT) break;

        char c = (char)ch;
        if (c == '\r') continue;

        if (c == '\n') {
            buffer[index] = '\0';
            // Parse patterns like "FINGER:5", "CONFIRMED: 5", or "Fingers: 5"
            int val = -1;
            if (strncmp(buffer, "FINGER:", 7) == 0) {
                val = atoi(buffer + 7);
            } else if (strncmp(buffer, "CONFIRMED:", 10) == 0) {
                val = atoi(buffer + 10);
            } else {
                // Fallback: find first number in the line
                for (int i = 0; buffer[i]; ++i) {
                    if (buffer[i] >= '0' && buffer[i] <= '9') {
                        val = atoi(&buffer[i]);
                        break;
                    }
                }
            }

            index = 0;
            if (val >= 0) return val;
        } else {
            if (index < (int)sizeof(buffer) - 1) buffer[index++] = c;
        }
    }
    return -1;
}

// Parse ANGLE:a1,a2 from USB CDC console and store in ctx->cv_angle1/2.
// Returns true if new angles were parsed in this call.
// Robust angle parser with debug output
static bool parse_console_angles(station_ctx_t *ctx) {
    static char buf[64];
    static int idx = 0;
    bool updated = false;

    while (true) {
        int ch = getchar_timeout_us(0);
        if (ch == PICO_ERROR_TIMEOUT) break;
        char c = (char)ch;
        if (c == '\r') continue;
        if (c == '\n') {
            buf[idx] = '\0';
            printf("[ANGLE DEBUG] Received: '%s'\n", buf); // Debug: print every line
            // Look for ANGLE: prefix anywhere in the line (robust)
            char *start = strstr(buf, "ANGLE:");
            if (start) {
                char *p = start + 6;
                while (*p == ' ' || *p == '\t') p++; // skip whitespace
                char *comma = strchr(p, ',');
                if (comma) {
                    char a1_str[24], a2_str[24];
                    int len1 = (int)(comma - p);
                    if (len1 > 0 && len1 < (int)sizeof(a1_str)) {
                        memcpy(a1_str, p, len1);
                        a1_str[len1] = '\0';
                        strncpy(a2_str, comma + 1, sizeof(a2_str) - 1);
                        a2_str[sizeof(a2_str) - 1] = '\0';
                        float a1 = (float)atof(a1_str);
                        float a2 = (float)atof(a2_str);
                        ctx->cv_angle1 = a1;
                        ctx->cv_angle2 = a2;
                        ctx->cv_data_ready = true;
                        printf("[ANGLE PARSED] %.2f, %.2f\n", a1, a2);
                        updated = true;
                    }
                }
            }
            idx = 0;
        } else {
            if (idx < (int)sizeof(buf) - 1) buf[idx++] = c;
        }
    }
    return updated;
}


void station_update(station_ctx_t *ctx) {

    switch (ctx->state) {


    // --------------------------------------------------
    case ST_CODE_INPUT:
    // --------------------------------------------------
    {
        char line1[17];
        snprintf(line1, sizeof(line1), "STAGE %d | CODE:", ctx->stage);
        LCDmsg(line1, "_ _ _ _");
        
        char code[5] = {0};
        int  idx = 0;

        while (idx < 4) {
            char k = keypad_get_key(&ctx->keypad);
            if (k != '\0' && k >= '0' && k <= '9') {
                code[idx++] = k;

                char line2[17];
                snprintf(line2, sizeof(line2), "%c %c %c %c",
                         code[0] ? code[0] : '_',
                         code[1] ? code[1] : '_',
                         code[2] ? code[2] : '_',
                         code[3] ? code[3] : '_');
                lcd_set_cursor(0, 1);
                lcd_print(line2);
            }
            sleep_ms(20);
        }

        // Extract D1 and D2 for servo target angles
        uint8_t d1 = (uint8_t)(code[0] - '0');
        uint8_t d2 = (uint8_t)(code[1] - '0');
        
        // Calculate target angles based on D1 and D2
        ctx->servo1_target_angle = d1 * 10;  // D1 × 10 (e.g. 4→40°)
        ctx->servo2_target_angle = d2 * 9;  // D2 × 20 (e.g. 3→60°)

        // Extract target count for current stage (D1 or D2 based on stage)
        ctx->targetCount = (uint8_t)(code[ctx->stage - 1] - '0');

        // Calculate target distance from D4
        uint8_t d4 = (uint8_t)(code[3] - '0');
        ctx->targetDistance = d4 * 20.0f;  // e.g. 4→80cm

        LCDmsg("Code OK!", "Starting...");
        sleep_ms(1000);

        ctx->state = ST_FINGER_SCAN_WAIT;
        break;
    }

    // --------------------------------------------------
case ST_FINGER_SCAN_WAIT:
{
    char line1[17];
    char line2[17];
    snprintf(line1, sizeof(line1), "STAGE %d|TGT:%d", ctx->stage, ctx->targetCount);
    snprintf(line2, sizeof(line2), "SHOW HAND 5s");
    LCDmsg(line1, line2);

    arcade_button_led_on(&ctx->correctBtn);
    arcade_button_led_on(&ctx->wrongBtn);

    uint32_t start = to_ms_since_boot(get_absolute_time());
    printf("START_FINGER_SCAN\n");  // Debug message to USB terminal

    while (true) {
        // Read from UART (where Python sends data)
        int fc = read_finger_count_from_uart();  // ← Changed function name
        
        if (fc != -1) {
            ctx->detected_finger_count = fc;
            printf("Received finger count: %d\n", fc);  // Debug
            ctx->state = ST_FINGER_CONFIRM;
            break;
        }

        // Timeout after 10 seconds
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - start > 10000) {
            LCDmsg("NO CV INPUT!", "Retry...");
            sleep_ms(1000);
            break;
        }

        sleep_ms(20);
    }

    break;
}

    // --------------------------------------------------
    case ST_FINGER_CONFIRM:
    // --------------------------------------------------
    {
        char line1[17];
        char line2[17];
        // If CV sent an invalid value (10), show error and request a new scan
        if (ctx->detected_finger_count == 10) {
            LCDmsg("INPUT ERROR", "Scan again");
            sleep_ms(1000);
            ctx->state = ST_FINGER_SCAN_WAIT;
            break;
        }
        snprintf(line1, sizeof(line1), "DETECTED: %d", ctx->detected_finger_count);
        snprintf(line2, sizeof(line2), "CORRECT or WRONG");
        LCDmsg(line1, line2);

        // Wait for user confirmation
        while (1) {
            if (arcade_button_is_pressed(&ctx->correctBtn)) {
                arcade_button_led_off(&ctx->correctBtn);
                arcade_button_led_off(&ctx->wrongBtn);
                
                // Check if detection matches target
                if (ctx->detected_finger_count == ctx->targetCount) {
                    // buzzer_beep_ms(100);
                    sleep_ms(300);
                    ctx->state = ST_DISTANCE_HOLD;
                    break;
                } else {
                    // Correct button pressed but wrong count
                    LCDmsg("MISMATCH!", "Try Again");
                    // buzzer_beep_ms(200);
                    sleep_ms(1000);
                    ctx->state = ST_FINGER_SCAN_WAIT;
                    break;
                }
            }
            
            if (arcade_button_is_pressed(&ctx->wrongBtn)) {
                arcade_button_led_off(&ctx->correctBtn);
                arcade_button_led_off(&ctx->wrongBtn);
                
                // User rejected the detection, retry
                LCDmsg("Retrying...", "");
                // buzzer_beep_ms(150);
                sleep_ms(800);
                ctx->state = ST_FINGER_SCAN_WAIT;
                break;
            }
            
            sleep_ms(20);
        }
        break;
    }

    // --------------------------------------------------
    case ST_DISTANCE_HOLD:
    // --------------------------------------------------
    {
        float d;
        HCSR04_Status_t st = HCSR04_MeasureDistance(&ctx->ultrasonic, &d);

        if (st != HCSR04_OK) {
            char err_line[17];
            snprintf(err_line, sizeof(err_line), "US ERR: %s", HCSR04_GetErrorString(st));
            LCDmsg("Sensor Error", err_line);
            sleep_ms(500);
            break;  // Retry measurement
        }

        char line1[17];
        char line2[17];
        snprintf(line1, sizeof(line1), "TGT DIST: %.0f CM", ctx->targetDistance);
        snprintf(line2, sizeof(line2), "CUR: %.1f CM", d);
        LCDmsg(line1, line2);

        // Check if distance is in range
        if (d >= ctx->targetDistance - ctx->tolerance &&
            d <= ctx->targetDistance + ctx->tolerance) {

            // buzzer_beep_ms(100);
            LCDmsg("Distance OK!", "Moving Arm...");
            sleep_ms(500);

            ctx->verification_in_progress = false;
            ctx->cv_started = false;  // Reset to allow new CV tracking
            ctx->cv_data_ready = false;  // Reset CV data for this stage
            ctx->state = ST_SERVO_MOVE_AND_VERIFY;
        }

        break;
    }

    // --------------------------------------------------
   // --------------------------------------------------
case ST_SERVO_MOVE_AND_VERIFY:
// --------------------------------------------------
{
    // 1) Command servos to target positions immediately
    // Convert target angles to pulse widths (500-1500us for 0-180°)
    uint16_t pulse1 = 1000 + (uint16_t)((ctx->servo1_target_angle  / 180.0f) * 1000) + (40/180) * 1000;
    uint16_t pulse2 = 1000 + (uint16_t)((ctx->servo2_target_angle / 180.0f) * 1000) + (90/180) * 1000;
    
    // Command servos to targets every loop iteration
    set_servo_pulse(SERVO1_PIN, pulse1);
    set_servo_pulse(SERVO2_PIN, pulse2);

    // 2) Read distance from ultrasonic sensor
    float current_distance;
    HCSR04_Status_t st = HCSR04_MeasureDistance(&ctx->ultrasonic, &current_distance);

    if (st != HCSR04_OK) {
        LCDmsg("Distance Lost!", "Reposition");
        // buzzer_beep_ms(200);
        ctx->verification_in_progress = false;
        ctx->state = ST_DISTANCE_HOLD;
        break;
    }



    // 3) Wait for CV angles from console (like finger count)
    ctx->cv_data_ready = false;
    bool done = false;
    while (!done) {
        parse_console_angles(ctx);
        float current_distance_live;
        HCSR04_Status_t st_live = HCSR04_MeasureDistance(&ctx->ultrasonic, &current_distance_live);
        char line1[17];
        char line2[17];
        snprintf(line1, sizeof(line1), "D:%.0f C1:%.0f C2:%.0f",
                 current_distance_live, ctx->cv_angle1, ctx->cv_angle2);
        snprintf(line2, sizeof(line2), "T:%.0f T1:%d T2:%d",
                 ctx->targetDistance, ctx->servo1_target_angle, ctx->servo2_target_angle);
        LCDmsg(line1, line2);

        // Mark CV started when first pair arrives
        if (ctx->cv_data_ready && !ctx->cv_started) {
            ctx->cv_started = true;
        }

        // Verify: Check if CV-detected angles match target angles
        bool targets_met = check_targets_met(ctx, current_distance_live);
        if (targets_met) {
            if (!ctx->verification_in_progress) {
                ctx->verification_in_progress = true;
                ctx->verification_start_time = to_ms_since_boot(get_absolute_time());
            } else {
                uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - ctx->verification_start_time;
                if (elapsed >= 3000) {
                    // buzzer_beep_ms(300);
                    sleep_ms(100);
                    // buzzer_beep_ms(300);
                    LCDmsg("TARGET REACHED!", "CONFIRM (GREEN)");
                    ctx->state = ST_WAIT_FINAL_CONFIRM;
                    done = true;
                    break;
                }
            }
        } else {
            ctx->verification_in_progress = false;
        }
        sleep_ms(40);
    }
    break;
}


    // --------------------------------------------------
    case ST_WAIT_FINAL_CONFIRM:
    // --------------------------------------------------
    {
        arcade_button_led_on(&ctx->correctBtn);
        
        if (arcade_button_is_pressed(&ctx->correctBtn)) {
            arcade_button_led_off(&ctx->correctBtn);
            // buzzer_beep_ms(150);
            sleep_ms(500);
            ctx->state = ST_NEXT_STAGE;
        }
        break;
    }

    // --------------------------------------------------
    case ST_NEXT_STAGE:
    // --------------------------------------------------
    {
        // Skip Stage 2, go directly to pulley/final reward after Stage 1
        // If you want to re-enable Stage 2, uncomment the code below
        /*
        if (ctx->stage == 1) {
            ctx->stage = 2;
            LCDmsg("Stage 1 OK", "Starting Stage 2");
            sleep_ms(1500);
            // Reset CV data for next stage
            ctx->cv_data_ready = false;
            ctx->cv_started = false;
            ctx->detected_finger_count = 0;
            ctx->verification_in_progress = false;
            // Stage 2 uses D2 from the same code (already stored in ctx->servo2_target_angle)
            // targetCount already set to D2 during code input for stage 2
            ctx->state = ST_FINGER_SCAN_WAIT;  // Go directly to finger scan, skip code input
        } else {
            ctx->state = ST_FINAL_REWARD;
        }
        break;
        */
        ctx->state = ST_FINAL_REWARD;
        break;
    }

    // --------------------------------------------------
    case ST_FINAL_REWARD:
    // --------------------------------------------------
    {
  // Ensure door is closed before unlocking
        LCDmsg("UNLOCKING...", "Stand Back!");
        // buzzer_beep_ms(200);
        sleep_ms(1000);

        // Open door using pulley with smooth ramping
        openDoor();
        LCDmsg("MISSION COMPLETE", "Clue Revealed!");
        // buzzer_beep_ms(200);
        sleep_ms(1800);
        // buzzer_beep_ms(200);
        sleep_ms(200);

        closeDoor();
        LCDmsg("DOOR CLOSED", "Thank You!");
        // buzzer_beep_ms(400);
        // Final state - infinite loop
        while (1) {
            sleep_ms(1000);
        }
    }

    // --------------------------------------------------
    case ST_ERROR:
    // --------------------------------------------------
    {
        LCDmsg("SYSTEM ERROR", "Reset Required");
        // buzzer_beep_ms(400);
        sleep_ms(1000);
        
        // Stay in error state
        break;
    }

    default:
        ctx->state = ST_ERROR;
        break;
    }
}