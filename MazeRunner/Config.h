#pragma once

// =====================================================
// Config.h — MazeRunner
// All pin assignments, physical constants, and tuning
// parameters in one place. Edit here, nowhere else.
// =====================================================


// ================= BOARD =================
// Arduino UNO: Interrupt-capable pins: 2,3


// ================= MOTOR DRIVER PINS =================
#define EN_PIN      4       // Motor driver enable (active HIGH)
#define SF_PIN      12      // Motor driver fault (active LOW)

#define M1_DIR      7       // Left motor direction
#define M1_PWM      9       // Left motor PWM
#define M2_DIR      8       // Right motor direction
#define M2_PWM      10      // Right motor PWM


// ================= ENCODER PINS =================
// Must be interrupt-capable pins
#define ENC_L       2       // Left encoder (INT0)
#define ENC_R       3       // Right encoder (INT1)


// ================= IR SENSOR PINS =================
#define FRONT_IR_PIN    A2
#define LEFT_IR_PIN     A3


// ================= BREAK BEAM PIN =================
#define BREAK_BEAM_PIN  A5   // Digital input, INPUT_PULLUP
                            // Triggered when reads LOW

inline bool breakBeamTriggered() {
    return digitalRead(BREAK_BEAM_PIN) == LOW;
}


// ================= PHYSICAL CONSTANTS =================
#define WHEEL_DIAMETER_MM   65.0f
#define WHEELBASE_MM        340.0f
#define CPR                 360.0f      // Encoder counts per revolution


// ================= MOTOR PI GAINS =================
// Inner loop: controls each wheel to a target RPM
#define MOTOR_KP            1.5f
#define MOTOR_KI            0.5f


// ================= WALL FOLLOW PD GAINS =================
// Outer loop: sets target RPMs based on IR distance error
#define WALL_KP             1.0f
#define WALL_KD             0.0f        // Tune upward from zero


// ================= SPEED CONSTANTS =================
#define BASE_SPEED          15.0f       // Normal forward RPM
#define SLOW_SPEED          7.0f        // Approach / cautious RPM
#define TURN_OUTER_RPM      5.0f        // Outer wheel during 90-deg turn
#define TURN_INNER_RPM      1.0f        // Inner wheel during 90-deg turn
#define MAX_SPEED           20.0f       // Hard clamp on any wheel RPM


// ================= WALL FOLLOW THRESHOLDS =================
#define TARGET_LEFT_DIST    10.0f       // Target distance from left wall (cm)
#define FRONT_STOP_DIST     25.0f       // Hard stop threshold (cm)
#define FRONT_SLOW_DIST     30.0f       // Begin slowing threshold (cm)
#define LEFT_OPEN_DIST      20.0f       // Left gap large enough to turn left (cm)
#define WALL_CORRECTION_MAX 5.0f        // Max RPM correction from PD (clamp)


// ================= IR CALIBRATION =================
// Personal note: We use our sensor 2 for front and sensor 1 for left.
// Model: distance = A / (adc + B) + C
// Fit from sensor calibration data

#define FRONT_IR_A      1.23072369e+04f
#define FRONT_IR_B      1.12642133e+01f
#define FRONT_IR_C      1.74338869e+00f

#define LEFT_IR_A       7.68904930e+03f
#define LEFT_IR_B       1.00000065e-03f
#define LEFT_IR_C      -2.64920279e+00f


// ================= IR FILTER =================
// 4th-order Butterworth (two biquad stages) + moving average

#define MA_WINDOW       6               // Moving average window size

// Biquad stage 1 coefficients
#define BIQUAD1_B0      2.91464944656977e-05f
#define BIQUAD1_B1      2.91464944656977e-05f
#define BIQUAD1_B2      0.0f
#define BIQUAD1_A1     -0.939062505817492f
#define BIQUAD1_A2      0.0f

// Biquad stage 2 coefficients
#define BIQUAD2_B0      1.0f
#define BIQUAD2_B1      2.0f
#define BIQUAD2_B2      1.0f
#define BIQUAD2_A1     -1.93529438685999f
#define BIQUAD2_A2      0.939120798806424f


// ================= TIMING =================
#define CONTROL_INTERVAL_MS     100     // Motor PI update period
#define IR_SAMPLE_INTERVAL_US   10000   // IR filter update period (10ms)