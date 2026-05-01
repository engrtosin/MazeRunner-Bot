#pragma once

// =====================================================
// Config.h — MazeRunner
// All pin assignments, physical constants, and tuning
// parameters in one place. Edit here, nowhere else.
// =====================================================

// ================= BOARD =================
// Arduino UNO: Interrupt-capable pins: 2,3


// ================= DEBUG =================
#define DEBUG_SERIAL    0   // set to 0 when communicating with RPi


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

// ================= ROLLER PIN =====================
#define ROLLER_PIN      5

// ================= BREAK BEAM PIN =================
#define BREAK_BEAM_PIN  A5   // Digital input, INPUT_PULLUP
                            // Triggered when reads LOW
inline bool breakBeamTriggered() {
    return digitalRead(BREAK_BEAM_PIN) == LOW;
}


// ================= PHYSICAL CONSTANTS =================
#define WHEEL_DIAMETER_MM   65.0f
#define WHEELBASE_MM        340.0f
#define CPR_L               342.0f      // Left encoder counts per revolution
#define CPR_R               347.0f      // Right encoder counts per revolution


// ================= MOTOR PI GAINS =================
// Inner loop: controls each wheel to a target RPM
#define MOTOR_KP            1.5f
#define MOTOR_KI            0.5f


// ================= WALL FOLLOW PD GAINS =================
// Outer loop: sets target RPMs based on IR distance error
#define WALL_KP             1.0f
#define WALL_KD             0.0f        // Tune upward from zero


// ================= SPEED CONSTANTS =================
#define BASE_SPEED          20.0f       // Normal forward RPM
#define SLOW_SPEED          7.0f        // Approach / cautious RPM
#define TURN_SPEED          7.0f        // Pivot turn RPM (both wheels)
#define ARC_OUTER_SPEED     5.0f        // Right (outer) wheel during arc-left
#define ARC_INNER_SPEED     2.0f        // Left (inner) wheel during arc-left
#define WALL_LOST_OL_SPEED  3.0f        // Open-loop reverse/creep speed (~12 PWM)
#define MAX_SPEED           20.0f       // Hard clamp on any wheel RPM


// ================= WALL FOLLOW THRESHOLDS =================
#define TARGET_LEFT_DIST    20.0f       // Target distance from left wall (cm)
#define FRONT_STOP_DIST     30.0f       // Hard stop / turn threshold (cm)
#define FRONT_SLOW_DIST     30.0f       // Begin slowing threshold (cm)
#define LEFT_OPEN_DIST      20.0f       // Left gap large enough to turn left (cm)
#define WALL_LOST_DIST      50.0f       // Left reading above this → wall lost (cm)
#define WALL_RECOVERY_DIST  30.0f       // Left reading below this → wall reacquired (cm)
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


// ================= BALL COLLECTION =================
// Serial protocol: 3-byte packets [ 0xFF | x_byte | y_byte ]
// x/y are 0-254 (never 0xFF so they can't mimic the header)
#define BALL_TIMEOUT_MS         300     // ms of silence → ball lost
#define BALL_ENTRY_DELAY_MS     300     // settle delay for ball sub-states
 
// Tracking
#define X_CENTER                127     // pixel x midpoint
#define DEAD_BAND                 8     // ignore error smaller than this
#define TRACK_SPEED               7     // RPM forward during tracking
#define MAX_CORRECTION            3     // max RPM differential during tracking
#define KP_HEADING                0.030f
 
// Collection
#define CREEP_SPEED_BALL          3     // RPM forward during collection creep
#define COLLECT_TIMEOUT_MS     3000     // bail if break beam never fires
#define COLLECTED_PAUSE_MS     2000     // pause after collection before resuming


// ================= TIMING =================
#define CONTROL_INTERVAL_MS     100     // Motor PI update period
#define IR_SAMPLE_INTERVAL_US   10000   // IR filter update period (10ms)
#define STATE_ENTRY_DELAY_MS    400     // Settle time on every state/phase transition


// ================= PIVOT TURN GEOMETRY (encoder counts) =================
#define TURN_COUNTS_L       171
#define TURN_COUNTS_R       173


// ================= WALL-LOST RECOVERY GEOMETRY =================
#define REVERSE_MAX_COUNTS_L    262     // ~16 cm max reverse, left wheel
#define REVERSE_MAX_COUNTS_R    265     // ~16 cm max reverse, right wheel
#define CREEP_COUNTS_L          279     // Forward creep before arcing, left
#define CREEP_COUNTS_R          283     // Forward creep before arcing, right
#define REALIGN_COUNTS           82     // Straight-drive settle after arc (unused)
#define ARC_TIMEOUT_MS        10000     // Bail out if arc never finds wall
#define ARC_MAX_COUNTS          327     // Max arc travel (currently unused)