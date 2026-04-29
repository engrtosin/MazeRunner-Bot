#pragma once
// =====================================================
// MotorController.h — MazeRunner
//
// PI speed control for a single DC motor with encoder.
// Owns all encoder state. Instantiate as a global.
//
// Usage:
//   motorLeft.init(M1_DIR, M1_PWM, ENC_L, CPR_L, false, MOTOR_KP, MOTOR_KI);
//   motorLeft.setTargetRPM(15.0f);
//   motorLeft.update(dt);   // call every control interval
//
// ISR wiring (Arduino limitation — must be in .ino):
//   void leftISR() { motorLeft.onEncoderPulse(); }
//   attachInterrupt(digitalPinToInterrupt(ENC_L), leftISR, RISING);
// =====================================================

#include <Arduino.h>
#include "Config.h"

class MotorController {
public:
    // ---- Construction ----
    MotorController();

    // ---- Initialisation ----
    // Call once in setup(). Configures all pins and resets state.
    // encPin must be interrupt-capable (pin 2 or 3 on Uno R3).
    // attachInterrupt() stays in .ino — Arduino ISR constraint.
    void init(uint8_t dirPin, uint8_t pwmPin, uint8_t encPin,
              float cpr, bool invertDir,
              float kp, float ki);

    // ---- ISR callback ----
    // Call from the interrupt service routine for this motor's encoder.
    // Wrapper in .ino must be a one-liner:
    //   void leftISR() { motorLeft.onEncoderPulse(); }
    void onEncoderPulse();

    // ---- Main update ----
    // Call once per control interval from loop().
    // dt = elapsed seconds since last call.
    void update(float dt);

    // ---- Commands ----
    void setTargetRPM(float rpm);   // positive = forward only under PI
    void stop();                    // immediate stop — clears integral & PWM

    // ---- Open-loop direct PWM ----
    // Bypasses PI — used for pivot turns and wall-lost recovery.
    // Pass positive for forward, negative for reverse.
    void setPWM(float pwm);

    // ---- Telemetry ----
    float   getMeasuredRPM() const { return _measuredRPM; }
    float   getTargetRPM()   const { return _targetRPM;   }
    float   getPWM()         const { return _pwm;         }
    uint8_t getEncPin()      const { return _encPin;      }

    // ---- Encoder count ----
    // Raw cumulative pulse count — used by FSM for dead-reckoning.
    int32_t getCount() const { return _pulseCount; }

private:
    // Pins
    uint8_t _dirPin;
    uint8_t _pwmPin;
    uint8_t _encPin;

    // Motor geometry
    float _cpr;          // counts per revolution (per-motor, e.g. 342 vs 347)
    bool  _invertDir;    // flip forward/reverse if motor is wired backwards

    // PI gains
    float _kp;
    float _ki;

    // Encoder — _pulseCount written by ISR, must be volatile
    volatile int32_t _pulseCount;
    int32_t          _lastPulseCount;

    // Speed tracking
    float _targetRPM;
    float _measuredRPM;
    float _pwm;

    // PI state
    float _integral;

    // Internal helpers
    void computeRPM(float dt);
    void computePI(float dt);
    void applyPWM(float pwm);
};