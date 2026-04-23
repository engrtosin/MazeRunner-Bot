// =====================================================
// MotorController.cpp — MazeRunner
// =====================================================

#include "MotorController.h"

// -------------------------------------------------------
// Construction
// -------------------------------------------------------

MotorController::MotorController()
    : _dirPin(0), _pwmPin(0), _encPin(0),
      _kp(0.0f), _ki(0.0f),
      _pulseCount(0), _lastPulseCount(0),
      _targetRPM(0.0f), _measuredRPM(0.0f),
      _pwm(0.0f), _integral(0.0f)
{}

// -------------------------------------------------------
// Initialisation
// -------------------------------------------------------

void MotorController::init(uint8_t dirPin, uint8_t pwmPin, uint8_t encPin,
                           float kp, float ki) {
    _dirPin = dirPin;
    _pwmPin = pwmPin;
    _encPin = encPin;
    _kp     = kp;
    _ki     = ki;

    // Motor driver pins
    pinMode(_dirPin, OUTPUT);
    pinMode(_pwmPin, OUTPUT);

    // Encoder pin — pullup so it reads HIGH at rest
    // attachInterrupt() is handled in .ino (Arduino constraint)
    pinMode(_encPin, INPUT_PULLUP);

    applyPWM(0.0f);
}

// -------------------------------------------------------
// ISR callback — must stay minimal, called from interrupt
// -------------------------------------------------------

void MotorController::onEncoderPulse() {
    _pulseCount++;
}

// -------------------------------------------------------
// Main update — call once per control interval
// -------------------------------------------------------

void MotorController::update(float dt) {
    computeRPM(dt);
    computePI();
}

// -------------------------------------------------------
// Commands
// -------------------------------------------------------

void MotorController::setTargetRPM(float rpm) {
    _targetRPM = rpm;
}

void MotorController::stop() {
    _targetRPM    = 0.0f;
    _integral     = 0.0f;
    _pwm          = 0.0f;
    applyPWM(0.0f);
}

// -------------------------------------------------------
// Private: RPM measurement from encoder pulses
// -------------------------------------------------------

void MotorController::computeRPM(float dt) {
    // Atomically snapshot the ISR-written pulse count
    noInterrupts();
    int32_t count = _pulseCount;
    interrupts();

    int32_t delta   = count - _lastPulseCount;
    _lastPulseCount = count;

    // pulses / CPR = revolutions, over dt seconds → RPM
    _measuredRPM = ((float)delta / CPR) / dt * 60.0f;
}

// -------------------------------------------------------
// Private: PI control — outputs updated PWM
// -------------------------------------------------------

void MotorController::computePI() {
    float error = _targetRPM - _measuredRPM;

    // Accumulate integral with anti-windup clamp.
    // Limits integral so its contribution never exceeds
    // half the PWM range in either direction.
    _integral += error;
    _integral  = constrain(_integral,
                           -128.0f / _ki,
                            128.0f / _ki);

    // Incremental output: add correction to running PWM
    float output = _kp * error + _ki * _integral;
    _pwm += output;

    applyPWM(_pwm);
}

// -------------------------------------------------------
// Private: write PWM and direction to motor driver pins
// -------------------------------------------------------

void MotorController::applyPWM(float pwm) {
    _pwm = constrain(pwm, -255.0f, 255.0f);

    if (_pwm >= 0.0f) {
        digitalWrite(_dirPin, HIGH);
        analogWrite(_pwmPin, (int)_pwm);
    } else {
        digitalWrite(_dirPin, LOW);
        analogWrite(_pwmPin, (int)(-_pwm));
    }
}