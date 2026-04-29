// =====================================================
// MotorController.cpp — MazeRunner
// =====================================================
#include "MotorController.h"

// -------------------------------------------------------
// Construction
// -------------------------------------------------------
MotorController::MotorController()
    : _dirPin(0), _pwmPin(0), _encPin(0),
      _cpr(0.0f), _invertDir(false),
      _kp(0.0f), _ki(0.0f),
      _pulseCount(0), _lastPulseCount(0),
      _targetRPM(0.0f), _measuredRPM(0.0f),
      _pwm(0.0f), _integral(0.0f)
{}

// -------------------------------------------------------
// Initialisation
// -------------------------------------------------------
void MotorController::init(uint8_t dirPin, uint8_t pwmPin, uint8_t encPin,
                           float cpr, bool invertDir,
                           float kp, float ki) {
    _dirPin    = dirPin;
    _pwmPin    = pwmPin;
    _encPin    = encPin;
    _cpr       = cpr;
    _invertDir = invertDir;
    _kp        = kp;
    _ki        = ki;

    pinMode(_dirPin, OUTPUT);
    pinMode(_pwmPin, OUTPUT);
    // attachInterrupt() handled in .ino (Arduino constraint)
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
    computePI(dt);
}

// -------------------------------------------------------
// Commands
// -------------------------------------------------------
void MotorController::setTargetRPM(float rpm) {
    _targetRPM = rpm;
}

void MotorController::stop() {
    _targetRPM = 0.0f;
    _integral  = 0.0f;
    _pwm       = 0.0f;
    applyPWM(0.0f);
}

// -------------------------------------------------------
// Open-loop direct PWM — bypasses PI entirely.
// Used for pivot turns and wall-lost recovery phases.
// -------------------------------------------------------
void MotorController::setPWM(float pwm) {
    _targetRPM = 0.0f;   // prevent PI from fighting open-loop command
    _integral  = 0.0f;
    applyPWM(pwm);
}

// -------------------------------------------------------
// Private: RPM measurement from encoder pulses
// -------------------------------------------------------
void MotorController::computeRPM(float dt) {
    noInterrupts();
    int32_t count = _pulseCount;
    interrupts();

    int32_t delta   = count - _lastPulseCount;
    _lastPulseCount = count;

    // pulses / CPR = revolutions; divide by dt → rev/s; × 60 → RPM
    _measuredRPM = ((float)delta / _cpr) / dt * 60.0f;
}

// -------------------------------------------------------
// Private: PI control — matches monolithic formulation
//
// Key decisions aligned to working monolithic code:
//   - Integral is time-scaled (error * dt), not bare error
//   - Anti-windup clamps integral to ±100
//   - Incremental output: _pwm += Kp*error + Ki*integral
//   - PWM clamped to [0, 255] — no reverse under closed loop
//     (reverse is open-loop only, via setPWM())
//   - Early return zeros output when targetRPM == 0
// -------------------------------------------------------
void MotorController::computePI(float dt) {
    if (_targetRPM == 0.0f) {
        _integral = 0.0f;
        _pwm      = 0.0f;
        applyPWM(0.0f);
        return;
    }

    float error = _targetRPM - _measuredRPM;

    // Anti-windup: only integrate when not saturated
    bool saturatedHigh = (_pwm >= 255.0f && error > 0.0f);
    bool saturatedLow  = (_pwm <= 0.0f   && error < 0.0f);
    if (!saturatedHigh && !saturatedLow) {
        _integral = constrain(_integral + error * dt, -100.0f, 100.0f);
    }

    float output = _kp * error + _ki * _integral;
    _pwm = constrain(_pwm + output, 0.0f, 255.0f);
    applyPWM(_pwm);
}

// -------------------------------------------------------
// Private: write PWM and direction to motor driver pins
// -------------------------------------------------------
void MotorController::applyPWM(float pwm) {
    _pwm = constrain(pwm, -255.0f, 255.0f);

    bool goForward = (_pwm >= 0.0f);
    if (_invertDir) goForward = !goForward;

    digitalWrite(_dirPin, goForward ? HIGH : LOW);
    analogWrite(_pwmPin, (int)abs(_pwm));
}