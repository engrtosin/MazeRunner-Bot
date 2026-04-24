// =====================================================
// WallFollowFSM.cpp — MazeRunner
// =====================================================

#include "WallFollowFSM.h"

// -------------------------------------------------------
// Construction
// -------------------------------------------------------

WallFollowFSM::WallFollowFSM(MotorController &motorLeft,  MotorController &motorRight,
                               IRSensor        &irFront,
                               IRSensor        &irLeft,
                               IRSensor        &irRight)
    : _motorLeft(motorLeft),   _motorRight(motorRight),
      _irFront(irFront),
      _irLeft(irLeft),
      _irRight(irRight),
      _state(WallFollowState::FOLLOW),
      _turnLeft(motorLeft, motorRight),
      _uturn(motorLeft, motorRight, irFront, irLeft),
      _lastError(0.0f),
      _lastPDTime(0),
      _turnRightStartCount(0)
{}

// -------------------------------------------------------
// Main update — runs every loop iteration
// -------------------------------------------------------

void WallFollowFSM::update() {
    switch (_state) {
        case WallFollowState::FOLLOW:      handleFollow();     break;
        case WallFollowState::TURN_RIGHT:  handleTurnRight();  break;
        case WallFollowState::TURN_LEFT:   handleTurnLeft();   break;
        case WallFollowState::UTURN:       handleUturn();      break;
    }
}

// -------------------------------------------------------
// Transition evaluation
// Called from FOLLOW to check if a state change is needed.
// Priority order matters — dead end checked before right turn.
// -------------------------------------------------------

WallFollowState WallFollowFSM::evaluateTransitions() {
    bool frontBlocked = _irFront.getDistance() < FRONT_STOP_DIST;
    bool rightBlocked = _irRight.getDistance() < RIGHT_STOP_DIST;
    bool leftOpen     = _irLeft.getDistance()  > LEFT_OPEN_DIST;

    // Priority 1: dead end — front AND right blocked
    if (frontBlocked && rightBlocked) return WallFollowState::UTURN;

    // Priority 2: corner — front blocked only
    if (frontBlocked)                 return WallFollowState::TURN_RIGHT;

    // Priority 3: open space to left
    if (leftOpen)                     return WallFollowState::TURN_LEFT;

    // Default: keep following
    return WallFollowState::FOLLOW;
}

// -------------------------------------------------------
// FOLLOW — PD correction on left IR distance
// -------------------------------------------------------

void WallFollowFSM::handleFollow() {
    // Check for state transitions first
    WallFollowState next = evaluateTransitions();
    if (next != WallFollowState::FOLLOW) {
        enterState(next);
        return;
    }

    // PD wall following
    unsigned long now = millis();
    float dt = (now - _lastPDTime) / 1000.0f;

    if (dt <= 0.0f) return;     // guard against zero dt
    _lastPDTime = now;

    float error      = _irLeft.getDistance() - TARGET_LEFT_DIST;
    float derivative = (error - _lastError) / dt;
    _lastError       = error;

    float correction = WALL_KP * error + WALL_KD * derivative;
    correction       = constrain(correction, -WALL_CORRECTION_MAX, WALL_CORRECTION_MAX);

    float leftRPM  = constrain(BASE_SPEED - correction, 0.0f, MAX_SPEED);
    float rightRPM = constrain(BASE_SPEED + correction, 0.0f, MAX_SPEED);

    _motorLeft.setTargetRPM(leftRPM);
    _motorRight.setTargetRPM(rightRPM);
}

// -------------------------------------------------------
// TURN_RIGHT — 90° right turn, inner (right) wheel fixed
// Completion: encoder count on outer (left) wheel reached
// -------------------------------------------------------

void WallFollowFSM::handleTurnRight() {
    // TODO: implement encoder-counted 90° right turn
    // Left wheel drives at TURN_OUTER_RPM
    // Right wheel holds at TURN_INNER_RPM
    // Exit when left encoder delta >= TURN_90_PULSES
    // enterState(WallFollowState::FOLLOW) on completion

    // Placeholder — will be implemented when encoder counts are defined
    enterState(WallFollowState::FOLLOW);
}

// -------------------------------------------------------
// TURN_LEFT — delegate to TurnLeftFSM
// -------------------------------------------------------

void WallFollowFSM::handleTurnLeft() {
    _turnLeft.update();

    if (_turnLeft.isDone()) {
        enterState(WallFollowState::FOLLOW);
    }
}

// -------------------------------------------------------
// UTURN — delegate to UTurnFSM
// -------------------------------------------------------

void WallFollowFSM::handleUturn() {
    _uturn.update();

    if (_uturn.isDone()) {
        enterState(WallFollowState::FOLLOW);
    }
}

// -------------------------------------------------------
// enterState — resets entry conditions for new state
// -------------------------------------------------------

void WallFollowFSM::enterState(WallFollowState newState) {
    _state = newState;

    switch (_state) {
        case WallFollowState::FOLLOW:
            _lastError  = 0.0f;
            _lastPDTime = millis();
            break;

        case WallFollowState::TURN_RIGHT:
            // TODO: snapshot left encoder count here for delta tracking
            // _turnRightStartCount = atomically read left motor pulse count
            _motorLeft.setTargetRPM(TURN_OUTER_RPM);
            _motorRight.setTargetRPM(TURN_INNER_RPM);
            break;

        case WallFollowState::TURN_LEFT:
            _turnLeft.start();
            break;

        case WallFollowState::UTURN:
            _uturn.start();
            break;
    }
}