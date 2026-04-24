#pragma once

// =====================================================
// WallFollowFSM.h — MazeRunner
//
// Inner FSM for wall following behavior.
// Follows the left wall using PD control.
// Handles right turns, left turns, and U-turns.
//
// Child FSMs:
//   TurnLeftFSM — move forward then turn left
//   UTurnFSM    — multi-step U-turn maneuver
//
// Usage:
//   wallFollowFSM.update();   // call every loop
//
// Dependencies:
//   motorLeft, motorRight     — motor controllers
//   irFront, irLeft, irRight  — IR sensors
// =====================================================

#include <Arduino.h>
#include "Config.h"
#include "MotorController.h"
#include "IRSensor.h"
#include "TurnLeftFSM.h"
#include "UTurnFSM.h"

// -------------------------------------------------------
// Wall follow states
// -------------------------------------------------------

enum class WallFollowState : uint8_t {
    FOLLOW,         // PD correction on left distance, straight ahead
    TURN_RIGHT,     // 90° right turn, inner wheel fixed
    TURN_LEFT,      // delegate to TurnLeftFSM
    UTURN           // delegate to UTurnFSM
};

// -------------------------------------------------------
// WallFollowFSM class
// -------------------------------------------------------

class WallFollowFSM {
public:

    // ---- Construction ----
    // Takes references to shared motors and sensors.
    // No dynamic allocation — all objects pre-exist as globals.
    WallFollowFSM(MotorController &motorLeft,  MotorController &motorRight,
                  IRSensor        &irFront,
                  IRSensor        &irLeft,
                  IRSensor        &irRight);

    // ---- Main update ----
    // Call every loop iteration from RobotFSM.
    void update();

    // ---- Accessors ----
    WallFollowState getState() const { return _state; }

private:

    // References to shared hardware — not owned here
    MotorController &_motorLeft;
    MotorController &_motorRight;
    IRSensor        &_irFront;
    IRSensor        &_irLeft;
    IRSensor        &_irRight;

    // Current state
    WallFollowState _state;

    // Child FSMs — always allocated, only active when relevant
    TurnLeftFSM _turnLeft;
    UTurnFSM    _uturn;

    // PD state for FOLLOW
    float        _lastError;
    unsigned long _lastPDTime;

    // TURN_RIGHT encoder tracking
    int32_t _turnRightStartCount;   // encoder count at turn entry

    // State handlers — one per state
    void handleFollow();
    void handleTurnRight();
    void handleTurnLeft();
    void handleUturn();

    // Transition helper — resets state on entry
    void enterState(WallFollowState newState);

    // Transition logic — evaluates sensor conditions
    WallFollowState evaluateTransitions();
};