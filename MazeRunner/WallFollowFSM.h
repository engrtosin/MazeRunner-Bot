#pragma once
// =====================================================
// WallFollowFSM.h — MazeRunner
//
// State machine for wall-following behavior.
// Follows the left wall using P control.
// Handles right turns (inner corner) and
// wall-lost recovery (outer corner).
//
// States:
//   FOLLOW      — P correction on left distance
//   TURN_RIGHT  — encoder-counted 90° pivot right
//   WALL_LOST   — multi-phase outer-corner recovery
//   STOPPED     — terminal, motors off
//   EXIT        — terminal, maze exit detected
//
// Wall-lost recovery phases (sub-state of WALL_LOST):
//   WL_REVERSE  — back up until wall reacquired or max distance
//   WL_CREEP    — creep forward before arcing
//   WL_ARC      — arc left until wall reacquired
//
// Usage:
//   WallFollowFSM fsm(motorLeft, motorRight, irFront, irLeft);
//   fsm.update();   // call every loop iteration
//
// Dependencies:
//   motorLeft, motorRight  — MotorController instances
//   irFront, irLeft        — IRSensor instances
// =====================================================

#include <Arduino.h>
#include "Config.h"
#include "MotorController.h"
#include "IRSensor.h"

// -------------------------------------------------------
// Top-level robot states
// -------------------------------------------------------
enum class NavState : uint8_t {
    FOLLOW,
    TURN_RIGHT,
    WALL_LOST,
    STOPPED,
    EXIT
};

// -------------------------------------------------------
// Wall-lost recovery sub-phases
// -------------------------------------------------------
enum class WallLostPhase : uint8_t {
    WL_REVERSE,
    WL_CREEP,
    WL_ARC
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
                  IRSensor        &irLeft);

    // ---- Main update ----
    // Call every loop iteration from MazeRunner.ino.
    void update();

    // ---- Accessors ----
    NavState getState() const { return _state; }

private:
    // References to shared hardware — not owned here
    MotorController &_motorLeft;
    MotorController &_motorRight;
    IRSensor        &_irFront;
    IRSensor        &_irLeft;

    // Current top-level state
    NavState _state;

    // Entry timestamp — set on every state and phase transition.
    // Each handler returns early until STATE_ENTRY_DELAY_MS has elapsed,
    // giving the robot time to physically stop and IR filters to settle.
    unsigned long _stateEntryMs;

    // FOLLOW — P control state
    float _lastError;

    // TURN_RIGHT — encoder snapshot at turn entry
    int32_t _turnStartCountL;
    int32_t _turnStartCountR;

    // WALL_LOST — sub-phase and encoder snapshots
    WallLostPhase _wallLostPhase;
    int32_t       _wallLostStartL;
    int32_t       _wallLostStartR;
    unsigned long _arcStartMs;

    // ---- State handlers ----
    void handleFollow();
    void handleTurnRight();
    void handleWallLost();

    // ---- Wall-lost phase handlers ----
    void handleWLReverse();
    void handleWLCreep();
    void handleWLArc();

    // ---- Transition helper ----
    // Resets entry timestamp and any phase-specific state.
    void enterState(NavState newState);
    void enterWallLostPhase(WallLostPhase phase);
};