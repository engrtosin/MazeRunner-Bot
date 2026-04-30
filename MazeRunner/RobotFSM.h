#pragma once
// =====================================================
// RobotFSM.h — MazeRunner
//
// Top-level state machine. Owns the two behaviors:
//   WALL_FOLLOW  — delegates to WallFollowFSM
//   BALL_OVERRIDE — delegates to BallFSM
//
// Ball override is only triggered from WALL_FOLLOW
// (not during a turn or wall-lost recovery) so the
// robot never interrupts a maneuver mid-way.
//
// TODO (future): to support ball collection during
// wall-lost recovery, add suspend()/resume() calls
// on WallFollowFSM here before entering BALL_OVERRIDE,
// and restore on return. WallFollowFSM will need to
// expose its phase + encoder snapshot state for this.
//
// Usage:
//   RobotFSM robotFSM(motorLeft, motorRight,
//                     irFront, irLeft,
//                     rpiComms);
//   robotFSM.update();  // call every loop
// =====================================================
#include <Arduino.h>
#include "Config.h"
#include "MotorController.h"
#include "IRSensor.h"
#include "RPiComms.h"
#include "WallFollowFSM.h"
#include "BallFSM.h"

// -------------------------------------------------------
// Top-level robot states
// -------------------------------------------------------
enum class TopState : uint8_t {
    WALL_FOLLOW,
    BALL_OVERRIDE
};

// -------------------------------------------------------
// RobotFSM class
// -------------------------------------------------------
class RobotFSM {
public:
    RobotFSM(MotorController &motorLeft,  MotorController &motorRight,
             IRSensor        &irFront,    IRSensor        &irLeft,
             RPiComms       &rpiComms);

    // ---- Main update ----
    // Call every loop iteration from MazeRunner.ino.
    void update();

    // ---- Accessors ----
    TopState getState() const { return _state; }

private:
    MotorController &_motorLeft;
    MotorController &_motorRight;
    RPiComms       &_rpiComms;

    TopState      _state;
    WallFollowFSM _wallFollow;
    BallFSM       _ball;

    void enterState(TopState newState);
};