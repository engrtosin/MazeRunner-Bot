#pragma once
// =====================================================
// RobotFSM.h — MazeRunner
//
// Top-level state machine. Owns the two behaviors:
//   WALL_FOLLOW   — delegates to WallFollowFSM
//   BALL_OVERRIDE — delegates to BallFSM
//
// Ball override triggers from two places:
//   1. NavState::FOLLOW        — existing behavior
//   2. NavState::WALL_LOST / WL_ARC — new: ball is close
//      enough during arc (getBallY() >= BALL_CLOSE_Y).
//      WallFollowFSM is suspended, BallFSM runs, then
//      WallFollowFSM is resumed to continue the arc.
//
// Usage:
//   RobotFSM robotFSM(motorLeft, motorRight,
//                     irFront, irLeft, rpiComms);
//   robotFSM.update();  // call every loop
// =====================================================
#include <Arduino.h>
#include "Config.h"
#include "MotorController.h"
#include "IRSensor.h"
#include "RPiComms.h"
#include "WallFollowFSM.h"
#include "BallFSM.h"

enum class TopState : uint8_t {
    WALL_FOLLOW,
    BALL_OVERRIDE
};

class RobotFSM {
public:
    RobotFSM(MotorController &motorLeft,  MotorController &motorRight,
             IRSensor        &irFront,    IRSensor        &irLeft,
             RPiComms        &rpiComms);

    void update();

    TopState getState() const { return _state; }

private:
    MotorController &_motorLeft;
    MotorController &_motorRight;
    RPiComms        &_rpiComms;

    TopState      _state;
    WallFollowFSM _wallFollow;
    BallFSM       _ball;

    void enterState(TopState newState, bool fromArc = false);
};