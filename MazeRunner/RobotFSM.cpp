// =====================================================
// RobotFSM.cpp — MazeRunner
// =====================================================
#include "RobotFSM.h"

// -------------------------------------------------------
// Construction
// -------------------------------------------------------
RobotFSM::RobotFSM(MotorController &motorLeft,  MotorController &motorRight,
                   IRSensor        &irFront,    IRSensor        &irLeft,
                   RPiComms        &rpiComms)
    : _motorLeft(motorLeft),
      _motorRight(motorRight),
      _rpiComms(rpiComms),
      _state(TopState::WALL_FOLLOW),
      _wallFollow(motorLeft, motorRight, irFront, irLeft),
      _ball(motorLeft, motorRight, rpiComms)
{}

// -------------------------------------------------------
// update()
// -------------------------------------------------------
void RobotFSM::update() {
    switch (_state) {

        case TopState::WALL_FOLLOW: {

            // ── Ball during WL_ARC (new) ──────────────────────────
            // Check before _wallFollow.update() so we intercept the
            // arc on this tick rather than one tick later.
            if (_wallFollow.getState()         == NavState::WALL_LOST  &&
                _wallFollow.getWallLostPhase() == WallLostPhase::WL_ARC &&
                _rpiComms.isBallVisible()                               &&
                _rpiComms.getBallY()           >= BALL_CLOSE_Y) {
                enterState(TopState::BALL_OVERRIDE, true);  // true = from arc
                break;
            }

            // ── Ball during normal FOLLOW (existing) ──────────────
            if (_wallFollow.getState() == NavState::FOLLOW &&
                _rpiComms.isBallVisible()) {
                enterState(TopState::BALL_OVERRIDE, false);
                break;
            }

            _wallFollow.update();
            break;
        }

        case TopState::BALL_OVERRIDE: {
            _ball.update();

            if (_ball.isDone()) {
                // If we suspended an arc, resume it; otherwise just go back
                if (_wallFollow.isSuspended()) {
                    _wallFollow.resume();
#if DEBUG_SERIAL
                    Serial.println("RobotFSM | ball done -> arc resumed");
#endif
                } else {
#if DEBUG_SERIAL
                    Serial.println("RobotFSM | ball done -> WALL_FOLLOW");
#endif
                }
                _state = TopState::WALL_FOLLOW;
            }
            break;
        }
    }
}

// -------------------------------------------------------
// enterState
// -------------------------------------------------------
void RobotFSM::enterState(TopState newState, bool fromArc) {
    _state = newState;

    if (_state == TopState::BALL_OVERRIDE) {
        if (fromArc) {
            _wallFollow.suspend();   // freeze arc, stop motors
        }
        _ball.start();
#if DEBUG_SERIAL
        Serial.print("RobotFSM | -> BALL_OVERRIDE fromArc=");
        Serial.println(fromArc ? "true" : "false");
#endif
    }
}