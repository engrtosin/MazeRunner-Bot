// =====================================================
// RobotFSM.cpp — MazeRunner
// =====================================================
#include "RobotFSM.h"

// -------------------------------------------------------
// Construction
// -------------------------------------------------------
RobotFSM::RobotFSM(MotorController &motorLeft,  MotorController &motorRight,
                   IRSensor        &irFront,    IRSensor        &irLeft,
                   RPiComms       &rpiComms)
    : _motorLeft(motorLeft), _motorRight(motorRight),
      _rpiComms(rpiComms),
      _state(TopState::WALL_FOLLOW),
      _wallFollow(motorLeft, motorRight, irFront, irLeft),
      _ball(motorLeft, motorRight, rpiComms)
{}

// -------------------------------------------------------
// Main update
// -------------------------------------------------------
void RobotFSM::update() {
    switch (_state) {

        case TopState::WALL_FOLLOW:
            // Ball override: only trigger when wall-follow is in
            // steady FOLLOW state — not mid-turn or mid-recovery.
            // TODO (future): expand this condition to also allow
            // override during WALL_LOST by calling
            // _wallFollow.suspend() before enterState(BALL_OVERRIDE)
            // and _wallFollow.resume() on return.
            if (_rpiComms.isBallVisible() &&
                _wallFollow.getState() == RobotState::FOLLOW) {
                _motorLeft.stop();
                _motorRight.stop();
                enterState(TopState::BALL_OVERRIDE);
#if DEBUG_SERIAL
                Serial.println("Ball detected -> BALL_OVERRIDE");
#endif
                return;
            }
            _wallFollow.update();
            break;

        case TopState::BALL_OVERRIDE:
            _ball.update();
            if (_ball.isDone()) {
                enterState(TopState::WALL_FOLLOW);
#if DEBUG_SERIAL
                Serial.println("Ball done -> WALL_FOLLOW");
#endif
            }
            break;
    }
}

// -------------------------------------------------------
// enterState
// -------------------------------------------------------
void RobotFSM::enterState(TopState newState) {
    _state = newState;

    if (_state == TopState::BALL_OVERRIDE) {
        _ball.start();
    }
}