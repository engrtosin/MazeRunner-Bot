// =====================================================
// BallFSM.cpp — MazeRunner
// =====================================================
#include "BallFSM.h"

// -------------------------------------------------------
// Construction
// -------------------------------------------------------
BallFSM::BallFSM(MotorController &motorLeft, MotorController &motorRight,
                 RPiComms       &rpiComms)
    : _motorLeft(motorLeft), _motorRight(motorRight),
      _rpiComms(rpiComms),
      _phase(BallPhase::TRACK),
      _done(false),
      _phaseEntryMs(0)
{}

// -------------------------------------------------------
// Lifecycle
// -------------------------------------------------------
void BallFSM::start() {
    _done = false;
    enterPhase(BallPhase::TRACK);
}

void BallFSM::update() {
    if (_done) return;

    switch (_phase) {
        case BallPhase::TRACK:      handleTrack();      break;
        case BallPhase::COLLECT:    handleCollect();    break;
        case BallPhase::COLLECTED:  handleCollected();  break;
    }
}

// -------------------------------------------------------
// TRACK — differential steer toward ball using pixel x
// -------------------------------------------------------
void BallFSM::handleTrack() {
    if (millis() - _phaseEntryMs < BALL_ENTRY_DELAY_MS) return;

    // Ball has left the frame — it has reached the robot
    if (!_rpiComms.isBallVisible()) {
        _motorLeft.stop();
        _motorRight.stop();
        rollersOn();
        enterPhase(BallPhase::COLLECT);
#if DEBUG_SERIAL
        Serial.println("Ball | frame exit -> COLLECT");
#endif
        return;
    }

    float error      = (float)_rpiComms.getBallX() - (float)X_CENTER;
    float correction = 0.0f;
    if (abs(error) > DEAD_BAND) {
        correction = constrain(KP_HEADING * error,
                               -(float)MAX_CORRECTION,
                                (float)MAX_CORRECTION);
    }

    float leftRPM  = constrain((float)TRACK_SPEED + correction,
                               0.0f, (float)(TRACK_SPEED + MAX_CORRECTION));
    float rightRPM = constrain((float)TRACK_SPEED - correction,
                               0.0f, (float)(TRACK_SPEED + MAX_CORRECTION));

    _motorLeft.setTargetRPM(leftRPM);
    _motorRight.setTargetRPM(rightRPM);

#if DEBUG_SERIAL
    Serial.print("Ball TRACK | x: "); Serial.print(_rpiComms.getBallX());
    Serial.print(" | err: "); Serial.print(error, 1);
    Serial.print(" | tL: "); Serial.print(leftRPM, 1);
    Serial.print(" | tR: "); Serial.println(rightRPM, 1);
#endif
}

// -------------------------------------------------------
// COLLECT — rollers on, creep until break beam or timeout
// -------------------------------------------------------
void BallFSM::handleCollect() {
    rollersOn();

    if (breakBeamTriggered()) {
        _motorLeft.stop();
        _motorRight.stop();
        rollersOff();
        enterPhase(BallPhase::COLLECTED);
#if DEBUG_SERIAL
        Serial.println("Ball | break beam -> COLLECTED");
#endif
        return;
    }

    if (millis() - _phaseEntryMs > COLLECT_TIMEOUT_MS) {
        _motorLeft.stop();
        _motorRight.stop();
        rollersOff();
        enterPhase(BallPhase::COLLECTED);
#if DEBUG_SERIAL
        Serial.println("Ball | collect timeout -> COLLECTED");
#endif
        return;
    }

    _motorLeft.setTargetRPM(CREEP_SPEED_BALL);
    _motorRight.setTargetRPM(CREEP_SPEED_BALL);
}

// -------------------------------------------------------
// COLLECTED — pause then signal completion to RobotFSM
// -------------------------------------------------------
void BallFSM::handleCollected() {
    _motorLeft.stop();
    _motorRight.stop();
    rollersOff();

    if (millis() - _phaseEntryMs > COLLECTED_PAUSE_MS) {
        _rpiComms.flush();   // discard stale packets before resuming
        _done = true;
#if DEBUG_SERIAL
        Serial.println("Ball | done -> resuming wall follow");
#endif
    }
}

// -------------------------------------------------------
// Helpers
// -------------------------------------------------------
void BallFSM::enterPhase(BallPhase phase) {
    _phase        = phase;
    _phaseEntryMs = millis();
}

void BallFSM::rollersOn() {
    digitalWrite(ROLLER_PIN, HIGH);
}

void BallFSM::rollersOff() {
    digitalWrite(ROLLER_PIN, LOW);
}