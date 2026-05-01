// =====================================================
// WallFollowFSM.cpp — MazeRunner
// =====================================================
#include "WallFollowFSM.h"

// -------------------------------------------------------
// Construction
// -------------------------------------------------------
WallFollowFSM::WallFollowFSM(MotorController &motorLeft,  MotorController &motorRight,
                               IRSensor        &irFront,
                               IRSensor        &irLeft)
    : _motorLeft(motorLeft),   _motorRight(motorRight),
      _irFront(irFront),
      _irLeft(irLeft),
      _state(NavState::FOLLOW),
      _stateEntryMs(0),
      _lastError(0.0f),
      _turnStartCountL(0),
      _turnStartCountR(0),
      _wallLostPhase(WallLostPhase::WL_REVERSE),
      _wallLostStartL(0),
      _wallLostStartR(0),
      _arcStartMs(0)
{}

// -------------------------------------------------------
// Main update — runs every loop iteration
// -------------------------------------------------------
void WallFollowFSM::update() {
    switch (_state) {
        case NavState::FOLLOW:      handleFollow();     break;
        case NavState::TURN_RIGHT:  handleTurnRight();  break;
        case NavState::WALL_LOST:   handleWallLost();   break;
        case NavState::STOPPED:                         break;
        case NavState::EXIT:                            break;
    }
}

// -------------------------------------------------------
// FOLLOW — P correction on left IR distance
// -------------------------------------------------------
void WallFollowFSM::handleFollow() {
    if (millis() - _stateEntryMs < STATE_ENTRY_DELAY_MS) return;

    float frontDist = _irFront.getDistance();
    float leftDist  = _irLeft.getDistance();

#if DEBUG_SERIAL
    Serial.print("F: "); Serial.print(frontDist);
    Serial.print(" | L: "); Serial.print(leftDist);
#endif

    // ── Outer corner / wall lost ──
    if (leftDist > WALL_LOST_DIST) {
        _motorLeft.stop();
        _motorRight.stop();
        _wallLostStartL = _motorLeft.getCount();
        _wallLostStartR = _motorRight.getCount();
        enterWallLostPhase(WallLostPhase::WL_REVERSE);
        enterState(NavState::WALL_LOST);
#if DEBUG_SERIAL
        Serial.println(" | -> WALL_LOST");
#endif
        return;
    }

    // ── Inner corner / front obstacle ──
    if (frontDist > 0 && frontDist < FRONT_STOP_DIST) {
        _motorLeft.stop();
        _motorRight.stop();
        _turnStartCountL = _motorLeft.getCount();
        _turnStartCountR = _motorRight.getCount();
        enterState(NavState::TURN_RIGHT);
#if DEBUG_SERIAL
        Serial.println(" | -> TURN_RIGHT");
#endif
        return;
    }

    // ── P wall-follow ──
    float baseRPM = (frontDist > 0 && frontDist < FRONT_SLOW_DIST)
                    ? SLOW_SPEED
                    : BASE_SPEED;

    float error      = leftDist - TARGET_LEFT_DIST;
    float correction = constrain(WALL_KP * error, -WALL_CORRECTION_MAX, WALL_CORRECTION_MAX);

    float leftRPM  = constrain(baseRPM - correction, 0.0f, MAX_SPEED);
    float rightRPM = constrain(baseRPM + correction, 0.0f, MAX_SPEED);

    _motorLeft.setTargetRPM(leftRPM);
    _motorRight.setTargetRPM(rightRPM);

#if DEBUG_SERIAL
    Serial.print(" | Err: "); Serial.print(error, 1);
    Serial.print(" | tL: "); Serial.print(leftRPM, 1);
    Serial.print(" | tR: "); Serial.print(rightRPM, 1);
    Serial.print(" | aL: "); Serial.print(_motorLeft.getMeasuredRPM(), 1);
    Serial.print(" | aR: "); Serial.println(_motorRight.getMeasuredRPM(), 1);
#endif
}

// -------------------------------------------------------
// TURN_RIGHT — encoder-counted 90° pivot right
// Left wheel forward, right wheel backward
// -------------------------------------------------------
void WallFollowFSM::handleTurnRight() {
    if (millis() - _stateEntryMs < STATE_ENTRY_DELAY_MS) return;

    int32_t travelL = abs(_motorLeft.getCount()  - _turnStartCountL);
    int32_t travelR = abs(_motorRight.getCount() - _turnStartCountR);

#if DEBUG_SERIAL
    Serial.print("Pivot L: "); Serial.print(travelL);
    Serial.print(" / ");       Serial.print(TURN_COUNTS_L);
    Serial.print(" | R: ");    Serial.print(travelR);
    Serial.print(" / ");       Serial.println(TURN_COUNTS_R);
#endif

    if (travelL >= TURN_COUNTS_L && travelR >= TURN_COUNTS_R) {
        _motorLeft.stop();
        _motorRight.stop();
        enterState(NavState::FOLLOW);
#if DEBUG_SERIAL
        Serial.println("Turn complete -> FOLLOW");
#endif
        return;
    }

    int pwmVal = map(TURN_SPEED, 0, 20, 0, 255);
    if (travelL < TURN_COUNTS_L) _motorLeft.setPWM( pwmVal);
    else                         _motorLeft.setPWM(0);
    if (travelR < TURN_COUNTS_R) _motorRight.setPWM(-pwmVal);
    else                         _motorRight.setPWM(0);
}

// -------------------------------------------------------
// WALL_LOST — dispatches to active sub-phase handler
// -------------------------------------------------------
void WallFollowFSM::handleWallLost() {
    switch (_wallLostPhase) {
        case WallLostPhase::WL_REVERSE:  handleWLReverse();  break;
        case WallLostPhase::WL_CREEP:    handleWLCreep();     break;
        case WallLostPhase::WL_ARC:      handleWLArc();       break;
    }
}

// ── Phase 1: reverse until wall reacquired or max distance hit ──
void WallFollowFSM::handleWLReverse() {
    if (millis() - _stateEntryMs < STATE_ENTRY_DELAY_MS) return;

    int32_t travelL = abs(_motorLeft.getCount() - _wallLostStartL);

#if DEBUG_SERIAL
    Serial.print("WL_REVERSE | L_dist: "); Serial.print(_irLeft.getDistance());
    Serial.print(" | rev_L: "); Serial.println(travelL);
#endif

    bool wallBack   = (_irLeft.getDistance() <= WALL_RECOVERY_DIST);
    bool maxReached = (travelL >= REVERSE_MAX_COUNTS_L);

    if (wallBack || maxReached) {
        _motorLeft.stop();
        _motorRight.stop();
        _wallLostStartL = _motorLeft.getCount();
        _wallLostStartR = _motorRight.getCount();
#if DEBUG_SERIAL
        Serial.print("  -> WL_CREEP (wallBack=");
        Serial.print(wallBack);
        Serial.print(", maxReached=");
        Serial.print(maxReached);
        Serial.println(")");
#endif
        enterWallLostPhase(WallLostPhase::WL_CREEP);
        return;
    }

    int pwmVal = map(WALL_LOST_OL_SPEED, 0, 20, 0, 255);
    _motorLeft.setPWM(-pwmVal);
    _motorRight.setPWM(-pwmVal);
}

// ── Phase 2: creep forward a fixed distance before arcing ──
void WallFollowFSM::handleWLCreep() {
    if (millis() - _stateEntryMs < STATE_ENTRY_DELAY_MS) return;

    int32_t creepL = abs(_motorLeft.getCount()  - _wallLostStartL);
    int32_t creepR = abs(_motorRight.getCount() - _wallLostStartR);

#if DEBUG_SERIAL
    Serial.print("WL_CREEP | creep_L: "); Serial.print(creepL);
    Serial.print(" / "); Serial.println(CREEP_COUNTS_L);
#endif

    if (creepL >= CREEP_COUNTS_L && creepR >= CREEP_COUNTS_R) {
        _motorLeft.stop();
        _motorRight.stop();
        _arcStartMs = millis();
#if DEBUG_SERIAL
        Serial.println("  -> WL_ARC");
#endif
        enterWallLostPhase(WallLostPhase::WL_ARC);
        return;
    }

    int pwmVal = map(WALL_LOST_OL_SPEED, 0, 20, 0, 255);
    _motorLeft.setPWM(pwmVal);
    _motorRight.setPWM(pwmVal);
}

// ── Phase 3: arc left until wall reacquired ──
void WallFollowFSM::handleWLArc() {
    if (millis() - _stateEntryMs < STATE_ENTRY_DELAY_MS) return;

#if DEBUG_SERIAL
    Serial.print("WL_ARC | L_dist: "); Serial.print(_irLeft.getDistance());
    Serial.print(" | t_ms: "); Serial.println(millis() - _arcStartMs);
#endif

    if (_irLeft.getDistance() <= WALL_RECOVERY_DIST) {
        _motorLeft.stop();
        _motorRight.stop();
        enterState(NavState::FOLLOW);
#if DEBUG_SERIAL
        Serial.println("  -> FOLLOW");
#endif
        return;
    }

    if (millis() - _arcStartMs > ARC_TIMEOUT_MS) {
        _motorLeft.stop();
        _motorRight.stop();
        enterState(NavState::STOPPED);
#if DEBUG_SERIAL
        Serial.println("  WL_ARC timeout -> STOPPED");
#endif
        return;
    }

    // Right (outer) wheel faster, left (inner) slower → arc left
    int outerPWM = map(ARC_OUTER_SPEED, 0, 20, 0, 255);
    int innerPWM = map(ARC_INNER_SPEED, 0, 20, 0, 255);
    _motorRight.setPWM(outerPWM);
    _motorLeft.setPWM(innerPWM);
}

// -------------------------------------------------------
// enterState — resets entry timestamp for new top-level state
// -------------------------------------------------------
void WallFollowFSM::enterState(NavState newState) {
    _state        = newState;
    _stateEntryMs = millis();

    if (_state == NavState::FOLLOW) {
        _lastError = 0.0f;
    }
}

// -------------------------------------------------------
// enterWallLostPhase — resets entry timestamp for new sub-phase
// -------------------------------------------------------
void WallFollowFSM::enterWallLostPhase(WallLostPhase phase) {
    _wallLostPhase = phase;
    _stateEntryMs  = millis();
}