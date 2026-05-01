#pragma once
// =====================================================
// WallFollowFSM.h — MazeRunner
// =====================================================
#include <Arduino.h>
#include "Config.h"
#include "MotorController.h"
#include "IRSensor.h"

enum class NavState : uint8_t {
    FOLLOW,
    TURN_RIGHT,
    WALL_LOST,
    STOPPED,
    EXIT
};

enum class WallLostPhase : uint8_t {
    WL_REVERSE,
    WL_CREEP,
    WL_ARC
};

class WallFollowFSM {
public:
    WallFollowFSM(MotorController &motorLeft,  MotorController &motorRight,
                  IRSensor        &irFront,
                  IRSensor        &irLeft);

    void update();

    // Arc-interrupt API — called by RobotFSM only when in WL_ARC.
    // suspend() freezes progress by recording how much of the arc
    // has elapsed. resume() rebuilds _arcStartMs so the remaining
    // timeout budget is preserved and the arc continues seamlessly.
    void suspend();
    void resume();
    bool isSuspended() const { return _suspended; }

    NavState      getState()         const { return _state; }
    WallLostPhase getWallLostPhase() const { return _wallLostPhase; }

private:
    MotorController &_motorLeft;
    MotorController &_motorRight;
    IRSensor        &_irFront;
    IRSensor        &_irLeft;

    NavState      _state;
    unsigned long _stateEntryMs;
    float         _lastError;

    int32_t       _turnStartCountL;
    int32_t       _turnStartCountR;

    WallLostPhase _wallLostPhase;
    int32_t       _wallLostStartL;
    int32_t       _wallLostStartR;
    unsigned long _arcStartMs;

    // --- Arc-interrupt state (new) ---
    bool          _suspended;
    unsigned long _arcElapsedAtSuspend;

    void handleFollow();
    void handleTurnRight();
    void handleWallLost();
    void handleWLReverse();
    void handleWLCreep();
    void handleWLArc();

    void enterState(NavState newState);
    void enterWallLostPhase(WallLostPhase phase);
};