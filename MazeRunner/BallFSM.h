#pragma once
// =====================================================
// BallFSM.h — MazeRunner
//
// Manages the full ball collection sequence:
//   TRACK     → differential steer toward ball using
//               RPi pixel x coordinate
//   COLLECT   → rollers on, creep forward until break
//               beam triggers (or timeout)
//   COLLECTED → motors stopped, pause then signal done
//
// Caller (RobotFSM) checks isDone() each loop and
// returns to wall-following when true.
//
// TODO (future): add suspend()/resume() to support
// pausing mid-wall-lost-recovery for ball collection,
// saving and restoring WallFollowFSM phase + encoder
// snapshots on re-entry.
//
// Usage:
//   ballFSM.start();             // call on entering BALL_OVERRIDE
//   ballFSM.update();            // call every loop
//   if (ballFSM.isDone()) ...    // true when COLLECTED pause elapsed
// =====================================================
#include <Arduino.h>
#include "Config.h"
#include "MotorController.h"
#include "RPiComms.h"

// -------------------------------------------------------
// Ball collection sub-phases
// -------------------------------------------------------
enum class BallPhase : uint8_t {
    TRACK,        // steer toward ball using pixel x
    COLLECT,      // rollers on, creep until break beam
    COLLECTED     // pause then signal completion
};

// -------------------------------------------------------
// BallFSM class
// -------------------------------------------------------
class BallFSM {
public:
    // ---- Construction ----
    BallFSM(MotorController &motorLeft, MotorController &motorRight,
            RPiComms       &rpiComms);

    // ---- Lifecycle ----
    void start();               // reset and begin from TRACK
    void update();              // call every loop while active
    bool isDone() const { return _done; }

    // ---- Accessors ----
    BallPhase getPhase() const { return _phase; }

private:
    MotorController &_motorLeft;
    MotorController &_motorRight;
    RPiComms       &_rpiComms;

    BallPhase     _phase;
    bool          _done;
    unsigned long _phaseEntryMs;

    // Phase handlers
    void handleTrack();
    void handleCollect();
    void handleCollected();

    // Helpers
    void enterPhase(BallPhase phase);
    void rollersOn();
    void rollersOff();
};