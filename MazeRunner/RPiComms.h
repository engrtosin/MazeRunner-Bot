#pragma once
// =====================================================
// RPiComms.h — MazeRunner
//
// Manages all serial communication between the Arduino
// and the Raspberry Pi.
//
// --- Incoming (RPi → Arduino) ---
// Ball detection packets: 3 bytes per frame
//   [0] 0xFF      : sync/header byte
//   [1] x_byte    : 0–254 mapped from pixel x
//   [2] y_byte    : 0–254 mapped from pixel y
//
// x/y are clamped to 0–254 on the RPi side so they
// can never accidentally match the 0xFF header byte,
// allowing re-sync after any framing error.
//
// --- Outgoing (Arduino → RPi) ---
// TODO: robot state and sensor telemetry.
// Use sendState() / sendTelemetry() when implemented.
//
// Usage:
//   RPiComms rpiComms;
//   rpiComms.update();              // call every loop
//   if (rpiComms.isBallVisible())   // true within timeout
//   uint8_t x = rpiComms.getBallX();
//   uint8_t y = rpiComms.getBallY();
//   rpiComms.flush();               // discard stale packets
// =====================================================
#include <Arduino.h>
#include "Config.h"

class RPiComms {
public:
    RPiComms();

    // ---- Main update ----
    // Call every loop iteration. Non-blocking.
    // Reads available bytes and updates internal state.
    void update();

    // ---- Incoming: ball detection ----
    bool    isBallVisible() const;
    uint8_t getBallX()      const { return _ballX; }
    uint8_t getBallY()      const { return _ballY; }

    // ---- Flush ----
    // Discard all buffered serial bytes and reset ball timeout.
    // Call when resuming wall-follow after ball collection.
    void flush();

    // ---- Outgoing (TODO) ----
    // void sendState(uint8_t state);
    // void sendTelemetry(float frontDist, float leftDist);

private:
    // Incoming ball state
    uint8_t       _ballX;
    uint8_t       _ballY;
    unsigned long _lastPacketMs;

    // Internal helpers
    void readIncoming();
};