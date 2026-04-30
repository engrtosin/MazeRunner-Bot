// =====================================================
// RPiComms.cpp — MazeRunner
// =====================================================
#include "RPiComms.h"

// -------------------------------------------------------
// Construction
// -------------------------------------------------------
RPiComms::RPiComms()
    : _ballX(0), _ballY(0), _lastPacketMs(0)
{}

// -------------------------------------------------------
// Main update — call every loop
// -------------------------------------------------------
void RPiComms::update() {
    readIncoming();
    // TODO: service any outgoing send queue here
}

// -------------------------------------------------------
// Incoming: ball detection accessors
// -------------------------------------------------------
bool RPiComms::isBallVisible() const {
    return (_lastPacketMs > 0) &&
           (millis() - _lastPacketMs < BALL_TIMEOUT_MS);
}

// -------------------------------------------------------
// Flush — discard stale bytes and reset ball timeout
// -------------------------------------------------------
void RPiComms::flush() {
    while (Serial.available()) Serial.read();
    _lastPacketMs = 0;
}

// -------------------------------------------------------
// Private: parse incoming serial packets
// Protocol: [ 0xFF | x_byte | y_byte ]
// Scans for 0xFF header to re-sync after framing errors.
// -------------------------------------------------------
void RPiComms::readIncoming() {
    while (Serial.available() >= 3) {
        if (Serial.peek() != 0xFF) {
            Serial.read();  // discard — not aligned to header
            continue;
        }
        Serial.read();              // consume 0xFF header
        _ballX        = Serial.read();
        _ballY        = Serial.read();
        _lastPacketMs = millis();
    }
}