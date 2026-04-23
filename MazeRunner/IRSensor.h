#pragma once

// =====================================================
// IRSensor.h — MazeRunner
//
// Reads and filters an analog IR distance sensor.
//
// Filter pipeline per update():
//   1. ADC double-read (capacitor settling)
//   2. 4th-order Butterworth — two biquad stages
//   3. Moving average
//   → getDistance() returns distance in cm
//
// Butterworth coefficients are shared across all instances
// (same filter order and cutoff frequency for all sensors).
// Calibration (A, B, C) is per-instance.
//
// Usage:
//   IRSensor irFront(FRONT_IR_PIN, FRONT_IR_A, FRONT_IR_B, FRONT_IR_C);
//   irFront.update();
//   float dist = irFront.getDistance();
// =====================================================

#include <Arduino.h>
#include "Config.h"

// -------------------------------------------------------
// Biquad filter types — shared, no per-instance overhead
// -------------------------------------------------------

struct BiquadCoeff {
    float b0, b1, b2;
    float a1, a2;
};

struct BiquadState {
    float z1;
    float z2;
    BiquadState() : z1(0.0f), z2(0.0f) {}
};

struct ButterworthState {
    BiquadState s1;     // stage 1
    BiquadState s2;     // stage 2
};

// Shared Butterworth coefficients — defined once in IRSensor.cpp.
// Same filter design for all IR sensors.
extern const BiquadCoeff biquadStage1;
extern const BiquadCoeff biquadStage2;

// -------------------------------------------------------
// IRSensor class
// -------------------------------------------------------

class IRSensor {
public:

    // ---- Construction ----
    // pin:     analog pin (e.g. A2)
    // a, b, c: calibration coefficients for A / (adc + B) + C
    IRSensor(uint8_t pin, float a, float b, float c);

    // ---- Main update ----
    // Call on every IR sample interval from loop().
    // Reads ADC, runs filter pipeline, updates distance.
    void update();

    // ---- Accessors ----
    float getDistance() const { return _distanceCm; }  // filtered, in cm
    float getRawADC()   const { return _rawADC;      }  // last raw ADC read

private:

    // Pin and calibration
    uint8_t _pin;
    float   _a, _b, _c;

    // Filter state — per instance, Butterworth + moving average
    ButterworthState _bwState;

    float   _maBuffer[MA_WINDOW];
    float   _maSum;
    uint8_t _maIndex;

    // Output
    float _distanceCm;
    float _rawADC;

    // Internal helpers
    float readADC();                        // double-read for ADC settling
    float applyButterworth(float x);        // two biquad stages in series
    float applyMovingAverage(float x);      // circular buffer MA
    float adcToDistance(float adc);         // calibration model
};