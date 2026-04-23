// =====================================================
// IRSensor.cpp — MazeRunner
// =====================================================

#include "IRSensor.h"

// -------------------------------------------------------
// Shared Butterworth coefficients — defined once here.
// All IRSensor instances reference these from Flash.
// Designed for 4th-order Butterworth at the same cutoff
// frequency for all IR sensors on this robot.
// -------------------------------------------------------

const BiquadCoeff biquadStage1 = {
    BIQUAD1_B0, BIQUAD1_B1, BIQUAD1_B2,
    BIQUAD1_A1, BIQUAD1_A2
};

const BiquadCoeff biquadStage2 = {
    BIQUAD2_B0, BIQUAD2_B1, BIQUAD2_B2,
    BIQUAD2_A1, BIQUAD2_A2
};

// -------------------------------------------------------
// Construction
// -------------------------------------------------------

IRSensor::IRSensor(uint8_t pin, float a, float b, float c)
    : _pin(pin), _a(a), _b(b), _c(c),
      _maSum(0.0f), _maIndex(0),
      _distanceCm(0.0f), _rawADC(0.0f)
{
    // Initialise moving average buffer to zero
    for (uint8_t i = 0; i < MA_WINDOW; i++) {
        _maBuffer[i] = 0.0f;
    }
}

// -------------------------------------------------------
// Main update — runs full filter pipeline
// -------------------------------------------------------

void IRSensor::update() {
    _rawADC = readADC();

    float filtered  = applyButterworth(_rawADC);
    float smoothed  = applyMovingAverage(filtered);
    _distanceCm     = adcToDistance(smoothed);
}

// -------------------------------------------------------
// Private: ADC read with double-read for mux settling
// -------------------------------------------------------

float IRSensor::readADC() {
    analogRead(_pin);           // discard — lets mux and capacitor settle
    return (float)analogRead(_pin);  // accurate read
}

// -------------------------------------------------------
// Private: 4th-order Butterworth — two biquad stages
// -------------------------------------------------------

float IRSensor::applyButterworth(float x) {
    // Stage 1
    float y1 = biquadStage1.b0 * x + _bwState.s1.z1;
    _bwState.s1.z1 = biquadStage1.b1 * x - biquadStage1.a1 * y1 + _bwState.s1.z2;
    _bwState.s1.z2 = biquadStage1.b2 * x - biquadStage1.a2 * y1;

    // Stage 2 — fed by stage 1 output
    float y2 = biquadStage2.b0 * y1 + _bwState.s2.z1;
    _bwState.s2.z1 = biquadStage2.b1 * y1 - biquadStage2.a1 * y2 + _bwState.s2.z2;
    _bwState.s2.z2 = biquadStage2.b2 * y1 - biquadStage2.a2 * y2;

    return y2;
}

// -------------------------------------------------------
// Private: moving average — circular buffer
// -------------------------------------------------------

float IRSensor::applyMovingAverage(float x) {
    _maSum -= _maBuffer[_maIndex];  // remove oldest value
    _maBuffer[_maIndex] = x;        // insert new value
    _maSum += x;

    _maIndex = (_maIndex + 1) % MA_WINDOW;

    return _maSum / MA_WINDOW;
}

// -------------------------------------------------------
// Private: calibration model  A / (adc + B) + C
// -------------------------------------------------------

float IRSensor::adcToDistance(float adc) {
    return max(_a / (adc + _b) + _c, 0.0f);
}