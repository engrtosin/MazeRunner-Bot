/* PIN DEFINITIONS */
int IRpin = A3;

// /* SENSOR CALIBRATION (same as before) - FRONT */
// const float A = 1.23072369e+04;
// const float B = 1.12642133e+01;
// const float C = 1.74338869e+00;

// Sensor 1 - Left
// Best-fit A, B, C: [ 7.68904930e+03  1.00000065e-03 -2.64920279e+00]
const float A = 7.68904930e+03;
const float B = 1.00000065e-03;
const float C = -2.64920279e+00;

/* SAMPLING TIME */
const unsigned long sampleTime = 100000; // 100 ms
unsigned long prevTime = 0;

void setup() {
  Serial.begin(115200);
  prevTime = micros();
}

void loop() {
  unsigned long currTime = micros();

  if (currTime - prevTime >= sampleTime) {

    // === READ RAW ADC ===
    int adcValue = analogRead(IRpin);

    // === CONVERT TO DISTANCE ===
    float distance = A / (adcValue + B) + C;

    // Optional: clamp unrealistic values
    if (distance < 0.9) distance = 0.0;

    // === PRINT RESULTS ===
    Serial.print("ADC: ");
    Serial.print(adcValue);
    Serial.print(" | Distance (cm): ");
    Serial.println(distance);

    prevTime += sampleTime;
  }
}