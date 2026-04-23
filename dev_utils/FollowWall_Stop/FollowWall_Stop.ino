// ================= PIN DEFINITIONS =================
#define M1_DIR 7
#define M1_PWM 9
#define M2_DIR 8
#define M2_PWM 10

#define EN_PIN 4
#define SF_PIN 12

#define ENC_L 2
#define ENC_R 3

#define FRONT_IR_PIN A2
#define LEFT_IR_PIN  A3

// ================= MOTOR STRUCT =================
struct Motor {
  int dirPin, pwmPin, encPin;
  volatile long count;
  long lastCount;
  float cpr;
  float targetRPM;
  float currentRPM;
  float pwm;
  float Kp;
  float Ki;
  float integral;
};

// ================= FILTER STRUCTS =================
struct BiquadCoeff {
  float b0, b1, b2;
  float a1, a2;
};

struct BiquadState {
  float z1 = 0;
  float z2 = 0;
};

struct ButterworthState {
  BiquadState s1;
  BiquadState s2;
};

#define MA_WINDOW 6
struct MovingAverage {
  float buffer[MA_WINDOW] = {0};
  float sum = 0;
  int index = 0;
};

// ================= GLOBAL =================
Motor leftMotor;
Motor rightMotor;

ButterworthState frontFilter;
ButterworthState leftFilter;

MovingAverage frontMA;
MovingAverage leftMA;

unsigned long lastControlTime = 0;
unsigned long prevIRTime = 0;

// ================= FILTER COEFF =================
const BiquadCoeff stage1Coeff = {
  2.91464944656977e-05f,
  2.91464944656977e-05f,
  0.0f,
  -0.939062505817492f,
  0.0f
};

const BiquadCoeff stage2Coeff = {
  1.0f, 2.0f, 1.0f,
  -1.93529438685999f,
  0.939120798806424f
};

// ================= CALIBRATION =================
const float A_front = 1.23072369e+04;
const float B_front = 1.12642133e+01;
const float C_front = 1.74338869e+00;

const float A_left = 7.68904930e+03;
const float B_left = 1.00000065e-03;
const float C_left = -2.64920279e+00;

// ================= CONSTANTS =================
const int intervalMs = 100;
const unsigned long IR_SAMPLE_TIME = 10000;

#define TARGET_LEFT_DIST 10.0
#define FRONT_STOP_DIST  20.0

// VERY SAFE SPEED
#define BASE_SPEED 15

// ================= ENCODER =================
void leftISR() { leftMotor.count++; }
void rightISR() { rightMotor.count++; }

// ================= MOTOR =================
void initMotor(Motor &m, int dirPin, int pwmPin, int encPin, float cpr) {
  m.dirPin = dirPin;
  m.pwmPin = pwmPin;
  m.encPin = encPin;

  m.count = 0;
  m.lastCount = 0;

  m.cpr = cpr;

  m.targetRPM = 0;
  m.currentRPM = 0;

  m.pwm = 0;

  m.Kp = 1.5;
  m.Ki = 0.5;
  m.integral = 0;

  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(encPin, INPUT_PULLUP);
}

void setMotorPWM(Motor &m, float pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm >= 0) {
    digitalWrite(m.dirPin, HIGH);
    analogWrite(m.pwmPin, (int)pwm);
  } else {
    digitalWrite(m.dirPin, LOW);
    analogWrite(m.pwmPin, (int)(-pwm));
  }

  m.pwm = pwm;
}

// 🔥 HARD STOP (REAL FIX)
void hardStop() {
  leftMotor.targetRPM = 0;
  rightMotor.targetRPM = 0;

  leftMotor.integral = 0;
  rightMotor.integral = 0;

  leftMotor.pwm = 0;
  rightMotor.pwm = 0;

  setMotorPWM(leftMotor, 0);
  setMotorPWM(rightMotor, 0);
}

// ================= DRIVER =================
void enableDriver() {
  digitalWrite(EN_PIN, LOW);
  delay(5);
  digitalWrite(EN_PIN, HIGH);
  delay(5);
}

// ================= FILTER =================
float processBiquad(const BiquadCoeff &c, BiquadState &s, float x) {
  float y = c.b0 * x + s.z1;
  s.z1 = c.b1 * x - c.a1 * y + s.z2;
  s.z2 = c.b2 * x - c.a2 * y;
  return y;
}

float processButterworth(ButterworthState &f, float x) {
  float y = processBiquad(stage1Coeff, f.s1, x);
  y = processBiquad(stage2Coeff, f.s2, y);
  return y;
}

// ================= MOVING AVG =================
float applyMA(MovingAverage &ma, float x) {
  ma.sum -= ma.buffer[ma.index];
  ma.buffer[ma.index] = x;
  ma.sum += x;

  ma.index = (ma.index + 1) % MA_WINDOW;
  return ma.sum / MA_WINDOW;
}

// ================= IR =================
float frontDistanceCM = 0;
float leftDistanceCM  = 0;

float computeFront(float adc) {
  return max(A_front / (adc + B_front) + C_front, 0.0f);
}

float computeLeft(float adc) {
  return max(A_left / (adc + B_left) + C_left, 0.0f);
}

void updateIR() {
  unsigned long now = micros();

  if (now - prevIRTime >= IR_SAMPLE_TIME) {

    float rawF = analogRead(FRONT_IR_PIN);
    float rawL = analogRead(LEFT_IR_PIN);

    float fF = processButterworth(frontFilter, rawF);
    float fL = processButterworth(leftFilter, rawL);

    float dF = computeFront(fF);
    float dL = computeLeft(fL);

    frontDistanceCM = applyMA(frontMA, dF);
    leftDistanceCM  = applyMA(leftMA, dL);

    prevIRTime += IR_SAMPLE_TIME;
  }
}

// ================= CONTROL =================
void updateRPM(Motor &m, float dt) {
  long delta = m.count - m.lastCount;
  m.lastCount = m.count;

  float revs = (float)delta / m.cpr;
  m.currentRPM = (revs / dt) * 60.0;
}

void updatePID(Motor &m, float dt) {
  float error = m.targetRPM - m.currentRPM;

  m.integral += error * dt;

  float output = m.Kp * error + m.Ki * m.integral;

  m.pwm += output;

  setMotorPWM(m, m.pwm);
}

void updateControl() {
  unsigned long now = millis();
  if (now - lastControlTime < intervalMs) return;

  float dt = (now - lastControlTime) / 1000.0;
  lastControlTime = now;

  updateRPM(leftMotor, dt);
  updateRPM(rightMotor, dt);

  updatePID(leftMotor, dt);
  updatePID(rightMotor, dt);
}

// ================= WALL FOLLOW =================
void wallControl() {

  Serial.print("F: ");
  Serial.print(frontDistanceCM);
  Serial.print(" | L: ");
  Serial.println(leftDistanceCM);

  // 🔥 HARD STOP FIRST (PRIORITY)
  if (frontDistanceCM < FRONT_STOP_DIST) {
    hardStop();
    return;
  }

  float error = leftDistanceCM - TARGET_LEFT_DIST;

  // VERY SMALL CORRECTION
  float Kp_wall = 1.0;
  float correction = Kp_wall * error;

  correction = constrain(correction, -5, 5);

  float leftRPM  = BASE_SPEED - correction;
  float rightRPM = BASE_SPEED + correction;

  // LIMIT SPEED HARD
  leftRPM  = constrain(leftRPM, 0, 20);
  rightRPM = constrain(rightRPM, 0, 20);

  leftMotor.targetRPM  = leftRPM;
  rightMotor.targetRPM = rightRPM;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  pinMode(SF_PIN, INPUT);

  pinMode(FRONT_IR_PIN, INPUT);
  pinMode(LEFT_IR_PIN, INPUT);

  initMotor(leftMotor, M1_DIR, M1_PWM, ENC_L, 360);
  initMotor(rightMotor, M2_DIR, M2_PWM, ENC_R, 360);

  attachInterrupt(digitalPinToInterrupt(ENC_L), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), rightISR, RISING);

  enableDriver();

  lastControlTime = millis();
  prevIRTime = micros();
}

// ================= LOOP =================
void loop() {

  updateIR();
  updateControl();
  wallControl();
}