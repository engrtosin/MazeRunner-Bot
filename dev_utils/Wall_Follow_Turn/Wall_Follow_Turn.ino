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
  bool invertDir;
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

// ================= GLOBALS =================
Motor leftMotor;
Motor rightMotor;

ButterworthState frontFilter;
ButterworthState leftFilter;

MovingAverage frontMA;
MovingAverage leftMA;

unsigned long lastControlTime = 0;
unsigned long prevIRTime = 0;

// ================= FILTER COEFFICIENTS =================
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

// ================= IR CALIBRATION =================
const float A_front = 1.23072369e+04;
const float B_front = 1.12642133e+01;
const float C_front = 1.74338869e+00;

const float A_left = 7.68904930e+03;
const float B_left = 1.00000065e-03;
const float C_left = -2.64920279e+00;

// ================= TIMING =================
const int intervalMs = 100;
const unsigned long IR_SAMPLE_TIME = 10000;

// ================= DISTANCE THRESHOLDS =================
#define TARGET_LEFT_DIST   20.0
#define FRONT_STOP_DIST    30.0
#define FRONT_SLOW_DIST    30.0
#define EXIT_FRONT_DIST  500.0  // cm — front reads open air
#define EXIT_FRONT_DIST_MIN 250.0
#define EXIT_LEFT_DIST   300.0  // cm — left reads open air
#define EXIT_LEFT_DIST_MAX   1000.0  // cm — left reads open air

// ================= SPEEDS =================
#define BASE_SPEED         15
#define SLOW_SPEED         7
#define TURN_SPEED         7    // RPM for both wheels during pivot
#define OUTER_TURN_SPEED   3

// ================= PIVOT TURN GEOMETRY =================
// Each wheel travels π × wheelbase / 4 for a 90° pivot
// counts = arc_mm / circumference_mm × CPR
// arc_mm = π × 340 / 4 ≈ 267 mm
// circumference = π × 65 ≈ 204 mm
#define TURN_COUNTS_L  188
#define TURN_COUNTS_R  190
#define OUTER_TURN_COUNTS_L  171
#define OUTER_TURN_COUNTS_R  173
// #define OUTER_TURN_COUNTS_L  96    // half of TURN_COUNTS_L
// #define OUTER_TURN_COUNTS_R  97    // half of TURN_COUNTS_R
#define CREEP_COUNTS_L  383   // ~23 cm
#define CREEP_COUNTS_R  389
#define WALL_LOST_DIST       90.0  // cm — left sensor above this = wall lost

// ================= STATE MACHINE =================
enum RobotState {
  STATE_WALL_FOLLOW,
  STATE_TURNING,
  STATE_OUTER_CORNER,
  STATE_EXIT,
  STATE_STOPPED
};

RobotState robotState = STATE_WALL_FOLLOW;

long turnStartCountL = 0;
long turnStartCountR = 0;
bool creepDone = false;

// ================= IR READINGS =================
float frontDistanceCM = 0;
float leftDistanceCM  = 0;

// ================= ENCODER ISR =================
void leftISR()  { leftMotor.count++; }
void rightISR() { rightMotor.count++; }

// ================= MOTOR INIT =================
void initMotor(Motor &m, int dirPin, int pwmPin, int encPin, float cpr, bool invertDir) {
  m.dirPin     = dirPin;
  m.pwmPin     = pwmPin;
  m.encPin     = encPin;
  m.count      = 0;
  m.lastCount  = 0;
  m.cpr        = cpr;
  m.targetRPM  = 0;
  m.currentRPM = 0;
  m.pwm        = 0;
  m.Kp         = 1.5;
  m.Ki         = 0.5;
  m.integral   = 0;
  m.invertDir  = invertDir;

  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(encPin, INPUT_PULLUP);
}

// ================= MOTOR PWM =================
void setMotorPWM(Motor &m, float pwm) {
  pwm = constrain(pwm, -255, 255);
  bool goForward = (pwm >= 0);
  if (m.invertDir) goForward = !goForward;

  digitalWrite(m.dirPin, goForward ? HIGH : LOW);
  analogWrite(m.pwmPin, (int)abs(pwm));
  m.pwm = pwm;
}

// ================= HARD STOP =================
void hardStop() {
  leftMotor.targetRPM  = 0;
  rightMotor.targetRPM = 0;
  leftMotor.integral   = 0;
  rightMotor.integral  = 0;
  leftMotor.pwm        = 0;
  rightMotor.pwm       = 0;
  setMotorPWM(leftMotor,  0);
  setMotorPWM(rightMotor, 0);
}

// ================= DRIVER ENABLE =================
void enableDriver() {
  digitalWrite(EN_PIN, LOW);
  delay(5);
  digitalWrite(EN_PIN, HIGH);
  delay(5);
}

// ================= BIQUAD FILTER =================
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

// ================= MOVING AVERAGE =================
float applyMA(MovingAverage &ma, float x) {
  ma.sum -= ma.buffer[ma.index];
  ma.buffer[ma.index] = x;
  ma.sum += x;
  ma.index = (ma.index + 1) % MA_WINDOW;
  return ma.sum / MA_WINDOW;
}

// ================= IR SENSOR =================
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
    float fL = processButterworth(leftFilter,  rawL);

    frontDistanceCM = applyMA(frontMA, computeFront(fF));
    leftDistanceCM  = applyMA(leftMA,  computeLeft(fL));

    prevIRTime += IR_SAMPLE_TIME;
  }
}

// ================= RPM + PID =================
void updateRPM(Motor &m, float dt) {
  long delta   = m.count - m.lastCount;
  m.lastCount  = m.count;
  float revs   = (float)delta / m.cpr;
  m.currentRPM = (revs / dt) * 60.0;
}

void updatePID(Motor &m, float dt) {
  if (m.targetRPM == 0) {
    m.integral = 0;
    m.pwm      = 0;
    setMotorPWM(m, 0);
    return;
  }

  float error = m.targetRPM - m.currentRPM;

  bool saturatedHigh = (m.pwm >= 255 && error > 0);
  bool saturatedLow  = (m.pwm <= 0   && error < 0);
  if (!saturatedHigh && !saturatedLow) {
    m.integral = constrain(m.integral + error * dt, -100, 100);
  }

  float output = m.Kp * error + m.Ki * m.integral;
  m.pwm = constrain(m.pwm + output, 0, 255);
  setMotorPWM(m, m.pwm);
}

void updateControl() {
  unsigned long now = millis();
  if (now - lastControlTime < intervalMs) return;

  float dt        = (now - lastControlTime) / 1000.0;
  lastControlTime = now;

  updateRPM(leftMotor,  dt);
  updateRPM(rightMotor, dt);

  if (robotState == STATE_WALL_FOLLOW || robotState == STATE_OUTER_CORNER) {
    updatePID(leftMotor,  dt);
    updatePID(rightMotor, dt);
  }
}

// ================= STATE: WALL FOLLOW =================
void doWallFollow() {
  Serial.print("F: "); Serial.print(frontDistanceCM);
  Serial.print(" | L: "); Serial.print(leftDistanceCM);

  if (leftDistanceCM > WALL_LOST_DIST  && frontDistanceCM < 91.5f) {
    hardStop();
    delay(200);
    turnStartCountL = leftMotor.count;
    turnStartCountR = rightMotor.count;
    robotState = STATE_OUTER_CORNER;
    Serial.println(" | -> OUTER_CORNER");
    return;
  }

  if (frontDistanceCM > 0 && frontDistanceCM < FRONT_STOP_DIST) {
    hardStop();
    delay(200);

    turnStartCountL = leftMotor.count;
    turnStartCountR = rightMotor.count;

    robotState = STATE_TURNING;
    Serial.println(" | -> TURNING");
    return;
  }

  float baseRPM = (frontDistanceCM > 0 && frontDistanceCM < FRONT_SLOW_DIST)
                  ? SLOW_SPEED
                  : BASE_SPEED;

  float error      = leftDistanceCM - TARGET_LEFT_DIST;
  float Kp_wall    = 1.0;
  float correction = constrain(Kp_wall * error, -5, 5);

  leftMotor.targetRPM  = constrain(baseRPM - correction, 0, 20);
  rightMotor.targetRPM = constrain(baseRPM + correction, 0, 20);

  Serial.print(" | Err: "); Serial.print(error, 1);
  Serial.print(" | tL: "); Serial.print(leftMotor.targetRPM, 1);
  Serial.print(" | tR: "); Serial.print(rightMotor.targetRPM, 1);
  Serial.print(" | aL: "); Serial.print(leftMotor.currentRPM, 1);
  Serial.print(" | aR: "); Serial.println(rightMotor.currentRPM, 1);
}

// ================= STATE: PIVOT TURN 90° RIGHT =================
void doTurning() {
  long travelL = abs(leftMotor.count  - turnStartCountL);
  long travelR = abs(rightMotor.count - turnStartCountR);

  Serial.print("Pivot L: "); Serial.print(travelL);
  Serial.print(" / "); Serial.print(TURN_COUNTS_L);
  Serial.print(" | R: "); Serial.print(travelR);
  Serial.print(" / "); Serial.println(TURN_COUNTS_R);

  if (travelL >= TURN_COUNTS_L && travelR >= TURN_COUNTS_R) {
    hardStop();
    Serial.println("Turn complete -> WALL_FOLLOW");
    robotState = STATE_WALL_FOLLOW;
    return;
  }

  // Left wheel forward, right wheel backward — pivot right about center
  int pwmVal = map(TURN_SPEED, 0, 20, 0, 255);
  if (travelL < TURN_COUNTS_L) setMotorPWM(leftMotor,   pwmVal);
  else                          setMotorPWM(leftMotor,   0);
  if (travelR < TURN_COUNTS_R) setMotorPWM(rightMotor, -pwmVal);
  else                          setMotorPWM(rightMotor,  0);
}

// ================= STATE: PIVOT TURN 90° LEFT =================
void doOuterCorner() {

  // ── Phase 1: approach front wall to TARGET_LEFT_DIST ──
  if (!creepDone) {
    Serial.print("Approach front: "); Serial.println(frontDistanceCM);

    if (frontDistanceCM <= FRONT_STOP_DIST) {
      hardStop();
      delay(200);
      creepDone = true;
      turnStartCountL = leftMotor.count;
      turnStartCountR = rightMotor.count;
      return;
    }

    // Simple proportional approach — reuse SLOW_SPEED as base
    float error      = frontDistanceCM - TARGET_LEFT_DIST;
    float correction = constrain(1.0 * error, -5, 5);
    int leftRpm      = constrain(SLOW_SPEED + (int)correction, 0, 20);
    int rightRpm     = constrain(SLOW_SPEED + (int)correction, 0, 20);

    leftMotor.targetRPM  = leftRpm;
    rightMotor.targetRPM = rightRpm;
    return;
  }

  // ── Phase 2: pivot left 90° ──
  long pivotL = abs(leftMotor.count - turnStartCountL);
  long pivotR = abs(rightMotor.count - turnStartCountR);

  Serial.print("Outer pivot L: "); Serial.print(pivotL);
  Serial.print(" / "); Serial.print(OUTER_TURN_COUNTS_L);
  Serial.print(" | R: "); Serial.print(pivotR);
  Serial.print(" / "); Serial.println(OUTER_TURN_COUNTS_R);

  if (pivotL >= OUTER_TURN_COUNTS_L && pivotR >= OUTER_TURN_COUNTS_R) {
    hardStop();
    Serial.println("Outer turn complete -> WALL_FOLLOW");
    creepDone = false;
    robotState = STATE_WALL_FOLLOW;
    return;
  }

  int pwmVal = map(OUTER_TURN_SPEED, 0, 20, 0, 255);
  setMotorPWM(leftMotor,  pivotL < OUTER_TURN_COUNTS_L ? -pwmVal : 0);
  setMotorPWM(rightMotor, pivotR < OUTER_TURN_COUNTS_R ?  pwmVal : 0);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  pinMode(SF_PIN, INPUT);
  pinMode(FRONT_IR_PIN, INPUT);
  pinMode(LEFT_IR_PIN,  INPUT);

  initMotor(leftMotor,  M1_DIR, M1_PWM, ENC_L, 342.0, false);
  initMotor(rightMotor, M2_DIR, M2_PWM, ENC_R, 347.0, false);

  attachInterrupt(digitalPinToInterrupt(ENC_L), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), rightISR, RISING);

  enableDriver();
  delay(2000);  // let IR sensors stabilize before starting

  lastControlTime = millis();
  prevIRTime      = micros();

  Serial.println("=== Wall-Following Robot Started ===");
  Serial.print("Target dist: "); Serial.print(TARGET_LEFT_DIST); Serial.println(" cm");
  Serial.print("Base speed : "); Serial.print(BASE_SPEED);       Serial.println(" RPM");
  Serial.print("Turn counts: L="); Serial.print(TURN_COUNTS_L);
  Serial.print(" R="); Serial.println(TURN_COUNTS_R);
  Serial.println("====================================");
}

// ================= LOOP =================
void loop() {
  updateIR();
  updateControl();

  if (robotState != STATE_STOPPED && robotState != STATE_EXIT) {
    if (frontDistanceCM < EXIT_FRONT_DIST && leftDistanceCM > EXIT_LEFT_DIST && leftDistanceCM < EXIT_LEFT_DIST_MAX) {
      hardStop();
      robotState = STATE_EXIT;
      Serial.println("EXIT detected — stopping permanently.");
    }
  }

  switch (robotState) {
    case STATE_WALL_FOLLOW:  doWallFollow();  break;
    case STATE_TURNING:      doTurning();     break;
    case STATE_OUTER_CORNER: doOuterCorner(); break;
    case STATE_EXIT:                          break;
    case STATE_STOPPED:                       break;
  }
}