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

// ================= SPEEDS =================
#define BASE_SPEED         15
#define SLOW_SPEED         7
#define TURN_OUTER_RPM     5
#define TURN_INNER_RPM     1

// ================= ARC TURN GEOMETRY =================
#define TURN_OUTER_COUNTS  2353
#define TURN_INNER_COUNTS  1412

// ================= FORWARD PHASE =================
#define FORWARD_DURATION_MS 1000

// ================= STATE MACHINE =================
enum RobotState {
  STATE_WALL_FOLLOW,
  STATE_TURNING,
  STATE_FORWARD,
  STATE_STOPPED
};

RobotState robotState = STATE_WALL_FOLLOW;

unsigned long forwardStartTime = 0;
long turnStartCountL = 0;
long turnStartCountR = 0;

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

  // Anti-windup: only integrate when not saturated, or when integration helps
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

  if (robotState == STATE_WALL_FOLLOW || robotState == STATE_FORWARD) {
    updatePID(leftMotor,  dt);
    updatePID(rightMotor, dt);
  }
}

// ================= STATE: WALL FOLLOW =================
void doWallFollow() {
  Serial.print("F: "); Serial.print(frontDistanceCM);
  Serial.print(" | L: "); Serial.print(leftDistanceCM);

  if (frontDistanceCM > 0 && frontDistanceCM < FRONT_STOP_DIST) {
    hardStop();
    delay(200);

    turnStartCountL = leftMotor.count;
    turnStartCountR = rightMotor.count;

    leftMotor.integral  = 0;
    rightMotor.integral = 0;
    leftMotor.pwm       = 0;
    rightMotor.pwm      = 0;

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

// ================= STATE: ARC TURN 90° RIGHT =================
void doTurning() {
  long outerTravel = leftMotor.count  - turnStartCountL;
  long innerTravel = rightMotor.count - turnStartCountR;

  Serial.print("Arc L(outer): "); Serial.print(outerTravel);
  Serial.print(" / R(inner): "); Serial.println(innerTravel);

  if (outerTravel >= TURN_OUTER_COUNTS) {
    hardStop();
    delay(200);

    leftMotor.integral   = 0;
    rightMotor.integral  = 0;
    leftMotor.pwm        = 0;
    rightMotor.pwm       = 0;
    leftMotor.targetRPM  = BASE_SPEED;
    rightMotor.targetRPM = BASE_SPEED;

    forwardStartTime = millis();
    robotState = STATE_FORWARD;
    Serial.println("-> FORWARD");
    return;
  }

  setMotorPWM(leftMotor,  map(TURN_OUTER_RPM, 0, 20, 0, 255));
  setMotorPWM(rightMotor, map(TURN_INNER_RPM, 0, 20, 0, 255));
}

// ================= STATE: FORWARD 1s =================
void doForward() {
  Serial.println("Forward...");

  if (millis() - forwardStartTime >= FORWARD_DURATION_MS) {
    hardStop();
    robotState = STATE_STOPPED;
    Serial.println("DONE. Robot stopped permanently.");
    return;
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  pinMode(SF_PIN, INPUT);
  pinMode(FRONT_IR_PIN, INPUT);
  pinMode(LEFT_IR_PIN,  INPUT);

  // CPR = counts per revolution in RISING mode (verified per motor)
  // Left motor: 342 RISING → 684 CHANGE. Right motor: 347 RISING → 694 CHANGE.
  // Using RISING here so CPR = 342 / 347 respectively.
  initMotor(leftMotor,  M1_DIR, M1_PWM, ENC_L, 342.0, false);
  initMotor(rightMotor, M2_DIR, M2_PWM, ENC_R, 347.0, false);

  attachInterrupt(digitalPinToInterrupt(ENC_L), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), rightISR, RISING);

  enableDriver();

  lastControlTime = millis();
  prevIRTime      = micros();

  Serial.println("=== Wall-Following Robot Started ===");
  Serial.print("Target dist: "); Serial.print(TARGET_LEFT_DIST); Serial.println(" cm");
  Serial.print("Base speed : "); Serial.print(BASE_SPEED);       Serial.println(" RPM");
  Serial.println("====================================");
}

// ================= LOOP =================
void loop() {
  updateIR();
  updateControl();

  switch (robotState) {
    case STATE_WALL_FOLLOW: doWallFollow(); break;
    case STATE_TURNING:     doTurning();    break;
    case STATE_FORWARD:     doForward();    break;
    case STATE_STOPPED:     break;
  }
}