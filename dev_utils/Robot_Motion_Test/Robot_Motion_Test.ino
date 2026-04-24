// ================= PIN DEFINITIONS =================
#define M1_DIR  7
#define M1_PWM  9
#define M2_DIR  8
#define M2_PWM  10

#define EN_PIN  4
#define SF_PIN  12

#define ENC_L   2
#define ENC_R   3

// ================= MOTOR STRUCT =================
struct Motor {
  int dirPin;
  int pwmPin;
  int encPin;
  bool invertDir;

  volatile long count;
  long lastCount;

  float cpr;

  float targetRPM;
  float targetCountsPerSec;
  float currentCountsPerSec;
  float currentRPM;

  float pwm;
  float Kp;
  float Ki;
  float integral;
};

// ================= GLOBALS =================
Motor leftMotor;
Motor rightMotor;

unsigned long lastTime = 0;
const int intervalMs = 100;

// ================= ISR =================
void leftISR()  { leftMotor.count++; }
void rightISR() { rightMotor.count++; }

// ================= HELPERS =================
float rpmToCountsPerSec(float rpm, float cpr) {
  return rpm * cpr / 60.0;
}

float countsPerSecToRPM(float cps, float cpr) {
  return cps * 60.0 / cpr;
}

float clampFloat(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// ================= MOTOR INIT =================
void initMotor(Motor &m,
               int dirPin, int pwmPin, int encPin,
               float cpr, bool invertDir,
               float Kp, float Ki) {
  m.dirPin    = dirPin;
  m.pwmPin    = pwmPin;
  m.encPin    = encPin;
  m.invertDir = invertDir;
  m.cpr       = cpr;

  m.Kp = Kp;
  m.Ki = Ki;

  m.count               = 0;
  m.lastCount           = 0;
  m.targetRPM           = 0;
  m.targetCountsPerSec  = 0;
  m.currentCountsPerSec = 0;
  m.currentRPM          = 0;
  m.pwm                 = 0;
  m.integral            = 0;

  pinMode(dirPin,  OUTPUT);
  pinMode(pwmPin,  OUTPUT);
  pinMode(encPin,  INPUT_PULLUP);
}

// ================= LOW LEVEL =================
void updateSpeed(Motor &m, float dt) {
  long delta  = m.count - m.lastCount;
  m.lastCount = m.count;

  m.currentCountsPerSec = delta / dt;
  m.currentRPM = countsPerSecToRPM(m.currentCountsPerSec, m.cpr);
}

void updatePID(Motor &m, float dt) {
  if (m.targetRPM == 0) {
    m.integral = 0;
    m.pwm      = 0;
    return;
  }

  float error = m.targetCountsPerSec - m.currentCountsPerSec;

  m.integral += error * dt;
  m.integral  = clampFloat(m.integral, -300, 300);

  float correction = m.Kp * error + m.Ki * m.integral;

  m.pwm += correction;
  m.pwm  = clampFloat(m.pwm, 0, 255);
}

void applyMotor(Motor &m) {
  if (m.targetRPM == 0) {
    analogWrite(m.pwmPin, 0);
    return;
  }

  bool goForward = (m.targetRPM > 0);
  if (m.invertDir) goForward = !goForward;

  digitalWrite(m.dirPin, goForward ? HIGH : LOW);
  analogWrite(m.pwmPin, (int)m.pwm);
}

// ================= SET WHEEL SPEEDS =================
void setWheelRPM(float leftRPM, float rightRPM) {
  leftMotor.targetRPM  = leftRPM;
  rightMotor.targetRPM = rightRPM;

  leftMotor.targetCountsPerSec  = rpmToCountsPerSec(leftRPM,  leftMotor.cpr);
  rightMotor.targetCountsPerSec = rpmToCountsPerSec(rightRPM, rightMotor.cpr);
}

// ================= MOVEMENT FUNCTIONS =================

// Stop immediately
void stopRobot() {
  setWheelRPM(0, 0);
}

// Drive straight forward
void moveForward(float rpm) {
  setWheelRPM(rpm, rpm);
}

// Drive straight backward
void moveReverse(float rpm) {
  setWheelRPM(-rpm, -rpm);
}

// Spin left in place (left wheel back, right wheel forward)
void turnLeft(float rpm) {
  setWheelRPM(-rpm, rpm);
}

// Spin right in place (left wheel forward, right wheel back)
void turnRight(float rpm) {
  setWheelRPM(rpm, -rpm);
}

// Gentle arc left while moving forward
void arcLeft(float rpm, float turnFactor) {
  // turnFactor 0.0 = straight, 1.0 = full spin
  // Inner (left) wheel slows down, outer (right) speeds up
  turnFactor = clampFloat(turnFactor, 0.0, 1.0);
  setWheelRPM(rpm * (1.0 - turnFactor), rpm);
}

// Gentle arc right while moving forward
void arcRight(float rpm, float turnFactor) {
  turnFactor = clampFloat(turnFactor, 0.0, 1.0);
  setWheelRPM(rpm, rpm * (1.0 - turnFactor));
}

// ================= CONTROL LOOP =================
void updateControl() {
  unsigned long now = millis();
  if (now - lastTime < intervalMs) return;

  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  updateSpeed(leftMotor,  dt);
  updateSpeed(rightMotor, dt);

  updatePID(leftMotor,  dt);
  updatePID(rightMotor, dt);

  applyMotor(leftMotor);
  applyMotor(rightMotor);

  // Debug — open Serial Monitor at 115200 baud
  Serial.print("L_RPM: ");    Serial.print(leftMotor.currentRPM);
  Serial.print(" | R_RPM: "); Serial.print(rightMotor.currentRPM);
  Serial.print(" | L_PWM: "); Serial.print(leftMotor.pwm);
  Serial.print(" | R_PWM: "); Serial.print(rightMotor.pwm);
  Serial.print(" | L_cnt: "); Serial.print(leftMotor.count);
  Serial.print(" | R_cnt: "); Serial.println(rightMotor.count);
}

// ================= DRIVER =================
void enableDriver() {
  digitalWrite(EN_PIN, LOW);
  delay(5);
  digitalWrite(EN_PIN, HIGH);
  delay(5);
}

bool isFault() {
  return digitalRead(SF_PIN) == LOW;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  pinMode(SF_PIN, INPUT);

  // Left motor:  CPR=684 (CHANGE mode), invertDir=true,  Kp=0.08, Ki=0.03
  // Right motor: CPR=694 (CHANGE mode), invertDir=false, Kp=0.08, Ki=0.03
  // Tune Kp/Ki independently per motor if response differs
  initMotor(leftMotor,  M1_DIR, M1_PWM, ENC_L, 684.0, false,  0.08, 0.03);
  initMotor(rightMotor, M2_DIR, M2_PWM, ENC_R, 694.0, false, 0.08, 0.03);

  attachInterrupt(digitalPinToInterrupt(ENC_L), leftISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R), rightISR, CHANGE);

  enableDriver();

  lastTime = millis();

  Serial.println("=== Robot Motor Control Ready ===");
}

// ================= LOOP =================
void loop() {

  // ── Example sequence ──────────────────────────────────
  // Swap these out to test each movement function.
  // Call updateControl() in every loop — it is non-blocking.

  moveForward(80);    // drive forward at 80 RPM
  // moveReverse(80); // drive backward at 80 RPM
  // turnLeft(60);    // spin left in place at 60 RPM
  // turnRight(60);   // spin right in place at 60 RPM
  // arcLeft(80, 0.5);  // arc left, inner wheel at 50% speed
  // arcRight(80, 0.5); // arc right, inner wheel at 50% speed
  // stopRobot();

  updateControl();

  if (isFault()) {
    Serial.println("DRIVER FAULT — stopping");
    stopRobot();
  }
}
