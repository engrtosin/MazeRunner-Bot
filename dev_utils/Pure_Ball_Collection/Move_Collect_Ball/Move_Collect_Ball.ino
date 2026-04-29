// =============================================================
//  Ball Tracker + Collector — Arduino
//
//  Serial protocol (RPi -> Arduino, 115200 baud):
//    2 bytes sent ONLY when ball detected AND position changed:
//      [0] x_byte : 0-255 mapped from pixel x
//      [1] y_byte : 0-255 mapped from pixel y
//
//    No packet = no ball. Arduino detects ball loss via timeout.
//
//  readSerial() is fully non-blocking — it only reads bytes
//  already in the buffer and never stalls the control loop.
// =============================================================

// ================= PIN DEFINITIONS =================
#define M1_DIR 7
#define M1_PWM 9
#define M2_DIR 8
#define M2_PWM 10

#define EN_PIN 4
#define SF_PIN 12

#define ENC_L 2
#define ENC_R 3

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

Motor leftMotor;
Motor rightMotor;

// ================= TIMING =================
const int intervalMs          = 100;
unsigned long lastControlTime = 0;

// ================= STATE ENTRY DELAY =================
unsigned long stateEntryMs = 0;
#define STATE_ENTRY_DELAY_MS 300

// ================= SPEEDS =================
#define SLOW_SPEED   7    // RPM — forward toward ball
#define ALIGN_SPEED  5    // RPM — pivot during heading correction

// ================= PD HEADING CORRECTION =================
#define X_CENTER   127    // midpoint of 0-255 x space
#define DEAD_BAND    8    // ± tolerance (~±6 px on 480-wide frame)

float lastHeadingError = 0.0f;
float Kp_heading       = 0.020f;
float Kd_heading       = 0.008f;

// ================= BALL TIMEOUT =================
#define BALL_TIMEOUT_MS 300   // no packet for this long = ball lost

unsigned long lastPacketMs = 0;
uint8_t pkt[2] = {0, 0};     // [0]=x_byte, [1]=y_byte

bool ballVisible() {
  return (millis() - lastPacketMs < BALL_TIMEOUT_MS);
}

// ================= STATE MACHINE =================
enum RobotState {
  STATE_SEARCH,     // waiting for ball
  STATE_ALIGN,      // PD pivot to centre ball horizontally
  STATE_APPROACH,   // drive straight; ball timeout = collected
  STATE_COLLECTED,  // brief pause then back to search
  STATE_STOPPED
};

RobotState robotState = STATE_SEARCH;

// ================= ENCODER ISR =================
void leftISR()  { leftMotor.count++; }
void rightISR() { rightMotor.count++; }

// ================= MOTOR HELPERS =================
void initMotor(Motor &m, int dirPin, int pwmPin, int encPin,
               float cpr, bool invertDir) {
  m.dirPin     = dirPin;
  m.pwmPin     = pwmPin;
  m.encPin     = encPin;
  m.count      = 0;
  m.lastCount  = 0;
  m.cpr        = cpr;
  m.targetRPM  = 0;
  m.currentRPM = 0;
  m.pwm        = 0;
  m.Kp         = 1.5f;
  m.Ki         = 0.5f;
  m.integral   = 0;
  m.invertDir  = invertDir;
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(encPin, INPUT_PULLUP);
}

void setMotorPWM(Motor &m, float pwm) {
  pwm = constrain(pwm, -255, 255);
  bool goForward = (pwm >= 0);
  if (m.invertDir) goForward = !goForward;
  digitalWrite(m.dirPin, goForward ? HIGH : LOW);
  analogWrite(m.pwmPin, (int)abs(pwm));
  m.pwm = pwm;
}

void hardStop() {
  leftMotor.targetRPM  = 0;
  rightMotor.targetRPM = 0;
  leftMotor.integral   = 0;
  rightMotor.integral  = 0;
  setMotorPWM(leftMotor,  0);
  setMotorPWM(rightMotor, 0);
}

void enableDriver() {
  digitalWrite(EN_PIN, LOW);  delay(5);
  digitalWrite(EN_PIN, HIGH); delay(5);
}

// ================= RPM + PID =================
void updateRPM(Motor &m, float dt) {
  long delta   = m.count - m.lastCount;
  m.lastCount  = m.count;
  m.currentRPM = ((float)delta / m.cpr / dt) * 60.0f;
}

void updatePID(Motor &m, float dt) {
  if (m.targetRPM == 0) {
    m.integral = 0; m.pwm = 0;
    setMotorPWM(m, 0);
    return;
  }
  float error  = m.targetRPM - m.currentRPM;
  bool satHigh = (m.pwm >= 255 && error > 0);
  bool satLow  = (m.pwm <=   0 && error < 0);
  if (!satHigh && !satLow)
    m.integral = constrain(m.integral + error * dt, -100, 100);
  float output = m.Kp * error + m.Ki * m.integral;
  m.pwm = constrain(m.pwm + output, 0, 255);
  setMotorPWM(m, m.pwm);
}

void updateControl() {
  unsigned long now = millis();
  if (now - lastControlTime < (unsigned long)intervalMs) return;
  float dt        = (now - lastControlTime) / 1000.0f;
  lastControlTime = now;
  updateRPM(leftMotor,  dt);
  updateRPM(rightMotor, dt);
  if (robotState == STATE_APPROACH) {
    updatePID(leftMotor,  dt);
    updatePID(rightMotor, dt);
  }
}

// ================= SERIAL READ (NON-BLOCKING) =================
// Drains all complete 2-byte packets from the buffer each call,
// keeping only the most recent. Never waits for bytes — if fewer
// than 2 are available it returns immediately.
void readSerial() {
  while (Serial.available() >= 2) {
    pkt[0] = Serial.read();  // x_byte
    pkt[1] = Serial.read();  // y_byte
    lastPacketMs = millis();
  }
}

// ================= STATE: SEARCH =================
void doSearch() {
  hardStop();
}

// ================= STATE: ALIGN =================
void doAlign() {
  if (millis() - stateEntryMs < STATE_ENTRY_DELAY_MS) return;

  float error = (float)pkt[0] - (float)X_CENTER;  // +ve = ball to the right

  float dt         = (float)intervalMs / 1000.0f;
  float derivative = (error - lastHeadingError) / dt;
  lastHeadingError  = error;

  float correction = Kp_heading * error + Kd_heading * derivative;
  correction = constrain(correction, -(float)ALIGN_SPEED, (float)ALIGN_SPEED);

  if (abs(error) <= DEAD_BAND) {
    hardStop();
    lastHeadingError = 0;
    stateEntryMs     = millis();
    robotState       = STATE_APPROACH;
    Serial.println("ALIGNED -> APPROACH");
    return;
  }

  int maxPWM  = map(ALIGN_SPEED, 0, 20, 0, 255);
  int corrPWM = (int)(abs(correction) / (float)ALIGN_SPEED * maxPWM);
  corrPWM = constrain(corrPWM, 0, maxPWM);

  if (correction > 0) {
    // Ball to the right → pivot right: left fwd, right back
    setMotorPWM(leftMotor,   corrPWM);
    setMotorPWM(rightMotor, -corrPWM);
  } else {
    // Ball to the left → pivot left: right fwd, left back
    setMotorPWM(leftMotor,  -corrPWM);
    setMotorPWM(rightMotor,  corrPWM);
  }

  Serial.print("ALIGN | x="); Serial.print(pkt[0]);
  Serial.print(" err="); Serial.print(error, 1);
  Serial.print(" corr="); Serial.println(correction, 3);
}

// ================= STATE: APPROACH =================
void doApproach() {
  if (millis() - stateEntryMs < STATE_ENTRY_DELAY_MS) return;

  // Re-align if heading drifts
  float error = (float)pkt[0] - (float)X_CENTER;
  if (abs(error) > DEAD_BAND * 2) {
    hardStop();
    lastHeadingError = 0;
    stateEntryMs     = millis();
    robotState       = STATE_ALIGN;
    Serial.println("Drift -> ALIGN");
    return;
  }

  leftMotor.targetRPM  = SLOW_SPEED;
  rightMotor.targetRPM = SLOW_SPEED;

  Serial.print("APPROACH | x="); Serial.println(pkt[0]);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN, OUTPUT);
  pinMode(SF_PIN, INPUT);

  initMotor(leftMotor,  M1_DIR, M1_PWM, ENC_L, 342.0f, false);
  initMotor(rightMotor, M2_DIR, M2_PWM, ENC_R, 347.0f, false);

  attachInterrupt(digitalPinToInterrupt(ENC_L), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), rightISR, RISING);

  enableDriver();
  delay(1000);

  lastControlTime = millis();
  stateEntryMs    = millis();

  Serial.println("=== Ball Collector Started ===");
  Serial.print("Kp="); Serial.print(Kp_heading, 4);
  Serial.print("  Kd="); Serial.println(Kd_heading, 4);
  Serial.print("Dead band="); Serial.print(DEAD_BAND);
  Serial.print("  Speed="); Serial.print(SLOW_SPEED);
  Serial.println(" RPM");
  Serial.println("==============================");
}

// ================= LOOP =================
void loop() {
  readSerial();     // non-blocking, always runs first
  updateControl();  // RPM + PID, always runs

  // ── Global state transitions based on ball visibility ──

  if (robotState == STATE_SEARCH && ballVisible()) {
    hardStop();
    lastHeadingError = 0;
    stateEntryMs     = millis();
    robotState       = STATE_ALIGN;
    Serial.println("Ball found -> ALIGN");
  }

  if (robotState == STATE_ALIGN && !ballVisible()) {
    hardStop();
    stateEntryMs = millis();
    robotState   = STATE_SEARCH;
    Serial.println("Ball lost -> SEARCH");
  }

  if (robotState == STATE_APPROACH && !ballVisible()) {
    hardStop();
    stateEntryMs = millis();
    robotState   = STATE_COLLECTED;
    Serial.println("Ball out of frame -> COLLECTED");
  }

  // ── State handlers ──
  switch (robotState) {
    case STATE_SEARCH:    doSearch();    break;
    case STATE_ALIGN:     doAlign();     break;
    case STATE_APPROACH:  doApproach();  break;

    case STATE_COLLECTED:
      hardStop();
      if (millis() - stateEntryMs > 2000) {
        stateEntryMs = millis();
        robotState   = STATE_SEARCH;
        Serial.println("Ready -> SEARCH");
      }
      break;

    case STATE_STOPPED:
      hardStop();
      break;
  }
}
