// =============================================================
//  Wall-Following + Ball Collector — Arduino
//
//  Default behaviour : wall following (left wall, IR sensors).
//  Override          : when RPi sends 2-byte ball packets, the
//                      robot suspends wall-following and runs the
//                      full TRACK → COLLECT → COLLECTED cycle,
//                      then resumes wall-following automatically.
//
//  RPi → Arduino serial protocol (115200 baud):
//    3 bytes per detected ball frame:
//      [0] 0xFF   : header / sync byte
//      [1] x_byte : 0-255 mapped from pixel x (clamped to 0-254)
//      [2] y_byte : 0-255 mapped from pixel y (clamped to 0-254)
//    Silence for > BALL_TIMEOUT_MS → ball lost.
//    Arduino re-syncs by scanning for 0xFF, so stale debug bytes
//    cannot cause misalignment. x/y bytes are clamped to 0-254
//    so they never accidentally look like the header.
// =============================================================

// ================= PIN DEFINITIONS =================
#define M1_DIR       7
#define M1_PWM       9
#define M2_DIR       8
#define M2_PWM       10

#define EN_PIN       4
#define SF_PIN       12

#define ENC_L        2
#define ENC_R        3

#define ROLLER_PIN   5    // HIGH = rollers on
#define BREAK_BEAM   A4   // LOW  = ball inside

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
struct BiquadCoeff { float b0, b1, b2, a1, a2; };
struct BiquadState  { float z1 = 0, z2 = 0; };
struct ButterworthState { BiquadState s1, s2; };

#define MA_WINDOW 6
struct MovingAverage {
  float buffer[MA_WINDOW] = {0};
  float sum   = 0;
  int   index = 0;
};

// ================= GLOBALS =================
Motor leftMotor;
Motor rightMotor;

ButterworthState frontFilter;
ButterworthState leftFilter;

MovingAverage frontMA;
MovingAverage leftMA;

unsigned long lastControlTime = 0;
unsigned long prevIRTime      = 0;

// ================= STATE ENTRY DELAY =================
unsigned long stateEntryMs = 0;
#define STATE_ENTRY_DELAY_MS  400   // wall-follow states
#define BALL_ENTRY_DELAY_MS   300   // ball sub-states

// ================= FILTER COEFFICIENTS =================
const BiquadCoeff stage1Coeff = {
  2.91464944656977e-05f, 2.91464944656977e-05f, 0.0f,
  -0.939062505817492f, 0.0f
};
const BiquadCoeff stage2Coeff = {
  1.0f, 2.0f, 1.0f,
  -1.93529438685999f, 0.939120798806424f
};

// ================= IR CALIBRATION =================
const float A_front = 1.23072369e+04f, B_front = 1.12642133e+01f, C_front =  1.74338869e+00f;
const float A_left  = 7.68904930e+03f, B_left  = 1.00000065e-03f, C_left  = -2.64920279e+00f;

// ================= TIMING =================
const int          intervalMs      = 100;
const unsigned long IR_SAMPLE_TIME = 10000;  // microseconds

// ================= WALL-FOLLOW THRESHOLDS =================
#define TARGET_LEFT_DIST     20.0f
#define FRONT_STOP_DIST      30.0f
#define FRONT_SLOW_DIST      30.0f
#define WALL_LOST_DIST       50.0f
#define WALL_RECOVERY_DIST   30.0f

// ================= WALL-FOLLOW SPEEDS =================
#define BASE_SPEED           20
#define SLOW_SPEED_WF         7
#define WALL_LOST_OL_SPEED    3
#define TURN_SPEED            7
#define ARC_OUTER_SPEED       5
#define ARC_INNER_SPEED       2

// ================= PIVOT / RECOVERY GEOMETRY =================
#define TURN_COUNTS_L        171
#define TURN_COUNTS_R        173
#define REVERSE_MAX_COUNTS_L 262
#define REVERSE_MAX_COUNTS_R 265
#define CREEP_COUNTS_L_WF    279   // wall-follow creep (renamed to avoid clash)
#define CREEP_COUNTS_R_WF    283
#define REALIGN_COUNTS        82
#define ARC_TIMEOUT_MS       10000
#define ARC_MAX_COUNTS       327

// ================= BALL-TRACK SPEEDS =================
#define TRACK_SPEED          7    // RPM forward during tracking
#define CREEP_SPEED_BALL     3    // RPM forward during collection
#define MAX_CORRECTION       3    // RPM differential cap

// ================= BALL HEADING CORRECTION =================
#define X_CENTER   127
#define DEAD_BAND    8
float Kp_heading = 0.030f;

// ================= BALL TIMEOUT =================
#define BALL_TIMEOUT_MS  300
unsigned long lastPacketMs = 0;
uint8_t pkt[2] = {0, 0};

bool ballVisible() { return (millis() - lastPacketMs < BALL_TIMEOUT_MS); }

// ================= IR READINGS =================
float frontDistanceCM = 0;
float leftDistanceCM  = 0;

// ================= TOP-LEVEL STATE MACHINE =================
enum RobotState {
  STATE_WALL_FOLLOW,   // default — IR-guided wall following
  STATE_TURNING,       // 90° pivot right (inner corner)
  STATE_WALL_LOST,     // outer-corner recovery sequence
  STATE_BALL_OVERRIDE, // ball detected — suspends wall follow
  STATE_EXIT,
  STATE_STOPPED
};
RobotState robotState = STATE_WALL_FOLLOW;

// ── Pivot turn ──
long turnStartCountL = 0, turnStartCountR = 0;

// ── Wall-lost recovery ──
enum WallLostPhase { WL_REVERSE, WL_CREEP, WL_ARC, WL_REALIGN };
WallLostPhase wallLostPhase  = WL_REVERSE;
long          wallLostStartL = 0, wallLostStartR = 0;
unsigned long arcStartMs     = 0;

// ── Ball sub-state ──
enum BallPhase { BP_TRACK, BP_COLLECT, BP_COLLECTED };
BallPhase     ballPhase    = BP_TRACK;
unsigned long ballPhaseMs  = 0;   // entry timestamp for ball sub-states

// ================= ENCODER ISR =================
void leftISR()  { leftMotor.count++; }
void rightISR() { rightMotor.count++; }

// ================= MOTOR HELPERS =================
void initMotor(Motor &m, int dirPin, int pwmPin, int encPin,
               float cpr, bool invertDir) {
  m.dirPin = dirPin; m.pwmPin = pwmPin; m.encPin = encPin;
  m.count = 0; m.lastCount = 0; m.cpr = cpr;
  m.targetRPM = 0; m.currentRPM = 0; m.pwm = 0;
  m.Kp = 1.5f; m.Ki = 0.5f; m.integral = 0;
  m.invertDir = invertDir;
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
  leftMotor.targetRPM  = 0; rightMotor.targetRPM = 0;
  leftMotor.integral   = 0; rightMotor.integral  = 0;
  leftMotor.pwm        = 0; rightMotor.pwm       = 0;
  setMotorPWM(leftMotor, 0);
  setMotorPWM(rightMotor, 0);
}

void enableDriver() {
  digitalWrite(EN_PIN, LOW);  delay(5);
  digitalWrite(EN_PIN, HIGH); delay(5);
}

void rollersOn()  { digitalWrite(ROLLER_PIN, HIGH); }
void rollersOff() { digitalWrite(ROLLER_PIN, LOW);  }

// ================= BIQUAD / BUTTERWORTH =================
float processBiquad(const BiquadCoeff &c, BiquadState &s, float x) {
  float y = c.b0 * x + s.z1;
  s.z1 = c.b1 * x - c.a1 * y + s.z2;
  s.z2 = c.b2 * x - c.a2 * y;
  return y;
}
float processButterworth(ButterworthState &f, float x) {
  return processBiquad(stage2Coeff, f.s2, processBiquad(stage1Coeff, f.s1, x));
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
float computeFront(float adc) { return max(A_front / (adc + B_front) + C_front, 0.0f); }
float computeLeft(float adc)  { return constrain(A_left / (adc + B_left) + C_left, 0.0f, 500.0f); }

void updateIR() {
  unsigned long now = micros();
  if (now - prevIRTime >= IR_SAMPLE_TIME) {
    float fF = processButterworth(frontFilter, analogRead(FRONT_IR_PIN));
    float fL = processButterworth(leftFilter,  analogRead(LEFT_IR_PIN));
    frontDistanceCM = applyMA(frontMA, computeFront(fF));
    leftDistanceCM  = applyMA(leftMA,  computeLeft(fL));
    prevIRTime += IR_SAMPLE_TIME;
  }
}

// ================= RPM + PID =================
void updateRPM(Motor &m, float dt) {
  long delta   = m.count - m.lastCount;
  m.lastCount  = m.count;
  m.currentRPM = ((float)delta / m.cpr / dt) * 60.0f;
}

void updatePID(Motor &m, float dt) {
  if (m.targetRPM == 0) { m.integral = 0; m.pwm = 0; setMotorPWM(m, 0); return; }
  float error  = m.targetRPM - m.currentRPM;
  bool satHigh = (m.pwm >= 255 && error > 0);
  bool satLow  = (m.pwm <=   0 && error < 0);
  if (!satHigh && !satLow)
    m.integral = constrain(m.integral + error * dt, -100, 100);
  m.pwm = constrain(m.pwm + m.Kp * error + m.Ki * m.integral, 0, 255);
  setMotorPWM(m, m.pwm);
}

void updateControl() {
  unsigned long now = millis();
  if (now - lastControlTime < (unsigned long)intervalMs) return;
  float dt = (now - lastControlTime) / 1000.0f;
  lastControlTime = now;
  updateRPM(leftMotor,  dt);
  updateRPM(rightMotor, dt);
  // Run PID for all motion states
  if (robotState == STATE_WALL_FOLLOW ||
      robotState == STATE_BALL_OVERRIDE) {
    updatePID(leftMotor,  dt);
    updatePID(rightMotor, dt);
  }
}

// ================= SERIAL READ (NON-BLOCKING) =================
// Protocol: 3-byte packets  [ 0xFF | x_byte | y_byte ]
// The header byte 0xFF lets us re-sync after any framing error or
// leftover bytes from the Serial debug stream.
void readSerial() {
  while (Serial.available() >= 3) {
    if (Serial.peek() != 0xFF) {
      Serial.read();   // discard until aligned to header
      continue;
    }
    Serial.read();           // consume 0xFF header
    pkt[0] = Serial.read();  // x_byte
    pkt[1] = Serial.read();  // y_byte
    lastPacketMs = millis();
  }
}

// ================= BREAK BEAM =================
bool breakBeamTriggered() { return (digitalRead(BREAK_BEAM) == LOW); }

// ====================================================================
//  STATE HANDLERS
// ====================================================================

// ── Wall Follow ─────────────────────────────────────────────────────
void doWallFollow() {
  if (millis() - stateEntryMs < STATE_ENTRY_DELAY_MS) return;

  // Outer corner / wall lost
  if (leftDistanceCM > WALL_LOST_DIST) {
    hardStop();
    wallLostPhase  = WL_REVERSE;
    wallLostStartL = leftMotor.count;
    wallLostStartR = rightMotor.count;
    stateEntryMs   = millis();
    robotState     = STATE_WALL_LOST;
    // Serial.println("WF | -> WALL_LOST");
    return;
  }

  // Inner corner / front obstacle
  if (frontDistanceCM > 0 && frontDistanceCM < FRONT_STOP_DIST) {
    hardStop();
    turnStartCountL = leftMotor.count;
    turnStartCountR = rightMotor.count;
    stateEntryMs    = millis();
    robotState      = STATE_TURNING;
    // Serial.println("WF | -> TURNING");
    return;
  }

  float baseRPM  = (frontDistanceCM > 0 && frontDistanceCM < FRONT_SLOW_DIST)
                   ? SLOW_SPEED_WF : BASE_SPEED;
  float error    = leftDistanceCM - TARGET_LEFT_DIST;
  float correction = constrain(1.0f * error, -5.0f, 5.0f);

  leftMotor.targetRPM  = constrain(baseRPM - correction, 0, 20);
  rightMotor.targetRPM = constrain(baseRPM + correction, 0, 20);
}

// ── Pivot Turn ──────────────────────────────────────────────────────
void doTurning() {
  if (millis() - stateEntryMs < STATE_ENTRY_DELAY_MS) return;

  long travelL = abs(leftMotor.count  - turnStartCountL);
  long travelR = abs(rightMotor.count - turnStartCountR);

  if (travelL >= TURN_COUNTS_L && travelR >= TURN_COUNTS_R) {
    hardStop();
    stateEntryMs = millis();
    robotState   = STATE_WALL_FOLLOW;
    // Serial.println("Turn complete -> WALL_FOLLOW");
    return;
  }

  int pwmVal = map(TURN_SPEED, 0, 20, 0, 255);
  setMotorPWM(leftMotor,  (travelL < TURN_COUNTS_L) ?  pwmVal : 0);
  setMotorPWM(rightMotor, (travelR < TURN_COUNTS_R) ? -pwmVal : 0);
}

// ── Wall Lost Recovery ──────────────────────────────────────────────
void doWallLost() {
  if (millis() - stateEntryMs < STATE_ENTRY_DELAY_MS) return;

  switch (wallLostPhase) {

    case WL_REVERSE: {
      long travelL = abs(leftMotor.count  - wallLostStartL);
      bool wallBack   = (leftDistanceCM <= WALL_RECOVERY_DIST);
      bool maxReached = (travelL >= REVERSE_MAX_COUNTS_L);
      if (wallBack || maxReached) {
        hardStop();
        wallLostStartL = leftMotor.count;
        wallLostStartR = rightMotor.count;
        wallLostPhase  = WL_CREEP;
        stateEntryMs   = millis();
        // Serial.println("WL | -> WL_CREEP");
        return;
      }
      int pwmVal = map(WALL_LOST_OL_SPEED, 0, 20, 0, 255);
      setMotorPWM(leftMotor,  -pwmVal);
      setMotorPWM(rightMotor, -pwmVal);
      break;
    }

    case WL_CREEP: {
      long creepL = abs(leftMotor.count  - wallLostStartL);
      long creepR = abs(rightMotor.count - wallLostStartR);
      if (creepL >= CREEP_COUNTS_L_WF && creepR >= CREEP_COUNTS_R_WF) {
        hardStop();
        arcStartMs    = millis();
        wallLostPhase = WL_ARC;
        stateEntryMs  = millis();
        // Serial.println("WL | -> WL_ARC");
        return;
      }
      int pwmVal = map(WALL_LOST_OL_SPEED, 0, 20, 0, 255);
      setMotorPWM(leftMotor,  pwmVal);
      setMotorPWM(rightMotor, pwmVal);
      break;
    }

    case WL_ARC: {
      if (leftDistanceCM <= WALL_RECOVERY_DIST) {
        hardStop();
        stateEntryMs = millis();
        robotState   = STATE_WALL_FOLLOW;
        // Serial.println("WL | -> WALL_FOLLOW");
        return;
      }
      if (millis() - arcStartMs > ARC_TIMEOUT_MS) {
        hardStop();
        robotState = STATE_STOPPED;
        // Serial.println("WL | ARC timeout -> STOPPED");
        return;
      }
      setMotorPWM(rightMotor, map(ARC_OUTER_SPEED, 0, 20, 0, 255));
      setMotorPWM(leftMotor,  map(ARC_INNER_SPEED, 0, 20, 0, 255));
      break;
    }

    case WL_REALIGN: {
      long sL = abs(leftMotor.count  - wallLostStartL);
      long sR = abs(rightMotor.count - wallLostStartR);
      if (sL >= REALIGN_COUNTS && sR >= REALIGN_COUNTS) {
        hardStop();
        stateEntryMs = millis();
        robotState   = STATE_WALL_FOLLOW;
        // Serial.println("WL | -> WALL_FOLLOW");
        return;
      }
      int pwmVal = map(SLOW_SPEED_WF, 0, 20, 0, 255);
      setMotorPWM(leftMotor,  pwmVal);
      setMotorPWM(rightMotor, pwmVal);
      break;
    }
  }
}

// ── Ball Override (TRACK → COLLECT → COLLECTED) ─────────────────────
void doBallOverride() {

  switch (ballPhase) {

    // ── Track: differential steer toward ball ──────────────────────
    case BP_TRACK: {
      if (millis() - ballPhaseMs < BALL_ENTRY_DELAY_MS) return;

      // Ball disappeared from frame → it has reached the robot
      if (!ballVisible()) {
        rollersOn();
        hardStop();
        ballPhase   = BP_COLLECT;
        ballPhaseMs = millis();
        // Serial.println("Ball | frame exit -> BP_COLLECT");
        return;
      }

      float error      = (float)pkt[0] - (float)X_CENTER;
      float correction = 0;
      if (abs(error) > DEAD_BAND) {
        correction = constrain(Kp_heading * error,
                               -(float)MAX_CORRECTION,
                                (float)MAX_CORRECTION);
      }
      leftMotor.targetRPM  = constrain((float)TRACK_SPEED + correction,
                                       0, (float)(TRACK_SPEED + MAX_CORRECTION));
      rightMotor.targetRPM = constrain((float)TRACK_SPEED - correction,
                                       0, (float)(TRACK_SPEED + MAX_CORRECTION));
      break;
    }

    // ── Collect: rollers on, creep until break beam ────────────────
    case BP_COLLECT: {
      rollersOn();

      if (breakBeamTriggered()) {
        hardStop();
        rollersOff();
        ballPhase   = BP_COLLECTED;
        ballPhaseMs = millis();
        // Serial.println("Ball | break beam -> BP_COLLECTED");
        return;
      }

      // Safety timeout: if break beam never fires, give up after 3 s
      if (millis() - ballPhaseMs > 3000) {
        hardStop();
        rollersOff();
        ballPhase   = BP_COLLECTED;
        ballPhaseMs = millis();
        // Serial.println("Ball | collect timeout -> BP_COLLECTED");
        return;
      }

      leftMotor.targetRPM  = CREEP_SPEED_BALL;
      rightMotor.targetRPM = CREEP_SPEED_BALL;
      break;
    }

    // ── Collected: pause then return to wall follow ────────────────
    case BP_COLLECTED: {
      hardStop();
      rollersOff();
      if (millis() - ballPhaseMs > 2000) {
        // Flush any stale ball packets so we don't immediately re-trigger
        while (Serial.available()) Serial.read();
        lastPacketMs = 0;

        stateEntryMs = millis();
        robotState   = STATE_WALL_FOLLOW;
        // Serial.println("Ball | done -> WALL_FOLLOW");
      }
      break;
    }
  }
}

// ====================================================================
//  SETUP
// ====================================================================
void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN,     OUTPUT);
  pinMode(SF_PIN,     INPUT);
  pinMode(FRONT_IR_PIN, INPUT);
  pinMode(LEFT_IR_PIN,  INPUT);
  pinMode(ROLLER_PIN, OUTPUT);
  pinMode(BREAK_BEAM, INPUT_PULLUP);

  rollersOff();

  initMotor(leftMotor,  M1_DIR, M1_PWM, ENC_L, 342.0f, false);
  initMotor(rightMotor, M2_DIR, M2_PWM, ENC_R, 347.0f, false);

  attachInterrupt(digitalPinToInterrupt(ENC_L), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), rightISR, RISING);

  enableDriver();
  delay(2000);  // IR sensor stabilisation

  lastControlTime = millis();
  prevIRTime      = micros();
  stateEntryMs    = millis();

  Serial.println("=== Wall-Follow + Ball Collector Started ===");
  Serial.print("Target left dist : "); Serial.print(TARGET_LEFT_DIST);   Serial.println(" cm");
  Serial.print("Wall lost dist   : "); Serial.print(WALL_LOST_DIST);     Serial.println(" cm");
  Serial.print("Base speed       : "); Serial.print(BASE_SPEED);         Serial.println(" RPM");
  Serial.print("Track speed      : "); Serial.print(TRACK_SPEED);        Serial.println(" RPM");
  Serial.print("Creep speed      : "); Serial.print(CREEP_SPEED_BALL);   Serial.println(" RPM");
  Serial.print("Ball timeout     : "); Serial.print(BALL_TIMEOUT_MS);    Serial.println(" ms");
  Serial.print("Entry delay (WF) : "); Serial.print(STATE_ENTRY_DELAY_MS); Serial.println(" ms");
  Serial.println("============================================");
}

// ====================================================================
//  LOOP
// ====================================================================
void loop() {
  readSerial();     // non-blocking — updates pkt[] and lastPacketMs
  updateIR();       // Butterworth + MA on both IR channels
  updateControl();  // RPM measurement + PID (runs on interval)

  // ── Global override: ball detected while wall-following ──────────
  // Only trigger from wall-follow states (not during a pivot or recovery)
  // so we don't interrupt a turn mid-way.
  if (robotState == STATE_WALL_FOLLOW && ballVisible()) {
    hardStop();
    ballPhase   = BP_TRACK;
    ballPhaseMs = millis();
    robotState  = STATE_BALL_OVERRIDE;
    // Serial.println("Ball detected -> BALL_OVERRIDE");
  }

  // ── State dispatch ───────────────────────────────────────────────
  switch (robotState) {
    case STATE_WALL_FOLLOW:  doWallFollow();  break;
    case STATE_TURNING:      doTurning();     break;
    case STATE_WALL_LOST:    doWallLost();    break;
    case STATE_BALL_OVERRIDE: doBallOverride(); break;
    case STATE_EXIT:                           break;
    case STATE_STOPPED:      hardStop();       break;
  }
}
