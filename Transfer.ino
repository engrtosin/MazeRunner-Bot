// =============================================================
//  Wall-Following + Ball Collector — Arduino
//
//  Default behaviour : wall following (left wall, IR sensors).
//  Override          : when RPi sends 2-byte ball packets, the
//                      robot suspends wall-following and runs the
//                      full TRACK -> COLLECT -> COLLECTED cycle,
//                      then resumes wall-following automatically.
//
//  Wall-proximity reverse: if the robot crossed the FRONT_STOP_DIST
//  threshold while tracking or collecting a ball (detected by the
//  front sensor ever reading above FRONT_IR_AMBIGUOUS_CM during the
//  approach, indicating it entered the close-range curl-back zone),
//  the post-collection BP_COLLECTED phase reverses the robot instead
//  of immediately returning to wall-follow.  Reverse continues until
//  the front IR reads reliably below FRONT_STOP_DIST for
//  FRONT_CLEAR_CONFIRM_COUNT consecutive samples, meaning the robot
//  has backed out of the ambiguous zone and the wall is genuinely
//  at ~FRONT_STOP_DIST again.  At that point normal wall-follow
//  resumes.
//
//  RPi -> Arduino serial protocol (115200 baud):
//    3 bytes per detected ball frame:
//      [0] 0xFF   : header / sync byte
//      [1] x_byte : 0-255 mapped from pixel x (clamped to 0-254)
//      [2] y_byte : 0-255 mapped from pixel y (clamped to 0-254)
//    Silence for > BALL_TIMEOUT_MS -> ball lost.
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
#define CREEP_COUNTS_L_WF    279
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

// ================= FRONT IR PROXIMITY DETECTION =================
//
// The Sharp IR sensors curl back at very close range: the output
// voltage (and therefore the computed distance) starts rising again
// once the wall is closer than ~10-15 cm.  This means a reading of,
// say, 21 cm could mean "wall is 21 cm away" (reliable) OR "wall is
// ~5 cm away and the sensor has wrapped" (unreliable).
//
// Strategy: during ball collection we watch for the sensor reading
// rising above FRONT_IR_AMBIGUOUS_CM.  That rising trend is only
// possible if the robot has driven close enough to enter the wrap
// zone — a genuine 21 cm wall would have triggered the normal pivot
// before ball collection even started.  So any value above this
// threshold during collection means "we are very close to the wall".
//
// On reverse, we wait for the reading to fall back BELOW
// FRONT_STOP_DIST and stay there for FRONT_CLEAR_CONFIRM_COUNT
// consecutive IR samples (~100 ms each by default).  That confirms
// the sensor has exited the ambiguous zone and the robot is at a
// safe distance again.
//
// FRONT_IR_AMBIGUOUS_CM: readings above this during approach signal
//   that the sensor has entered the curl-back zone.  Set just above
//   the typical "true" FRONT_STOP_DIST reading so normal stopping
//   distances don't accidentally arm the flag.
#define FRONT_IR_AMBIGUOUS_CM     23.0f  // cm — arm threshold during approach
#define FRONT_CLEAR_CONFIRM_COUNT  4     // consecutive samples needed to confirm clear

// Flag set when the front sensor entered the ambiguous zone during
// ball approach / collection.  Cleared when the robot backs clear.
bool  frontProximityArmed = false;
// Counter incremented each IR cycle that frontDistanceCM < FRONT_STOP_DIST
// while reversing; reset whenever the condition is not met.
int   frontClearCount     = 0;

// ================= IR READINGS =================
float frontDistanceCM = 0;
float leftDistanceCM  = 0;

// ================= TOP-LEVEL STATE MACHINE =================
enum RobotState {
  STATE_WALL_FOLLOW,
  STATE_TURNING,
  STATE_WALL_LOST,
  STATE_BALL_OVERRIDE,
  STATE_EXIT,
  STATE_STOPPED
};
RobotState robotState = STATE_WALL_FOLLOW;

// -- Pivot turn --
long turnStartCountL = 0, turnStartCountR = 0;

// -- Wall-lost recovery --
enum WallLostPhase { WL_REVERSE, WL_CREEP, WL_ARC, WL_REALIGN };
WallLostPhase wallLostPhase  = WL_REVERSE;
long          wallLostStartL = 0, wallLostStartR = 0;
unsigned long arcStartMs     = 0;

// -- Ball sub-state --
enum BallPhase { BP_TRACK, BP_COLLECT, BP_COLLECTED };
BallPhase     ballPhase   = BP_TRACK;
unsigned long ballPhaseMs = 0;

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

    // -- Proximity arm: if we are in ball override and the front sensor
    //    rises above the ambiguous threshold, record that the robot has
    //    entered the curl-back zone. --
    if (robotState == STATE_BALL_OVERRIDE &&
        (ballPhase == BP_TRACK || ballPhase == BP_COLLECT)) {
      if (frontDistanceCM > FRONT_IR_AMBIGUOUS_CM) {
        frontProximityArmed = true;
      }
    }
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
  if (robotState == STATE_WALL_FOLLOW ||
      robotState == STATE_BALL_OVERRIDE) {
    updatePID(leftMotor,  dt);
    updatePID(rightMotor, dt);
  }
}

// ================= SERIAL READ (NON-BLOCKING) =================
void readSerial() {
  while (Serial.available() >= 3) {
    if (Serial.peek() != 0xFF) { Serial.read(); continue; }
    Serial.read();
    pkt[0] = Serial.read();
    pkt[1] = Serial.read();
    lastPacketMs = millis();
  }
}

// ================= BREAK BEAM =================
bool breakBeamTriggered() { return (digitalRead(BREAK_BEAM) == LOW); }

// ====================================================================
//  STATE HANDLERS
// ====================================================================

// -- Wall Follow -----------------------------------------------------
void doWallFollow() {
  if (millis() - stateEntryMs < STATE_ENTRY_DELAY_MS) return;

  if (leftDistanceCM > WALL_LOST_DIST) {
    hardStop();
    wallLostPhase  = WL_REVERSE;
    wallLostStartL = leftMotor.count;
    wallLostStartR = rightMotor.count;
    stateEntryMs   = millis();
    robotState     = STATE_WALL_LOST;
    return;
  }

  if (frontDistanceCM > 0 && frontDistanceCM < FRONT_STOP_DIST) {
    hardStop();
    turnStartCountL = leftMotor.count;
    turnStartCountR = rightMotor.count;
    stateEntryMs    = millis();
    robotState      = STATE_TURNING;
    return;
  }

  float baseRPM    = (frontDistanceCM > 0 && frontDistanceCM < FRONT_SLOW_DIST)
                     ? SLOW_SPEED_WF : BASE_SPEED;
  float error      = leftDistanceCM - TARGET_LEFT_DIST;
  float correction = constrain(1.0f * error, -5.0f, 5.0f);

  leftMotor.targetRPM  = constrain(baseRPM - correction, 0, 20);
  rightMotor.targetRPM = constrain(baseRPM + correction, 0, 20);
}

// -- Pivot Turn ------------------------------------------------------
void doTurning() {
  if (millis() - stateEntryMs < STATE_ENTRY_DELAY_MS) return;

  long travelL = abs(leftMotor.count - turnStartCountL);
  long travelR = abs(rightMotor.count - turnStartCountR);

  if (travelL >= TURN_COUNTS_L && travelR >= TURN_COUNTS_R) {
    hardStop();
    stateEntryMs = millis();
    robotState   = STATE_WALL_FOLLOW;
    return;
  }

  int pwmVal = map(TURN_SPEED, 0, 20, 0, 255);
  setMotorPWM(leftMotor,  (travelL < TURN_COUNTS_L) ?  pwmVal : 0);
  setMotorPWM(rightMotor, (travelR < TURN_COUNTS_R) ? -pwmVal : 0);
}

// -- Wall Lost Recovery ----------------------------------------------
void doWallLost() {
  if (millis() - stateEntryMs < STATE_ENTRY_DELAY_MS) return;

  switch (wallLostPhase) {

    case WL_REVERSE: {
      long travelL    = abs(leftMotor.count - wallLostStartL);
      bool wallBack   = (leftDistanceCM <= WALL_RECOVERY_DIST);
      bool maxReached = (travelL >= REVERSE_MAX_COUNTS_L);
      if (wallBack || maxReached) {
        hardStop();
        wallLostStartL = leftMotor.count;
        wallLostStartR = rightMotor.count;
        wallLostPhase  = WL_CREEP;
        stateEntryMs   = millis();
        return;
      }
      int pwmVal = map(WALL_LOST_OL_SPEED, 0, 20, 0, 255);
      setMotorPWM(leftMotor,  -pwmVal);
      setMotorPWM(rightMotor, -pwmVal);
      break;
    }

    case WL_CREEP: {
      long creepL = abs(leftMotor.count - wallLostStartL);
      long creepR = abs(rightMotor.count - wallLostStartR);
      if (creepL >= CREEP_COUNTS_L_WF && creepR >= CREEP_COUNTS_R_WF) {
        hardStop();
        arcStartMs    = millis();
        wallLostPhase = WL_ARC;
        stateEntryMs  = millis();
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
        return;
      }
      if (millis() - arcStartMs > ARC_TIMEOUT_MS) {
        hardStop();
        robotState = STATE_STOPPED;
        return;
      }
      setMotorPWM(rightMotor, map(ARC_OUTER_SPEED, 0, 20, 0, 255));
      setMotorPWM(leftMotor,  map(ARC_INNER_SPEED, 0, 20, 0, 255));
      break;
    }

    case WL_REALIGN: {
      long sL = abs(leftMotor.count - wallLostStartL);
      long sR = abs(rightMotor.count - wallLostStartR);
      if (sL >= REALIGN_COUNTS && sR >= REALIGN_COUNTS) {
        hardStop();
        stateEntryMs = millis();
        robotState   = STATE_WALL_FOLLOW;
        return;
      }
      int pwmVal = map(SLOW_SPEED_WF, 0, 20, 0, 255);
      setMotorPWM(leftMotor,  pwmVal);
      setMotorPWM(rightMotor, pwmVal);
      break;
    }
  }
}

// -- Ball Override (TRACK -> COLLECT -> COLLECTED) -------------------
void doBallOverride() {

  switch (ballPhase) {

    // Track: differential steer toward ball
    case BP_TRACK: {
      if (millis() - ballPhaseMs < BALL_ENTRY_DELAY_MS) return;

      if (!ballVisible()) {
        rollersOn();
        hardStop();
        ballPhase   = BP_COLLECT;
        ballPhaseMs = millis();
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

    // Collect: rollers on, creep until break beam or timeout
    case BP_COLLECT: {
      rollersOn();

      if (breakBeamTriggered()) {
        hardStop();
        rollersOff();
        ballPhase   = BP_COLLECTED;
        ballPhaseMs = millis();
        return;
      }

      if (millis() - ballPhaseMs > 3000) {
        hardStop();
        rollersOff();
        ballPhase   = BP_COLLECTED;
        ballPhaseMs = millis();
        return;
      }

      leftMotor.targetRPM  = CREEP_SPEED_BALL;
      rightMotor.targetRPM = CREEP_SPEED_BALL;
      break;
    }

    // Collected: 2 s pause, then either reverse-to-clear or return to wall-follow.
    //
    // Reverse logic runs every loop tick (not gated on the pause timer) so
    // the IR check and motor commands execute at the full loop rate.
    // A one-shot flag (reversing) activates after the pause expires and stays
    // set until the front sensor is confirmed clear.
    case BP_COLLECTED: {
      rollersOff();

      // ---- Pause phase: hold still for 2 s ----
      if (millis() - ballPhaseMs <= 2000) {
        hardStop();
        break;
      }

      // ---- One-time setup on first tick after pause ----
      // Flush stale serial packets so we don't immediately re-trigger ball
      // detection once we return to wall-follow.
      static bool postPauseInit = false;
      if (!postPauseInit) {
        while (Serial.available()) Serial.read();
        lastPacketMs    = 0;
        frontClearCount = 0;
        postPauseInit   = true;
      }

      // ---- No proximity issue: return to wall-follow immediately ----
      if (!frontProximityArmed) {
        hardStop();
        postPauseInit       = false;
        frontProximityArmed = false;
        stateEntryMs        = millis();
        robotState          = STATE_WALL_FOLLOW;
        break;
      }

      // ---- Proximity armed: reverse until front IR is reliably clear ----
      // Drive both motors backward at the same open-loop PWM used elsewhere.
      int pwmVal = map(WALL_LOST_OL_SPEED, 0, 20, 0, 255);
      setMotorPWM(leftMotor,  -pwmVal);
      setMotorPWM(rightMotor, -pwmVal);

      // Increment confirmation counter only while the reading is below
      // FRONT_STOP_DIST (sensor has exited the curl-back zone).
      // Reset it any time the reading is ambiguous (high or zero) so we
      // require a clean consecutive run.
      if (frontDistanceCM > 0.0f && frontDistanceCM < FRONT_STOP_DIST) {
        frontClearCount++;
      } else {
        frontClearCount = 0;
      }

      if (frontClearCount >= FRONT_CLEAR_CONFIRM_COUNT) {
        hardStop();
        postPauseInit       = false;
        frontProximityArmed = false;
        frontClearCount     = 0;
        stateEntryMs        = millis();
        robotState          = STATE_WALL_FOLLOW;
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

  pinMode(EN_PIN,       OUTPUT);
  pinMode(SF_PIN,       INPUT);
  pinMode(FRONT_IR_PIN, INPUT);
  pinMode(LEFT_IR_PIN,  INPUT);
  pinMode(ROLLER_PIN,   OUTPUT);
  pinMode(BREAK_BEAM,   INPUT_PULLUP);

  rollersOff();

  initMotor(leftMotor,  M1_DIR, M1_PWM, ENC_L, 342.0f, false);
  initMotor(rightMotor, M2_DIR, M2_PWM, ENC_R, 347.0f, false);

  attachInterrupt(digitalPinToInterrupt(ENC_L), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), rightISR, RISING);

  enableDriver();
  delay(2000);

  lastControlTime = millis();
  prevIRTime      = micros();
  stateEntryMs    = millis();

  Serial.println("=== Wall-Follow + Ball Collector Started ===");
  Serial.print("Target left dist    : "); Serial.print(TARGET_LEFT_DIST);              Serial.println(" cm");
  Serial.print("Wall lost dist      : "); Serial.print(WALL_LOST_DIST);                Serial.println(" cm");
  Serial.print("Base speed          : "); Serial.print(BASE_SPEED);                    Serial.println(" RPM");
  Serial.print("Track speed         : "); Serial.print(TRACK_SPEED);                   Serial.println(" RPM");
  Serial.print("Creep speed         : "); Serial.print(CREEP_SPEED_BALL);              Serial.println(" RPM");
  Serial.print("Ball timeout        : "); Serial.print(BALL_TIMEOUT_MS);               Serial.println(" ms");
  Serial.print("Entry delay (WF)    : "); Serial.print(STATE_ENTRY_DELAY_MS);          Serial.println(" ms");
  Serial.print("Front ambiguous thr : "); Serial.print(FRONT_IR_AMBIGUOUS_CM);         Serial.println(" cm");
  Serial.print("Front clear confirm : "); Serial.print(FRONT_CLEAR_CONFIRM_COUNT);     Serial.println(" samples");
  Serial.println("============================================");
}

// ====================================================================
//  LOOP
// ====================================================================
void loop() {
  readSerial();
  updateIR();
  updateControl();

  // Global override: ball detected while wall-following
  if (robotState == STATE_WALL_FOLLOW && ballVisible()) {
    hardStop();
    frontProximityArmed = false;   // fresh collection cycle
    frontClearCount     = 0;
    ballPhase   = BP_TRACK;
    ballPhaseMs = millis();
    robotState  = STATE_BALL_OVERRIDE;
  }

  switch (robotState) {
    case STATE_WALL_FOLLOW:   doWallFollow();   break;
    case STATE_TURNING:       doTurning();      break;
    case STATE_WALL_LOST:     doWallLost();     break;
    case STATE_BALL_OVERRIDE: doBallOverride(); break;
    case STATE_EXIT:                            break;
    case STATE_STOPPED:       hardStop();       break;
  }
}
