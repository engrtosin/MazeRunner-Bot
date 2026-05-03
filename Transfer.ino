// =============================================================
//  Wall-Following + Ball Collector -- Arduino
//
//  Default behaviour : wall following (left wall, IR sensors).
//  Override          : when RPi sends 2-byte ball packets, the
//                      robot suspends wall-following and runs the
//                      full TRACK -> COLLECT -> COLLECTED cycle,
//                      then resumes wall-following automatically.
//
//  Arc roller assist:
//    While executing WL_ARC, if a ball is visible the rollers are
//    switched on so the ball can be ingested passively as the robot
//    sweeps past.  The rollers turn off as soon as the break beam
//    fires (ball secured) or the arc ends.  The arc motion itself
//    is never interrupted.
//
//  Post-collection reverse:
//    After collecting a ball, if the front IR reads within the
//    danger band [FRONT_REV_BAND_LO, FRONT_REV_BAND_HI] the robot
//    reverses.  Two cases are handled:
//
//    Case A -- reading rises during reverse (robot was in normal
//    range but just slightly too close):
//      Stop once the reading has increased FRONT_REV_RISE_CLEAR cm
//      above the value recorded at the start of reverse.
//
//    Case B -- reading falls during reverse (robot is in the IR
//    curl-back zone; the sensor wraps and reads high even though
//    the wall is very close):
//      The reading will dip through a minimum then start climbing.
//      Keep reversing until the reading climbs back up to
//      FRONT_STOP_DIST, at which point the sensor is back in the
//      reliable region and the robot is at a safe distance.
//
//  The two cases are distinguished automatically: if the first
//  directional movement of the reading (after a small settling
//  window) is upward it is Case A; if it is downward it is Case B.
//
//  RPi -> Arduino serial protocol (115200 baud):
//    3 bytes per detected ball frame:
//      [0] 0xFF   : header / sync byte
//      [1] x_byte : 0-255 mapped from pixel x (clamped to 0-254)
//      [2] y_byte : 0-255 mapped from pixel y (clamped to 0-254)
//    Silence for > BALL_TIMEOUT_MS -> ball lost.
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

#define ROLLER_PIN   5
#define BREAK_BEAM   A4

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
  float Kp, Ki, integral;
  bool invertDir;
};

// ================= FILTER STRUCTS =================
struct BiquadCoeff { float b0, b1, b2, a1, a2; };
struct BiquadState  { float z1 = 0, z2 = 0; };
struct ButterworthState { BiquadState s1, s2; };

#define MA_WINDOW 6
struct MovingAverage {
  float buffer[MA_WINDOW] = {0};
  float sum = 0;
  int   index = 0;
};

// ================= GLOBALS =================
Motor leftMotor, rightMotor;
ButterworthState frontFilter, leftFilter;
MovingAverage frontMA, leftMA;

unsigned long lastControlTime = 0;
unsigned long prevIRTime      = 0;

// ================= STATE ENTRY DELAY =================
unsigned long stateEntryMs = 0;
#define STATE_ENTRY_DELAY_MS  400
#define BALL_ENTRY_DELAY_MS   300

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
const int           intervalMs     = 100;
const unsigned long IR_SAMPLE_TIME = 10000;

// ================= WALL-FOLLOW THRESHOLDS =================
#define TARGET_LEFT_DIST    20.0f
#define FRONT_STOP_DIST     30.0f
#define FRONT_SLOW_DIST     30.0f
#define WALL_LOST_DIST      50.0f
#define WALL_RECOVERY_DIST  30.0f

// ================= WALL-FOLLOW SPEEDS =================
#define BASE_SPEED          20
#define SLOW_SPEED_WF        7
#define WALL_LOST_OL_SPEED   3
#define TURN_SPEED           7
#define ARC_OUTER_SPEED      5
#define ARC_INNER_SPEED      2

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
#define TRACK_SPEED       7
#define CREEP_SPEED_BALL  3
#define MAX_CORRECTION    3

// ================= BALL HEADING CORRECTION =================
#define X_CENTER  127
#define DEAD_BAND   8
float Kp_heading = 0.030f;

// ================= BALL TIMEOUT =================
#define BALL_TIMEOUT_MS  300
unsigned long lastPacketMs = 0;
uint8_t pkt[2] = {0, 0};
bool ballVisible() { return (millis() - lastPacketMs < BALL_TIMEOUT_MS); }

// ================= ARC ROLLER ASSIST THRESHOLD =================
// Rollers activate during WL_ARC only when the ball Y pixel is at or
// above this value (0 = top of frame / far, 254 = bottom / close).
// Keeps rollers off for distant balls that are unlikely to reach the
// intake during the arc sweep.
#define BALL_ARC_Y_THRESHOLD  127

// ================= POST-COLLECTION REVERSE THRESHOLDS =================
//
// After collecting, if frontDistanceCM is inside [FRONT_REV_BAND_LO,
// FRONT_REV_BAND_HI] the robot needs to reverse before wall-following.
//
// FRONT_REV_BAND_LO : lower bound of the danger band.  Below this the
//   reading is reliable and the robot is far enough to turn normally.
// FRONT_REV_BAND_HI : upper bound.  Above this the wall is not a concern
//   (open space ahead).
// FRONT_REV_RISE_CLEAR : Case A exit -- stop once the reading has risen
//   this many cm above the value seen at the start of reverse.
//   (robot was close but not in curl-back; a small retreat is enough)
// FRONT_STOP_DIST is reused as the Case B exit target -- once the sensor
//   climbs back to this value after the curl-back dip the robot is at
//   the normal pivot distance and wall-follow can resume safely.
//
#define FRONT_REV_BAND_LO    20.0f   // cm -- start reversing if above this
#define FRONT_REV_BAND_HI    33.0f   // cm -- start reversing if below this
#define FRONT_REV_RISE_CLEAR  5.0f   // cm -- Case A: rise needed to stop

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

long turnStartCountL = 0, turnStartCountR = 0;

enum WallLostPhase { WL_REVERSE, WL_CREEP, WL_ARC, WL_REALIGN };
WallLostPhase wallLostPhase  = WL_REVERSE;
long          wallLostStartL = 0, wallLostStartR = 0;
unsigned long arcStartMs     = 0;

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
  bool fwd = (pwm >= 0);
  if (m.invertDir) fwd = !fwd;
  digitalWrite(m.dirPin, fwd ? HIGH : LOW);
  analogWrite(m.pwmPin, (int)abs(pwm));
  m.pwm = pwm;
}

void hardStop() {
  leftMotor.targetRPM = rightMotor.targetRPM = 0;
  leftMotor.integral  = rightMotor.integral  = 0;
  leftMotor.pwm       = rightMotor.pwm       = 0;
  setMotorPWM(leftMotor,  0);
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
  return processBiquad(stage2Coeff, f.s2,
         processBiquad(stage1Coeff, f.s1, x));
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
  return constrain(A_left / (adc + B_left) + C_left, 0.0f, 500.0f);
}

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
  long delta  = m.count - m.lastCount;
  m.lastCount = m.count;
  m.currentRPM = ((float)delta / m.cpr / dt) * 60.0f;
}

void updatePID(Motor &m, float dt) {
  if (m.targetRPM == 0) {
    m.integral = 0; m.pwm = 0; setMotorPWM(m, 0); return;
  }
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

// ================= SERIAL READ =================
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
bool breakBeamTriggered() { return digitalRead(BREAK_BEAM) == LOW; }

// ====================================================================
//  STATE HANDLERS
// ====================================================================

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
      // Roller assist: spin rollers whenever a ball is visible so it
      // can be ingested as the robot sweeps past.  Stop rollers the
      // moment the break beam confirms the ball is secured, or when
      // the arc ends (both exit paths call rollersOff() below).
      if (breakBeamTriggered()) {
        rollersOff();
      } else if (ballVisible() && pkt[1] >= BALL_ARC_Y_THRESHOLD) {
        rollersOn();
      }

      // Arc complete: wall re-acquired
      if (leftDistanceCM <= WALL_RECOVERY_DIST) {
        hardStop();
        rollersOff();
        stateEntryMs = millis();
        robotState   = STATE_WALL_FOLLOW;
        return;
      }

      // Arc timeout
      if (millis() - arcStartMs > ARC_TIMEOUT_MS) {
        hardStop();
        rollersOff();
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

// ── Ball Override ───────────────────────────────────────────────────
void doBallOverride() {

  switch (ballPhase) {

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
      if (abs(error) > DEAD_BAND)
        correction = constrain(Kp_heading * error,
                               -(float)MAX_CORRECTION, (float)MAX_CORRECTION);

      leftMotor.targetRPM  = constrain((float)TRACK_SPEED + correction, 0,
                                       (float)(TRACK_SPEED + MAX_CORRECTION));
      rightMotor.targetRPM = constrain((float)TRACK_SPEED - correction, 0,
                                       (float)(TRACK_SPEED + MAX_CORRECTION));
      break;
    }

    case BP_COLLECT: {
      rollersOn();

      if (breakBeamTriggered() || millis() - ballPhaseMs > 3000) {
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

    // ── Collected ───────────────────────────────────────────────────
    //
    // After the 2 s pause, check whether the front IR is in the
    // danger band.  If it is, reverse.  While reversing:
    //
    //   - Track the reading at reverse start (revStartReading) and
    //     the running minimum seen so far (revMinReading).
    //
    //   - If the reading rises from the start without ever going
    //     below revStartReading - 1 cm (no dip), it is Case A:
    //     stop once the reading has risen FRONT_REV_RISE_CLEAR above
    //     revStartReading.
    //
    //   - If the reading drops below revStartReading - 1 cm at any
    //     point, the curl-back zone is confirmed (Case B): keep
    //     going until the reading climbs back up to FRONT_STOP_DIST.
    //
    //  The 1 cm tolerance absorbs sensor noise when deciding whether
    //  a dip has occurred.
    //
    case BP_COLLECTED: {
      rollersOff();

      // 2 s pause
      if (millis() - ballPhaseMs <= 2000) {
        hardStop();
        break;
      }

      // Persistent state for the reverse phase, initialised once per
      // collection cycle using a static + a fresh-cycle flag.
      static bool  revInitDone    = false;
      static float revStartReading = 0.0f;
      static float revMinReading   = 0.0f;
      static bool  revNeedReverse  = false;
      static bool  revCurlBack     = false;   // Case B confirmed

      // One-time init on first tick after the pause
      if (!revInitDone) {
        while (Serial.available()) Serial.read();
        lastPacketMs = 0;

        float f = frontDistanceCM;
        revNeedReverse = (f > FRONT_REV_BAND_LO && f < FRONT_REV_BAND_HI);
        revStartReading = f;
        revMinReading   = f;
        revCurlBack     = false;
        revInitDone     = true;
      }

      // No reverse needed: return to wall-follow
      if (!revNeedReverse) {
        hardStop();
        revInitDone = false;
        stateEntryMs = millis();
        robotState   = STATE_WALL_FOLLOW;
        break;
      }

      // --- Reversing ---
      int pwmVal = map(WALL_LOST_OL_SPEED, 0, 20, 0, 255);
      setMotorPWM(leftMotor,  -pwmVal);
      setMotorPWM(rightMotor, -pwmVal);

      float f = frontDistanceCM;

      // Update running minimum
      if (f > 0.0f && f < revMinReading) revMinReading = f;

      // Detect curl-back: reading dropped more than 1 cm below start
      if (f > 0.0f && f < revStartReading - 1.0f) revCurlBack = true;

      bool done = false;
      if (!revCurlBack) {
        // Case A: reading going up from the start -- stop once it has
        // risen FRONT_REV_RISE_CLEAR above the starting value.
        done = (f > revStartReading + FRONT_REV_RISE_CLEAR);
      } else {
        // Case B: curl-back confirmed -- stop once the reading has
        // climbed back up to FRONT_STOP_DIST after the dip.
        done = (f >= FRONT_STOP_DIST);
      }

      if (done) {
        hardStop();
        revInitDone = false;
        stateEntryMs = millis();
        robotState   = STATE_WALL_FOLLOW;
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
  Serial.print("Target left dist   : "); Serial.print(TARGET_LEFT_DIST);        Serial.println(" cm");
  Serial.print("Wall lost dist     : "); Serial.print(WALL_LOST_DIST);          Serial.println(" cm");
  Serial.print("Base speed         : "); Serial.print(BASE_SPEED);              Serial.println(" RPM");
  Serial.print("Track speed        : "); Serial.print(TRACK_SPEED);             Serial.println(" RPM");
  Serial.print("Creep speed        : "); Serial.print(CREEP_SPEED_BALL);        Serial.println(" RPM");
  Serial.print("Ball timeout       : "); Serial.print(BALL_TIMEOUT_MS);         Serial.println(" ms");
  Serial.print("Entry delay (WF)   : "); Serial.print(STATE_ENTRY_DELAY_MS);    Serial.println(" ms");
  Serial.print("Rev band lo        : "); Serial.print(FRONT_REV_BAND_LO);       Serial.println(" cm");
  Serial.print("Rev band hi        : "); Serial.print(FRONT_REV_BAND_HI);       Serial.println(" cm");
  Serial.print("Rev rise clear     : "); Serial.print(FRONT_REV_RISE_CLEAR);    Serial.println(" cm");
  Serial.print("Arc roller Y thr   : "); Serial.print(BALL_ARC_Y_THRESHOLD);    Serial.println(" px");
  Serial.println("============================================");
}

// ====================================================================
//  LOOP
// ====================================================================
void loop() {
  readSerial();
  updateIR();
  updateControl();

  if (robotState == STATE_WALL_FOLLOW && ballVisible()) {
    hardStop();
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
