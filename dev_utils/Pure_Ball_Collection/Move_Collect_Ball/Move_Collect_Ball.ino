// =============================================================
//  Ball Tracker + Collector — Arduino
//
//  Serial protocol (RPi -> Arduino, 115200 baud):
//    2 bytes sent every frame when ball is detected:
//      [0] x_byte : 0-255 mapped from pixel x
//      [1] y_byte : 0-255 mapped from pixel y
//
//    No packet = no ball. Arduino detects ball loss via timeout.
//
//  Motion strategy:
//    Robot drives forward at SLOW_SPEED while applying a gentle
//    left/right speed differential to keep the ball centred on
//    the vertical centreline.
//
//    error    = x_byte - 127
//    leftRPM  = SLOW_SPEED + correction
//    rightRPM = SLOW_SPEED - correction
//
//  Collection trigger:
//    Ball disappears from frame while robot is in STATE_TRACK
//    (ball has reached the robot and exited the bottom of frame).
//    → Rollers turn on, robot creeps straight forward.
//    → Break beam (A5 == 0) confirms ball is inside → stop everything.
//
//  State flow:
//    SEARCH -> TRACK -> COLLECT -> COLLECTED -> SEARCH
// =============================================================

// ================= PIN DEFINITIONS =================
#define M1_DIR 7
#define M1_PWM 9
#define M2_DIR 8
#define M2_PWM 10

#define EN_PIN       4
#define SF_PIN       12

#define ENC_L        2
#define ENC_R        3

#define ROLLER_PIN   5    // HIGH = rollers on, LOW = rollers off
#define BREAK_BEAM   A4   // analogRead == 0 when ball crosses beam

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
#define SLOW_SPEED      7    // RPM — forward speed during tracking
#define CREEP_SPEED     3    // RPM — forward speed during collection
#define MAX_CORRECTION  3    // RPM — max differential per wheel during tracking

// ================= HEADING CORRECTION =================
#define X_CENTER   127       // midpoint of 0-255 x space
#define DEAD_BAND    8       // ± no-correction zone around centre

float Kp_heading = 0.030f;

// ================= BALL TIMEOUT =================
#define BALL_TIMEOUT_MS  300  // no packet for this long = ball lost

unsigned long lastPacketMs = 0;
uint8_t pkt[2] = {0, 0};     // [0]=x_byte, [1]=y_byte

bool ballVisible() {
  return (millis() - lastPacketMs < BALL_TIMEOUT_MS);
}

// ================= STATE MACHINE =================
enum RobotState {
  STATE_SEARCH,     // stopped, waiting for ball to appear
  STATE_TRACK,      // drive + differential steer toward ball
  STATE_COLLECT,    // ball left frame — rollers on, creep forward until break beam
  STATE_COLLECTED,  // break beam fired — stop everything, pause, reset
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

void rollersOn()  { digitalWrite(ROLLER_PIN, HIGH); }
void rollersOff() { digitalWrite(ROLLER_PIN, LOW);  }

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
    m.integral = 0;
    m.pwm      = 0;
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
  if (robotState == STATE_TRACK || robotState == STATE_COLLECT) {
    updatePID(leftMotor,  dt);
    updatePID(rightMotor, dt);
  }
}

// ================= SERIAL READ (NON-BLOCKING) =================
void readSerial() {
  while (Serial.available() >= 2) {
    pkt[0] = Serial.read();  // x_byte
    pkt[1] = Serial.read();  // y_byte
    lastPacketMs = millis();
  }
}

// ================= BREAK BEAM =================
bool breakBeamTriggered() {
  return (digitalRead(BREAK_BEAM) == LOW);
 
}

// ================= STATE: SEARCH =================
void doSearch() {
  hardStop();
  rollersOff();
}

// ================= STATE: TRACK =================
// Drive forward with proportional differential steering.
// Ball disappearing = it has reached the robot → handled in loop().
// No Serial.print() — runs every loop iteration.
void doTrack() {
  if (millis() - stateEntryMs < STATE_ENTRY_DELAY_MS) return;

  float error = (float)pkt[0] - (float)X_CENTER;

  float correction = 0;
  if (abs(error) > DEAD_BAND) {
    correction = Kp_heading * error;
    correction = constrain(correction,
                           -(float)MAX_CORRECTION,
                            (float)MAX_CORRECTION);
  }

  float leftRPM  = SLOW_SPEED + correction;
  float rightRPM = SLOW_SPEED - correction;

  leftRPM  = constrain(leftRPM,  0, (float)(SLOW_SPEED + MAX_CORRECTION));
  rightRPM = constrain(rightRPM, 0, (float)(SLOW_SPEED + MAX_CORRECTION));

  leftMotor.targetRPM  = leftRPM;
  rightMotor.targetRPM = rightRPM;
}

// ================= STATE: COLLECT =================
// Ball has left the frame — it is right in front of the robot.
// Rollers stay on. Robot creeps straight forward.
// Only exits when break beam confirms ball is inside.
// No Serial.print() — runs every loop iteration.
void doCollect() {
  rollersOn();

  if (breakBeamTriggered()) {
    hardStop();
    rollersOff();
    stateEntryMs = millis();
    robotState   = STATE_COLLECTED;
    Serial.println("Break beam -> COLLECTED");
    return;
  }

  leftMotor.targetRPM  = CREEP_SPEED;
  rightMotor.targetRPM = CREEP_SPEED;
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN,     OUTPUT);
  pinMode(SF_PIN,     INPUT);
  pinMode(ROLLER_PIN, OUTPUT);
  pinMode(BREAK_BEAM, INPUT_PULLUP);
  
  rollersOff();

  initMotor(leftMotor,  M1_DIR, M1_PWM, ENC_L, 342.0f, false);
  initMotor(rightMotor, M2_DIR, M2_PWM, ENC_R, 347.0f, false);

  attachInterrupt(digitalPinToInterrupt(ENC_L), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), rightISR, RISING);

  enableDriver();
  delay(1000);

  lastControlTime = millis();
  stateEntryMs    = millis();

  Serial.println("=== Ball Collector Started ===");
  Serial.print("Track speed=");    Serial.print(SLOW_SPEED);     Serial.println(" RPM");
  Serial.print("Creep speed=");    Serial.print(CREEP_SPEED);    Serial.println(" RPM");
  Serial.print("Max correction="); Serial.print(MAX_CORRECTION); Serial.println(" RPM");
  Serial.print("Kp_heading=");     Serial.print(Kp_heading, 4);
  Serial.print("  Dead band=");    Serial.println(DEAD_BAND);
  Serial.println("==============================");
}

// ================= LOOP =================
void loop() {
  readSerial();     // non-blocking, always first
  updateControl();  // RPM + PID, always runs

  // ── Global state transitions ──

  // Ball appeared while searching → start tracking
  if (robotState == STATE_SEARCH && ballVisible()) {
    hardStop();
    stateEntryMs = millis();
    robotState   = STATE_TRACK;
    Serial.println("Ball found -> TRACK");
  }

  // Ball lost during tracking → ball has reached the robot, start collection
  // (distinguished from ball never appearing by only firing from STATE_TRACK)
  if (robotState == STATE_TRACK && !ballVisible()) {
    rollersOn();
    stateEntryMs = millis();
    robotState   = STATE_COLLECT;
    Serial.println("Ball left frame -> COLLECT");
  }

  // ── State handlers ──
  switch (robotState) {
    case STATE_SEARCH:    doSearch();    break;
    case STATE_TRACK:     doTrack();     break;
    case STATE_COLLECT:   doCollect();   break;

    case STATE_COLLECTED:
      hardStop();
      rollersOff();
      if (millis() - stateEntryMs > 2000) {
        stateEntryMs = millis();
        robotState   = STATE_SEARCH;
        Serial.println("Ready -> SEARCH");
      }
      break;

    case STATE_STOPPED:
      hardStop();
      rollersOff();
      break;
  }
}
