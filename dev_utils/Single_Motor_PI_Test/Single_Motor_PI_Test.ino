// ================= PIN DEFINITIONS =================
#define EN_PIN  4

// ================= WHICH MOTOR TO TEST =================
// Uncomment ONE of the two blocks below:

// --- LEFT MOTOR (M1) ---
// #define DIR_PIN    7
// #define PWM_PIN    9
// #define ENC_PIN    2
// #define CPR        684.0   // CHANGE mode, 342 * 2
// #define INVERT_DIR false
// #define MOTOR_NAME "LEFT"

// --- RIGHT MOTOR (M2) ---
#define DIR_PIN    8
#define PWM_PIN    10
#define ENC_PIN    3
#define CPR        694.0   // CHANGE mode, 347 * 2
#define INVERT_DIR false
#define MOTOR_NAME "RIGHT"

// ================= SETTINGS =================
#define TARGET_RPM  80     // <-- SET YOUR DESIRED RPM HERE

// ================= MOTOR STRUCT =================
struct Motor {
  volatile long count;
  long lastCount;

  float targetRPM;
  float targetCountsPerSec;
  float currentCountsPerSec;
  float currentRPM;

  float pwm;
  float Kp;
  float Ki;
  float integral;
};

Motor motor;

unsigned long lastTime = 0;
const int intervalMs = 100;

// ================= ISR =================
void encoderISR() { motor.count++; }

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

// ================= MOTOR =================
void updateSpeed(float dt) {
  long delta      = motor.count - motor.lastCount;
  motor.lastCount = motor.count;

  motor.currentCountsPerSec = delta / dt;
  motor.currentRPM = countsPerSecToRPM(motor.currentCountsPerSec, CPR);
}

void updatePID(float dt) {
  if (motor.targetRPM == 0) {
    motor.integral = 0;
    motor.pwm      = 0;
    return;
  }

  float error = motor.targetCountsPerSec - motor.currentCountsPerSec;

  motor.integral += error * dt;
  motor.integral  = clampFloat(motor.integral, -300, 300);

  float correction = motor.Kp * error + motor.Ki * motor.integral;

  motor.pwm += correction;
  motor.pwm  = clampFloat(motor.pwm, 0, 255);
}

void applyMotor() {
  if (motor.targetRPM == 0) {
    analogWrite(PWM_PIN, 0);
    return;
  }

  bool goForward = (motor.targetRPM > 0);
  if (INVERT_DIR) goForward = !goForward;

  digitalWrite(DIR_PIN, goForward ? HIGH : LOW);
  analogWrite(PWM_PIN, (int)motor.pwm);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  pinMode(EN_PIN,  OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(ENC_PIN, INPUT_PULLUP);

  digitalWrite(EN_PIN, HIGH);

  motor.count               = 0;
  motor.lastCount           = 0;
  motor.targetRPM           = TARGET_RPM;
  motor.targetCountsPerSec  = rpmToCountsPerSec(TARGET_RPM, CPR);
  motor.currentCountsPerSec = 0;
  motor.currentRPM          = 0;
  motor.pwm                 = 0;
  motor.integral            = 0;

  // ── Per-motor PID gains ──────────────────────────────
  // These may need separate tuning once you test each motor.
  // Start with the same values and adjust if one overshoots
  // or responds slower than the other.
#ifdef MOTOR_NAME
  #if defined(DIR_PIN) && DIR_PIN == 7   // Left motor
    motor.Kp = 0.08;
    motor.Ki = 0.03;
  #else                                  // Right motor
    motor.Kp = 0.08;
    motor.Ki = 0.03;
  #endif
#endif

  attachInterrupt(digitalPinToInterrupt(ENC_PIN), encoderISR, CHANGE);

  lastTime = millis();

  Serial.print("=== Single Motor PI Test: ");
  Serial.print(MOTOR_NAME);
  Serial.println(" ===");
  Serial.print("Target RPM: "); Serial.println(TARGET_RPM);
  Serial.print("CPR: ");        Serial.println(CPR);
  Serial.println("Time(ms),Target,Actual,PWM,Counts");
}

// ================= LOOP =================
void loop() {
  unsigned long now = millis();

  if (now - lastTime >= intervalMs) {
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    updateSpeed(dt);
    updatePID(dt);
    applyMotor();

    // CSV format — open Serial Plotter or paste into Excel
    Serial.print(now);                  Serial.print(",");
    Serial.print(TARGET_RPM);          Serial.print(",");
    Serial.print(motor.currentRPM);    Serial.print(",");
    Serial.print(motor.pwm);           Serial.print(",");
    Serial.println(motor.count);
  }
}
