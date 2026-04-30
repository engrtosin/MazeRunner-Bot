// =====================================================
// MazeRunner.ino — main entry point
//
// Globals are instantiated here and passed by reference
// to all FSMs. No business logic lives here.
// =====================================================
#include "Config.h"
#include "IRSensor.h"
#include "MotorController.h"
#include "RPiComms.h"
#include "RobotFSM.h"

// -------------------------------------------------------
// Hardware instances
// -------------------------------------------------------
MotorController motorLeft;
MotorController motorRight;

IRSensor irFront(FRONT_IR_PIN, FRONT_IR_A, FRONT_IR_B, FRONT_IR_C);
IRSensor irLeft (LEFT_IR_PIN,  LEFT_IR_A,  LEFT_IR_B,  LEFT_IR_C);

RPiComms rpiComms;

// -------------------------------------------------------
// Top-level FSM
// -------------------------------------------------------
RobotFSM robotFSM(motorLeft, motorRight, irFront, irLeft, rpiComms);

// -------------------------------------------------------
// Encoder ISRs — must be free functions in .ino
// -------------------------------------------------------
void leftISR()  { motorLeft.onEncoderPulse();  }
void rightISR() { motorRight.onEncoderPulse(); }

// -------------------------------------------------------
// Control loop — called on fixed interval from loop()
// Runs RPM measurement + PID for both motors.
// -------------------------------------------------------
static unsigned long lastControlMs = 0;

void updateControl() {
    unsigned long now = millis();
    if (now - lastControlMs < CONTROL_INTERVAL_MS) return;
    float dt      = (now - lastControlMs) / 1000.0f;
    lastControlMs = now;
    motorLeft.update(dt);
    motorRight.update(dt);
}

// -------------------------------------------------------
// Setup
// -------------------------------------------------------
void setup() {
    Serial.begin(115200);

    pinMode(EN_PIN,         OUTPUT);
    pinMode(SF_PIN,         INPUT);
    pinMode(ROLLER_PIN,     OUTPUT);
    pinMode(BREAK_BEAM_PIN, INPUT_PULLUP);
    digitalWrite(ROLLER_PIN, LOW);

    motorLeft.init (M1_DIR, M1_PWM, ENC_L, CPR_L, false, MOTOR_KP, MOTOR_KI);
    motorRight.init(M2_DIR, M2_PWM, ENC_R, CPR_R, false, MOTOR_KP, MOTOR_KI);

    attachInterrupt(digitalPinToInterrupt(ENC_L), leftISR,  RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R), rightISR, RISING);

    // Enable motor driver
    digitalWrite(EN_PIN, LOW);  delay(5);
    digitalWrite(EN_PIN, HIGH); delay(5);

    delay(2000);    // IR sensor stabilisation

    lastControlMs = millis();

#if DEBUG_SERIAL
    Serial.println("=== MazeRunner Started ===");
    Serial.print("Target left dist : "); Serial.print(TARGET_LEFT_DIST);   Serial.println(" cm");
    Serial.print("Wall lost dist   : "); Serial.print(WALL_LOST_DIST);     Serial.println(" cm");
    Serial.print("Base speed       : "); Serial.print(BASE_SPEED);         Serial.println(" RPM");
    Serial.print("Track speed      : "); Serial.print(TRACK_SPEED);        Serial.println(" RPM");
    Serial.print("Creep speed      : "); Serial.print(CREEP_SPEED_BALL);   Serial.println(" RPM");
    Serial.print("Ball timeout     : "); Serial.print(BALL_TIMEOUT_MS);    Serial.println(" ms");
    Serial.print("Entry delay (WF) : "); Serial.print(STATE_ENTRY_DELAY_MS); Serial.println(" ms");
    Serial.println("==========================");
#endif
}

// -------------------------------------------------------
// Loop
// -------------------------------------------------------
void loop() {
    rpiComms.update();     // non-blocking serial read
    irFront.update();       // Butterworth + MA filter
    irLeft.update();        // Butterworth + MA filter
    updateControl();        // RPM + PID on fixed interval
    robotFSM.update();      // top-level state machine
}
