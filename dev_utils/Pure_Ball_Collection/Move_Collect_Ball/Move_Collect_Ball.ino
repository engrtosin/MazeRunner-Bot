// ===== PARAMETERS =====
#define CENTER 125
#define CENTER_TOL 12

#define Y_THRESHOLD 125       // must be below midline
#define CLOSE_THRESHOLD 220   // close to robot

#define NO_BALL 255

#define TURN_SPEED 80
#define FORWARD_SPEED 110

// ===== GLOBAL =====
uint8_t x_pos = NO_BALL;
uint8_t y_pos = NO_BALL;

enum State {
  SEARCH,
  ALIGN,
  FORWARD,
  STOPPED
};

State state = SEARCH;

// ===== MOTOR CONTROL =====
// Replace with your driver code
void setMotor(int left, int right);

void stopMotors() {
  setMotor(0, 0);
}

void spinLeft() {
  setMotor(-TURN_SPEED, TURN_SPEED);
}

void spinRight() {
  setMotor(TURN_SPEED, -TURN_SPEED);
}

void forward() {
  setMotor(FORWARD_SPEED, FORWARD_SPEED);
}

// ===== SERIAL =====
void readCamera() {
  if (Serial.available() >= 2) {
    x_pos = Serial.read();
    y_pos = Serial.read();
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(9600);
}

// ===== LOOP =====
void loop() {

  readCamera();

  switch(state) {

    // ===== NO BALL =====
    case SEARCH:
      stopMotors();

      if (x_pos != NO_BALL && y_pos != NO_BALL && y_pos > Y_THRESHOLD) {
        state = ALIGN;
      }
      break;

    // ===== ALIGN (SPIN ONLY) =====
    case ALIGN:

      if (x_pos == NO_BALL) {
        state = SEARCH;
        break;
      }

      int error = x_pos - CENTER;

      if (abs(error) <= CENTER_TOL) {
        stopMotors();
        delay(200);  // stabilize before moving
        state = FORWARD;
      }
      else if (error > 0) {
        spinRight();
      }
      else {
        spinLeft();
      }

      break;

    // ===== GO STRAIGHT =====
    case FORWARD:

      if (x_pos == NO_BALL) {
        state = SEARCH;
        break;
      }

      // Optional: small correction allowed
      int error2 = x_pos - CENTER;

      if (abs(error2) > 25) {
        // lost alignment → re-align
        state = ALIGN;
        break;
      }

      if (y_pos < CLOSE_THRESHOLD) {
        forward();
      } else {
        stopMotors();
        state = STOPPED;
      }

      break;

    // ===== COLLECT =====
    case STOPPED:
      stopMotors();
      delay(1500);  // simulate pickup
      state = SEARCH;
      break;
  }
}