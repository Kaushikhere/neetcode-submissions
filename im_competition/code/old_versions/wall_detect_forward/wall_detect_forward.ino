/*
  ==========================================
       WALL DETECT -> FORWARD + HOLD DISTANCE
       Raspberry Pi Pico + L298N
       6 Optocouplers (inverted PWM)
       Right-side HC-SR04
  ==========================================

  BEHAVIOUR:

  Wall detected on the right  -> move forward and hold the same
                                 distance from the wall
  Wall lost while moving      -> turn about 10 degrees in ONE
                                 direction (SEARCH_DIRECTION); if the
                                 wall is seen again, carry on forward,
                                 otherwise stop

  The robot holds TARGET_DISTANCE from the wall by changing only
  the right motor. Anywhere between TARGET_DISTANCE and
  WALL_DETECT_DISTANCE it keeps driving and steers back.

  MOTOR PINS (UNCHANGED):

  ENA = GP10   IN1 = GP11   IN2 = GP12
  ENB = GP15   IN3 = GP13   IN4 = GP14

  RIGHT HC-SR04:

  TRIG = GP16
  ECHO = GP17

  CALIBRATED RAW PWM (UNCHANGED):

  Left motor  = 0
  Right motor = 73

  Lower PWM = Faster, Higher PWM = Slower, 255 = Stop
*/

// ==========================================
// MOTOR PINS
// ==========================================

#define ENA 10
#define IN1 11
#define IN2 12

#define IN3 13
#define IN4 14
#define ENB 15

// ==========================================
// SENSOR PINS
// ==========================================

#define TRIG_PIN 16
#define ECHO_PIN 17

// ==========================================
// SETTINGS
// ==========================================

// A wall counts as "detected" when it is closer than this (cm).
// Keep this well above the distance you start at, otherwise the
// robot stops instead of steering back when it drifts away.
float WALL_DETECT_DISTANCE = 40.0;

// Distance to hold from the wall (cm).
float TARGET_DISTANCE = 15.0;

// Steering strength: PWM change per cm of distance error.
// Robot corrects too slowly  -> increase.
// Robot zig-zags             -> decrease.
float KP = 5.0;

// No correction while the error is smaller than this (cm).
float DEAD_BAND = 1.0;

// Largest PWM change the steering may apply.
// Kept small on purpose: the steering only nudges the right motor
// around its calibrated speed, so the robot can never spin in circles.
int MAX_CORRECTION = 35;

// Smoothing of the distance readings (0.0 - 1.0).
// Lower = smoother but slower to react.
float SMOOTHING = 0.5;

// Readings in a row without a wall before the robot starts searching.
int MISS_LIMIT = 2;

// Which way the robot turns to get back to the wall.
//  1 = turn RIGHT
// -1 = turn LEFT
// Used both for steering back when the robot is too far from the
// wall and for the search turn when the wall is lost.
// If the robot turns away from the wall, change this value.
int SEARCH_DIRECTION = -1;

// How long the robot spins to turn about 10 degrees (milliseconds).
// There is no angle sensor, so this is set by time:
// turns too far    -> decrease
// turns too little -> increase
int SEARCH_TURN_MS = 80;

// ==========================================
// MOTOR CALIBRATION (UNCHANGED)
// ==========================================

int LEFT_BASE_PWM = 0;
int RIGHT_BASE_PWM = 72;

// ==========================================
// STATE
// ==========================================

int missCount = 0;
bool moving = false;

float smoothDistance = 0;

// ==========================================
// MOTOR STOP
// ==========================================

void stopMotors() {

  // Inverted PWM: 255 = stop

  analogWrite(ENA, 255);
  analogWrite(ENB, 255);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ==========================================
// MOVE FORWARD
// ==========================================

void moveForward(int leftPWM, int rightPWM) {

  // LEFT MOTOR
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // RIGHT MOTOR (mounted on the opposite side)
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, constrain(leftPWM, 0, 255));
  analogWrite(ENB, constrain(rightPWM, 0, 255));
}

// ==========================================
// RIGHT ULTRASONIC SENSOR
// Returns distance in cm, or -1 if no echo
// ==========================================

float getDistance() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    return -1;
  }

  return duration * 0.0343 / 2.0;
}

// ==========================================
// SPIN ON THE SPOT
// ==========================================

void spinRight() {

  // Left motor forward, right motor backward

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, LEFT_BASE_PWM);
  analogWrite(ENB, RIGHT_BASE_PWM);
}

void spinLeft() {

  // Left motor backward, right motor forward

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, LEFT_BASE_PWM);
  analogWrite(ENB, RIGHT_BASE_PWM);
}

// ==========================================
// SPIN FOR A GIVEN TIME WHILE WATCHING THE SENSOR
// Returns true (and stops spinning) as soon as the wall is seen
// ==========================================

bool spinAndLook(bool toRight, int durationMs) {

  if (toRight) {
    spinRight();
  } else {
    spinLeft();
  }

  unsigned long start = millis();

  while (millis() - start < (unsigned long)durationMs) {

    float d = getDistance();

    if (d > 0 && d <= WALL_DETECT_DISTANCE) {
      stopMotors();
      smoothDistance = d;
      return true;
    }

    delay(10);
  }

  stopMotors();
  delay(100);

  return false;
}

// ==========================================
// SEARCH FOR THE WALL
// Turns about 10 degrees in one direction only.
// Returns true if the wall was found.
// ==========================================

bool searchForWall() {

  stopMotors();
  delay(100);

  Serial.println("WALL LOST - SEARCHING");

  if (spinAndLook(SEARCH_DIRECTION > 0, SEARCH_TURN_MS)) {
    return true;
  }

  Serial.println("WALL NOT FOUND - STOP");

  return false;
}

// ==========================================
// SETUP
// ==========================================

void setup() {

  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  analogWriteRange(255);
  analogWriteFreq(500);

  stopMotors();

  delay(1000);

  Serial.println("==============================");
  Serial.println(" WALL DETECT + HOLD DISTANCE");
  Serial.println("==============================");
  Serial.println("Ready!");
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop() {

  float distance = getDistance();

  bool wallDetected = (distance > 0 && distance <= WALL_DETECT_DISTANCE);

  int leftPWM = LEFT_BASE_PWM;
  int rightPWM = RIGHT_BASE_PWM;

  float error = 0;

  if (wallDetected) {

    missCount = 0;

    // ----------------------------------------
    // Starting to move: reset the smoothing
    // ----------------------------------------

    if (!moving) {
      smoothDistance = distance;
      moving = true;
    }

    // Smooth the readings so one jumpy echo cannot jerk the robot
    smoothDistance += SMOOTHING * (distance - smoothDistance);

    // ----------------------------------------
    // Steering
    //
    // The left motor always stays on its calibrated speed.
    // Only the right motor is nudged around its calibrated speed:
    //
    // error > 0 : too far from the wall  -> turn TOWARDS the wall
    // error < 0 : too close to the wall  -> turn AWAY from the wall
    //
    // "Towards the wall" is the direction set by SEARCH_DIRECTION.
    // ----------------------------------------

    error = smoothDistance - TARGET_DISTANCE;

    int correction = 0;

    if (abs(error) > DEAD_BAND) {
      correction = constrain(
        (int)(KP * error) * SEARCH_DIRECTION,
        -MAX_CORRECTION,
        MAX_CORRECTION
      );
    }

    rightPWM = constrain(RIGHT_BASE_PWM + correction, 0, 255);

    moveForward(leftPWM, rightPWM);

  } else {

    // ----------------------------------------
    // No wall -> stop once the miss limit is reached.
    // Until then keep driving straight on the base speeds.
    // ----------------------------------------

    if (missCount < MISS_LIMIT) {
      missCount++;
    }

    if (missCount >= MISS_LIMIT) {

      if (moving && searchForWall()) {

        // Wall found again: carry on forward from the next loop
        missCount = 0;

      } else {

        stopMotors();
        moving = false;
      }

    } else if (moving) {
      moveForward(LEFT_BASE_PWM, RIGHT_BASE_PWM);
    }
  }

  // ----------------------------------------
  // SERIAL MONITOR
  // ----------------------------------------

  Serial.print("Distance: ");

  if (distance < 0) {
    Serial.print("no echo");
  } else {
    Serial.print(distance);
    Serial.print(" cm");
  }

  Serial.print(" | Target: ");
  Serial.print(TARGET_DISTANCE);

  Serial.print(" | Error: ");
  Serial.print(error);

  Serial.print(" | ENA: ");
  Serial.print(moving ? leftPWM : 255);

  Serial.print(" | ENB: ");
  Serial.print(moving ? rightPWM : 255);

  Serial.print(" | ");
  Serial.println(moving ? "FORWARD" : "STOPPED");

  delay(30);
}
