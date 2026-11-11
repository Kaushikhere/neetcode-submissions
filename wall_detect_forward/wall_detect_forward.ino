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
  Wall NOT detected           -> stop

  The distance to hold is the distance measured at the moment the
  robot starts moving. If the robot drifts away from the wall it
  steers right; if it drifts towards the wall it steers left.

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
float WALL_DETECT_DISTANCE = 50.0;

// Distance to hold from the wall (cm).
// 0 = automatic: hold whatever distance the robot starts at.
float FIXED_TARGET_DISTANCE = 0.0;

// Steering strength: PWM change per cm of distance error.
// Robot corrects too slowly  -> increase.
// Robot zig-zags             -> decrease.
float KP = 2.0;

// No correction while the error is smaller than this (cm).
float DEAD_BAND = 1.0;

// Largest PWM change the steering may apply.
// Kept small on purpose: the steering only nudges the right motor
// around its calibrated speed, so the robot can never spin in circles.
int MAX_CORRECTION = 35;

// Steering direction.
//  1 = normal
// -1 = reversed (use this if the robot steers AWAY from the
//      distance it should hold instead of back towards it)
int STEER_SIGN = 1;

// Smoothing of the distance readings (0.0 - 1.0).
// Lower = smoother but slower to react.
float SMOOTHING = 0.5;

// Readings in a row without a wall before stopping.
int MISS_LIMIT = 2;

// ==========================================
// MOTOR CALIBRATION (UNCHANGED)
// ==========================================

int LEFT_BASE_PWM = 0;
int RIGHT_BASE_PWM = 73;

// ==========================================
// STATE
// ==========================================

int missCount = 0;
bool moving = false;

float targetDistance = 0;
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
    // Starting to move: lock the distance to hold
    // ----------------------------------------

    if (!moving) {

      if (FIXED_TARGET_DISTANCE > 0) {
        targetDistance = FIXED_TARGET_DISTANCE;
      } else {
        targetDistance = distance;
      }

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
    // error > 0 : drifted AWAY from the wall
    //             -> right motor a little SLOWER -> steer RIGHT
    // error < 0 : drifted TOWARDS the wall
    //             -> right motor a little FASTER -> steer LEFT
    // ----------------------------------------

    error = smoothDistance - targetDistance;

    int correction = 0;

    if (abs(error) > DEAD_BAND) {
      correction = constrain(
        (int)(KP * error) * STEER_SIGN,
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
      stopMotors();
      moving = false;
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
  Serial.print(targetDistance);

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
