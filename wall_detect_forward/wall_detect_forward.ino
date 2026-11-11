/*
  ==========================================
       WALL DETECT -> MOVE FORWARD
       Raspberry Pi Pico + L298N
       6 Optocouplers (inverted PWM)
       Right-side HC-SR04
  ==========================================

  BEHAVIOUR:

  Wall detected on the right  -> move forward (straight)
  Wall NOT detected           -> stop immediately

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
float WALL_DETECT_DISTANCE = 30.0;

// Number of readings in a row without a wall before stopping.
// 1 = stop on the very first missed reading (fastest, but the robot
//     may stutter if the HC-SR04 drops a single echo).
// 2 = ignores one bad reading (still stops in well under 0.1 s).
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
  Serial.println(" WALL DETECT -> MOVE FORWARD");
  Serial.println("==============================");
  Serial.println("Ready!");
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop() {

  float distance = getDistance();

  bool wallDetected = (distance > 0 && distance <= WALL_DETECT_DISTANCE);

  if (wallDetected) {

    // Wall on the right -> go straight with calibrated speeds

    missCount = 0;

    moveForward(LEFT_BASE_PWM, RIGHT_BASE_PWM);
    moving = true;

  } else {

    // No wall -> stop as soon as the miss limit is reached

    if (missCount < MISS_LIMIT) {
      missCount++;
    }

    if (missCount >= MISS_LIMIT) {
      stopMotors();
      moving = false;
    }
  }

  Serial.print("Distance: ");

  if (distance < 0) {
    Serial.print("no echo");
  } else {
    Serial.print(distance);
    Serial.print(" cm");
  }

  Serial.print(" | Wall: ");
  Serial.print(wallDetected ? "YES" : "NO");

  Serial.print(" | Motors: ");
  Serial.println(moving ? "FORWARD" : "STOPPED");

  delay(30);
}
