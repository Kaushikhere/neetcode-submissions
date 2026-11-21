/*
  ==========================================
       LINE FOLLOWING ROBOT
       Raspberry Pi Pico + L298N
       6 Optocouplers
       2 IR line sensors + 2 wheel encoders
  ==========================================

  WHAT THE ENCODERS DO HERE

  On straight sections the code compares how far each wheel has
  turned and trims the right motor's PWM until both wheels turn at
  the same rate. So the robot drives straight by itself without a
  hand calibration.

  Run encoder_test first: both counts must go UP when driving
  forward (fix with LEFT_SIGN / RIGHT_SIGN below).

  HOW IT STEERS

  The two IR sensors sit on either side of the black line.

  Both on white   -> line is between them -> go straight
  Left on black   -> robot drifted right  -> turn LEFT
                     (left wheel slows down, it does NOT stop)
  Right on black  -> robot drifted left   -> turn RIGHT
                     (right wheel slows down, it does NOT stop)
  Both on black   -> crossing line        -> go straight

  MOTOR PINS (UNCHANGED, through the optocouplers):

  ENA = GP10
  IN1 = GP11
  IN2 = GP12

  IN3 = GP13
  IN4 = GP14
  ENB = GP15

  LINE SENSORS:

  Left IR  OUT = GP1
  Right IR OUT = GP0

  ENCODERS:

  Left  encoder C1 = GP20, C2 = GP21
  Right encoder C1 = GP18, C2 = GP19

  BASE PWM:

  Both motors = 30 (your encoder log showed both motors turn at the
  same speed for the same PWM, so no fixed offset is needed; the
  encoder trim handles the rest)

  Lower PWM = Faster
  Higher PWM = Slower
  255 = Stop
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
// LINE SENSOR PINS
// ==========================================

#define LEFT_IR  1
#define RIGHT_IR 0

// What the IR module outputs when it is over the BLACK line.
// Most TCRT5000 modules give HIGH on black (their LED goes off).
// If the Serial Monitor shows "black" on white paper, change to LOW.
#define ON_BLACK HIGH

// ==========================================
// ENCODER PINS
// ==========================================

// The encoder plugged into GP18/GP19 is on the RIGHT motor,
// and the one on GP20/GP21 is on the LEFT motor.
#define ENC_L_A 20   // left encoder C1
#define ENC_L_B 21   // left encoder C2
#define ENC_R_A 18   // right encoder C1
#define ENC_R_B 19   // right encoder C2

// -1 if that wheel counts down while driving forward.
// The right encoder counted down in your test, so it is -1.
int LEFT_SIGN = 1;
int RIGHT_SIGN = -1;

// How often the wheel speeds are compared (ms)
unsigned long SPEED_CHECK_MS = 50;

// How strongly the right motor is trimmed per count of difference.
// Robot still drifts on straights -> increase.
// Right motor speed hunts up and down -> decrease.
float TRIM_GAIN = 0.5;

// Largest trim allowed on the right motor (PWM steps)
int MAX_TRIM = 60;

// ==========================================
// SPEEDS (raw PWM, lower = faster, 255 = stop)
// ==========================================

// Straight: same value on both motors. Not 0, so the encoder trim
// has room to make the right motor faster as well as slower.
int LEFT_BASE_PWM  = 30;
int RIGHT_BASE_PWM = 30;

// Turning: the wheel on the inside of the turn only slows down.
// Same value on both, because both motors behave the same.
// Turns too gently (runs off curves) -> raise towards 255.
// Turns too sharply (wobbles)        -> lower towards the base value.
// Keep them below 255, or the wheel stops.
int LEFT_SLOW_PWM  = 170;   // left wheel while turning LEFT
int RIGHT_SLOW_PWM = 170;   // right wheel while turning RIGHT

// ==========================================
// ENCODERS
// ==========================================

volatile long rawLeft = 0;
volatile long rawRight = 0;

void onLeftA() {
  if (digitalRead(ENC_L_A) != digitalRead(ENC_L_B)) rawLeft++;
  else rawLeft--;
}

void onRightA() {
  if (digitalRead(ENC_R_A) != digitalRead(ENC_R_B)) rawRight++;
  else rawRight--;
}

long leftCount() {
  noInterrupts();
  long c = rawLeft;
  interrupts();
  return c * LEFT_SIGN;
}

long rightCount() {
  noInterrupts();
  long c = rawRight;
  interrupts();
  return c * RIGHT_SIGN;
}

// Correction added to RIGHT_BASE_PWM, learned on straights.
// Positive = right motor slowed down.
float rightTrim = 0;

long lastLeft = 0;
long lastRight = 0;
unsigned long lastSpeedCheck = 0;
long leftRate = 0;    // counts in the last SPEED_CHECK_MS
long rightRate = 0;

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

  // LEFT MOTOR - ORIGINAL DIRECTION
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // RIGHT MOTOR - ORIGINAL DIRECTION
  // Electrical direction is opposite because
  // the motors are mounted on opposite sides.
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, constrain(leftPWM, 0, 255));
  analogWrite(ENB, constrain(rightPWM, 0, 255));
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

  pinMode(LEFT_IR, INPUT);
  pinMode(RIGHT_IR, INPUT);

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), onLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), onRightA, CHANGE);

  analogWriteRange(255);
  analogWriteFreq(500);

  stopMotors();

  delay(1000);

  Serial.println("==============================");
  Serial.println(" LINE FOLLOWING + ENCODERS");
  Serial.println("==============================");
  Serial.println("Ready!");
}

// ==========================================
// MAIN LOOP
// ==========================================

unsigned long lastPrint = 0;

void loop() {

  // ----------------------------------------
  // 1. READ BOTH LINE SENSORS
  // ----------------------------------------

  bool leftBlack  = (digitalRead(LEFT_IR)  == ON_BLACK);
  bool rightBlack = (digitalRead(RIGHT_IR) == ON_BLACK);

  // ----------------------------------------
  // 2. START WITH CALIBRATED SPEEDS
  // ----------------------------------------

  int leftPWM  = LEFT_BASE_PWM;
  int rightPWM = RIGHT_BASE_PWM + (int)rightTrim;

  bool straight = false;

  const char* action;

  // ----------------------------------------
  // 3. STEER
  // ----------------------------------------

  if (leftBlack && !rightBlack) {

    // LINE UNDER LEFT SENSOR -> TURN LEFT
    // Left wheel slower, right wheel keeps its speed.

    leftPWM = LEFT_SLOW_PWM;
    action = "TURN LEFT";

  } else if (rightBlack && !leftBlack) {

    // LINE UNDER RIGHT SENSOR -> TURN RIGHT
    // Right wheel slower, left wheel keeps its speed.

    rightPWM = RIGHT_SLOW_PWM;
    action = "TURN RIGHT";

  } else if (leftBlack && rightBlack) {

    // BOTH ON BLACK -> crossing line, keep going

    action = "CROSSING - STRAIGHT";
    straight = true;

  } else {

    // BOTH ON WHITE -> line is between the sensors

    action = "STRAIGHT";
    straight = true;
  }

  // ----------------------------------------
  // 3b. ENCODERS: MATCH WHEEL SPEEDS ON STRAIGHTS
  // ----------------------------------------

  if (millis() - lastSpeedCheck >= SPEED_CHECK_MS) {

    lastSpeedCheck = millis();

    long l = leftCount();
    long r = rightCount();

    leftRate = l - lastLeft;
    rightRate = r - lastRight;

    lastLeft = l;
    lastRight = r;

    // Only learn while driving straight with both wheels moving.
    if (straight && leftRate > 0 && rightRate > 0) {

      // Right wheel faster than left -> slow it (raise its PWM).
      rightTrim += TRIM_GAIN * (rightRate - leftRate);
      rightTrim = constrain(rightTrim, (float)-MAX_TRIM, (float)MAX_TRIM);
    }
  }

  if (straight) {
    rightPWM = RIGHT_BASE_PWM + (int)rightTrim;
  }

  // ----------------------------------------
  // 4. MOVE MOTORS
  // ----------------------------------------

  moveForward(leftPWM, rightPWM);

  // ----------------------------------------
  // 5. SERIAL MONITOR (5 times a second, so
  //    printing does not slow the steering)
  // ----------------------------------------

  if (millis() - lastPrint >= 200) {

    lastPrint = millis();

    Serial.print("L: ");
    Serial.print(leftBlack ? "black" : "white");

    Serial.print(" | R: ");
    Serial.print(rightBlack ? "black" : "white");

    Serial.print(" | ");
    Serial.print(action);

    Serial.print(" | ENA: ");
    Serial.print(leftPWM);

    Serial.print(" | ENB: ");
    Serial.print(rightPWM);

    Serial.print(" | counts/50ms L: ");
    Serial.print(leftRate);

    Serial.print(" R: ");
    Serial.print(rightRate);

    Serial.print(" | trim: ");
    Serial.println((int)rightTrim);
  }

  delay(5);
}
