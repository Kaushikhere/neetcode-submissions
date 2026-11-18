/*
  ==========================================
       LINE FOLLOWING ROBOT
       Raspberry Pi Pico + L298N
       6 Optocouplers
       2 IR line sensors
  ==========================================

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

  Left IR  OUT = GP0
  Right IR OUT = GP1

  CALIBRATED RAW PWM (UNCHANGED):

  Left motor  = 0
  Right motor = 73

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

#define LEFT_IR  0
#define RIGHT_IR 1

// What the IR module outputs when it is over the BLACK line.
// Most TCRT5000 modules give HIGH on black (their LED goes off).
// If the Serial Monitor shows "black" on white paper, change to LOW.
#define ON_BLACK HIGH

// ==========================================
// SPEEDS (raw PWM, lower = faster, 255 = stop)
// ==========================================

// Straight: your calibration
int LEFT_BASE_PWM  = 0;
int RIGHT_BASE_PWM = 73;

// Turning: the wheel on the inside of the turn only slows down.
// Both values give about 37% of straight speed on each motor.
// Turns too gently (runs off curves) -> raise towards 255.
// Turns too sharply (wobbles)        -> lower towards the base value.
// Keep them below 255, or the wheel stops.
int LEFT_SLOW_PWM  = 160;   // left wheel while turning LEFT
int RIGHT_SLOW_PWM = 185;   // right wheel while turning RIGHT

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

  analogWriteRange(255);
  analogWriteFreq(500);

  stopMotors();

  delay(1000);

  Serial.println("==============================");
  Serial.println(" LINE FOLLOWING ROBOT");
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
  int rightPWM = RIGHT_BASE_PWM;

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

  } else {

    // BOTH ON WHITE -> line is between the sensors

    action = "STRAIGHT";
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
    Serial.println(rightPWM);
  }

  delay(5);
}
