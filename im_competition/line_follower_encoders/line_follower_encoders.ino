/*
  ==========================================
       LINE FOLLOWING ROBOT
       Raspberry Pi Pico + L298N
       6 Optocouplers
       2 IR line sensors + 2 wheel encoders
  ==========================================

  WHY THIS VERSION

  Lowering a motor's PWM does not stop it quickly: during the "off"
  part of each PWM cycle the motor coasts, and one of your motors
  coasts much longer than the other. So when the robot turned, one
  wheel slowed at once and the other kept spinning, and the robot
  went round in circles.

  Now each wheel has a target SPEED (encoder counts per 50 ms), not a
  fixed PWM. Every 25 ms the code measures each wheel:
    - too slow  -> more power
    - too fast  -> less power
    - much too fast -> BRAKE that wheel (L298N shorts the motor),
                       so it slows down as fast as the other one.

  HOW IT STEERS

  Both on white   -> straight: both wheels at STRAIGHT_RATE
  Left on black   -> turn LEFT: left wheel at TURN_INNER_RATE
  Right on black  -> turn RIGHT: right wheel at TURN_INNER_RATE
  Both on black   -> crossing line: straight

  PINS

  Motors (through optocouplers): ENA GP10, IN1 GP11, IN2 GP12,
                                 IN3 GP13, IN4 GP14, ENB GP15
  Line sensors: Left IR GP1, Right IR GP0
  Encoders:     Left C1 GP20, C2 GP21 / Right C1 GP18, C2 GP19

  Lower PWM = Faster, Higher PWM = Slower, 255 = Stop (inverted)
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

// What the IR module outputs over the BLACK line.
// If the Serial Monitor shows "black" on white paper, change to LOW.
#define ON_BLACK HIGH

// ==========================================
// ENCODER PINS
// ==========================================

// The encoder on GP18/GP19 is on the RIGHT motor,
// the one on GP20/GP21 is on the LEFT motor.
#define ENC_L_A 20
#define ENC_L_B 21
#define ENC_R_A 18
#define ENC_R_B 19

// -1 if that wheel counts DOWN while driving forward.
// Check in the Serial Monitor: both rates must be positive going forward.
int LEFT_SIGN = 1;
int RIGHT_SIGN = -1;

// ==========================================
// SPEEDS (encoder counts per 50 ms)
// ==========================================

// Your log showed about 60 counts/50ms at full power (wheels lifted).
float MAX_RATE = 60;

// Straight speed for both wheels. Lower if it overshoots curves.
float STRAIGHT_RATE = 40;

// Inner wheel speed during a turn (slowed, never stopped).
// Runs off curves -> lower it. Wobbles -> raise it.
float TURN_INNER_RATE = 12;

// ==========================================
// SPEED CONTROL
// ==========================================

unsigned long CONTROL_MS = 25;  // how often wheels are measured

float KP_SPEED = 0.012;   // extra power per count of speed error
float KI_SPEED = 0.002;   // slowly removes leftover speed error

// Brake a wheel when it is this many counts/50ms too fast.
float BRAKE_MARGIN = 6;

// ==========================================
// WHEEL STATE
// (defined before any function so the Arduino IDE
//  can use it in the functions below)
// ==========================================

struct Wheel {
  float target = 0;     // wanted counts/50ms
  float rate = 0;       // measured counts/50ms
  float integral = 0;
  float duty = 0;       // 0 = off, 1 = full power
  bool brake = false;
  long last = 0;
};

Wheel L, R;

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

// ==========================================
// ONE WHEEL'S SPEED CONTROLLER
// ==========================================

void updateWheel(Wheel &w, long count) {

  long delta = count - w.last;
  w.last = count;
  w.rate = delta * (50.0 / CONTROL_MS);

  float error = w.target - w.rate;

  // Much too fast (for example just after a turn starts): brake.
  if (w.rate > w.target + BRAKE_MARGIN) {
    w.brake = true;
    w.integral = 0;
    w.duty = w.target / MAX_RATE;
    return;
  }

  w.brake = false;

  w.integral += error;
  w.integral = constrain(w.integral, -150.0f, 150.0f);

  w.duty = w.target / MAX_RATE + KP_SPEED * error + KI_SPEED * w.integral;
  w.duty = constrain(w.duty, 0.0f, 1.0f);
}

int dutyToPwm(float duty) {
  // Inverted: 0 = full speed, 255 = off
  return (int)(255 - duty * 255);
}

// ==========================================
// MOTOR OUTPUT
// ==========================================

void driveLeft(Wheel &w) {
  if (w.brake) {
    // Both inputs the same + enable on = L298N brakes the motor
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  } else {
    // LEFT MOTOR - ORIGINAL DIRECTION
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, dutyToPwm(w.duty));
  }
}

void driveRight(Wheel &w) {
  if (w.brake) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  } else {
    // RIGHT MOTOR - ORIGINAL DIRECTION (mounted opposite)
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, dutyToPwm(w.duty));
  }
}

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

  L.last = leftCount();
  R.last = rightCount();

  Serial.println("==============================");
  Serial.println(" LINE FOLLOWING + SPEED CONTROL");
  Serial.println("==============================");
  Serial.println("Ready!");
}

// ==========================================
// MAIN LOOP
// ==========================================

unsigned long lastControl = 0;
unsigned long lastPrint = 0;
const char* action = "STRAIGHT";

void loop() {

  // ----------------------------------------
  // 1. READ LINE SENSORS -> SET WHEEL TARGETS
  // ----------------------------------------

  bool leftBlack  = (digitalRead(LEFT_IR)  == ON_BLACK);
  bool rightBlack = (digitalRead(RIGHT_IR) == ON_BLACK);

  if (leftBlack && !rightBlack) {
    L.target = TURN_INNER_RATE;
    R.target = STRAIGHT_RATE;
    action = "TURN LEFT";
  } else if (rightBlack && !leftBlack) {
    L.target = STRAIGHT_RATE;
    R.target = TURN_INNER_RATE;
    action = "TURN RIGHT";
  } else if (leftBlack && rightBlack) {
    L.target = STRAIGHT_RATE;
    R.target = STRAIGHT_RATE;
    action = "CROSSING";
  } else {
    L.target = STRAIGHT_RATE;
    R.target = STRAIGHT_RATE;
    action = "STRAIGHT";
  }

  // ----------------------------------------
  // 2. MEASURE BOTH WHEELS AND SET THEIR POWER
  // ----------------------------------------

  if (millis() - lastControl >= CONTROL_MS) {
    lastControl = millis();

    updateWheel(L, leftCount());
    updateWheel(R, rightCount());

    driveLeft(L);
    driveRight(R);
  }

  // ----------------------------------------
  // 3. SERIAL MONITOR
  // ----------------------------------------

  if (millis() - lastPrint >= 200) {
    lastPrint = millis();

    Serial.print("L: ");
    Serial.print(leftBlack ? "black" : "white");
    Serial.print(" | R: ");
    Serial.print(rightBlack ? "black" : "white");
    Serial.print(" | ");
    Serial.print(action);

    Serial.print(" | Left want/got: ");
    Serial.print((int)L.target);
    Serial.print("/");
    Serial.print((int)L.rate);
    Serial.print(L.brake ? " BRAKE" : "");

    Serial.print(" | Right want/got: ");
    Serial.print((int)R.target);
    Serial.print("/");
    Serial.print((int)R.rate);
    Serial.println(R.brake ? " BRAKE" : "");
  }
}
