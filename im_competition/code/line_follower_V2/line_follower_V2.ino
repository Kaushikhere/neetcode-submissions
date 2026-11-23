/*
  ==========================================
       LINE FOLLOWING ROBOT  -  V2
       Raspberry Pi Pico + L298N
       6 Optocouplers
       2 IR line sensors + 2 wheel encoders
  ==========================================

  SAME AS line_follower_FINAL (same pins, same speed numbers, same
  encoder speed control with braking) PLUS:

  1. LINE LOST -> SEARCH
     If both IRs stay on white for longer than LOST_MS, the line has
     slipped outside the sensors (sharp curve, or too fast). The robot
     then turns towards the side where it last saw black, until an IR
     finds the line again. Before, it just drove straight off the line.

  2. TEST MODES (TEST_MODE at the top), so you can find a hardware
     problem without guessing:
       0 = normal line following
       1 = IR test   : motors off, prints what both IRs see
       2 = motor test: both wheels forward at straight speed, prints
                       want/got of both wheels (lift the wheels)

  3. Serial printing only when the USB cable is connected, so it can
     never slow the robot down on battery.

  HOW IT STEERS (unchanged)

  Both on white   -> straight       (STRAIGHT_... values)
  Left on black   -> turn RIGHT     (LEFT_BLACK_... values)
  Right on black  -> turn LEFT      (RIGHT_BLACK_... values)
  Both on black   -> crossing line, straight values
  Both white too long -> search towards the last black side

  PINS
    RIGHT wheel: ENA GP0, IN1 GP1, IN2 GP2
    LEFT  wheel: ENB GP5, IN3 GP3, IN4 GP4
    Line sensors: Left IR GP16, Right IR GP17
    Encoders:     Left C1 GP14, C2 GP15 / Right C1 GP12, C2 GP13

  Lower PWM = Faster, Higher PWM = Slower, 255 = Stop (inverted)
*/

// ---------------- TEST MODE ----------------
// 0 = follow the line, 1 = IR test (motors off), 2 = motor test
int TEST_MODE = 0;

// ##########################################################
//   SPEED SETTINGS  -  the only numbers you need to change
//
//   Same scale as your PWM:  0 = FULL speed,  255 = STOPPED
//   (the encoders then hold each wheel at that speed)
// ##########################################################

// 1. GOING STRAIGHT (both IR sensors on white)
int STRAIGHT_LEFT_PWM  = 0 + 20;
int STRAIGHT_RIGHT_PWM = 80 + 30;

// 2. LEFT IR SEES BLACK (robot turns RIGHT)
int LEFT_BLACK_LEFT_PWM  = 10;     // left wheel
int LEFT_BLACK_RIGHT_PWM = 250;    // right wheel

// 3. RIGHT IR SEES BLACK (robot turns LEFT)
int RIGHT_BLACK_LEFT_PWM  = 250;   // left wheel
int RIGHT_BLACK_RIGHT_PWM = 10;    // right wheel

// 4. LINE LOST (both white for longer than LOST_MS)
unsigned long LOST_MS = 400;       // 0 = never search
int SEARCH_FAST_PWM = 60;          // outer wheel while searching
int SEARCH_SLOW_PWM = 255;         // inner wheel while searching (255 = stopped)

// ##########################################################

// ==========================================
// MOTOR PINS
// ==========================================

#define ENA 0
#define IN1 1
#define IN2 2

#define IN3 3
#define IN4 4
#define ENB 5

// ==========================================
// LINE SENSOR PINS
// ==========================================

#define LEFT_IR  16
#define RIGHT_IR 17

// What the IR module outputs over the BLACK line.
// If TEST_MODE 1 shows "black" on white paper, change to LOW.
#define ON_BLACK HIGH

// ==========================================
// ENCODER PINS
// ==========================================

#define ENC_L_A 14
#define ENC_L_B 15
#define ENC_R_A 12
#define ENC_R_B 13

// -1 if that wheel counts DOWN while driving forward.
// Check with TEST_MODE 2: both "got" values must be positive.
int LEFT_SIGN = 1;
int RIGHT_SIGN = -1;

// Full-power speed of a wheel (counts/50ms)
float MAX_RATE = 70;

// ==========================================
// SPEED CONTROL
// ==========================================

unsigned long CONTROL_MS = 25;  // how often wheels are measured

float KP_SPEED = 0.012;   // extra power per count of speed error
float KI_SPEED = 0.002;   // slowly removes leftover speed error

// Brake a wheel when it is this many counts/50ms too fast.
float BRAKE_MARGIN = 10;

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

// Your PWM scale (0 = full, 255 = stop) -> wheel speed target
float pwmToRate(int pwm) {
  pwm = constrain(pwm, 0, 255);
  return (255 - pwm) / 255.0 * MAX_RATE;
}

int dutyToPwm(float duty) {
  // Inverted: 0 = full speed, 255 = off
  return (int)(255 - duty * 255);
}

void setTargets(int leftPwm, int rightPwm) {
  L.target = pwmToRate(leftPwm);
  R.target = pwmToRate(rightPwm);
}

// ==========================================
// MOTOR OUTPUT
// ==========================================

// LEFT wheel = motor on OUT3/OUT4 (ENB, IN3, IN4)
void driveLeft(Wheel &w) {
  if (w.brake) {
    // Both inputs the same + enable on = L298N brakes the motor
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, dutyToPwm(w.duty));
  }
}

// RIGHT wheel = motor on OUT1/OUT2 (ENA, IN1, IN2)
void driveRight(Wheel &w) {
  if (w.brake) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, dutyToPwm(w.duty));
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

  if (Serial) {
    Serial.println("==============================");
    Serial.println(" LINE FOLLOWER V2");
    Serial.print(" TEST_MODE = ");
    Serial.println(TEST_MODE);
    Serial.println("==============================");
  }
}

// ==========================================
// MAIN LOOP
// ==========================================

unsigned long lastControl = 0;
unsigned long lastPrint = 0;
unsigned long whiteSince = 0;   // when both IRs went white
int lastSide = 0;               // +1 left IR saw black last (line to the right)
                                // -1 right IR saw black last (line to the left)
const char* action = "STRAIGHT";

void loop() {

  bool leftBlack  = (digitalRead(LEFT_IR)  == ON_BLACK);
  bool rightBlack = (digitalRead(RIGHT_IR) == ON_BLACK);

  // ---------- TEST MODE 1: IRs only, motors off ----------
  if (TEST_MODE == 1) {
    stopMotors();
    if (millis() - lastPrint >= 200) {
      lastPrint = millis();
      Serial.print("IR TEST   Left IR (GP16): ");
      Serial.print(leftBlack ? "BLACK" : "white");
      Serial.print("   Right IR (GP17): ");
      Serial.println(rightBlack ? "BLACK" : "white");
    }
    return;
  }

  // ----------------------------------------
  // 1. READ LINE SENSORS -> SET WHEEL TARGETS
  // ----------------------------------------

  if (TEST_MODE == 2) {
    setTargets(STRAIGHT_LEFT_PWM, STRAIGHT_RIGHT_PWM);
    action = "MOTOR TEST";
  } else if (leftBlack && !rightBlack) {
    setTargets(LEFT_BLACK_LEFT_PWM, LEFT_BLACK_RIGHT_PWM);
    lastSide = +1;
    whiteSince = 0;
    action = "LEFT IR BLACK -> TURN RIGHT";
  } else if (rightBlack && !leftBlack) {
    setTargets(RIGHT_BLACK_LEFT_PWM, RIGHT_BLACK_RIGHT_PWM);
    lastSide = -1;
    whiteSince = 0;
    action = "RIGHT IR BLACK -> TURN LEFT";
  } else if (leftBlack && rightBlack) {
    // crossing line
    setTargets(STRAIGHT_LEFT_PWM, STRAIGHT_RIGHT_PWM);
    whiteSince = 0;
    action = "BOTH BLACK -> STRAIGHT";
  } else {
    // both white
    if (whiteSince == 0) whiteSince = millis();
    bool lost = LOST_MS > 0 && lastSide != 0 && millis() - whiteSince > LOST_MS;

    if (lost && lastSide > 0) {
      setTargets(SEARCH_FAST_PWM, SEARCH_SLOW_PWM);     // turn right
      action = "LOST -> SEARCH RIGHT";
    } else if (lost && lastSide < 0) {
      setTargets(SEARCH_SLOW_PWM, SEARCH_FAST_PWM);     // turn left
      action = "LOST -> SEARCH LEFT";
    } else {
      setTargets(STRAIGHT_LEFT_PWM, STRAIGHT_RIGHT_PWM);
      action = "STRAIGHT";
    }
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
  // 3. SERIAL MONITOR (only with the USB cable in)
  // ----------------------------------------

  if (Serial && millis() - lastPrint >= 200) {
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
