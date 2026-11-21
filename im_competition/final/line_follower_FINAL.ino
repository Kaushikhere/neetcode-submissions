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

  Both on white   -> straight       (STRAIGHT_... values)
  Left on black   -> turn RIGHT     (LEFT_BLACK_... values)
  Right on black  -> turn LEFT      (RIGHT_BLACK_... values)
  Both on black   -> crossing line, straight values

  All speeds are in the SPEED SETTINGS block just below.

  PINS

  MOTORS SWAPPED SIDES: the motor on L298N OUT1/OUT2 (ENA) is now
  the RIGHT wheel, and the motor on OUT3/OUT4 (ENB) is now the LEFT
  wheel. Each motor kept its own wires and encoder.

  Motors (through optocouplers):
    RIGHT wheel: ENA GP10, IN1 GP11, IN2 GP12
    LEFT  wheel: ENB GP15, IN3 GP13, IN4 GP14
  Line sensors: Left IR GP1, Right IR GP0
  Encoders:     Left C1 GP18, C2 GP19 / Right C1 GP4, C2 GP5

  Lower PWM = Faster, Higher PWM = Slower, 255 = Stop (inverted)
*/

// ##########################################################
//   SPEED SETTINGS  -  the only numbers you need to change
//
//   Same scale as your PWM:  0 = FULL speed,  255 = STOPPED
//   (the encoders then hold each wheel at that speed)
// ##########################################################

// 1. GOING STRAIGHT (both IR sensors on white)
int STRAIGHT_LEFT_PWM  = 73;
int STRAIGHT_RIGHT_PWM = 0;

// 2. LEFT IR SEES BLACK (robot turns RIGHT)
int LEFT_BLACK_LEFT_PWM  = 0;     // left wheel
int LEFT_BLACK_RIGHT_PWM = 200;   // right wheel

// 3. RIGHT IR SEES BLACK (robot turns LEFT)
int RIGHT_BLACK_LEFT_PWM  = 200;  // left wheel
int RIGHT_BLACK_RIGHT_PWM = 0;    // right wheel

// ##########################################################

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

// Motors swapped sides, and each encoder moved with its motor:
// GP18/GP19 encoder is now on the LEFT wheel,
// GP4/GP5 encoder is now on the RIGHT wheel.
#define ENC_L_A 18
#define ENC_L_B 19
#define ENC_R_A 4
#define ENC_R_B 5

// -1 if that wheel counts DOWN while driving forward.
// A motor on the other side turns the other way when going forward,
// so both signs flipped compared with before the swap.
// Check in the Serial Monitor: both "got" values must be positive
// going forward.
int LEFT_SIGN = 1;
int RIGHT_SIGN = -1;

// Full-power speed of a wheel (counts/50ms), used internally as the
// starting guess for how much power a speed needs.
float MAX_RATE = 60;

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

// Your PWM scale (0 = full, 255 = stop) -> wheel speed target
float pwmToRate(int pwm) {
  pwm = constrain(pwm, 0, 255);
  return (255 - pwm) / 255.0 * MAX_RATE;
}

int dutyToPwm(float duty) {
  // Inverted: 0 = full speed, 255 = off
  return (int)(255 - duty * 255);
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
    // Forward on the left side (reversed from when this motor
    // was on the right)
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
    // Forward on the right side (reversed from when this motor
    // was on the left)
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
    L.target = pwmToRate(LEFT_BLACK_LEFT_PWM);
    R.target = pwmToRate(LEFT_BLACK_RIGHT_PWM);
    action = "LEFT IR BLACK -> TURN RIGHT";
  } else if (rightBlack && !leftBlack) {
    L.target = pwmToRate(RIGHT_BLACK_LEFT_PWM);
    R.target = pwmToRate(RIGHT_BLACK_RIGHT_PWM);
    action = "RIGHT IR BLACK -> TURN LEFT";
  } else {
    // both white, or both black (crossing line)
    L.target = pwmToRate(STRAIGHT_LEFT_PWM);
    R.target = pwmToRate(STRAIGHT_RIGHT_PWM);
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
