/*
  ==========================================
       IM COMPETITION 1 - FULL RUN
       Raspberry Pi Pico + L298N + 6 optocouplers
       2 encoder motors, 2 IR line sensors,
       right HC-SR04, servo for the drop
  ==========================================

  WHAT IT DOES (from any start position and direction)

   1. FIND POSITION  a) Spins once on the spot, reading the right
                        ultrasonic. The three start walls show up as
                        close readings, the open side (up the field)
                        as far ones -> rough x, y and heading.
                     b) Turns so the sensor faces the back wall and
                        scans it slowly -> exact heading and y.
                     c) Turns so the sensor faces the NEARER side wall
                        and reads it standing still -> exact x.
                     Works next to the left wall, the right wall or in
                     the middle, facing any direction.
   2. GO TO T1       Turns to face T1 and drives straight to it with
                     the encoders. When an IR crosses the black loop
                     around T1, it corrects its position from the map.
   3. DROP           Servo opens, the item falls on T1.
   4. TO THE LINE    Faces right, drives until an IR sees the loop.
   5. FOLLOW LINE    Your line follower (same speeds as your final
                     code) up to the end of the line.
   6. GO TO T2       Resets its position at the line end (known on the
                     map), drives into the pocket and turns so the
                     pointer ends on T2.
   7. STOP           Motors off. Everything stops at 88 s anyway.

  FIRST: set RUN_MODE below to the calibration modes, one by one,
  and fill in COUNTS_PER_CM and WHEEL_BASE_CM. The whole run depends
  on those two numbers.

  BOARD COORDINATES (cm), from the field map
    origin = start-end LEFT corner of the board,
    x = to the right (0..122), y = up the field (0..244)
    heading: 0 = facing right, 90 = facing up the field

  PINS
    Left wheel  : ENB GP15, IN3 GP13, IN4 GP14, encoder GP18/GP19
    Right wheel : ENA GP10, IN1 GP11, IN2 GP12, encoder GP4/GP5
    Left IR GP1, Right IR GP0
    Right HC-SR04: TRIG GP16, ECHO GP17
    Drop servo   : GP2
*/

#include <Servo.h>

// ##########################################################
//   RUN MODE
// ##########################################################
//   0 = full competition run
//   1 = COUNT TEST: push the robot by hand exactly 100 cm along a
//       ruler; the Serial Monitor shows counts -> COUNTS_PER_CM
//   2 = DRIVE TEST: robot drives 100 cm. Measure it; if it went
//       102 cm, multiply COUNTS_PER_CM by 1.02
//   3 = TURN TEST: robot spins 360 degrees. Turned too far ->
//       raise WHEEL_BASE_CM; not far enough -> lower it
//   4 = POSITION TEST: does step 1 only and prints x, y, heading.
//       Put the robot on a known spot and compare.
//   5 = T1 TEST: steps 1-3 only (find position, go to T1, drop)
int RUN_MODE = 4;

// ##########################################################
//   CALIBRATION  (measure these on the real board)
// ##########################################################

float COUNTS_PER_CM = 12.0;    // from RUN_MODE 1 and 2
float WHEEL_BASE_CM = 15.0;    // centre of left wheel to centre of
                               // right wheel, then tune with mode 3

// Where the sensors are, measured from the robot's turning point
// (the middle between the two wheels):
float US_SIDE_CM   = 10.0;  // ultrasonic face, to the right of centre
float IR_FORWARD_CM = 8.0;  // IR sensors, in front of centre
float IR_SIDE_CM    = 2.5;  // each IR, left/right of centre

// How far the pointer reaches in front of the turning point
float POINTER_REACH_CM = 14.0;

// Drop servo angles
int SERVO_CLOSED = 0;
int SERVO_OPEN   = 90;

// ##########################################################
//   FIELD MAP (cm)  - tape-measure and correct these
// ##########################################################

float WALL_LEFT_X  = 0;     // inner face of the left start wall
float WALL_RIGHT_X = 122;   // inner face of the right start wall
float WALL_BACK_Y  = 0;     // inner face of the back start wall

float T1_X = 58.4, T1_Y = 97.0;     // target 1 (green ring)
float T2_X = 51.1, T2_Y = 179.5;    // target 2 (estimate)
float POCKET_HEADING = -62.7;       // facing into the T2 pocket

float LINE_END_X = 50.5, LINE_END_Y = 202.0;  // end of the black line
float LINE_END_HEADING = -22.0;               // last part of the line

// Distances along the line from where the robot joins it (cm)
float FORK_AT_CM     = 125;   // fork just after the narrow passage
float LINE_LENGTH_CM = 228;   // join point to line end

// Black loop around T1 (for the position correction)
const int LOOP_N = 15;
float LOOP_X[LOOP_N] = { 94.0, 65.2, 42.1, 19.0, 16.2, 20.6, 28.4, 38.7, 52.1, 65.0, 77.8, 85.9, 91.9, 98.9, 102.9 };
float LOOP_Y[LOOP_N] = { 114.8, 123.3, 125.9, 122.8, 108.3, 91.2, 78.6, 68.9, 63.7, 63.4, 66.4, 73.4, 83.7, 101.2, 118.7 };

// ##########################################################
//   SPEEDS  (encoder counts per 50 ms, about 60 = full speed)
// ##########################################################

float DRIVE_RATE = 25;   // driving to a point
float SLOW_RATE  = 10;   // last 15 cm before a target
float TURN_RATE  = 10;   // spinning on the spot
float SWEEP_RATE = 5;    // slow spin while finding position
float SEARCH_RATE = 10;  // driving towards the line

// Line following: YOUR final values (0 = full, 255 = stop)
int STRAIGHT_LEFT_PWM  = 0 + 30;
int STRAIGHT_RIGHT_PWM = 70 + 30;
int LEFT_BLACK_LEFT_PWM  = 10;    // left IR black -> turn right
int LEFT_BLACK_RIGHT_PWM = 250;
int RIGHT_BLACK_LEFT_PWM  = 250;  // right IR black -> turn left
int RIGHT_BLACK_RIGHT_PWM = 10;

// At the fork, ignore this sensor so the robot keeps to the main
// line. 0 = ignore RIGHT IR, 1 = ignore LEFT IR. If the robot takes
// the dead-end branch, change it.
int FORK_IGNORE = 0;

unsigned long RUN_TIME_LIMIT_MS = 88000;   // rules: off by 90 s

// ##########################################################
//   PINS
// ##########################################################

#define ENA 10
#define IN1 11
#define IN2 12
#define IN3 13
#define IN4 14
#define ENB 15

#define LEFT_IR  1
#define RIGHT_IR 0
#define ON_BLACK HIGH

#define ENC_L_A 18
#define ENC_L_B 19
#define ENC_R_A 4
#define ENC_R_B 5
int LEFT_SIGN = 1;
int RIGHT_SIGN = -1;

#define TRIG_PIN 16
#define ECHO_PIN 17

#define SERVO_PIN 2

// ##########################################################
//   INTERNAL SETTINGS
// ##########################################################

float MAX_RATE = 60;
unsigned long CONTROL_MS = 25;
float KP_SPEED = 0.012;
float KI_SPEED = 0.002;
float BRAKE_MARGIN = 6;
float KP_HEADING = 25;    // steering per radian of heading error

// ==========================================
// TYPES (before any function, for the Arduino IDE)
// ==========================================

struct Wheel {
  float target = 0;     // counts/50ms, negative = backwards
  float rate = 0;       // measured counts/50ms
  float integral = 0;
  float duty = 0;
  int dir = 1;
  bool brake = true;
  long last = 0;
};

enum Step {
  S_SWEEP, S_ALIGN_BACK, S_SCAN_BACK, S_FACE_SIDE, S_READ_SIDE, S_FACE_T1, S_GO_T1, S_DROP, S_FACE_LINE, S_FIND_LINE,
  S_FOLLOW, S_FACE_T2, S_GO_T2, S_FINAL_TURN, S_DONE
};

Wheel L, R;
Servo dropServo;

// robot pose in board coordinates
float poseX = 0, poseY = 0, poseTh = 0;   // th in radians
long lastOdoL = 0, lastOdoR = 0;

// sweep data: closest reading per 5 degree bin of SENSOR direction
const int BINS = 72;
float sweepMin[BINS];

Step step = S_SWEEP;
unsigned long runStart = 0;
unsigned long lastControl = 0;
unsigned long lastPrint = 0;

float sweepStartTh = 0;

// fine scan of the back wall
const int SCAN_MAX = 200;
float scanTh[SCAN_MAX];
float scanD[SCAN_MAX];
int scanN = 0;
float SCAN_HALF_DEG = 30;     // scan back wall from -30 to +30 degrees

// side wall reading
bool useRightWall = true;
float sideReadings[9];
int sideN = 0;
bool loopFixed = false;
bool prevAnyBlack = false;
float followStartDist = 0;
float whiteSinceDist = -1;
float travelled = 0;          // total distance driven (cm)
float goalX = 0, goalY = 0, goalTh = 0;

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
// SMALL HELPERS
// ==========================================

float wrapPi(float a) {
  while (a > PI) a -= 2 * PI;
  while (a < -PI) a += 2 * PI;
  return a;
}

float deg2rad(float d) { return d * PI / 180.0; }
float rad2deg(float r) { return r * 180.0 / PI; }

float pwmToRate(int pwm) {
  pwm = constrain(pwm, 0, 255);
  return (255 - pwm) / 255.0 * MAX_RATE;
}

bool leftBlack()  { return digitalRead(LEFT_IR)  == ON_BLACK; }
bool rightBlack() { return digitalRead(RIGHT_IR) == ON_BLACK; }

// Right ultrasonic, cm. -1 = no echo (nothing within ~2.5 m)
float readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long us = pulseIn(ECHO_PIN, HIGH, 15000);
  if (us == 0) return -1;
  return us * 0.0343 / 2.0;
}

// ==========================================
// WHEEL SPEED CONTROL (with braking, both directions)
// ==========================================

void updateWheel(Wheel &w, long count) {
  long delta = count - w.last;
  w.last = count;
  w.rate = delta * (50.0 / CONTROL_MS);

  if (w.target == 0) {           // stop and hold
    w.brake = true;
    w.integral = 0;
    w.duty = 0;
    return;
  }

  int want = (w.target > 0) ? 1 : -1;
  float along = w.rate * want;   // speed in the wanted direction
  float mag = fabs(w.target);

  // too fast, or still rolling the wrong way: brake first
  if (along > mag + BRAKE_MARGIN || along < -BRAKE_MARGIN) {
    w.brake = true;
    w.integral = 0;
    w.duty = 0;
    return;
  }

  w.brake = false;
  w.dir = want;
  float err = mag - along;
  w.integral = constrain(w.integral + err, -150.0f, 150.0f);
  w.duty = constrain(mag / MAX_RATE + KP_SPEED * err + KI_SPEED * w.integral, 0.0f, 1.0f);
}

int dutyToPwm(float duty) {
  return (int)(255 - duty * 255);   // inverted: 0 = full, 255 = off
}

// LEFT wheel = OUT3/OUT4 motor (ENB, IN3, IN4)
void outputLeft(Wheel &w) {
  if (w.brake) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  } else if (w.dir > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, dutyToPwm(w.duty));
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, dutyToPwm(w.duty));
  }
}

// RIGHT wheel = OUT1/OUT2 motor (ENA, IN1, IN2)
void outputRight(Wheel &w) {
  if (w.brake) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  } else if (w.dir > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, dutyToPwm(w.duty));
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, dutyToPwm(w.duty));
  }
}

void motorsOff() {
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void setWheels(float leftRate, float rightRate) {
  L.target = leftRate;
  R.target = rightRate;
}

// ==========================================
// ODOMETRY: where the robot is, from the wheel counts
// ==========================================

void updatePose(long cl, long cr) {
  float dL = (cl - lastOdoL) / COUNTS_PER_CM;
  float dR = (cr - lastOdoR) / COUNTS_PER_CM;
  lastOdoL = cl;
  lastOdoR = cr;

  float d = (dL + dR) / 2.0;
  float dTh = (dR - dL) / WHEEL_BASE_CM;

  poseX += d * cos(poseTh + dTh / 2);
  poseY += d * sin(poseTh + dTh / 2);
  poseTh = wrapPi(poseTh + dTh);
  travelled += fabs(d);
}

// Runs every CONTROL_MS: measure, update pose, drive the motors
void controlTick() {
  long cl = leftCount();
  long cr = rightCount();
  updatePose(cl, cr);
  updateWheel(L, cl);
  updateWheel(R, cr);
  outputLeft(L);
  outputRight(R);
}

// ==========================================
// MOVES (each returns true when finished)
// ==========================================

// Spin on the spot to face a board heading (degrees)
bool faceHeading(float headingDeg) {
  float err = wrapPi(deg2rad(headingDeg) - poseTh);
  if (fabs(err) < deg2rad(2)) {
    setWheels(0, 0);
    return true;
  }
  float r = (fabs(err) < deg2rad(15)) ? TURN_RATE * 0.5 : TURN_RATE;
  if (err > 0) setWheels(-r, r);    // turn left
  else         setWheels(r, -r);    // turn right
  return false;
}

// Drive to a point, steering with the heading. Returns true there.
bool driveTo(float tx, float ty) {
  float dx = tx - poseX, dy = ty - poseY;
  float dist = sqrt(dx * dx + dy * dy);

  // arrived, or just passed it (target now slightly behind)
  float ahead = dx * cos(poseTh) + dy * sin(poseTh);
  if (dist < 0.8 || (dist < 3 && ahead < 0)) {
    setWheels(0, 0);
    return true;
  }

  float err = wrapPi(atan2(dy, dx) - poseTh);
  if (fabs(err) > deg2rad(30)) {       // way off: turn first
    float r = TURN_RATE;
    if (err > 0) setWheels(-r, r); else setWheels(r, -r);
    return false;
  }

  float base = (dist > 15) ? DRIVE_RATE : max(SLOW_RATE * 0.5f, SLOW_RATE * dist / 15.0f + 3);
  float steer = constrain(KP_HEADING * err, -base, base);
  setWheels(base - steer, base + steer);
  return false;
}

// ==========================================
// STEP 1: FIND POSITION FROM THE START WALLS
// ==========================================

// closest reading within +-10 degrees of a sensor direction
float windowMin(float dirDeg) {
  int c = (int)lround(dirDeg / 5.0);
  float m = 999;
  for (int k = -2; k <= 2; k++) {
    int b = ((c + k) % BINS + BINS) % BINS;
    if (sweepMin[b] < m) m = sweepMin[b];
  }
  return m;
}

void recordSweepReading() {
  float d = readUltrasonic();
  if (d < 0) return;
  // ultrasonic points to the robot's RIGHT
  float dirDeg = rad2deg(poseTh) - 90.0;
  int b = (((int)lround(dirDeg / 5.0)) % BINS + BINS) % BINS;
  if (d < sweepMin[b]) sweepMin[b] = d;
}

void solvePosition() {
  // 1. nearest wall direction
  int best = 0;
  for (int b = 1; b < BINS; b++) if (sweepMin[b] < sweepMin[best]) best = b;
  float a1 = best * 5.0;

  // 2. the four directions 90 degrees apart; the OPEN one
  //    (farthest / no echo) is up the field
  int upK = 0;
  float upD = -1;
  for (int k = 0; k < 4; k++) {
    float d = windowMin(a1 + 90.0 * k);
    if (d > upD) { upD = d; upK = k; }
  }
  float up = a1 + 90.0 * upK;          // sensor direction = up field

  float dBack  = windowMin(up + 180);
  float dLeft  = windowMin(up + 90);
  float dRight = windowMin(up - 90);

  // 3. position. Readings over 120 cm are not start walls.
  float y = (dBack < 120) ? WALL_BACK_Y + dBack + US_SIDE_CM : 25;

  bool okL = dLeft < 120, okR = dRight < 120;
  float xL = WALL_LEFT_X + dLeft + US_SIDE_CM;
  float xR = WALL_RIGHT_X - dRight - US_SIDE_CM;
  float x;
  if (okL && okR && fabs(xL - xR) < 15) x = (xL + xR) / 2;
  else if (okL && (!okR || dLeft < dRight)) x = xL;
  else if (okR) x = xR;
  else x = 61;

  // 4. heading: the 'up' sensor direction is board heading 90 deg
  //    for the sensor, so the robot heading shifts by (90 - up) deg
  poseTh = wrapPi(poseTh + deg2rad(90.0 - up));
  poseX = x;
  poseY = y;

  Serial.println("------ ROUGH POSITION ------");
  Serial.print("walls  back: "); Serial.print(dBack);
  Serial.print("  left: ");      Serial.print(dLeft);
  Serial.print("  right: ");     Serial.println(dRight);
  Serial.print("x = "); Serial.print(poseX);
  Serial.print("  y = "); Serial.print(poseY);
  Serial.print("  heading = "); Serial.println(rad2deg(poseTh));
}

// Fine scan of the back wall: the readings form a V around the
// direction where the sensor is square to the wall. The middle of
// the V's two sides gives the exact heading; the bottom gives y.
void solveBackWall() {
  if (scanN < 10) { Serial.println("BACK SCAN: too few readings, keeping rough heading"); return; }

  float m = 999;
  for (int i = 0; i < scanN; i++) if (scanD[i] < m) m = scanD[i];
  float thr = m + max(1.5f, 0.05f * m);

  int i1 = -1, i2 = -1;
  for (int i = 0; i < scanN; i++) if (scanD[i] <= thr) { i1 = i; break; }
  for (int i = scanN - 1; i >= 0; i--) if (scanD[i] <= thr) { i2 = i; break; }

  // both sides of the V must be inside the scan, otherwise another
  // wall got in the way: keep the rough heading
  if (i1 <= 0 || i2 >= scanN - 1 || i2 <= i1) {
    Serial.println("BACK SCAN: V not clean, keeping rough heading");
    return;
  }

  float squareTh = (scanTh[i1] + scanTh[i2]) / 2.0;   // robot heading when square

  // the rough heading is good to about 10 degrees; a bigger
  // correction means a side wall spoiled the scan
  if (fabs(squareTh) > deg2rad(12)) {
    Serial.println("BACK SCAN: correction too big, keeping rough heading");
    return;
  }

  // y = closest readings around the square direction
  float sum = 0; int n = 0;
  for (int i = 0; i < scanN; i++) {
    if (fabs(wrapPi(scanTh[i] - squareTh)) < deg2rad(4)) { sum += scanD[i]; n++; }
  }
  float dBack = (n > 0) ? sum / n : m;

  // when square to the back wall the robot truly faces heading 0
  poseTh = wrapPi(poseTh - squareTh);
  poseY = WALL_BACK_Y + dBack + US_SIDE_CM;

  Serial.print("BACK SCAN: heading corrected by ");
  Serial.print(rad2deg(-squareTh));
  Serial.print(" deg, y = ");
  Serial.println(poseY);
}

// ==========================================
// STEP 2 helper: correct position when crossing the T1 loop
// ==========================================

void loopCorrection(bool lBlack, bool rBlack) {
  // where the sensor that saw black should be
  float side = lBlack ? IR_SIDE_CM : -IR_SIDE_CM;
  float sx = poseX + IR_FORWARD_CM * cos(poseTh) - side * sin(poseTh);
  float sy = poseY + IR_FORWARD_CM * sin(poseTh) + side * cos(poseTh);

  // nearest point on the loop
  float bestD = 1e9, bx = sx, by = sy;
  for (int i = 0; i < LOOP_N - 1; i++) {
    float ax = LOOP_X[i], ay = LOOP_Y[i];
    float vx = LOOP_X[i + 1] - ax, vy = LOOP_Y[i + 1] - ay;
    float t = ((sx - ax) * vx + (sy - ay) * vy) / (vx * vx + vy * vy);
    t = constrain(t, 0.0f, 1.0f);
    float px = ax + t * vx, py = ay + t * vy;
    float d = (px - sx) * (px - sx) + (py - sy) * (py - sy);
    if (d < bestD) { bestD = d; bx = px; by = py; }
  }

  // only trust it if it is a small correction
  if (sqrt(bestD) < 15) {
    poseX += bx - sx;
    poseY += by - sy;
    Serial.print("LOOP CORRECTION: ");
    Serial.print(sqrt(bestD));
    Serial.println(" cm");
  }
}

// ==========================================
// STEP 5 helper: line following with your settings
// ==========================================

void followLine(bool lBlack, bool rBlack, float alongLine) {
  // at the fork, ignore one sensor so the robot stays on the line
  if (alongLine > FORK_AT_CM - 10 && alongLine < FORK_AT_CM + 20) {
    if (FORK_IGNORE == 0) rBlack = false;
    else lBlack = false;
  }

  int lp, rp;
  if (lBlack && !rBlack)      { lp = LEFT_BLACK_LEFT_PWM;  rp = LEFT_BLACK_RIGHT_PWM; }
  else if (rBlack && !lBlack) { lp = RIGHT_BLACK_LEFT_PWM; rp = RIGHT_BLACK_RIGHT_PWM; }
  else                        { lp = STRAIGHT_LEFT_PWM;    rp = STRAIGHT_RIGHT_PWM; }

  setWheels(pwmToRate(lp), pwmToRate(rp));
}

// ==========================================
// SETUP
// ==========================================

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  analogWriteRange(255);
  analogWriteFreq(500);
  motorsOff();

  pinMode(LEFT_IR, INPUT);
  pinMode(RIGHT_IR, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), onLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), onRightA, CHANGE);

  dropServo.attach(SERVO_PIN);
  dropServo.write(SERVO_CLOSED);

  for (int b = 0; b < BINS; b++) sweepMin[b] = 999;

  delay(2000);   // time to take your hand away

  L.last = lastOdoL = leftCount();
  R.last = lastOdoR = rightCount();

  Serial.print("RUN_MODE ");
  Serial.println(RUN_MODE);

  // mode-specific start
  if (RUN_MODE == 2) { poseX = 0; poseY = 0; poseTh = 0; goalX = 100; goalY = 0; }
  if (RUN_MODE == 3) { sweepStartTh = poseTh; }

  runStart = millis();
  lastControl = millis();
}

// ==========================================
// MAIN LOOP
// ==========================================

float turnProgress = 0;
float lastTh = 0;

void loop() {
  unsigned long now = millis();

  // ---------- mode 1: just print counts while pushed by hand
  if (RUN_MODE == 1) {
    if (now - lastPrint >= 300) {
      lastPrint = now;
      Serial.print("left counts: "); Serial.print(leftCount());
      Serial.print("   right counts: "); Serial.print(rightCount());
      Serial.print("   -> COUNTS_PER_CM after 100 cm = average / 100");
      Serial.println();
    }
    return;
  }

  // ---------- time limit (rules: robot shuts down by 90 s)
  if (now - runStart > RUN_TIME_LIMIT_MS) {
    motorsOff();
    if (step != S_DONE) Serial.println("TIME LIMIT - STOPPED");
    step = S_DONE;
    while (true) { delay(1000); }
  }

  if (now - lastControl < CONTROL_MS) return;
  lastControl = now;

  bool lB = leftBlack(), rB = rightBlack();

  // ---------- mode 2: drive 100 cm
  if (RUN_MODE == 2) {
    if (driveTo(goalX, goalY)) { controlTick(); motorsOff(); Serial.println("Drove 100 cm (by count). Measure it."); while (true) delay(1000); }
    controlTick();
    return;
  }

  // ---------- mode 3: spin 360 degrees
  if (RUN_MODE == 3) {
    turnProgress += fabs(wrapPi(poseTh - lastTh));
    lastTh = poseTh;
    if (turnProgress >= 2 * PI) { setWheels(0, 0); controlTick(); motorsOff(); Serial.println("Turned 360 (by count). Check it."); while (true) delay(1000); }
    setWheels(-TURN_RATE * 0.6, TURN_RATE * 0.6);
    controlTick();
    return;
  }

  // ---------- the run
  switch (step) {

    case S_SWEEP: {
      // spin slowly to the left once, recording the right ultrasonic
      turnProgress += fabs(wrapPi(poseTh - lastTh));
      lastTh = poseTh;
      recordSweepReading();
      if (turnProgress >= 2 * PI + deg2rad(20)) {
        setWheels(0, 0);
        solvePosition();
        step = S_ALIGN_BACK;
      } else {
        setWheels(-SWEEP_RATE, SWEEP_RATE);
      }
      break;
    }

    case S_ALIGN_BACK:
      // sensor on the right faces the back wall when heading = 0
      if (faceHeading(-SCAN_HALF_DEG)) { scanN = 0; step = S_SCAN_BACK; }
      break;

    case S_SCAN_BACK: {
      float d = readUltrasonic();
      if (d > 0 && scanN < SCAN_MAX) { scanTh[scanN] = poseTh; scanD[scanN] = d; scanN++; }
      if (poseTh >= deg2rad(SCAN_HALF_DEG)) {
        setWheels(0, 0);
        solveBackWall();
        useRightWall = poseX > 61;           // read the nearer side wall
        step = S_FACE_SIDE;
      } else {
        setWheels(-SWEEP_RATE * 0.6, SWEEP_RATE * 0.6);
      }
      break;
    }

    case S_FACE_SIDE:
      // sensor (on the right) faces the right wall at heading 90,
      // the left wall at heading -90
      if (faceHeading(useRightWall ? 90 : -90)) { sideN = 0; step = S_READ_SIDE; }
      break;

    case S_READ_SIDE: {
      setWheels(0, 0);
      float d = readUltrasonic();
      if (d > 0) sideReadings[sideN++] = d;
      if (sideN >= 7) {
        // median of 7 readings
        for (int i = 0; i < sideN; i++)
          for (int j = i + 1; j < sideN; j++)
            if (sideReadings[j] < sideReadings[i]) { float t = sideReadings[i]; sideReadings[i] = sideReadings[j]; sideReadings[j] = t; }
        float dSide = sideReadings[sideN / 2];
        if (dSide < 120) {
          poseX = useRightWall ? WALL_RIGHT_X - dSide - US_SIDE_CM
                               : WALL_LEFT_X + dSide + US_SIDE_CM;
        }
        Serial.println("------ EXACT POSITION ------");
        Serial.print("x = "); Serial.print(poseX);
        Serial.print("  y = "); Serial.print(poseY);
        Serial.print("  heading = "); Serial.println(rad2deg(poseTh));
        if (RUN_MODE == 4) { controlTick(); motorsOff(); while (true) delay(1000); }
        step = S_FACE_T1;
      }
      break;
    }

    case S_FACE_T1: {
      float h = rad2deg(atan2(T1_Y - poseY, T1_X - poseX));
      if (faceHeading(h)) step = S_GO_T1;
      break;
    }

    case S_GO_T1: {
      bool anyBlack = lB || rB;
      if (anyBlack && !prevAnyBlack && !loopFixed) {
        loopCorrection(lB, rB);
        loopFixed = true;
      }
      prevAnyBlack = anyBlack;
      if (driveTo(T1_X, T1_Y)) step = S_DROP;
      break;
    }

    case S_DROP: {
      setWheels(0, 0);
      controlTick();
      Serial.println("AT T1 - DROP");
      dropServo.write(SERVO_OPEN);
      delay(600);
      dropServo.write(SERVO_CLOSED);
      if (RUN_MODE == 5) { motorsOff(); while (true) delay(1000); }
      step = S_FACE_LINE;
      return;
    }

    case S_FACE_LINE:
      if (faceHeading(0)) {          // face right, towards the loop
        step = S_FIND_LINE;
        followStartDist = travelled;
      }
      break;

    case S_FIND_LINE:
      // drive right until an IR sees the loop (about 39 cm)
      if (lB || rB) {
        Serial.println("ON THE LINE - FOLLOWING");
        followStartDist = travelled;
        whiteSinceDist = -1;
        step = S_FOLLOW;
      } else if (travelled - followStartDist > 70) {
        Serial.println("LINE NOT FOUND - STOP");
        step = S_DONE;
      } else {
        setWheels(SEARCH_RATE, SEARCH_RATE);
      }
      break;

    case S_FOLLOW: {
      float along = travelled - followStartDist;
      followLine(lB, rB, along);

      // line end: near the expected length and both IR on white
      if (!lB && !rB) {
        if (whiteSinceDist < 0) whiteSinceDist = along;
      } else {
        whiteSinceDist = -1;
      }
      bool nearEnd = along > LINE_LENGTH_CM - 15;
      bool whiteLong = whiteSinceDist >= 0 && along - whiteSinceDist > 6;
      if ((nearEnd && whiteLong) || along > LINE_LENGTH_CM + 25) {
        // reset position: the line end is a known point on the map
        float past = (whiteSinceDist >= 0) ? along - whiteSinceDist : 0;
        poseTh = deg2rad(LINE_END_HEADING);
        poseX = LINE_END_X + (past - IR_FORWARD_CM) * cos(poseTh);
        poseY = LINE_END_Y + (past - IR_FORWARD_CM) * sin(poseTh);
        Serial.println("LINE END - GOING TO T2");
        step = S_FACE_T2;
      }
      break;
    }

    case S_FACE_T2: {
      // stop with the POINTER on T2: aim the turning point short of T2
      float h = deg2rad(POCKET_HEADING);
      goalX = T2_X - POINTER_REACH_CM * cos(h);
      goalY = T2_Y - POINTER_REACH_CM * sin(h);
      float toGoal = rad2deg(atan2(goalY - poseY, goalX - poseX));
      if (faceHeading(toGoal)) step = S_GO_T2;
      break;
    }

    case S_GO_T2:
      if (driveTo(goalX, goalY)) step = S_FINAL_TURN;
      break;

    case S_FINAL_TURN:
      // turn on the spot so the pointer swings onto T2
      if (faceHeading(POCKET_HEADING)) {
        Serial.println("AT T2 - POINTER ON TARGET");
        step = S_DONE;
      }
      break;

    case S_DONE:
      setWheels(0, 0);
      controlTick();
      motorsOff();
      Serial.println("RUN FINISHED");
      while (true) delay(1000);
  }

  controlTick();

  if (now - lastPrint >= 250) {
    lastPrint = now;
    Serial.print("step "); Serial.print((int)step);
    Serial.print(" | x "); Serial.print(poseX, 1);
    Serial.print(" y "); Serial.print(poseY, 1);
    Serial.print(" h "); Serial.print(rad2deg(poseTh), 0);
    Serial.print(" | IR L "); Serial.print(lB ? "B" : "w");
    Serial.print(" R "); Serial.print(rB ? "B" : "w");
    Serial.print(" | wheels want/got L "); Serial.print((int)L.target);
    Serial.print("/"); Serial.print((int)L.rate);
    Serial.print(" R "); Serial.print((int)R.target);
    Serial.print("/"); Serial.println((int)R.rate);
  }
}
