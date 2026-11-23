/*
  ==========================================
       IM COMPETITION 1 - FULL RUN, APPROACH 2
       "find T1 with the black loop"
       Raspberry Pi Pico + L298N + 6 optocouplers
       2 encoder motors, 2 IR line sensors,
       2 HC-SR04 (RIGHT side + FRONT), servo for the drop
  ==========================================

  HOW THIS DIFFERS FROM APPROACH 1 (mission.ino)
    Approach 1 measures its exact start position from the walls and
    drives a calculated path to T1.
    Approach 2 only takes a ROUGH position from the walls, then finds
    T1 from the black loop that circles it:
      - it drives across the loop (chord 1), noting the two points
        where an IR saw the line,
      - goes back to the middle of that chord, turns 90 degrees and
        crosses the loop again (chord 2): two more points,
      - it fits those 4 points onto the loop's shape from the map.
        That gives its exact position.
    The line itself places the robot, so wheel slip and errors from
    the start do not carry over to T1.
    It still DROPS WITH THE SERVO at T1, exactly like approach 1.

  WHAT IT DOES
   1. ROUGH POSITION  Spins once reading both ultrasonics -> rough
                      x, y and which way is up the field.
   2. FIND T1         Two chords across the black loop (above).
   3. DROP            Servo opens over T1.
   4. TO THE LINE     Faces right, drives until an IR sees the loop.
   5. FOLLOW LINE     Your line-follower speeds, to the end of the line.
   6. INTO POCKET     Lines up with the T2 pocket, stays centred with
                      the RIGHT sensor, stops with the FRONT sensor.
   7. STOP            Motors off. Everything stops at 88 s anyway.

  NEEDS the practice-board loop around T1. If the competition board
  has no loop, use approach 1 (mission.ino).

  The green ring at T1 may look black to the IR sensors. The code
  ignores black closer than the loop can be, but test it.

  PINS (same as mission.ino)
    Right wheel : ENA GP0, IN1 GP1, IN2 GP2   (L298N OUT1/OUT2)
    Left wheel  : IN3 GP3, IN4 GP4, ENB GP5   (L298N OUT3/OUT4)
    Encoders    : left C1 GP14, C2 GP15 / right C1 GP12, C2 GP13
    Line IR     : left GP16, right GP17
    RIGHT HC-SR04: TRIG GP6, ECHO GP7  (ECHO via 1k/2k divider)
    FRONT HC-SR04: TRIG GP8, ECHO GP9  (ECHO via 1k/2k divider)
    Drop servo  : GP10
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
//   4 = LOOP TEST: steps 1-2, prints the loop match, drives to T1
//       and stops there (no drop) - check how close it is
//   5 = T1 TEST: steps 1-3 only (rough position, find T1, drop)
int RUN_MODE = 4;

// ##########################################################
//   CALIBRATION  (measure these on the real board)
// ##########################################################

float COUNTS_PER_CM = 12.0;    // from RUN_MODE 1 and 2
float WHEEL_BASE_CM = 15.0;    // centre of left wheel to centre of
                               // right wheel, then tune with mode 3

// Where the sensors are, measured from the robot's turning point
// (the middle between the two wheels):
float US_SIDE_CM    = 10.0; // RIGHT ultrasonic face, right of centre
float US_FRONT_CM   = 12.0; // FRONT ultrasonic face, ahead of centre
float IR_FORWARD_CM = 8.0;  // IR sensors, in front of centre
float IR_SIDE_CM    = 2.5;  // each IR, left/right of centre

// How far the pointer tip sticks out IN FRONT OF THE FRONT SENSOR
float POINTER_BEYOND_SENSOR_CM = 2.0;

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
float T2_FROM_WALL_CM = 3.0;        // T2 is this far in front of the
                                    // pocket's back wall
float POCKET_MOUTH_X = 39.1, POCKET_MOUTH_Y = 202.6; // just outside the
                                    // pocket opening, on its centre line
float POCKET_CENTRE_TO_RAIL_CM = 19.6; // pocket centre line to the
                                    // inner face of either side wall
float POCKET_MAX_DRIVE_CM = 35;     // safety: never drive further in

float LINE_END_X = 50.5, LINE_END_Y = 202.0;  // end of the black line
float LINE_END_HEADING = -22.0;               // last part of the line

// Distances along the line from where the robot joins it (cm)
float FORK_AT_CM     = 125;   // fork just after the narrow passage
float LINE_LENGTH_CM = 228;   // join point to line end

// Black loop around T1 (for the position correction)
const int LOOP_N = 15;
float LOOP_X[LOOP_N] = { 94.0, 65.2, 42.1, 19.0, 16.2, 20.6, 28.4, 38.7, 52.1, 65.0, 77.8, 85.9, 91.9, 98.9, 102.9 };
float LOOP_Y[LOOP_N] = { 114.8, 123.3, 125.9, 122.8, 108.3, 91.2, 78.6, 68.9, 63.7, 63.4, 66.4, 73.4, 83.7, 101.2, 118.7 };

// Rough middle of the loop: the robot aims here from the start
float LOOP_MID_X = 58.0, LOOP_MID_Y = 93.9;

// Chord limits (cm): black closer than MIN is ignored (the T1 ring),
// no black within MAX means the chord failed.
float CHORD1_MIN = 40, CHORD1_MAX = 110;
float SIDE_MIN   = 20, SIDE_MAX   = 70;    // centre -> one side
float CHORD2_MIN = 50, CHORD2_MAX = 120;   // side -> other side
float LOOP_SEARCH_MAX = 120;               // start -> first black

// ##########################################################
//   SPEEDS  (encoder counts per 50 ms, about 60 = full speed)
// ##########################################################

float DRIVE_RATE = 25;   // driving to a point
float SLOW_RATE  = 10;   // last 15 cm before a target
float TURN_RATE  = 10;   // spinning on the spot
float SWEEP_RATE = 5;    // slow spin while finding position
float SEARCH_RATE = 10;  // driving towards the line

// Line following: YOUR final values (0 = full, 255 = stop)
int STRAIGHT_LEFT_PWM  = 0 + 20;
int STRAIGHT_RIGHT_PWM = 80 + 30;
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

#define ENA 0
#define IN1 1
#define IN2 2
#define IN3 3
#define IN4 4
#define ENB 5

#define LEFT_IR  16
#define RIGHT_IR 17
#define ON_BLACK HIGH

#define ENC_L_A 14
#define ENC_L_B 15
#define ENC_R_A 12
#define ENC_R_B 13
int LEFT_SIGN = 1;
int RIGHT_SIGN = -1;

#define R_TRIG 6     // RIGHT ultrasonic
#define R_ECHO 7
#define F_TRIG 8     // FRONT ultrasonic
#define F_ECHO 9

#define SERVO_PIN 10

// ##########################################################
//   INTERNAL SETTINGS
// ##########################################################

float MAX_RATE = 70;
unsigned long CONTROL_MS = 25;
float KP_SPEED = 0.012;
float KI_SPEED = 0.002;
float BRAKE_MARGIN = 10;
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
  S_SWEEP, S_AIM_LOOP, S_TO_LOOP, S_CROSS, S_TO_M1, S_FACE_A, S_FIND_A,
  S_FACE_B, S_FIND_B, S_MATCH, S_FACE_T1, S_GO_T1, S_DROP, S_FACE_LINE, S_FIND_LINE,
  S_FOLLOW, S_FACE_MOUTH, S_GO_MOUTH, S_FACE_POCKET, S_IN_POCKET, S_DONE
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

// chords across the loop (points where an IR first saw black)
float e1x = 0, e1y = 0;       // chord 1 entry
float m1x = 0, m1y = 0;       // middle of chord 1
float ax = 0, ay = 0;         // chord 2, first side
float chordDeg = 90;          // heading of chord 1
float legStart = 0;           // distance at the start of a leg
float whiteFrom = -1;         // distance where the IR went white

// every point where an IR met the loop, in the robot's own map
const int MAX_HITS = 6;
float hitX[MAX_HITS], hitY[MAX_HITS];
int nHits = 0;

float pocketStartDist = 0;
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

// One ultrasonic reading in cm. -1 = no echo (nothing within ~2.5 m)
float readUS(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long us = pulseIn(echo, HIGH, 15000);
  if (us == 0) return -1;
  return us * 0.0343 / 2.0;
}

float readRight() { return readUS(R_TRIG, R_ECHO); }
float readFront() { return readUS(F_TRIG, F_ECHO); }

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

// store a reading by the direction the SENSOR points. Distances are
// stored from the robot's turning point (sensor offset added).
void storeBin(float dirDeg, float dFromCentre) {
  int b = (((int)lround(dirDeg / 5.0)) % BINS + BINS) % BINS;
  if (dFromCentre < sweepMin[b]) sweepMin[b] = dFromCentre;
}

bool sweepFrontTurn = true;   // read the two sensors in turns

void recordSweepReading() {
  if (sweepFrontTurn) {
    float d = readFront();
    if (d > 0) storeBin(rad2deg(poseTh), d + US_FRONT_CM);
  } else {
    float d = readRight();
    if (d > 0) storeBin(rad2deg(poseTh) - 90.0, d + US_SIDE_CM);
  }
  sweepFrontTurn = !sweepFrontTurn;
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
  float y = (dBack < 130) ? WALL_BACK_Y + dBack : 25;

  bool okL = dLeft < 130, okR = dRight < 130;
  float xL = WALL_LEFT_X + dLeft;
  float xR = WALL_RIGHT_X - dRight;
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

// where the IR sensors are (point between them, in front of centre)
float sensorX() { return poseX + IR_FORWARD_CM * cos(poseTh); }
float sensorY() { return poseY + IR_FORWARD_CM * sin(poseTh); }

// drive straight along a board heading, steering to hold it
void driveStraight(float headingDeg, float rate) {
  float err = wrapPi(deg2rad(headingDeg) - poseTh);
  float steer = constrain(KP_HEADING * err, -rate, rate);
  setWheels(rate - steer, rate + steer);
}

void addHit() {
  if (nHits < MAX_HITS) { hitX[nHits] = sensorX(); hitY[nHits] = sensorY(); nHits++; }
}

// nearest point on the black loop (from the map) to (px, py)
void nearestOnLoop(float px, float py, float &qx, float &qy) {
  float bestD = 1e9;
  qx = px; qy = py;
  for (int i = 0; i < LOOP_N - 1; i++) {
    float ax0 = LOOP_X[i], ay0 = LOOP_Y[i];
    float vx = LOOP_X[i + 1] - ax0, vy = LOOP_Y[i + 1] - ay0;
    float t = constrain(((px - ax0) * vx + (py - ay0) * vy) / (vx * vx + vy * vy), 0.0f, 1.0f);
    float cx = ax0 + t * vx, cy = ay0 + t * vy;
    float d = (cx - px) * (cx - px) + (cy - py) * (cy - py);
    if (d < bestD) { bestD = d; qx = cx; qy = cy; }
  }
}

// Fit the points where the IRs met the line onto the loop's shape.
// Finds the shift that puts every point as close as possible to the
// loop, then moves the robot's position by that shift. (The heading
// is kept from the start sweep: on a round loop the turn cannot be
// measured reliably, and near T1 a few degrees hardly matter.)
void matchToLoop() {
  if (nHits == 0) { Serial.println("NO LINE POINTS - keeping rough position"); return; }

  float tx = 0, ty = 0;
  for (int it = 0; it < 20; it++) {
    float dx = 0, dy = 0;
    for (int i = 0; i < nHits; i++) {
      float qx, qy;
      nearestOnLoop(hitX[i] + tx, hitY[i] + ty, qx, qy);
      dx += qx - (hitX[i] + tx);
      dy += qy - (hitY[i] + ty);
    }
    tx += dx / nHits;
    ty += dy / nHits;
  }

  float moved = sqrt(tx * tx + ty * ty);
  // a huge correction means a wrong line was seen: do not trust it
  if (moved > 30) {
    Serial.println("LOOP MATCH REJECTED - keeping rough position");
    return;
  }
  poseX += tx;
  poseY += ty;

  Serial.print("LOOP MATCH with "); Serial.print(nHits);
  Serial.print(" points: moved "); Serial.print(moved);
  Serial.print(" cm -> x "); Serial.print(poseX);
  Serial.print(" y "); Serial.println(poseY);
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

  pinMode(R_TRIG, OUTPUT);
  pinMode(R_ECHO, INPUT);
  pinMode(F_TRIG, OUTPUT);
  pinMode(F_ECHO, INPUT);

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
      // spin slowly to the left once, reading both ultrasonics
      turnProgress += fabs(wrapPi(poseTh - lastTh));
      lastTh = poseTh;
      recordSweepReading();
      if (turnProgress >= 2 * PI + deg2rad(20)) {
        setWheels(0, 0);
        solvePosition();
        step = S_AIM_LOOP;
      } else {
        setWheels(-SWEEP_RATE, SWEEP_RATE);
      }
      break;
    }

    // ---------- 2. FIND T1 FROM THE LOOP
    case S_AIM_LOOP: {
      // aim at the middle of the loop from the rough position
      float h = rad2deg(atan2(LOOP_MID_Y - poseY, LOOP_MID_X - poseX));
      if (faceHeading(h)) { chordDeg = h; legStart = travelled; step = S_TO_LOOP; }
      break;
    }

    case S_TO_LOOP:
      // drive until an IR sees the loop: chord 1 entry
      if (lB || rB) {
        e1x = sensorX(); e1y = sensorY();
        addHit();
        legStart = travelled; whiteFrom = -1;
        Serial.println("LOOP ENTRY");
        step = S_CROSS;
      } else if (travelled - legStart > LOOP_SEARCH_MAX) {
        Serial.println("LOOP NOT FOUND - using rough position");
        step = S_MATCH;
      } else {
        driveStraight(chordDeg, SEARCH_RATE);
      }
      break;

    case S_CROSS: {
      // keep going across the loop to the far side: chord 1 exit
      float along = travelled - legStart;
      if (!lB && !rB) { if (whiteFrom < 0) whiteFrom = along; }
      if ((lB || rB) && whiteFrom >= 0 && along > CHORD1_MIN) {
        m1x = (e1x + sensorX()) / 2;
        m1y = (e1y + sensorY()) / 2;
        addHit();
        Serial.print("CHORD 1: "); Serial.print(along); Serial.println(" cm");
        step = S_TO_M1;
      } else if (along > CHORD1_MAX) {
        Serial.println("CHORD 1 FAILED - matching what it has");
        step = S_MATCH;
      } else {
        driveStraight(chordDeg, SEARCH_RATE);
      }
      break;
    }

    case S_TO_M1:
      // back to the middle of chord 1 (driveTo turns round first)
      if (driveTo(m1x, m1y)) step = S_FACE_A;
      break;

    case S_FACE_A:
      if (faceHeading(chordDeg + 90)) { legStart = travelled; step = S_FIND_A; }
      break;

    case S_FIND_A: {
      float along = travelled - legStart;
      if ((lB || rB) && along > SIDE_MIN) {
        ax = sensorX(); ay = sensorY();
        addHit();
        step = S_FACE_B;
      } else if (along > SIDE_MAX) {
        Serial.println("SIDE A FAILED - matching what it has");
        step = S_MATCH;
      } else {
        driveStraight(chordDeg + 90, SEARCH_RATE);
      }
      break;
    }

    case S_FACE_B:
      if (faceHeading(chordDeg - 90)) { legStart = travelled; step = S_FIND_B; }
      break;

    case S_FIND_B: {
      float along = travelled - legStart;
      if ((lB || rB) && along > CHORD2_MIN) {
        addHit();
        Serial.print("CHORD 2: "); Serial.print(along); Serial.println(" cm");
        step = S_MATCH;
      } else if (along > CHORD2_MAX) {
        Serial.println("SIDE B FAILED - matching what it has");
        step = S_MATCH;
      } else {
        driveStraight(chordDeg - 90, SEARCH_RATE);
      }
      break;
    }

    case S_MATCH:
      setWheels(0, 0);
      matchToLoop();
      step = S_FACE_T1;
      break;

    case S_FACE_T1: {
      goalX = T1_X;
      goalY = T1_Y;
      float h = rad2deg(atan2(goalY - poseY, goalX - poseX));
      if (faceHeading(h) || hypot(goalX - poseX, goalY - poseY) < 1.5) step = S_GO_T1;
      break;
    }

    case S_GO_T1:
      if (driveTo(goalX, goalY)) {
        if (RUN_MODE == 4) {
          setWheels(0, 0); controlTick(); motorsOff();
          Serial.println("AT T1 (loop test) - stopped");
          while (true) delay(1000);
        }
        step = S_DROP;
      }
      break;

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
        step = S_FACE_MOUTH;
      }
      break;
    }

    case S_FACE_MOUTH: {
      float h = rad2deg(atan2(POCKET_MOUTH_Y - poseY, POCKET_MOUTH_X - poseX));
      if (faceHeading(h)) step = S_GO_MOUTH;
      break;
    }

    case S_GO_MOUTH:
      if (driveTo(POCKET_MOUTH_X, POCKET_MOUTH_Y)) step = S_FACE_POCKET;
      break;

    case S_FACE_POCKET:
      if (faceHeading(POCKET_HEADING)) {
        pocketStartDist = travelled;
        step = S_IN_POCKET;
      }
      break;

    case S_IN_POCKET: {
      // FRONT sensor: stop when the pointer tip reaches T2
      float stopAt = T2_FROM_WALL_CM + POINTER_BEYOND_SENSOR_CM;
      float dF = readFront();
      if ((dF > 0 && dF <= stopAt) || travelled - pocketStartDist > POCKET_MAX_DRIVE_CM) {
        setWheels(0, 0);
        Serial.print("AT T2 - front wall at "); Serial.print(dF); Serial.println(" cm");
        step = S_DONE;
        break;
      }
      // RIGHT sensor: stay on the pocket's centre line
      float dR = readRight();
      float steer = 0;
      if (dR > 0 && dR < 40) {
        float err = dR - (POCKET_CENTRE_TO_RAIL_CM - US_SIDE_CM);   // + = too far left
        steer = constrain(err * 0.6f, -4.0f, 4.0f);
      }
      // keep the heading too
      float hErr = wrapPi(deg2rad(POCKET_HEADING) - poseTh);
      steer -= constrain(KP_HEADING * hErr, -4.0f, 4.0f);   // + hErr = turn left
      // slower as the wall gets close
      float base = (dF > 0 && dF < stopAt + 10) ? SLOW_RATE * 0.6 : SLOW_RATE;
      // too far left (err > 0) -> turn right: left wheel faster
      setWheels(base + steer, base - steer);
      break;
    }

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
