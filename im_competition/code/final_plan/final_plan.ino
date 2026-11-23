/*
  ==========================================================
       IM COMPETITION 1 - FINAL PLAN  (v2: line following fixed)
       App 2 finds the position, App 3 keeps the robot safe
  ==========================================================

  THE RUN
    1 ROOM    Front < 27 cm: back up. Else creep 10 cm forward.      (app 3 + app 2)
    2 SPIN    Spin once. Longest open stretch (nothing within 80 cm)  (app 2)
              = up the field. The shortest echoes behind, left and
              right give a ROUGH x, y, used only to aim at the curl.
    3 CURL    Drive toward the bottom of the curl with the wall       (app 2 + app 3)
              rules on, until the IRs see black.
    4 RIDE    Turn LEFT onto the curl and ride it backwards (round    (app 2)
              the bottom, up the left side, across the top).
    5 SQUARE  Just past the dead end the IRs cross the main line.     (new)
              Back up 4 cm and square up on it (each wheel creeps to
              the line edge with the IR on its own side). Exact heading
              and position = THE LANDMARK.
    6 T1      One turn, drive to T1, servo drops the item.            (app 2)
    7 TO A    Face right, drive to the line, turn left onto it.       (new shortcut)
    8 FOLLOW  Line follower to the end. Slow in the passage and       (app 3 + fixes)
              hairpin, keep left at the fork, search toward the last
              black side when lost. The hairpin is a landmark: the
              line ends HAIRPIN_TO_END_CM after it.
    9 POCKET  Reset position at the line end, go to the pocket        (app 3)
              mouth, face the pocket, stay centred with RIGHT, stop
              when FRONT reads 10 cm, flip the pointer.
    Everything stops at 88 s.

  WHAT v2 FIXES
    LINE FOLLOWING (why it was not following properly)
     a Straight speeds are back to your WORKING follower values
       (STRAIGHT_L 20, STRAIGHT_R 110). 110/110 was never tested.
     b The IRs are now read all the time (thousands of times a second)
       and remembered until the next control tick, so a thin line is
       never missed. Ultrasonics are only read in the steps that use
       them - before, a sensor read could blind the IRs for 20 ms.
     c Lost line: search only after LOST_CM of white: pivot up to 90 deg
       toward the last black side, then REVERSE along the track to where
       the line was, then 90 deg the other way, then carry on straight.
       After finding it, half speed for RECOVER_CM. It NEVER stops the run (before: "LINE LOST"
       ended the run). Search is off at the fork, near the line end and
       on the top of the curl, so it cannot turn round onto the line
       behind it.
     d Which IR is on which side is worked out from your follower
       values: a black IR always slows the wheel on its OWN side (the
       robot turns toward it). With LB 10/250 the LEFT_IR (GP16) sits on
       the robot's RIGHT. So "keep left at the fork", "ignore the decoy
       side" and SQUARE now use the correct sensor.
    THE LANDMARK
     e Crossing the main line = both IRs black within CROSS_PAIR_CM of
       each other, after the white gap of the dead end. Works when the
       crossing is at an angle (before: both at the very same moment).
     f On the top of the curl, before the decoy is passed, a decoy-side
       correction may not turn the robot more than DECOY_MAX_DEG off the
       top's direction (so it cannot swing onto the decoy), and off the dead end the robot holds its
       heading with the encoders (not "straight" with uneven speeds).
     g SQUARE: each wheel tracks the line edge with its own IR (forward
       on white, slowly back on black) until both settle - square within
       a few degrees. Before, each wheel just stopped at the first black,
       which only fixed about a third of the angle. Wrong sides (one IR on
       the line, robot turned over 45 deg) -> turn back, flip the sides,
       try once more. Works for approach angles up to about 40 deg.
    OTHER
     h The green ring at T1 is not taken as the line on the way to A.
     i Stuck while backing up in the start box -> spins right there.
       Stuck while reversing -> nudges forward.
     j Separate COUNTS_PER_CM for each wheel (mode 1 prints both).
     k RUN_MODE 7: line follower only, to check following on its own.

  WHAT COULD GO WRONG AT THE LANDMARK, AND WHAT THE CODE DOES
    1 DECOY   A decoy ends about 5 cm above the top of the curl.
              -> On the top, no left turn beyond DECOY_MAX_DEG, and
                 "crossing" only counts after TOP_CLEAR_CM along the top.
    2 NOISE   One flicker of black.
              -> Needs a white gap of OFF_END_CM first, then BOTH sides.
    3 SWAPPED Squaring turns AWAY from the line.
              -> Detected (one side locked, robot turning), retried once
                 with the sides flipped, right IR_SIDES_FLIP printed.
    4 MISSED  The crossing is never seen.
              -> After RIDE_GIVE_UP_CM on the top, carry on with the
                 position from odometry. Less accurate, run still ends.

  SAFETY (only while driving in open space: ROOM to CURL, T1, TO A)
    FRONT < 12 cm  -> back up until FRONT >= 27 cm, turn 35 deg away
    RIGHT < 27 cm  -> steer left (only on the way to the curl)
    WHEELS STUCK   -> 0.5 s: back up and turn; stuck while TURNING or
                      REVERSING: nudge 5 cm
    BLACK + wall within 15 cm ahead = border, not line
    Off while following the line and in the pocket.

  CALIBRATE IN THIS ORDER
    RUN_MODE 4 -> check sensors      RUN_MODE 7 -> check line following
    RUN_MODE 1 -> COUNTS_PER_CM_L/R  RUN_MODE 2 -> WHEEL_BASE_CM
    RUN_MODE 6 -> IR_SIDES_FLIP      RUN_MODE 5 -> T1_X, T1_Y (teach)
    then RUN_MODE 0

  PINS (same as approach 3)
    Right wheel : ENA GP0, IN1 GP1, IN2 GP2
    Left wheel  : IN3 GP3, IN4 GP4, ENB GP5
    Ultrasonic  : RIGHT TRIG GP6 / ECHO GP7, FRONT TRIG GP8 / ECHO GP9
    Servo GP10, encoders right GP12/13 left GP14/15, IR left GP16 right GP17
*/

#include <Servo.h>

// ===================== RUN MODE =====================
// 0 FULL RUN
// 1 COUNT TEST : push the robot 100 cm by hand. COUNTS_PER_CM_L = L counts / 100,
//                COUNTS_PER_CM_R = R counts / 100
// 2 TURN TEST  : spins 360 deg. Went too far -> raise WHEEL_BASE_CM
// 3 SAFETY TEST: drives forward with the wall rules on
// 4 SENSOR TEST: prints FRONT, RIGHT and both IRs, motors off
// 5 TEACH      : full run up to SQUARE, then motors off and prints x, y.
//                Push the robot by hand until the axle centre is over T1
//                and copy the printed x, y into T1_X, T1_Y.
// 6 SQUARE TEST: put the robot 10 cm before a line at an angle. It must
//                stop square to the line. If it turns away: flip IR_SIDES_FLIP.
// 7 LINE TEST  : put the robot on a line. It only follows the line
//                (no fork, no slow zones, no end), with the lost-line search.
int RUN_MODE = 0;

// ===================== CALIBRATION =====================
float COUNTS_PER_CM_L = 12.0;  // mode 1
float COUNTS_PER_CM_R = 12.0;  // mode 1
float WHEEL_BASE_CM  = 15.0;   // mode 2
float US_FRONT_CM    = 12.0;   // front sensor face, ahead of the axle centre
float US_SIDE_CM     = 10.0;   // right sensor face, right of the axle centre
float IR_FORWARD_CM  = 8.0;    // middle of the two IRs, ahead of the axle centre
float POINTER_BEYOND_SENSOR_CM = 7.0;  // unfolded pointer tip past the front sensor
bool  IR_SIDES_FLIP  = false;  // mode 6: flip only if SQUARE turns away from the line
int   SERVO_HOLD = 0, SERVO_DROP = 90, SERVO_POINTER = 180;

// ===================== SAFETY =====================
const float FRONT_STOP_CM = 12;   // closer than this: back up
const float FRONT_ROOM_CM = 27;   // back up until this much room (20 cm swing + 7 spare)
const float RIGHT_KEEP_CM = 27;   // keep the right wall at least this far
const float WALL_SEEN_CM  = 15;   // black with a wall this close ahead = border
const float OPEN_CM       = 80;   // further than this = open field
const float CREEP_FWD_CM  = 10;   // room step when the front is clear

// ===================== MAP (practice board, cm) =====================
// Frame: origin at the start-end left corner, x right, y up the field,
// heading 0 = facing right, 90 = up the field, left turn = +.
float T1_X = 58.4, T1_Y = 97.0;                        // mode 5 measures these
float SQ_X = 101.5, SQ_Y = 112.6, SQ_HEADING = -12.9;  // IRs squared on the main line
float CURL_AIM_X = 58, CURL_AIM_Y = 70;               // aim for the bottom middle of the curl
float LINE_END_X = 50.5, LINE_END_Y = 202.0, LINE_END_HEADING = -22.0;
float MOUTH_X = 36.8, MOUTH_Y = 207.2;                // pocket centre line, outside the rail ends
float POCKET_HEADING = -62.7;
float POCKET_CENTRE_TO_RAIL_CM = 19.6;
float T2_FROM_WALL_CM = 3.0;
// distances along the line, measured from A (where the robot joins the right side)
float FORK_AT_CM = 125, PASSAGE_FROM_CM = 65, PASSAGE_TO_CM = 130;
float HAIRPIN_AFTER_CM = 170, HAIRPIN_TO_END_CM = 34, LINE_LENGTH_CM = 228;
float MIN_RIDE_CM = 30;        // "on the top" not before this much riding
float TOP_HEADING_DEG = 0;     // heading along the top of the curl (riding backwards)
float TOP_CLEAR_CM = 35;       // along the top before the decoy end is passed
float DECOY_MAX_DEG = 20;      // in the decoy zone, no left turn beyond this off the top direction
float RIDE_GIVE_UP_CM = 120;   // along the top with no crossing -> carry on without it
float OFF_END_CM = 4;          // white gap that means "past the dead end"
float CROSS_PAIR_CM = 5;       // both IRs black within this distance = main line crossed
float A_IGNORE_CM = 15;        // on the way to A, ignore black this close to T1 (green ring)
unsigned long SQUARE_SETTLE_MS = 800;  // both IRs on the line edge this long = square

// ===================== SPEEDS (counts per 50 ms, ~60 = full) =====================
float DRIVE = 20, SLOW = 10, TURN = 9, BACK = 12, CREEP = 6, SEARCH_SPEED = 7;
// Line follower: YOUR WORKING VALUES, PWM scale (0 = full, 255 = stop).
int STRAIGHT_L = 20, STRAIGHT_R = 110;   // both IRs white
int LB_L = 10,  LB_R = 250;              // left IR (GP16) black
int RB_L = 250, RB_R = 10;               // right IR (GP17) black
float SLOW_ZONE = 0.6;           // speed factor in the passage and after the hairpin
float RIDE_TOP_K = 0.6;          // speed factor on the top of the curl
bool  FORK_KEEP_LEFT = true;     // keep left at the fork (false = keep right)
float LOST_CM = 8;               // both white this far while following -> search
float SEARCH_DEG = 90;           // search sweep each way
float RECOVER_CM = 15;           // half speed this far after finding the line again

// ===================== PINS =====================
#define ENA 0
#define IN1 1
#define IN2 2
#define IN3 3
#define IN4 4
#define ENB 5
#define R_TRIG 6
#define R_ECHO 7
#define F_TRIG 8
#define F_ECHO 9
#define SERVO_PIN 10
#define ENC_R_A 12
#define ENC_R_B 13
#define ENC_L_A 14
#define ENC_L_B 15
#define LEFT_IR 16
#define RIGHT_IR 17
#define ON_BLACK HIGH
int LEFT_SIGN = 1, RIGHT_SIGN = -1;
float MAX_RATE = 70;

// ===================== TYPES (before functions, for the IDE) =====================
struct Wheel { float target = 0, rate = 0, integ = 0, duty = 0; int dir = 1; bool brake = true; long last = 0; };

enum Step { S_ROOM, S_SPIN, S_AIM, S_TO_CURL, S_CURL_ONTO, S_CURL_TURN, S_RIDE,
            S_SQ_BACK, S_SQUARE, S_FACE_T1, S_TO_T1, S_DROP, S_FACE_A, S_TO_A,
            S_A_ONTO, S_A_TURN, S_FOLLOW, S_GO_MOUTH, S_FACE_POCKET, S_POCKET,
            S_POINTER, S_TEACH, S_DONE };
const char* NAMES[] = { "ROOM", "SPIN", "AIM", "TO CURL", "CURL ONTO", "CURL TURN", "RIDE",
                        "SQUARE BACK", "SQUARE", "FACE T1", "TO T1", "DROP", "FACE A", "TO A",
                        "A ONTO", "A TURN", "FOLLOW", "GO MOUTH", "FACE POCKET", "POCKET",
                        "POINTER", "TEACH", "DONE" };

// ===================== STATE =====================
Wheel L, R;
Servo servo;
Step step = S_ROOM;
float x = 61, y = 30, th = 0;        // position (cm) and heading (rad)
float fwd = 0, travelled = 0;        // signed forward distance, total distance
float legStart = 0, legTh = 0;       // fwd and th when the current step began
long oL = 0, oR = 0;
unsigned long runStart, lastTick, lastPrint, stuckSince = 0, stepT = 0;

int escape = 0, escNudge = 0;        // 0 none, 1 backing, 2 turning away, 3 nudge
float escStart = 0, escTurnTo = 0;

signed char openB[72];               // spin: 1 open, -1 wall, 0 not seen
float minD[72];                      // spin: shortest echo per 5 deg
float spun = 0, lastTh = 0, aimDeg = 90;
int roomMode = 0;                    // 1 backing up, 2 creeping forward

bool irL = false, irR = false;       // black seen since the last tick (fast sampling)
bool leftIrOnRight = true;           // worked out in setup() from the follower values

int lastTurn = 0;                    // +1 last correction was LEFT IR, -1 RIGHT IR
float whiteFrom = -1, searchTh = 0;
int searchPhase = 0;                 // 0 none, 1-5 search steps (see lineStep)
float backTo = 0, recoverUntil = -999;
float hairpinAt = -1, hist[16], histFwd = 0;
int histN = 0;

bool lockL = false, lockR = false, irSwap = false, swapTried = false;
unsigned long settleT = 0;
float sqStartTh = 0; bool sqStartSet = false, sqRefaceFirst = false;
float topTh = 0, topAt = -1, lastBlack = 0, holdTh = 0, seenL = -999, seenR = -999, offFirst = -1;
bool offEnd = false;
unsigned long servoT = 0;

volatile long rawL = 0, rawR = 0;
void isrL() { rawL += (digitalRead(ENC_L_A) != digitalRead(ENC_L_B)) ? 1 : -1; }
void isrR() { rawR += (digitalRead(ENC_R_A) != digitalRead(ENC_R_B)) ? 1 : -1; }
long cntL() { noInterrupts(); long c = rawL; interrupts(); return c * LEFT_SIGN; }
long cntR() { noInterrupts(); long c = rawR; interrupts(); return c * RIGHT_SIGN; }

float wrapPi(float a) { while (a > PI) a -= 2 * PI; while (a < -PI) a += 2 * PI; return a; }
float wrapDeg(float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
float rad(float d) { return d * PI / 180; }
float deg(float r) { return r * 180 / PI; }
float pwmToRate(int p) { return (255 - constrain(p, 0, 255)) / 255.0 * MAX_RATE; }

// ===================== SENSORS =====================
void sampleIR() {                                  // called all the time, latches black
  if (digitalRead(LEFT_IR) == ON_BLACK) irL = true;
  if (digitalRead(RIGHT_IR) == ON_BLACK) irR = true;
}

// black on the robot's right / left side (sides worked out from the follower)
bool blackRightSide(bool l, bool r) { return leftIrOnRight ? l : r; }
bool blackLeftSide(bool l, bool r)  { return leftIrOnRight ? r : l; }

// Ultrasonic, but keeps sampling the IRs while it waits for the echo.
float readUS(int t, int e, unsigned long timeoutUs) {
  unsigned long t0 = micros();
  while (digitalRead(e) == HIGH) { sampleIR(); if (micros() - t0 > timeoutUs) return -1; }  // old echo
  digitalWrite(t, LOW); delayMicroseconds(2);
  digitalWrite(t, HIGH); delayMicroseconds(10); digitalWrite(t, LOW);
  t0 = micros();
  while (digitalRead(e) == LOW) { sampleIR(); if (micros() - t0 > 4000) return -1; }
  unsigned long s = micros();
  while (digitalRead(e) == HIGH) { sampleIR(); if (micros() - s > timeoutUs) return -1; }
  return (micros() - s) * 0.0343 / 2;               // -1 = no echo (open)
}

// ===================== MOTORS + ODOMETRY =====================
void updateWheel(Wheel &w, long c) {
  w.rate = (c - w.last) * 2.0; w.last = c;        // counts per 50 ms (tick = 25 ms)
  if (w.target == 0) { w.brake = true; w.integ = 0; return; }
  int want = w.target > 0 ? 1 : -1;
  float along = w.rate * want, mag = fabs(w.target);
  if (along > mag + 10 || along < -10) { w.brake = true; w.integ = 0; return; }
  w.brake = false; w.dir = want;
  float err = mag - along;
  w.integ = constrain(w.integ + err, -150.0f, 150.0f);
  w.duty = constrain(mag / MAX_RATE + 0.012f * err + 0.002f * w.integ, 0.0f, 1.0f);
}

void outMotor(Wheel &w, int en, int a, int b, int fwdA) {
  if (w.brake) { digitalWrite(a, LOW); digitalWrite(b, LOW); analogWrite(en, 0); return; }
  bool f = w.dir > 0;
  digitalWrite(a, f ? fwdA : !fwdA);
  digitalWrite(b, f ? !fwdA : fwdA);
  analogWrite(en, (int)(255 - w.duty * 255));     // inverted PWM
}

void motorsOff() { analogWrite(ENA, 255); analogWrite(ENB, 255);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }
void wheels(float l, float r) { L.target = l; R.target = r; }

void odom() {
  long cl = cntL(), cr = cntR();
  float dL = (cl - oL) / COUNTS_PER_CM_L, dR = (cr - oR) / COUNTS_PER_CM_R;
  oL = cl; oR = cr;
  float d = (dL + dR) / 2, dTh = (dR - dL) / WHEEL_BASE_CM;
  x += d * cos(th + dTh / 2); y += d * sin(th + dTh / 2); th = wrapPi(th + dTh);
  fwd += d; travelled += fabs(d);
}

void tick() {
  odom();
  updateWheel(L, cntL()); updateWheel(R, cntR());
  outMotor(R, ENA, IN1, IN2, LOW);               // right: forward = IN1 LOW
  outMotor(L, ENB, IN3, IN4, HIGH);              // left:  forward = IN3 HIGH
}

void stepTo(Step s) {
  step = s; legStart = fwd; legTh = th; stepT = millis();
  Serial.print(">> "); Serial.print(NAMES[s]);
  Serial.print("  x "); Serial.print(x, 1); Serial.print(" y "); Serial.print(y, 1);
  Serial.print(" h "); Serial.println(deg(th), 1);
}

// Pose from a known spot where the IRs sit (px, py) with heading hDeg.
void setPoseFromIR(float px, float py, float hDeg) {
  th = rad(hDeg);
  x = px - IR_FORWARD_CM * cos(th);
  y = py - IR_FORWARD_CM * sin(th);
}

// ===================== MOVES =====================
bool face(float hDeg) {                           // spin on the spot
  float e = wrapPi(rad(hDeg) - th);
  if (fabs(e) < rad(2)) { wheels(0, 0); return true; }
  float s = fabs(e) < rad(15) ? TURN * 0.5 : TURN;
  if (e > 0) wheels(-s, s); else wheels(s, -s);
  return false;
}

void hold(float hDeg, float v) {                  // drive along a heading
  float e = wrapPi(rad(hDeg) - th);
  float s = constrain(25 * e, -v, v);
  wheels(v - s, v + s);
}

bool goTo(float tx, float ty) {
  float dx = tx - x, dy = ty - y, dist = sqrt(dx * dx + dy * dy);
  if (dist < 1 || (dist < 3 && dx * cos(th) + dy * sin(th) < 0)) { wheels(0, 0); return true; }
  float h = deg(atan2(dy, dx));
  if (fabs(wrapPi(rad(h) - th)) > rad(30)) { face(h); return false; }
  hold(h, dist > 15 ? DRIVE : fmaxf(SLOW * 0.6f, SLOW * dist / 15));
  return false;
}

// ===================== SAFETY =====================
bool stalled() {                                  // asked to move, wheels not moving
  bool asking = (fabs(L.target) > 3 && !L.brake) || (fabs(R.target) > 3 && !R.brake);
  bool moving = fabs(L.rate) > 1 || fabs(R.rate) > 1;
  if (!asking || moving) { stuckSince = 0; return false; }
  if (stuckSince == 0) stuckSince = millis();
  return millis() - stuckSince > 500;
}

void startEscape(int nudge) {                     // 0 = back up and turn, +-1 = nudge 5 cm
  escape = nudge ? 3 : 1; escNudge = nudge; escStart = fwd; stuckSince = 0;
  Serial.println(nudge ? "SAFETY: stuck, nudging 5 cm" : "SAFETY: backing up");
}

void runEscape(float f, float r) {
  if (escape == 1) {                              // back up until there is room
    if (f < 0 || f >= FRONT_ROOM_CM || escStart - fwd > 25 || stalled()) {
      escTurnTo = deg(th) + ((r > 0 && r < 30) ? 35 : -35);   // right close -> turn left
      escape = 2; wheels(0, 0);
    } else wheels(-BACK, -BACK);
  } else if (escape == 2) {
    if (face(escTurnTo) || stalled()) { escape = 0; stuckSince = 0; Serial.println("SAFETY: clear"); }
  } else {                                        // nudge 5 cm
    if (fabs(fwd - escStart) >= 5 || stalled()) { escape = 0; escNudge = 0; stuckSince = 0; wheels(0, 0); }
    else wheels(escNudge * BACK, escNudge * BACK);
  }
}

bool atEdge() {                                   // IRs over the board's black border?
  float sx = x + IR_FORWARD_CM * cos(th), sy = y + IR_FORWARD_CM * sin(th);
  return sx < 7 || sx > 115 || sy < 7 || sy > 237;
}

// ===================== SPIN ANALYSIS =====================
int binOf(float a) { return ((int)lround(deg(a) / 5) % 72 + 72) % 72; }

float minAround(float cDeg, float half) {         // shortest echo within +-half of cDeg
  float m = -1;
  for (int b = 0; b < 72; b++) {
    if (minD[b] > 900 || fabs(wrapDeg(b * 5 - cDeg)) > half) continue;
    if (m < 0 || minD[b] < m) m = minD[b];
  }
  return m;
}

void startSpin() {
  for (int i = 0; i < 72; i++) { openB[i] = 0; minD[i] = 999; }
  spun = 0; lastTh = th;
  stepTo(S_SPIN);
}

void analyseSpin() {
  int bestStart = 0, bestLen = 0;                 // longest run of open bins = up the field
  for (int s = 0; s < 72; s++) {
    int n = 0; while (n < 72 && openB[(s + n) % 72] == 1) n++;
    if (n > bestLen) { bestLen = n; bestStart = s; }
  }
  float up = bestLen ? (bestStart + (bestLen - 1) / 2.0) * 5 : deg(th);
  float back = minAround(up + 180, 30), left = minAround(up + 90, 25), right = minAround(up - 90, 25);

  th = wrapPi(th - rad(up) + rad(90));            // field frame from now on: 90 = up the field
  y = back > 0 ? back + US_FRONT_CM : 30;         // ROUGH - only used to aim
  if (left > 0 && (right < 0 || left < right)) x = left + US_FRONT_CM;
  else if (right > 0) x = 122 - right - US_FRONT_CM;
  else x = 61;
  x = constrain(x, 15.0f, 107.0f); y = constrain(y, 12.0f, 55.0f);
  aimDeg = 0.6 * deg(atan2(CURL_AIM_Y - y, CURL_AIM_X - x)) + 0.4 * 90;

  Serial.print("SPIN: open width "); Serial.print(bestLen * 5);
  Serial.print("  back "); Serial.print(back); Serial.print(" left "); Serial.print(left);
  Serial.print(" right "); Serial.print(right);
  Serial.print("  rough x "); Serial.print(x, 0); Serial.print(" y "); Serial.print(y, 0);
  Serial.print("  aim "); Serial.println(aimDeg, 0);
}

// ===================== LINE FOLLOWER =====================
// Your rules and speeds. When both IRs stay white for LOST_CM (and search
// is allowed), the line has slipped outside the sensors:
//   1 pivot up to SEARCH_DEG toward the side of the last correction
//   2 turn back, 3 REVERSE along the track to where the line was
//   4 pivot SEARCH_DEG the other way, 5 turn back and carry on straight
// As soon as an IR sees black it follows again, at half speed for
// RECOVER_CM so it does not overshoot the same corner again.
void pivot(int dir) { wheels(-dir * SEARCH_SPEED, dir * SEARCH_SPEED); }   // +1 = left

void lineStep(bool l, bool r, float k, bool allowSearch) {
  if (l || r) {
    if (searchPhase) recoverUntil = fwd + RECOVER_CM;
    searchPhase = 0; whiteFrom = -1;
  }
  if (fwd < recoverUntil) k *= 0.5;
  if (l && !r)      { wheels(k * pwmToRate(LB_L), k * pwmToRate(LB_R)); lastTurn = 1; return; }
  if (r && !l)      { wheels(k * pwmToRate(RB_L), k * pwmToRate(RB_R)); lastTurn = -1; return; }
  float sL = k * pwmToRate(STRAIGHT_L), sR = k * pwmToRate(STRAIGHT_R);
  if (l && r)       { wheels(sL, sR); return; }

  // both white
  if (whiteFrom < 0) whiteFrom = fwd;
  if (searchPhase == 0) {
    if (!allowSearch || lastTurn == 0 || fwd - whiteFrom < LOST_CM) { wheels(sL, sR); return; }
    searchPhase = 1; searchTh = th; backTo = whiteFrom - 3;
    Serial.println("LINE LOST - searching");
  }
  // direction the last correction turned: + = left
  int corr = lastTurn == 1 ? (pwmToRate(LB_R) > pwmToRate(LB_L) ? 1 : -1)
                           : (pwmToRate(RB_R) > pwmToRate(RB_L) ? 1 : -1);
  float turned = deg(wrapPi(th - searchTh));
  switch (searchPhase) {
    case 1: if (turned * corr >= SEARCH_DEG) searchPhase = 2; else pivot(corr); break;
    case 2: if (face(deg(searchTh))) searchPhase = 3; break;
    case 3: if (fwd <= backTo) searchPhase = 4;                         // reverse to the line
            else wheels(-SEARCH_SPEED, -SEARCH_SPEED); break;
    case 4: if (-turned * corr >= SEARCH_DEG) searchPhase = 5; else pivot(-corr); break;
    default:
      if (face(deg(searchTh))) {                  // not found: carry on straight
        searchPhase = 0; lastTurn = 0; whiteFrom = fwd;
        Serial.println("LINE NOT FOUND - carrying on straight");
      }
  }
}

void resetFollower() {
  lastTurn = 0; whiteFrom = -1; searchPhase = 0; recoverUntil = -999;
  hairpinAt = -1; histN = 0; histFwd = fwd;
}

void recordHeading() {                            // heading every 2 cm, last 32 cm
  if (fwd - histFwd < 2) return;
  histFwd = fwd;
  for (int i = 15; i > 0; i--) hist[i] = hist[i - 1];
  hist[0] = th;
  if (histN < 16) histN++;
}

float turnedOver(int n) { return histN > n ? fabs(wrapPi(th - hist[n])) : 0; }   // n x 2 cm back

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  int outs[] = { ENA, IN1, IN2, IN3, IN4, ENB, R_TRIG, F_TRIG };
  for (int p : outs) pinMode(p, OUTPUT);
  pinMode(R_ECHO, INPUT); pinMode(F_ECHO, INPUT);
  pinMode(LEFT_IR, INPUT); pinMode(RIGHT_IR, INPUT);
  int encs[] = { ENC_L_A, ENC_L_B, ENC_R_A, ENC_R_B };
  for (int p : encs) pinMode(p, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, CHANGE);
  analogWriteRange(255); analogWriteFreq(500);
  motorsOff();
  servo.attach(SERVO_PIN); servo.write(SERVO_HOLD);

  // A black IR slows the wheel on its own side. LEFT_IR black slows the
  // right wheel (LB_R > LB_L on the PWM scale) -> LEFT_IR is on the right.
  leftIrOnRight = (LB_R > LB_L) != IR_SIDES_FLIP;

  delay(2000);
  L.last = oL = cntL(); R.last = oR = cntR();
  runStart = lastTick = millis();
  Serial.print("LEFT_IR (GP16) is on the robot's "); Serial.println(leftIrOnRight ? "RIGHT" : "LEFT");
  if (RUN_MODE == 6) { lockL = lockR = false; settleT = 0; swapTried = false; irSwap = false; stepTo(S_SQUARE); }
  else if (RUN_MODE == 7) { resetFollower(); stepTo(S_FOLLOW); }
  else stepTo(S_ROOM);
}

// ===================== LOOP =====================
void loop() {
  sampleIR();                                     // as often as possible

  if (RUN_MODE == 1) {
    if (millis() - lastPrint > 300) { lastPrint = millis();
      Serial.print("counts L "); Serial.print(cntL()); Serial.print("  R "); Serial.print(cntR());
      Serial.println("   (after 100 cm: COUNTS_PER_CM_L = L/100, COUNTS_PER_CM_R = R/100)"); }
    return;
  }
  if (RUN_MODE != 7 && step != S_DONE && step != S_TEACH && millis() - runStart > 88000) {
    Serial.println("88 s - STOP"); stepTo(S_DONE);
  }
  if (millis() - lastTick < 25) return;
  lastTick = millis();

  // ---- sensors: ultrasonics only in the steps that use them
  bool guarded = step <= S_TO_CURL || step == S_FACE_T1 || step == S_TO_T1 || step == S_FACE_A || step == S_TO_A;
  bool test34 = RUN_MODE == 3 || RUN_MODE == 4;
  bool needFront = guarded || step == S_POCKET || escape || test34;
  bool needRight = step == S_TO_CURL || step == S_POCKET || escape || test34;
  float f = needFront ? readUS(F_TRIG, F_ECHO, 12000) : -1;
  float r = needRight ? readUS(R_TRIG, R_ECHO, 8000) : -1;
  sampleIR();
  bool lB = irL, rB = irR;                        // black at any moment since the last tick
  irL = irR = false;
  bool wallAhead = f > 0 && f < WALL_SEEN_CM;
  bool localised = step >= S_FACE_T1 && step <= S_POINTER;
  bool black = (lB || rB) && !wallAhead && !(localised && atEdge());   // not the border

  // ---- test modes
  if (RUN_MODE == 2) {
    spun += fabs(wrapPi(th - lastTh)); lastTh = th;
    if (spun >= 2 * PI) { wheels(0, 0); tick(); motorsOff(); Serial.println("360 done"); while (true) delay(1000); }
    wheels(-TURN, TURN); tick(); return;
  }
  if (RUN_MODE == 4) {
    motorsOff();
    if (millis() - lastPrint > 200) { lastPrint = millis();
      Serial.print("FRONT "); Serial.print(f, 1); Serial.print("  RIGHT "); Serial.print(r, 1);
      Serial.print("  IR L(GP16) "); Serial.print(lB ? "black" : "white");
      Serial.print("  IR R(GP17) "); Serial.println(rB ? "black" : "white"); }
    return;
  }
  if (step == S_TEACH) {                          // motors off, robot pushed by hand
    odom(); motorsOff();
    if (millis() - lastPrint > 300) { lastPrint = millis();
      Serial.print("TEACH  x "); Serial.print(x, 1); Serial.print("  y "); Serial.print(y, 1);
      Serial.print("  h "); Serial.println(deg(th), 1); }
    return;
  }

  // ---- safety layer: only while driving in open space
  if (escape) { runEscape(f, r); tick(); return; }
  if (guarded) {
    bool forward = L.target > 0 && R.target > 0;
    if (forward && f > 0 && f < FRONT_STOP_CM) { startEscape(0); tick(); return; }
    if (stalled()) {
      if (step == S_ROOM && roomMode == 1) { wheels(0, 0); Serial.println("ROOM: back wall reached"); startSpin(); }
      else if (L.target < 0 && R.target < 0) startEscape(1);                 // stuck reversing
      else if (L.target * R.target < 0) startEscape((f < 0 || f > 20) ? 1 : -1);  // stuck turning
      else startEscape(0);
      tick(); return;
    }
  }

  if (RUN_MODE == 3) {                            // safety test: forward, right rule on
    float h = deg(th) + ((r > 0 && r < RIGHT_KEEP_CM) ? (RIGHT_KEEP_CM - r) * 1.5 : 0);
    hold(h, DRIVE * 0.6); tick(); return;
  }

  switch (step) {
    // ---------- 1 ROOM: make space to spin
    case S_ROOM:
      if (roomMode == 0) roomMode = (f > 0 && f < FRONT_ROOM_CM) ? 1 : 2;
      if (roomMode == 1) {                        // wall ahead: back up (app 3)
        if (f < 0 || f >= FRONT_ROOM_CM || legStart - fwd > 20) { wheels(0, 0); startSpin(); }
        else wheels(-BACK, -BACK);
      } else {                                    // clear: creep forward (app 2)
        if (fwd - legStart >= CREEP_FWD_CM) { wheels(0, 0); startSpin(); }
        else hold(deg(legTh), SLOW);
      }
      break;

    // ---------- 2 SPIN: find the open side, rough x, y
    case S_SPIN: {
      int b = binOf(th);
      if (f < 0 || f > OPEN_CM) openB[b] = 1; else if (openB[b] == 0) openB[b] = -1;
      if (f > 0) minD[b] = fminf(minD[b], f);
      spun += fabs(wrapPi(th - lastTh)); lastTh = th;
      if (spun < 2 * PI + rad(10)) wheels(-TURN * 0.7, TURN * 0.7);
      else { wheels(0, 0); analyseSpin(); stepTo(S_AIM); }
      break;
    }

    case S_AIM:
      if (face(aimDeg)) stepTo(S_TO_CURL);
      break;

    // ---------- 3 CURL: drive up with the wall rules
    case S_TO_CURL: {
      float h = aimDeg + ((r > 0 && r < RIGHT_KEEP_CM) ? fminf(30.0f, (RIGHT_KEEP_CM - r) * 1.5f) : 0);
      if (black) { Serial.println("CURL FOUND"); stepTo(S_CURL_ONTO); }
      else if (fwd - legStart > 180) { Serial.println("NO CURL WITHIN 180 cm"); stepTo(S_DONE); }
      else hold(h, DRIVE);
      break;
    }

    case S_CURL_ONTO:                             // put the axle on the line
      if (fwd - legStart >= IR_FORWARD_CM) { wheels(0, 0); stepTo(S_CURL_TURN); }
      else hold(deg(legTh), SLOW * 0.6);
      break;

    case S_CURL_TURN: {                           // pivot LEFT until the IRs find the line
      float turned = wrapPi(th - legTh);
      if (turned > rad(30) && (lB || rB)) {
        wheels(0, 0); resetFollower();
        topAt = -1; offEnd = false; offFirst = -1; lastBlack = fwd; holdTh = th;
        stepTo(S_RIDE);
      }
      else if (turned > rad(150)) { Serial.println("CURL LOST IN TURN"); stepTo(S_DONE); }
      else wheels(-TURN * 0.6, TURN * 0.6);
      break;
    }

    // ---------- 4 RIDE the curl backwards to the main line crossing
    case S_RIDE: {
      float ride = fwd - legStart;
      if (topAt < 0 && ride > MIN_RIDE_CM && fabs(wrapDeg(deg(th) - TOP_HEADING_DEG)) < 30) {
        topAt = fwd; topTh = th; Serial.println("ON TOP OF THE CURL");
      }
      bool onTop = topAt >= 0;
      bool pastDecoy = onTop && fwd - topAt > TOP_CLEAR_CM;

      if (lB || rB) { lastBlack = fwd; holdTh = th; }
      if (pastDecoy && !offEnd && fwd - lastBlack >= OFF_END_CM) {
        offEnd = true; seenL = seenR = -999; offFirst = -1;
        Serial.println("CURL END - looking for the main line");
      }
      if (offEnd) {
        if (lB) seenL = fwd;
        if (rB) seenR = fwd;
        if ((lB || rB) && offFirst < 0) offFirst = fwd;
      }
      bool crossed = offEnd && fwd - seenL <= CROSS_PAIR_CM && fwd - seenR <= CROSS_PAIR_CM;

      if (crossed) {                              // both sides black close together
        wheels(0, 0); swapTried = false; irSwap = false; sqStartSet = false; sqRefaceFirst = false;
        Serial.println("MAIN LINE CROSSED"); stepTo(S_SQ_BACK);
      }
      else if ((onTop && fwd - topAt > RIDE_GIVE_UP_CM) || ride > 260) {
        wheels(0, 0); Serial.println("NO CROSSING SEEN - carrying on with odometry (less accurate)");
        stepTo(S_FACE_T1);
      }
      else if (offEnd && offFirst >= 0 && fwd - offFirst > CROSS_PAIR_CM) {
        offEnd = false;                           // only one side: still the curl, follow it
        Serial.println("ONE SIDE ONLY - still on the curl");
        lineStep(lB, rB, RIDE_TOP_K, false);
      }
      else if (offEnd || (pastDecoy && !lB && !rB)) hold(deg(holdTh), SLOW);   // straight off the end
      else {
        bool l = lB, rr = rB;
        // decoy zone: a left-side (decoy-side) correction may not turn the
        // robot more than DECOY_MAX_DEG left of the top's direction
        if (onTop && !pastDecoy && wrapDeg(deg(th) - deg(topTh)) > DECOY_MAX_DEG) {
          if (leftIrOnRight) rr = false; else l = false;
        }
        lineStep(l, rr, onTop ? RIDE_TOP_K : 1.0f, !onTop);
      }
      break;
    }

    // ---------- 5 SQUARE on the main line = the landmark
    case S_SQ_BACK:                               // (retry: face the old heading) back off the line
      if (sqRefaceFirst) { if (face(deg(sqStartTh))) { sqRefaceFirst = false; legStart = fwd; } break; }
      if (legStart - fwd >= 4) { wheels(0, 0); lockL = lockR = false; settleT = 0; stepTo(S_SQUARE); }
      else wheels(-CREEP, -CREEP);
      break;

    case S_SQUARE: {                              // each wheel tracks the line edge with the IR on its side
      bool sL = blackLeftSide(lB, rB), sR = blackRightSide(lB, rB);
      if (irSwap) { bool t = sL; sL = sR; sR = t; }
      if (sL) lockL = true;                       // this side has reached the line
      if (sR) lockR = true;
      float rot = fabs(wrapPi(th - legTh));
      // right sides: the 2nd IR reaches the line after turning about the
      // approach angle. Wrong sides: it never does and the robot keeps turning.
      bool away = lockL != lockR && rot > rad(45);
      if (!swapTried && !sqStartSet) { sqStartTh = legTh; sqStartSet = true; }
      if (away && !swapTried) {                   // wrong sides: it turns away - flip, try again
        swapTried = true; irSwap = !irSwap; wheels(0, 0); sqRefaceFirst = true;
        Serial.print("SQUARE: turned away - retrying. Set IR_SIDES_FLIP = ");
        Serial.println(IR_SIDES_FLIP ? "false" : "true");
        stepTo(S_SQ_BACK);
        break;
      }
      if (lockL && lockR && settleT == 0) settleT = millis();
      bool timeout = (!lockL && !lockR && fwd - legStart > 15) || millis() - stepT > 6000 || rot > rad(80);
      if ((settleT && millis() - settleT > SQUARE_SETTLE_MS) || timeout) {
        wheels(0, 0);
        setPoseFromIR(SQ_X, SQ_Y, SQ_HEADING);
        Serial.println(settleT ? "SQUARED - position known" : "SQUARE TIMEOUT - using crossing");
        if (RUN_MODE == 5 || RUN_MODE == 6) { tick(); motorsOff(); stepTo(S_TEACH); return; }
        stepTo(S_FACE_T1);
      } else {                                    // white: forward, black: slowly back
        float vl = sL ? -CREEP * 0.5f : (lockL ? CREEP * 0.5f : CREEP);
        float vr = sR ? -CREEP * 0.5f : (lockR ? CREEP * 0.5f : CREEP);
        wheels(vl, vr);
      }
      break;
    }

    // ---------- 6 T1: one turn, drive, drop
    case S_FACE_T1:
      if (face(deg(atan2(T1_Y - y, T1_X - x)))) stepTo(S_TO_T1);
      break;

    case S_TO_T1:
      if (goTo(T1_X, T1_Y)) { servoT = 0; stepTo(S_DROP); }
      break;

    case S_DROP:                                  // non-blocking: odometry keeps running
      wheels(0, 0);
      if (servoT == 0) { servo.write(SERVO_DROP); servoT = millis(); Serial.println("DROP"); }
      else if (millis() - servoT > 600) { servo.write(SERVO_HOLD); stepTo(S_FACE_A); }
      break;

    // ---------- 7 TO A: face right, find the line, turn left onto it
    case S_FACE_A:
      if (face(0)) stepTo(S_TO_A);
      break;

    case S_TO_A:
      if (black && fwd - legStart > A_IGNORE_CM) stepTo(S_A_ONTO);   // not the T1 ring
      else if (fwd - legStart > 60) { Serial.println("NO LINE AT A"); stepTo(S_DONE); }
      else hold(0, SLOW);
      break;

    case S_A_ONTO:
      if (fwd - legStart >= IR_FORWARD_CM) { wheels(0, 0); stepTo(S_A_TURN); }
      else hold(0, SLOW * 0.6);
      break;

    case S_A_TURN: {
      float turned = wrapPi(th - legTh);
      if (turned > rad(30) && (lB || rB)) { wheels(0, 0); resetFollower(); stepTo(S_FOLLOW); }
      else if (turned > rad(150)) { Serial.println("LINE LOST AT A"); stepTo(S_DONE); }
      else wheels(-TURN * 0.6, TURN * 0.6);
      break;
    }

    // ---------- 8 FOLLOW to the end
    case S_FOLLOW: {
      float along = fwd - legStart;               // forward distance only, from A
      bool test = RUN_MODE == 7;
      bool slowZone = !test && ((along > PASSAGE_FROM_CM && along < PASSAGE_TO_CM) || along > HAIRPIN_AFTER_CM);
      bool forkZone = !test && along > FORK_AT_CM - 10 && along < FORK_AT_CM + 20;
      bool endZone  = !test && (along > LINE_LENGTH_CM - 20 ||
                                (hairpinAt >= 0 && along - hairpinAt > HAIRPIN_TO_END_CM - 12));
      bool l = lB, rr = rB;
      if (forkZone) {                             // keep left: ignore the RIGHT-side IR
        if (FORK_KEEP_LEFT == leftIrOnRight) l = false; else rr = false;
      }
      lineStep(l, rr, slowZone ? SLOW_ZONE : 1.0f, !forkZone && !endZone);
      if (test) break;
      recordHeading();
      if (hairpinAt < 0 && along > HAIRPIN_AFTER_CM && turnedOver(12) > rad(100)) {
        hairpinAt = along; Serial.println("HAIRPIN");
      }
      if ((hairpinAt >= 0 && along - hairpinAt >= HAIRPIN_TO_END_CM) || along > LINE_LENGTH_CM + 25) {
        wheels(0, 0);
        setPoseFromIR(LINE_END_X, LINE_END_Y, LINE_END_HEADING);   // reset (app 3)
        Serial.println("LINE END - position reset");
        stepTo(S_GO_MOUTH);
      }
      break;
    }

    // ---------- 9 POCKET
    case S_GO_MOUTH:
      if (goTo(MOUTH_X, MOUTH_Y)) stepTo(S_FACE_POCKET);
      break;

    case S_FACE_POCKET:
      if (face(POCKET_HEADING)) stepTo(S_POCKET);
      break;

    case S_POCKET: {                              // centred by RIGHT, stopped by FRONT
      float stopAt = T2_FROM_WALL_CM + POINTER_BEYOND_SENSOR_CM;   // 10 cm
      if ((f > 0 && f <= stopAt) || fwd - legStart > 35 || stalled()) {
        wheels(0, 0); Serial.print("AT T2, front "); Serial.println(f);
        servoT = 0; stepTo(S_POINTER); break;
      }
      float h = POCKET_HEADING;
      if (r > 0 && r < 40) h -= constrain((r - (POCKET_CENTRE_TO_RAIL_CM - US_SIDE_CM)) * 1.5f, -15.0f, 15.0f);
      hold(h, (f > 0 && f < stopAt + 10) ? SLOW * 0.6 : SLOW);
      break;
    }

    case S_POINTER:
      wheels(0, 0);
      if (servoT == 0) { servo.write(SERVO_POINTER); servoT = millis(); }
      else if (millis() - servoT > 500) stepTo(S_DONE);
      break;

    case S_DONE:
      wheels(0, 0); tick(); motorsOff();
      Serial.print("RUN FINISHED at "); Serial.print((millis() - runStart) / 1000.0, 1); Serial.println(" s");
      while (true) delay(1000);

    default: break;
  }

  tick();
  if (Serial && millis() - lastPrint > 250) {
    lastPrint = millis();
    Serial.print(NAMES[step]); Serial.print(" x "); Serial.print(x, 0);
    Serial.print(" y "); Serial.print(y, 0); Serial.print(" h "); Serial.print(deg(th), 0);
    Serial.print(" | F "); Serial.print(f, 0); Serial.print(" R "); Serial.print(r, 0);
    Serial.print(" | IR "); Serial.print(lB); Serial.print(rB);
    if (searchPhase) Serial.print(" SEARCH");
    if (step == S_FOLLOW) { Serial.print(" | along "); Serial.print(fwd - legStart, 0); }
    Serial.print(" | L "); Serial.print(L.target, 0); Serial.print("/"); Serial.print(L.rate, 0);
    Serial.print(" R "); Serial.print(R.target, 0); Serial.print("/"); Serial.print(R.rate, 0);
    Serial.print(" | t "); Serial.println((millis() - runStart) / 1000.0, 1);
  }
}
