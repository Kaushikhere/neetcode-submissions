/*
  ==========================================
       IM COMPETITION 1 - APPROACH 3 "SAFE"
       short version with the team's wall rules
  ==========================================

  SAFETY RULES (always on while driving, team's idea, tuned)
    FRONT < 12 cm  -> back up until FRONT >= 27 cm (room to turn),
                      then turn 35 deg away from the nearer side
    RIGHT < 27 cm  -> steer left; when RIGHT is back over 27 cm,
                      return to the planned heading
    WHEELS STUCK   -> (motors on but encoders not moving for 0.5 s)
                      same as FRONT < 12: back up and turn away
    BLACK BORDER   -> black is NOT a line if a wall is closer than
                      15 cm ahead, or if the IRs are at the board edge

  THE RUN
    1. ROOM      If a wall is closer than 27 cm ahead, back up.
    2. OPEN SIDE Spin once; the direction with the longest open view
                 ahead (no wall within 80 cm) is up the field.
    3. SQUARE    Small turn left/right reading the RIGHT sensor: it is
                 square to the right wall where the reading is
                 shortest -> exact heading and x.
    4. UP        Drive up the field (right rule on) until the IRs see
                 the bottom of the black loop -> exact y from the map.
    5. T1        Turn to T1, drive there, servo drop.
    6. LINE      Go back just below the loop's bottom edge, drive up
                 to it, and when the IRs see BLACK rotate CLOCKWISE
                 (turn right) onto the line. From there the line runs
                 along the bottom, up the right side and on to T2:
                 follow it to its end.
    7. T2        Into the pocket, centred with RIGHT, stop with FRONT.
    Everything stops at 88 s.

  PINS
    Right wheel : ENA GP0, IN1 GP1, IN2 GP2
    Left wheel  : IN3 GP3, IN4 GP4, ENB GP5
    Ultrasonic  : RIGHT TRIG GP6 / ECHO GP7, FRONT TRIG GP8 / ECHO GP9
    Servo GP10, encoders right GP12/13 left GP14/15, IR left GP16 right GP17

  BEFORE ANYTHING: set COUNTS_PER_CM and WHEEL_BASE_CM (RUN_MODE 1 / 2).
  If they are wrong, every turn is wrong and the robot drives at walls.
*/

#include <Servo.h>

// ---------------- RUN MODE ----------------
// 0 = full run
// 1 = COUNT TEST: push robot 100 cm by hand, read counts -> COUNTS_PER_CM
// 2 = TURN TEST: robot spins 360 deg -> too far: raise WHEEL_BASE_CM
// 3 = SAFETY TEST: just drives forward with the safety rules on
int RUN_MODE = 3;

// ---------------- CALIBRATION ----------------
float COUNTS_PER_CM = 12.0;
float WHEEL_BASE_CM = 15.0;
float US_SIDE_CM = 10.0;    // right sensor face, right of turning point
float US_FRONT_CM = 12.0;   // front sensor face, ahead of turning point
float IR_FORWARD_CM = 8.0;  // IRs, ahead of turning point
float POINTER_BEYOND_SENSOR_CM = 2.0;
int SERVO_CLOSED = 0, SERVO_OPEN = 90;

// ---------------- SAFETY ----------------
float FRONT_STOP_CM = 12;   // closer than this: back up
float FRONT_ROOM_CM = 27;   // back up until this much room
float RIGHT_KEEP_CM = 27;   // keep the right wall at least this far
float WALL_SEEN_CM  = 15;   // black with a wall this close = border
float OPEN_CM       = 80;   // further than this = open field

// ---------------- MAP (practice board, cm) ----------------
float T1_X = 58.4, T1_Y = 97.0;
float T2_FROM_WALL_CM = 3.0;
float POCKET_HEADING = -62.7;
float POCKET_MOUTH_X = 39.1, POCKET_MOUTH_Y = 202.6;
float POCKET_CENTRE_TO_RAIL_CM = 19.6;
float LINE_END_X = 50.5, LINE_END_Y = 202.0, LINE_END_HEADING = -22.0;
float FORK_AT_CM = 125, LINE_LENGTH_CM = 228;  // from where the loop's right side
float JOIN_X = 97.2;                            // starts (x 97.2, y 97)

// When the IRs find the line: turn this way onto it, by this much.
// CLOCKWISE (true) = turn right. Coming up from below the loop, turning
// right leads along the bottom and up the right side towards T2.
bool  LINE_TURN_CLOCKWISE = true;
float LINE_TURN_DEG = 90;
float BELOW_LOOP_CM = 12;       // how far below the loop edge to restart
// bottom half of the loop round T1 (x increasing)
const int LB_N = 10;
float LB_X[LB_N] = { 16.2, 20.6, 28.4, 38.7, 52.1, 65.0, 77.8, 85.9, 91.9, 98.9 };
float LB_Y[LB_N] = { 108.3, 91.2, 78.6, 68.9, 63.7, 63.4, 66.4, 73.4, 83.7, 101.2 };

// ---------------- SPEEDS (counts per 50 ms, ~60 = full) ----------------
float DRIVE = 20, SLOW = 10, TURN = 9, BACK = 12;
int STRAIGHT_L = 20, STRAIGHT_R = 110;            // your line speeds
int LBLACK_L = 10, LBLACK_R = 250;               // left IR black
int RBLACK_L = 250, RBLACK_R = 10;               // right IR black
int FORK_IGNORE = 0;                             // 0 ignore right IR at fork

// ---------------- PINS ----------------
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

// ---------------- TYPES (before functions, for the IDE) ----------------
struct Wheel { float target = 0, rate = 0, integ = 0, duty = 0; int dir = 1; bool brake = true; long last = 0; };
enum Step { S_ROOM, S_FIND_OPEN, S_SQ_START, S_SQUARE, S_FACE_UP, S_UP, S_FACE_T1, S_GO_T1,
            S_DROP, S_BELOW, S_FACE_LINE, S_FIND_LINE, S_ONTO, S_TURN_ONTO, S_FOLLOW,
            S_GO_MOUTH, S_FACE_POCKET, S_POCKET, S_DONE };

Wheel L, R;
Servo servo;
Step step = S_ROOM;
float x = 61, y = 30, th = 0;          // position (cm) and heading (rad)
float travelled = 0;
long oL = 0, oR = 0;
unsigned long runStart, lastTick, lastPrint, stuckSince = 0;

// escape (safety) state
int escape = 0;                        // 0 none, 1 backing, 2 turning
float escStart = 0, escTurnTo = 0;

// scans
float openBin[72];                     // front reading per 5 deg
float spun = 0, lastTh = 0;
float sqTh[120], sqR[120]; int sqN = 0;
float legStart = 0, whiteFrom = -1, plannedDeg = 90;
float hitX = 58, preLen = 0, turnToDeg = 0;     // where the loop bottom was met

volatile long rawL = 0, rawR = 0;
void isrL() { rawL += (digitalRead(ENC_L_A) != digitalRead(ENC_L_B)) ? 1 : -1; }
void isrR() { rawR += (digitalRead(ENC_R_A) != digitalRead(ENC_R_B)) ? 1 : -1; }
long cntL() { noInterrupts(); long c = rawL; interrupts(); return c * LEFT_SIGN; }
long cntR() { noInterrupts(); long c = rawR; interrupts(); return c * RIGHT_SIGN; }

float wrapPi(float a) { while (a > PI) a -= 2 * PI; while (a < -PI) a += 2 * PI; return a; }
float rad(float d) { return d * PI / 180; }
float deg(float r) { return r * 180 / PI; }
float pwmToRate(int p) { return (255 - constrain(p, 0, 255)) / 255.0 * MAX_RATE; }

float readUS(int t, int e) {
  digitalWrite(t, LOW); delayMicroseconds(2);
  digitalWrite(t, HIGH); delayMicroseconds(10); digitalWrite(t, LOW);
  long us = pulseIn(e, HIGH, 12000);
  return us ? us * 0.0343 / 2 : -1;           // -1 = nothing within ~2 m
}

// ---------------- MOTORS ----------------
void updateWheel(Wheel &w, long c) {
  w.rate = (c - w.last) * 2.0; w.last = c;     // counts per 50 ms (tick = 25 ms)
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
  bool fwd = w.dir > 0;
  digitalWrite(a, fwd ? fwdA : !fwdA);
  digitalWrite(b, fwd ? !fwdA : fwdA);
  analogWrite(en, (int)(255 - w.duty * 255));   // inverted PWM
}

void motorsOff() { analogWrite(ENA, 255); analogWrite(ENB, 255);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }
void wheels(float l, float r) { L.target = l; R.target = r; }

void tick() {
  long cl = cntL(), cr = cntR();
  float dL = (cl - oL) / COUNTS_PER_CM, dR = (cr - oR) / COUNTS_PER_CM;
  oL = cl; oR = cr;
  float d = (dL + dR) / 2, dTh = (dR - dL) / WHEEL_BASE_CM;
  x += d * cos(th + dTh / 2); y += d * sin(th + dTh / 2); th = wrapPi(th + dTh);
  travelled += fabs(d);
  updateWheel(L, cl); updateWheel(R, cr);
  outMotor(R, ENA, IN1, IN2, LOW);              // right: forward = IN1 LOW
  outMotor(L, ENB, IN3, IN4, HIGH);             // left:  forward = IN3 HIGH
}

// ---------------- MOVES ----------------
bool face(float hDeg) {                          // spin on the spot
  float e = wrapPi(rad(hDeg) - th);
  if (fabs(e) < rad(2)) { wheels(0, 0); return true; }
  float s = fabs(e) < rad(15) ? TURN * 0.5 : TURN;
  if (e > 0) wheels(-s, s); else wheels(s, -s);
  return false;
}

void hold(float hDeg, float v) {                 // drive along a heading
  float e = wrapPi(rad(hDeg) - th);
  float s = constrain(25 * e, -v, v);
  wheels(v - s, v + s);
}

bool goTo(float tx, float ty) {
  float dx = tx - x, dy = ty - y, dist = sqrt(dx * dx + dy * dy);
  if (dist < 1 || (dist < 3 && dx * cos(th) + dy * sin(th) < 0)) { wheels(0, 0); return true; }
  float h = deg(atan2(dy, dx));
  if (fabs(wrapPi(rad(h) - th)) > rad(30)) { face(h); return false; }
  hold(h, dist > 15 ? DRIVE : max(SLOW * 0.6f, SLOW * dist / 15));
  return false;
}

// ---------------- SAFETY ----------------
bool stalled() {                                // motors asked to move, wheels not moving
  bool asking = (fabs(L.target) > 3 && !L.brake) || (fabs(R.target) > 3 && !R.brake);
  bool moving = fabs(L.rate) > 1 || fabs(R.rate) > 1;
  if (!asking || moving) { stuckSince = 0; return false; }
  if (stuckSince == 0) stuckSince = millis();
  return millis() - stuckSince > 500;
}

void startEscape() { escape = 1; escStart = travelled; stuckSince = 0; Serial.println("SAFETY: backing up"); }

void runEscape(float f, float r) {
  if (escape == 1) {                            // back up until there is room
    if ((f < 0 || f >= FRONT_ROOM_CM) || travelled - escStart > 25 || stalled()) {
      // turn away from the nearer side (right close -> turn left)
      escTurnTo = deg(th) + ((r > 0 && r < 30) ? 35 : -35);
      escape = 2; wheels(0, 0);
    } else wheels(-BACK, -BACK);
  } else if (face(escTurnTo) || stalled()) { escape = 0; stuckSince = 0; Serial.println("SAFETY: clear"); }
}

bool atEdge() {                                  // IRs over the board's black border?
  float sx = x + IR_FORWARD_CM * cos(th), sy = y + IR_FORWARD_CM * sin(th);
  return sx < 7 || sx > 115 || sy < 7 || sy > 237;
}

float loopBottomY(float sx) {                    // y of the loop's lower edge at sx
  for (int i = 0; i < LB_N - 1; i++)
    if (sx >= LB_X[i] && sx <= LB_X[i + 1])
      return LB_Y[i] + (LB_Y[i + 1] - LB_Y[i]) * (sx - LB_X[i]) / (LB_X[i + 1] - LB_X[i]);
  return -1;
}

float lenToJoin(float sx) {                      // line length from sx to the right side
  float len = 0, px = sx, py = loopBottomY(sx);
  if (py < 0) return 0;
  for (int i = 0; i < LB_N; i++) {
    if (LB_X[i] <= sx) continue;
    float nx = min(LB_X[i], JOIN_X), ny = loopBottomY(nx);
    len += sqrt((nx - px) * (nx - px) + (ny - py) * (ny - py));
    px = nx; py = ny;
    if (nx >= JOIN_X) break;
  }
  return len;
}

// ---------------- SETUP ----------------
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
  servo.attach(SERVO_PIN); servo.write(SERVO_CLOSED);
  for (int i = 0; i < 72; i++) openBin[i] = 0;
  delay(2000);
  L.last = oL = cntL(); R.last = oR = cntR();
  runStart = lastTick = millis();
}

// ---------------- LOOP ----------------
void loop() {
  if (RUN_MODE == 1) {
    if (millis() - lastPrint > 300) { lastPrint = millis();
      Serial.print("counts L "); Serial.print(cntL()); Serial.print("  R "); Serial.println(cntR()); }
    return;
  }
  if (millis() - runStart > 88000 && step != S_DONE) { step = S_DONE; Serial.println("88 s - STOP"); }
  if (millis() - lastTick < 25) return;
  lastTick = millis();

  float f = readUS(F_TRIG, F_ECHO);
  float r = readUS(R_TRIG, R_ECHO);
  bool lB = digitalRead(LEFT_IR) == ON_BLACK, rB = digitalRead(RIGHT_IR) == ON_BLACK;
  bool wallAhead = f > 0 && f < WALL_SEEN_CM;
  bool localised = step >= S_UP;
  bool line = (lB || rB) && !wallAhead && !(localised && atEdge());   // not the border

  if (RUN_MODE == 2) {                            // spin 360 test
    spun += fabs(wrapPi(th - lastTh)); lastTh = th;
    if (spun >= 2 * PI) { wheels(0, 0); tick(); motorsOff(); Serial.println("360 done"); while (true) delay(1000); }
    wheels(-TURN, TURN); tick(); return;
  }

  // ---- safety layer. Near T2 (pocket walls all round) only the stuck
  //      check is used: the walls there are expected, and at the very end
  //      driving up to the wall is the goal.
  if (escape) { runEscape(f, r); tick(); return; }
  bool drivingForward = L.target > 0 && R.target > 0;
  bool frontTooClose = drivingForward && f > 0 && f < FRONT_STOP_CM && step < S_GO_MOUTH;
  if (step < S_POCKET && (frontTooClose || stalled())) {
    startEscape(); tick(); return;
  }

  if (RUN_MODE == 3) {                            // safety test: forward, right rule on
    float h = deg(th) + ((r > 0 && r < RIGHT_KEEP_CM) ? (RIGHT_KEEP_CM - r) * 1.5 : 0);
    hold(h, DRIVE * 0.6); tick(); return;
  }

  switch (step) {
    case S_ROOM:                                  // room to spin: 20 cm ahead
      if (f > 0 && f < FRONT_ROOM_CM && travelled < 20) wheels(-BACK, -BACK);
      else { wheels(0, 0); spun = 0; lastTh = th; step = S_FIND_OPEN; }
      break;

    case S_FIND_OPEN: {                           // spin once, note open directions
      int b = ((int)lround(deg(th) / 5) % 72 + 72) % 72;
      if (f < 0 || f > OPEN_CM) openBin[b] = 1; else if (openBin[b] == 0) openBin[b] = -1;
      spun += fabs(wrapPi(th - lastTh)); lastTh = th;
      if (spun < 2 * PI + rad(10)) { wheels(-TURN * 0.7, TURN * 0.7); break; }
      // longest run of open bins = up the field
      int bestStart = 0, bestLen = 0;
      for (int s = 0; s < 72; s++) {
        int n = 0; while (n < 72 && openBin[(s + n) % 72] == 1) n++;
        if (n > bestLen) { bestLen = n; bestStart = s; }
      }
      plannedDeg = (bestStart + bestLen / 2.0) * 5;   // odometry heading of "up"
      Serial.print("OPEN SIDE at "); Serial.print(plannedDeg); Serial.print(" deg, width "); Serial.println(bestLen * 5);
      step = S_SQ_START;
      break;
    }

    case S_SQ_START:                              // sensor sweep of the right wall
      if (face(plannedDeg - 30)) { sqN = 0; step = S_SQUARE; }
      break;

    case S_SQUARE:
      if (sqN < 120 && r > 0) { sqTh[sqN] = th; sqR[sqN] = r; sqN++; }
      if (wrapPi(th - rad(plannedDeg)) < rad(30)) { wheels(-TURN * 0.4, TURN * 0.4); break; }
      {
        float m = 999; for (int i = 0; i < sqN; i++) m = min(m, sqR[i]);
        float sum = 0; int n = 0;
        for (int i = 0; i < sqN; i++) if (sqR[i] <= m + 1.5) { sum += wrapPi(sqTh[i] - rad(plannedDeg)); n++; }
        float squareTh = rad(plannedDeg) + (n ? sum / n : 0);   // here the robot faced straight up
        if (m < 120) {
          th = wrapPi(th - squareTh + rad(90));
          x = 122 - m - US_SIDE_CM;
        } else th = wrapPi(th - rad(plannedDeg) + rad(90));   // right wall too far: trust open side
        y = 30;
        Serial.print("SQUARED: x "); Serial.print(x); Serial.print("  heading "); Serial.println(deg(th));
      }
      step = S_FACE_UP;
      break;

    case S_FACE_UP:
      if (face(90)) { legStart = travelled; step = S_UP; }
      break;

    case S_UP: {                                  // up the field until the loop
      float h = 90 + ((r > 0 && r < RIGHT_KEEP_CM) ? min(30.0f, (RIGHT_KEEP_CM - r) * 1.5f) : 0);
      if (line) {
        float sx = x + IR_FORWARD_CM * cos(th);
        float ly = loopBottomY(sx);
        if (ly > 0) { y = ly - IR_FORWARD_CM * sin(th); hitX = sx; }
        Serial.print("LOOP EDGE: y set to "); Serial.println(y);
        step = S_FACE_T1;
      } else if (travelled - legStart > 150) { Serial.println("NO LOOP - going on dead reckoning"); step = S_FACE_T1; }
      else hold(h, DRIVE);
      break;
    }

    case S_FACE_T1:
      if (face(deg(atan2(T1_Y - y, T1_X - x)))) step = S_GO_T1;
      break;

    case S_GO_T1:
      if (goTo(T1_X, T1_Y)) step = S_DROP;
      break;

    case S_DROP:
      wheels(0, 0); tick();
      servo.write(SERVO_OPEN); delay(600); servo.write(SERVO_CLOSED);
      Serial.println("DROPPED AT T1");
      step = S_BELOW;
      return;

    case S_BELOW: {                               // just below the loop's bottom edge
      float by = loopBottomY(hitX);
      if (by < 0) by = 63;
      if (goTo(hitX, by - BELOW_LOOP_CM)) step = S_FACE_LINE;
      break;
    }

    case S_FACE_LINE:
      if (face(90)) { legStart = travelled; step = S_FIND_LINE; }
      break;

    case S_FIND_LINE:                             // up to the loop edge
      if (line) { legStart = travelled; step = S_ONTO; Serial.println("BLACK - turning onto the line"); }
      else if (travelled - legStart > BELOW_LOOP_CM + 20) { Serial.println("LINE NOT FOUND"); step = S_DONE; }
      else hold(90, SLOW);
      break;

    case S_ONTO:                                  // put the turning point on the line
      if (travelled - legStart >= IR_FORWARD_CM) {
        turnToDeg = deg(th) + (LINE_TURN_CLOCKWISE ? -LINE_TURN_DEG : LINE_TURN_DEG);
        step = S_TURN_ONTO;
      } else hold(90, SLOW * 0.6);
      break;

    case S_TURN_ONTO:                             // rotate clockwise onto the line
      if (face(turnToDeg)) {
        preLen = lenToJoin(x);                    // line from here to the right side
        legStart = travelled; whiteFrom = -1;
        Serial.println("ON THE LINE - FOLLOWING");
        step = S_FOLLOW;
      }
      break;

    case S_FOLLOW: {
      float along = travelled - legStart - preLen;  // measured from the right side
      bool l = lB, rr = rB;
      if (along > FORK_AT_CM - 10 && along < FORK_AT_CM + 20) { if (FORK_IGNORE == 0) rr = false; else l = false; }
      if (l && !rr) wheels(pwmToRate(LBLACK_L), pwmToRate(LBLACK_R));
      else if (rr && !l) wheels(pwmToRate(RBLACK_L), pwmToRate(RBLACK_R));
      else wheels(pwmToRate(STRAIGHT_L), pwmToRate(STRAIGHT_R));
      if (!lB && !rB) { if (whiteFrom < 0) whiteFrom = along; } else whiteFrom = -1;
      if ((along > LINE_LENGTH_CM - 15 && whiteFrom >= 0 && along - whiteFrom > 6) || along > LINE_LENGTH_CM + 25) {
        th = rad(LINE_END_HEADING);
        x = LINE_END_X - IR_FORWARD_CM * cos(th);
        y = LINE_END_Y - IR_FORWARD_CM * sin(th);
        Serial.println("LINE END");
        step = S_GO_MOUTH;
      }
      break;
    }

    case S_GO_MOUTH:
      if (goTo(POCKET_MOUTH_X, POCKET_MOUTH_Y)) step = S_FACE_POCKET;
      break;

    case S_FACE_POCKET:
      if (face(POCKET_HEADING)) { legStart = travelled; step = S_POCKET; }
      break;

    case S_POCKET: {                              // centred by RIGHT, stopped by FRONT
      float stopAt = T2_FROM_WALL_CM + POINTER_BEYOND_SENSOR_CM;
      if ((f > 0 && f <= stopAt) || travelled - legStart > 35 || stalled()) {
        wheels(0, 0); Serial.print("AT T2, front "); Serial.println(f); step = S_DONE; break;
      }
      float h = POCKET_HEADING;
      if (r > 0 && r < 40) h -= constrain((r - (POCKET_CENTRE_TO_RAIL_CM - US_SIDE_CM)) * 1.5f, -15.0f, 15.0f);
      hold(h, (f > 0 && f < stopAt + 10) ? SLOW * 0.6 : SLOW);
      break;
    }

    case S_DONE:
      wheels(0, 0); tick(); motorsOff();
      Serial.println("RUN FINISHED");
      while (true) delay(1000);
  }

  tick();
  if (millis() - lastPrint > 250) {
    lastPrint = millis();
    Serial.print("step "); Serial.print((int)step); Serial.print(" x "); Serial.print(x, 0);
    Serial.print(" y "); Serial.print(y, 0); Serial.print(" h "); Serial.print(deg(th), 0);
    Serial.print(" | F "); Serial.print(f, 0); Serial.print(" R "); Serial.print(r, 0);
    Serial.print(" | IR "); Serial.print(lB); Serial.print(rB); Serial.print(line ? " line" : "");
    Serial.print(" | L "); Serial.print(L.target, 0); Serial.print("/"); Serial.print(L.rate, 0);
    Serial.print(" R "); Serial.print(R.target, 0); Serial.print("/"); Serial.println(R.rate, 0);
  }
}
