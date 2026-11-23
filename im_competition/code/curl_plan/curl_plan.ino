/*
  ==========================================================
       IM COMPETITION 1 - CURL PLAN
       (the 6-stage plan from the screen recording)
  ==========================================================

  Never work out the start. Head for the open side, catch the curl
  around T1, ride it backwards to its dead end - a point the robot
  cannot mistake - and measure everything from there.

  STAGE 1  FIND THE OPEN SIDE
    One spin. Echoes come back from the three start walls; the side
    with no echo is up-field. Only open or not, no map. Face the open
    side - it only needs to be close.
  STAGE 2  CATCH THE CURL
    Drive straight. The first black line up-field is always the curl
    around T1 (about 80 cm wide). IR hits black: width check on the
    encoders - a 2 to 4 cm line, not the 5 cm border band.
  STAGE 3  RIDE TO THE DEAD END
    Turn left and ride the curl backwards: round the bottom, up the
    left side, across the top. The line ends and does not come back:
    the dead end. Stop at once (the main line is only ~9 cm ahead):
    after 3 cm of white on the top it stops and looks 25 deg left and
    right on the spot - nothing there = the line has really ended.
    Known point + known heading = exact pose.
  STAGE 4  DELIVER AT T1
    From the dead end, T1 is one fixed hop (HOP_TURN_DEG, HOP_CM -
    measure it once with a tape). STOP with the axle midpoint on T1 and
    wait T1_WAIT_MS (no servo for now - this is where the drop goes later).
    Face right, drive right until the IR sees black at A (about 39 cm).
  STAGE 5  FOLLOW THE LINE
    Roll onto the line at A, turn left onto it and follow it up the
    right side. Passage: go slowly. Fork: keep left. Hairpin (about
    140 deg left): slow down, pivot left if the line is lost.
  STAGE 6  INTO THE T2 POCKET
    Square up along the rails (heading about -63 deg), swing to the
    middle of the pocket with the right ultrasonic, stop at the pointer
    length from the rung (no servo for now: it just stops there).

  The start pose is never computed. The dead end resets everything, so
  everything after it is the same from any start. Stops at 88 s.

  RUN MODES (RUN_MODE below)
    0 FULL RUN
    1 COUNT TEST  motors off. Push the robot straight 100 cm by hand,
                  copy the printed COUNTS_PER_CM_L / _R.
    2 TURN TEST   motors off. Turn the robot by hand ONE full turn to the
                  LEFT on the spot, copy the printed WHEEL_BASE_CM.
    3 SENSOR TEST motors off. Prints FRONT, RIGHT and both IRs.
    4 LINE TEST   put it on a line: it only follows the line.
    5 DEAD END    stages 1-3, then stops at the dead end and prints.
    6 MOTOR TEST  no encoders, no IRs: both wheels forward at your straight
                  PWM for 3 s, then backward 3 s, repeating. Prints the
                  encoder counts. If the wheels do not turn here, the
                  problem is power or wiring, not the code.

  PINS (your connections)
    Right wheel : ENA GP0, IN1 GP1, IN2 GP2   (L298N OUT1/OUT2)
    Left wheel  : IN3 GP3, IN4 GP4, ENB GP5   (L298N OUT3/OUT4)
    Ultrasonic  : RIGHT TRIG GP6 / ECHO GP7, FRONT TRIG GP8 / ECHO GP9
    Encoders right GP12/GP13, left GP14/GP15   (servo GP10 not used now)
    IR left GP16, right GP17
    PWM through the optocouplers is inverted: 0 = full, 255 = stop.
*/


// ===================== RUN MODE =====================
int RUN_MODE = 0;

// ===================== CALIBRATION (modes 1 and 2) =====================
// These matter for the spin, the T1 hop and every turn. 12 / 15 are only
// placeholders - measure them with modes 1 and 2.
float COUNTS_PER_CM_L = 12.0;
float COUNTS_PER_CM_R = 12.0;
float WHEEL_BASE_CM   = 15.0;
float IR_FORWARD_CM   = 8.0;    // middle of the two IRs, ahead of the axle centre
float IR_GAP_CM       = 4.0;    // distance between the two IRs (width check)
float US_SIDE_CM      = 10.0;   // right ultrasonic face, right of the axle centre
float POINTER_CM      = 7.0;    // pointer tip past the front ultrasonic (stop distance at T2)
unsigned long T1_WAIT_MS = 1000; // stop this long on T1

// ===================== STAGE SETTINGS (practice board) =====================
// Stage 1
float OPEN_CM = 125;            // echo longer than this (or no echo) = open
// Stage 2
float CURL_MAX_CM   = 200;      // no curl after this far -> stop
float LINE_MAX_CM   = 4.5;      // black up to this wide = line (2-4 cm)
float BORDER_CM     = 6.0;      // black this long = the 5 cm border band
// Stage 3
float TOP_TURN_DEG  = 150;      // turned this much right while riding = on the top
float MIN_RIDE_CM   = 60;       // ... and at least this far along the curl
float DEAD_END_CM   = 3;        // white this far on the top -> stop and look for the line
float SCAN_DEG      = 25;       // ... by turning this much left and right on the spot
float DE_HEADING_DEG = -13;     // direction of the curl top at the dead end (0 = right, 90 = up-field)
// Stage 4 (measure once with a tape: robot stopped at the dead end -> T1)
float HOP_TURN_DEG  = -131;     // turn from the dead-end direction to face T1 (- = right)
float HOP_CM        = 33;       // axle travel from the dead-end stop to T1
float A_IGNORE_CM   = 12;       // ignore black this close to T1 (the green ring)
float A_MAX_CM      = 70;       // no line within this -> stop
// Stage 5 (distances along the line from A)
float PASSAGE_FROM_CM = 65, PASSAGE_TO_CM = 130, FORK_AT_CM = 125;
float HAIRPIN_AFTER_CM = 170, HAIRPIN_TO_END_CM = 34, LINE_LENGTH_CM = 228;
bool  FORK_KEEP_LEFT = true;
float LOST_CM = 8;              // both white this far (not on a straight end) -> search
// Stage 6
float LINE_END_X = 50.5, LINE_END_Y = 202.0, LINE_END_HEADING = -22.0;
float MOUTH_X = 36.8, MOUTH_Y = 207.2, POCKET_HEADING = -62.7;
float POCKET_CENTRE_TO_RAIL_CM = 19.6;
float T2_FROM_WALL_CM = 3.0;

// ===================== SPEEDS =====================
// Moves: encoder counts per 50 ms (about 60 = full speed)
float DRIVE = 18, SLOW = 10, TURN = 9, CREEP = 6, BACK = 12;
// Line follower: YOUR WORKING VALUES (0 = full, 255 = stop)
int STRAIGHT_L = 20, STRAIGHT_R = 110;   // both IRs white
int LB_L = 10,  LB_R = 250;              // left IR (GP16) black
int RB_L = 250, RB_R = 10;               // right IR (GP17) black
float SLOW_K = 0.6;                      // speed factor in passage / hairpin / curl top
float FRONT_STOP_CM = 12;                // open driving: wall this close -> back off

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
#define ENC_R_A 12
#define ENC_R_B 13
#define ENC_L_A 14
#define ENC_L_B 15
#define LEFT_IR 16
#define RIGHT_IR 17
#define ON_BLACK HIGH
int LEFT_SIGN = 1, RIGHT_SIGN = -1;
float MAX_RATE = 70;            // counts per 50 ms at full power

// ===================== TYPES (before any function, for the IDE) =====================
struct Wheel { float target = 0, rate = 0, integ = 0, duty = 0; int dir = 1; bool brake = true; long last = 0; };

enum Step { S_SPIN, S_FACE_OPEN, S_TO_CURL, S_WIDTH, S_BORDER, S_ONTO_CURL, S_TURN_CURL,
            S_RIDE, S_DE_BACK, S_HOP_FACE, S_HOP, S_DROP, S_FACE_RIGHT, S_TO_A,
            S_ONTO_A, S_TURN_A, S_FOLLOW, S_GO_MOUTH, S_FACE_POCKET, S_POCKET,
            S_POINTER, S_DONE };
const char* NAMES[] = { "1 SPIN", "1 FACE OPEN", "2 TO CURL", "2 WIDTH CHECK", "2 BORDER",
                        "2 ONTO CURL", "3 TURN ONTO CURL", "3 RIDE", "3 DEAD END",
                        "4 FACE T1", "4 HOP", "4 AT T1", "4 FACE RIGHT", "4 TO A",
                        "5 ONTO A", "5 TURN ONTO LINE", "5 FOLLOW", "6 TO MOUTH",
                        "6 SQUARE UP", "6 POCKET", "6 AT T2", "DONE" };

// ===================== STATE =====================
Wheel L, R;
Step step = S_SPIN;
float x = 0, y = 0, th = 0;          // odometry (cm, rad); the field frame from stage 1
float fwd = 0;                       // signed distance driven
float legStart = 0, legTh = 0;       // fwd / th when the step began
long oL = 0, oR = 0, startL = 0, startR = 0;
unsigned long runStart, lastTick, lastCtl, lastPrint, stuckSince = 0, waitT = 0;

bool irL = false, irR = false;       // black seen since the last tick
bool leftIrOnRight = true;           // from your follower values (setup)

int  escape = 0; float escStart = 0, escTo = 0;   // open-driving safety
bool spinOpen[36]; float spun = 0, prevTh = 0, upDeg = 90;

float lStart, lEnd, rStart, rEnd;    // width check
float lineW = 0;
float rideTurn = 0, rideTh = 0;      // stage 3: total turn while riding (+ = left)
bool  onTop = false;
float whiteFrom = -1, lastBlackTh = 0; int lastSide = 0;   // +1 = line last on the right
float hist[16]; int histN = 0; float histFwd = 0;
int   search = 0; float searchTh = 0, backTo = 0;
float hairpinAt = -1;
int   scan = 0; float scanTh = 0;     // stage 3: looking for the line on the spot

// ===================== SMALL HELPERS =====================
volatile long rawL = 0, rawR = 0;
void isrL() { rawL += (digitalRead(ENC_L_A) != digitalRead(ENC_L_B)) ? 1 : -1; }
void isrR() { rawR += (digitalRead(ENC_R_A) != digitalRead(ENC_R_B)) ? 1 : -1; }
long cntL() { noInterrupts(); long c = rawL; interrupts(); return c * LEFT_SIGN; }
long cntR() { noInterrupts(); long c = rawR; interrupts(); return c * RIGHT_SIGN; }

float wrapPi(float a) { while (a > PI) a -= 2 * PI; while (a < -PI) a += 2 * PI; return a; }
float rad(float d) { return d * PI / 180; }
float deg(float r) { return r * 180 / PI; }
float rate(int pwm) { return (255 - constrain(pwm, 0, 255)) / 255.0 * MAX_RATE; }   // your scale -> speed

void sampleIR() {
  if (digitalRead(LEFT_IR) == ON_BLACK) irL = true;
  if (digitalRead(RIGHT_IR) == ON_BLACK) irR = true;
}
// black on the robot's right / left side
bool rightSide(bool l, bool r) { return leftIrOnRight ? l : r; }
bool leftSide(bool l, bool r)  { return leftIrOnRight ? r : l; }

// HC-SR04, keeps reading the IRs while it waits. -1 = no echo.
float readUS(int trig, int echo, unsigned long timeoutUs) {
  unsigned long t0 = micros();
  while (digitalRead(echo) == HIGH) { sampleIR(); if (micros() - t0 > timeoutUs) return -1; }
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10); digitalWrite(trig, LOW);
  t0 = micros();
  while (digitalRead(echo) == LOW) { sampleIR(); if (micros() - t0 > 4000) return -1; }
  unsigned long s = micros();
  while (digitalRead(echo) == HIGH) { sampleIR(); if (micros() - s > timeoutUs) return -1; }
  return (micros() - s) * 0.0343 / 2;
}

// ===================== MOTORS (encoder speed control + brake) =====================
void wheels(float l, float r) { L.target = l; R.target = r; }

void control(Wheel &w, long c, float dtMs) {
  w.rate = (c - w.last) * 50.0f / dtMs; w.last = c;
  if (w.target == 0) { w.brake = true; w.integ = 0; return; }
  int want = w.target > 0 ? 1 : -1;
  float along = w.rate * want, mag = fabs(w.target);
  if (along > mag + 10 || along < -10) { w.brake = true; w.integ = 0; return; }   // too fast: brake
  w.brake = false; w.dir = want;
  float err = mag - along;
  w.integ = constrain(w.integ + err, -150.0f, 150.0f);
  w.duty = constrain(mag / MAX_RATE + 0.012f * err + 0.002f * w.integ, 0.0f, 1.0f);
}

void drive(Wheel &w, int en, int a, int b, int fwdA) {
  if (w.brake) { digitalWrite(a, LOW); digitalWrite(b, LOW); analogWrite(en, 0); return; }
  bool f = w.dir > 0;
  digitalWrite(a, f ? fwdA : !fwdA);
  digitalWrite(b, f ? !fwdA : fwdA);
  analogWrite(en, (int)(255 - w.duty * 255));      // inverted PWM
}

void motorsOff() {
  analogWrite(ENA, 255); analogWrite(ENB, 255);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void odometry() {
  long cl = cntL(), cr = cntR();
  float dL = (cl - oL) / COUNTS_PER_CM_L, dR = (cr - oR) / COUNTS_PER_CM_R;
  oL = cl; oR = cr;
  float d = (dL + dR) / 2, dTh = (dR - dL) / WHEEL_BASE_CM;
  x += d * cos(th + dTh / 2); y += d * sin(th + dTh / 2); th = wrapPi(th + dTh);
  fwd += d;
}

void tick() {
  odometry();
  unsigned long now = millis();
  float dt = constrain((float)(now - lastCtl), 10.0f, 100.0f);
  lastCtl = now;
  control(L, cntL(), dt); control(R, cntR(), dt);
  drive(R, ENA, IN1, IN2, LOW);                     // right wheel: forward = IN1 LOW
  drive(L, ENB, IN3, IN4, HIGH);                    // left wheel:  forward = IN3 HIGH
}

void go(Step s) {
  step = s; legStart = fwd; legTh = th;
  if (Serial) { Serial.print(">> "); Serial.print(NAMES[s]);
    Serial.print("   heading "); Serial.println(deg(th), 0); }
}

// ===================== MOVES =====================
bool face(float hDeg) {                             // turn on the spot
  float e = wrapPi(rad(hDeg) - th);
  if (fabs(e) < rad(2)) { wheels(0, 0); return true; }
  float s = fabs(e) < rad(15) ? TURN * 0.5 : TURN;
  if (e > 0) wheels(-s, s); else wheels(s, -s);
  return false;
}

void hold(float hDeg, float v) {                    // drive along a heading
  float s = constrain(25 * wrapPi(rad(hDeg) - th), -v, v);
  wheels(v - s, v + s);
}

bool goTo(float tx, float ty) {
  float dx = tx - x, dy = ty - y, d = sqrt(dx * dx + dy * dy);
  if (d < 1.5) { wheels(0, 0); return true; }
  float h = deg(atan2(dy, dx));
  if (fabs(wrapPi(rad(h) - th)) > rad(30)) { face(h); return false; }
  hold(h, d > 15 ? DRIVE : fmaxf(SLOW * 0.6f, SLOW * d / 15));
  return false;
}

void pivot(int dir) { wheels(-dir * TURN * 0.8, dir * TURN * 0.8); }   // +1 = left

// ===================== LINE FOLLOWER =====================
// Your follower. If both IRs stay white for LOST_CM (and search is on):
// pivot toward the side the line was last seen (left at the hairpin),
// then back up to where it was lost and pivot again.
void follow(bool l, bool r, float k, bool allowSearch) {
  if (l || r) { search = 0; whiteFrom = -1; lastBlackTh = th; }
  if (l && !r) { wheels(k * rate(LB_L), k * rate(LB_R)); lastSide = leftIrOnRight ? 1 : -1; return; }
  if (r && !l) { wheels(k * rate(RB_L), k * rate(RB_R)); lastSide = leftIrOnRight ? -1 : 1; return; }
  if (l && r)  { wheels(k * rate(STRAIGHT_L), k * rate(STRAIGHT_R)); return; }

  if (whiteFrom < 0) whiteFrom = fwd;
  if (search == 0) {
    if (!allowSearch || lastSide == 0 || fwd - whiteFrom < LOST_CM) {
      wheels(k * rate(STRAIGHT_L), k * rate(STRAIGHT_R)); return;
    }
    search = 1; searchTh = th; backTo = whiteFrom - 3;
    if (Serial) Serial.println("LINE LOST - pivot toward it");
  }
  int toward = -lastSide;                           // line on the right -> turn right (-)
  float turned = deg(wrapPi(th - searchTh)) * toward;
  if (search == 1) { if (turned < 100) pivot(toward); else search = 2; }
  else if (search == 2) { if (face(deg(searchTh))) search = 3; }
  else if (search == 3) { if (fwd > backTo) wheels(-CREEP, -CREEP); else { search = 4; searchTh = th; } }
  else if (search == 4) { if (turned < 100) pivot(toward); else search = 5; }
  else if (face(deg(searchTh))) { search = 0; lastSide = 0; whiteFrom = fwd;
    if (Serial) Serial.println("LINE NOT FOUND - straight on"); }
}

void rememberHeading() {                            // every 2 cm, the last 32 cm
  if (fabs(fwd - histFwd) < 2) return;
  histFwd = fwd;
  for (int i = 15; i > 0; i--) hist[i] = hist[i - 1];
  hist[0] = th;
  if (histN < 16) histN++;
}

float lineHeading(int n) {                          // average heading over the last n x 2 cm
  if (n > histN) n = histN;
  if (n == 0) return th;
  float sx = 0, sy = 0;
  for (int i = 0; i < n; i++) { sx += cos(hist[i]); sy += sin(hist[i]); }
  return atan2(sy, sx);
}

void startFollow() { search = 0; whiteFrom = -1; lastSide = 0; histN = 0; histFwd = fwd; hairpinAt = -1; }

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

  // Your follower turns TOWARD the IR that sees black. GP16 black slows the
  // right wheel (LB_R 250) = turns right, so GP16 sits on the robot's right.
  leftIrOnRight = LB_R > LB_L;

  delay(2000);
  L.last = oL = startL = cntL(); R.last = oR = startR = cntR();
  runStart = lastTick = lastCtl = millis();
  for (int i = 0; i < 36; i++) spinOpen[i] = false;
  prevTh = th;
  if (Serial && COUNTS_PER_CM_L == 12.0) Serial.println("NOTE: COUNTS_PER_CM is still the 12.0 placeholder - run modes 1 and 2");
  if (Serial) { Serial.print("CURL PLAN - RUN_MODE "); Serial.println(RUN_MODE); }
  if (RUN_MODE == 4) { startFollow(); go(S_FOLLOW); }
  else go(S_SPIN);
}

// ===================== LOOP =====================
void loop() {
  sampleIR();

  if (RUN_MODE == 1 || RUN_MODE == 2) {             // calibration by hand, motors off
    if (millis() - lastPrint > 300) { lastPrint = millis();
      long l = cntL() - startL, r = cntR() - startR;
      if (RUN_MODE == 1) {
        Serial.print("counts L "); Serial.print(l); Serial.print("  R "); Serial.print(r);
        Serial.print("   after 100 cm: COUNTS_PER_CM_L = "); Serial.print(l / 100.0, 2);
        Serial.print("  COUNTS_PER_CM_R = "); Serial.println(r / 100.0, 2);
      } else {
        float dL = l / COUNTS_PER_CM_L, dR = r / COUNTS_PER_CM_R;
        Serial.print("after ONE full turn left: WHEEL_BASE_CM = "); Serial.println((dR - dL) / (2 * PI), 2);
      }
    }
    return;
  }
  if (RUN_MODE == 6) {                              // raw motor test: no speed control at all
    bool forward = ((millis() - runStart) / 3000) % 2 == 0;
    digitalWrite(IN1, forward ? LOW : HIGH);  digitalWrite(IN2, forward ? HIGH : LOW);   // right wheel
    digitalWrite(IN3, forward ? HIGH : LOW);  digitalWrite(IN4, forward ? LOW : HIGH);   // left wheel
    analogWrite(ENA, STRAIGHT_R); analogWrite(ENB, STRAIGHT_L);                         // inverted PWM
    if (millis() - lastPrint > 300) { lastPrint = millis();
      Serial.print(forward ? "FORWARD " : "BACKWARD"); Serial.print("   ENA "); Serial.print(STRAIGHT_R);
      Serial.print(" ENB "); Serial.print(STRAIGHT_L);
      Serial.print("   encoder counts L "); Serial.print(cntL() - startL);
      Serial.print("  R "); Serial.println(cntR() - startR); }
    return;
  }
  if (RUN_MODE != 4 && step != S_DONE && millis() - runStart > 88000) { Serial.println("88 s - STOP"); go(S_DONE); }
  if (millis() - lastTick < 25) return;
  lastTick = millis();

  // ---- sensors (ultrasonics only where they are used)
  bool openDrive = step <= S_BORDER || (step >= S_HOP_FACE && step <= S_TO_A) || step == S_GO_MOUTH;
  bool needF = openDrive || step == S_POCKET || RUN_MODE == 3;
  bool needR = step == S_POCKET || RUN_MODE == 3;
  float f = needF ? readUS(F_TRIG, F_ECHO, 12000) : -1;
  float r = needR ? readUS(R_TRIG, R_ECHO, 8000) : -1;
  sampleIR();
  bool lB = irL, rB = irR; irL = irR = false;

  if (RUN_MODE == 3) {
    motorsOff();
    if (millis() - lastPrint > 200) { lastPrint = millis();
      Serial.print("FRONT "); Serial.print(f, 1); Serial.print("   RIGHT "); Serial.print(r, 1);
      Serial.print("   IR GP16 "); Serial.print(lB ? "BLACK" : "white");
      Serial.print("   IR GP17 "); Serial.println(rB ? "BLACK" : "white"); }
    return;
  }

  // ---- open driving: a wall too close ahead -> back off 10 cm, turn 35 deg
  if (escape) {
    if (escape == 1) { if (escStart - fwd < 10) wheels(-BACK, -BACK); else escape = 2; }
    else if (face(escTo)) escape = 0;
    tick(); return;
  }
  if (openDrive && step != S_SPIN && L.target > 0 && R.target > 0 && f > 0 && f < FRONT_STOP_CM) {
    escape = 1; escStart = fwd; escTo = deg(th) + 35;
    if (Serial) Serial.println("WALL AHEAD - backing off");
    tick(); return;
  }

  switch (step) {
    // ================= STAGE 1: FIND THE OPEN SIDE =================
    case S_SPIN: {                                  // one spin: no echo / far echo = open
      int b = ((int)lround(deg(th) / 10) % 36 + 36) % 36;
      if (f < 0 || f > OPEN_CM) spinOpen[b] = true;
      spun += fabs(wrapPi(th - prevTh)); prevTh = th;
      if (spun < 2 * PI + rad(10)) { wheels(-TURN * 0.7, TURN * 0.7); break; }
      wheels(0, 0);
      int bestS = 0, bestN = 0;                     // longest run of open = up-field
      for (int s = 0; s < 36; s++) {
        int n = 0; while (n < 36 && spinOpen[(s + n) % 36]) n++;
        if (n > bestN) { bestN = n; bestS = s; }
      }
      upDeg = (bestS + (bestN - 1) / 2.0) * 10;
      th = wrapPi(th - rad(upDeg) + rad(90));       // from now on 90 = up-field, 0 = right
      if (Serial) { Serial.print("OPEN SIDE found, "); Serial.print(bestN * 10); Serial.println(" deg wide"); }
      go(S_FACE_OPEN);
      break;
    }

    case S_FACE_OPEN:
      if (face(90)) go(S_TO_CURL);
      break;

    // ================= STAGE 2: CATCH THE CURL =================
    case S_TO_CURL:
      if (lB || rB) {
        lStart = lB ? fwd : -1; rStart = rB ? fwd : -1; lEnd = rEnd = -1;
        go(S_WIDTH);
      } else if (fwd - legStart > CURL_MAX_CM) { Serial.println("NO CURL FOUND"); go(S_DONE); }
      else hold(90, DRIVE);
      break;

    case S_WIDTH: {                                 // measure the black with the encoders
      if (lB && lStart < 0) lStart = fwd;
      if (rB && rStart < 0) rStart = fwd;
      if (!lB && lStart >= 0 && lEnd < 0) lEnd = fwd;
      if (!rB && rStart >= 0 && rEnd < 0) rEnd = fwd;
      bool longBlack = (lB && lStart >= 0 && fwd - lStart > BORDER_CM) || (rB && rStart >= 0 && fwd - rStart > BORDER_CM);
      if (longBlack) { wheels(0, 0); Serial.println("BLACK TOO WIDE - border band"); go(S_BORDER); break; }
      bool lDone = lStart < 0 || lEnd >= 0, rDone = rStart < 0 || rEnd >= 0;
      if ((lDone && rDone) || fwd - legStart > 12) {
        float lenL = lStart >= 0 && lEnd >= 0 ? lEnd - lStart : 99, lenR = rStart >= 0 && rEnd >= 0 ? rEnd - rStart : 99;
        float off = (lStart >= 0 && rStart >= 0) ? fabs(lStart - rStart) : 0;
        float cosA = IR_GAP_CM / sqrt(IR_GAP_CM * IR_GAP_CM + off * off);   // crossing angle
        lineW = fminf(lenL, lenR) * cosA;
        if (Serial) { Serial.print("WIDTH "); Serial.print(lineW, 1); Serial.println(" cm"); }
        if (lineW <= LINE_MAX_CM) go(S_ONTO_CURL);
        else { Serial.println("not a 2-4 cm line - border band"); go(S_BORDER); }
      } else hold(deg(legTh), SLOW);
      break;
    }

    case S_BORDER:                                  // back off the band, turn a little, try again
      if (legStart - fwd < 12) wheels(-BACK, -BACK);
      else if (face(deg(legTh) + 30)) go(S_TO_CURL);
      break;

    case S_ONTO_CURL:                               // axle over the line, then turn left onto it
      if (fwd - legStart >= IR_FORWARD_CM - lineW / 2) { wheels(0, 0); go(S_TURN_CURL); }
      else hold(deg(legTh), SLOW * 0.6);
      break;

    case S_TURN_CURL: {
      float turned = wrapPi(th - legTh);
      if (turned > rad(30) && (lB || rB)) {
        wheels(0, 0); startFollow(); rideTurn = 0; rideTh = th; onTop = false; scan = 0;
        go(S_RIDE);
      } else if (turned > rad(160)) { Serial.println("CURL LOST IN THE TURN - riding anyway"); startFollow(); rideTurn = 0; rideTh = th; go(S_RIDE); }
      else pivot(1);
      break;
    }

    // ================= STAGE 3: RIDE TO THE DEAD END =================
    case S_RIDE: {
      rideTurn += wrapPi(th - rideTh); rideTh = th;  // riding backwards turns RIGHT (-)
      rememberHeading();
      if (!onTop && -rideTurn > rad(TOP_TURN_DEG) && fwd - legStart > MIN_RIDE_CM) {
        onTop = true; histN = 0; Serial.println("ON THE TOP OF THE CURL");
      }
      if (!onTop) { follow(lB, rB, 1.0f, true); break; }

      if (lB || rB) {                               // line still there
        if (scan) { scan = 0; Serial.println("  line still there"); }
        follow(lB, rB, SLOW_K, false);
        break;
      }
      if (whiteFrom < 0) whiteFrom = fwd;
      if (!scan && fwd - whiteFrom >= DEAD_END_CM) { scan = 1; scanTh = th; }
      if (!scan) { follow(lB, rB, SLOW_K, false); break; }
      // both white: stop and look +-SCAN_DEG on the spot. If the line is
      // still there (between the IRs) one IR crosses it. If not: dead end.
      if (scan == 1) { if (face(deg(scanTh) + SCAN_DEG)) scan = 2; }
      else if (scan == 2) { if (face(deg(scanTh) - SCAN_DEG)) scan = 3; }
      else if (face(deg(scanTh))) {
        scan = 0; Serial.println("DEAD END");
        backTo = whiteFrom; go(S_DE_BACK);
      }
      break;
    }

    case S_DE_BACK:                                 // back up so the IRs sit on the dead end
      if (fwd > backTo) { wheels(-CREEP, -CREEP); break; }
      wheels(0, 0);
      // known heading: the robot's angle to the line + the line's direction
      th = wrapPi(rad(DE_HEADING_DEG) + wrapPi(th - lineHeading(4)));
      x = 0; y = 0;                                 // the dead end is the reference point now
      if (Serial) { Serial.print("POSE RESET at the dead end, heading "); Serial.println(deg(th), 1); }
      if (RUN_MODE == 5) { tick(); motorsOff(); Serial.println("MODE 5: stopped at the dead end"); while (true) delay(1000); }
      go(S_HOP_FACE);
      break;

    // ================= STAGE 4: DELIVER AT T1 =================
    case S_HOP_FACE:
      if (face(DE_HEADING_DEG + HOP_TURN_DEG)) go(S_HOP);
      break;

    case S_HOP:                                     // one fixed hop
      if (fwd - legStart >= HOP_CM) { wheels(0, 0); waitT = 0; go(S_DROP); }
      else hold(DE_HEADING_DEG + HOP_TURN_DEG, HOP_CM - (fwd - legStart) > 10 ? DRIVE : SLOW * 0.6);
      break;

    case S_DROP:
      wheels(0, 0);
      if (waitT == 0) { waitT = millis(); Serial.println("AT T1 - stopped"); }   // no servo for now
      else if (millis() - waitT > T1_WAIT_MS) go(S_FACE_RIGHT);
      break;

    case S_FACE_RIGHT:
      if (face(0)) go(S_TO_A);
      break;

    case S_TO_A:                                    // drive right until black at A
      if ((lB || rB) && fwd - legStart > A_IGNORE_CM) go(S_ONTO_A);
      else if (fwd - legStart > A_MAX_CM) { Serial.println("NO LINE AT A"); go(S_DONE); }
      else hold(0, SLOW);
      break;

    // ================= STAGE 5: FOLLOW THE LINE =================
    case S_ONTO_A:                                  // roll onto the line
      if (fwd - legStart >= IR_FORWARD_CM) { wheels(0, 0); go(S_TURN_A); }
      else hold(0, SLOW * 0.6);
      break;

    case S_TURN_A: {                                // turn left onto it
      float turned = wrapPi(th - legTh);
      if (turned > rad(30) && (lB || rB)) { wheels(0, 0); startFollow(); go(S_FOLLOW); }
      else if (turned > rad(160)) { Serial.println("LINE LOST AT A"); go(S_DONE); }
      else pivot(1);
      break;
    }

    case S_FOLLOW: {
      float along = fwd - legStart;
      bool test = RUN_MODE == 4;
      bool slow = !test && ((along > PASSAGE_FROM_CM && along < PASSAGE_TO_CM) || along > HAIRPIN_AFTER_CM);
      bool fork = !test && along > FORK_AT_CM - 10 && along < FORK_AT_CM + 20;
      bool nearEnd = !test && (along > LINE_LENGTH_CM - 20 || (hairpinAt >= 0 && along - hairpinAt > HAIRPIN_TO_END_CM - 12));
      bool l = lB, rr = rB;
      if (fork) {                                   // keep left: ignore the right-side IR
        bool ignoreGP16 = FORK_KEEP_LEFT == leftIrOnRight;
        if (ignoreGP16) l = false; else rr = false;
      }
      follow(l, rr, slow ? SLOW_K : 1.0f, !fork && !nearEnd);
      if (test) break;
      rememberHeading();
      if (hairpinAt < 0 && along > HAIRPIN_AFTER_CM && histN >= 12 && wrapPi(th - hist[11]) > rad(100)) {
        hairpinAt = along; Serial.println("HAIRPIN");
      }
      if ((hairpinAt >= 0 && along - hairpinAt >= HAIRPIN_TO_END_CM) || along > LINE_LENGTH_CM + 25) {
        wheels(0, 0);
        th = wrapPi(rad(LINE_END_HEADING) + wrapPi(th - lineHeading(4)));   // reset at the line end
        x = LINE_END_X - IR_FORWARD_CM * cos(th);
        y = LINE_END_Y - IR_FORWARD_CM * sin(th);
        Serial.println("LINE END - position reset");
        go(S_GO_MOUTH);
      }
      break;
    }

    // ================= STAGE 6: INTO THE T2 POCKET =================
    case S_GO_MOUTH:
      if (goTo(MOUTH_X, MOUTH_Y)) go(S_FACE_POCKET);
      break;

    case S_FACE_POCKET:                             // square up along the rails
      if (face(POCKET_HEADING)) go(S_POCKET);
      break;

    case S_POCKET: {                                // middle of the pocket with RIGHT, stop with FRONT
      float stopAt = T2_FROM_WALL_CM + POINTER_CM;
      if ((f > 0 && f <= stopAt) || fwd - legStart > 35) {
        wheels(0, 0); Serial.print("AT T2, front "); Serial.println(f);
        waitT = 0; go(S_POINTER); break;
      }
      float h = POCKET_HEADING;
      if (r > 0 && r < 40) h -= constrain((r - (POCKET_CENTRE_TO_RAIL_CM - US_SIDE_CM)) * 1.5f, -15.0f, 15.0f);
      hold(h, (f > 0 && f < stopAt + 10) ? SLOW * 0.6 : SLOW);
      break;
    }

    case S_POINTER:                                 // stopped at T2 (no pointer servo for now)
      wheels(0, 0);
      go(S_DONE);
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
    Serial.print(NAMES[step]); Serial.print(" | h "); Serial.print(deg(th), 0);
    Serial.print(" | F "); Serial.print(f, 0); Serial.print(" R "); Serial.print(r, 0);
    Serial.print(" | IR "); Serial.print(lB); Serial.print(rB);
    Serial.print(" | L "); Serial.print(L.target, 0); Serial.print("/"); Serial.print(L.rate, 0);
    Serial.print(" R "); Serial.print(R.target, 0); Serial.print("/"); Serial.print(R.rate, 0);
    Serial.print(" | t "); Serial.println((millis() - runStart) / 1000.0, 1);
  }
}
