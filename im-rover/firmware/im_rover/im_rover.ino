/*
  IM Competition I: random start -> T1 drop -> line -> T2
  Strategy "Square, slide, shoot"

    1. Spin slowly and find the box pattern: LEFT+RIGHT = box width, FRONT short (facing the back wall)
       or FRONT long (facing up-field). Turn back to the MIDDLE of that pattern, not its first sample.
    2. Face up-field and reverse into the 30 cm back wall until both rear corners touch (wall squaring).
       Heading is now exact and the axle is REAR_TO_AXLE from the wall.
    3. LEFT and RIGHT now hit the side walls square, so they give the true position across the box.
    4. Slide sideways under T1 (out 22 cm, turn, range FRONT to a side wall, turn back) and square again.
    5. One straight encoder drive up-field to T1. No turn after the last squaring. Drop.
    6. Face the right wall and drive to the oval's right side (black only accepted where it must be),
     turn onto it and follow the route: a table keyed to encoder distance ignores the oval's loose end
     and the branch, slows in the passage and makes the hairpin as a scripted pivot.
  7. At the end of the line, turn to face the upper rung and stop at the FRONT reading taught on T2.

  Board : Raspberry Pi Pico, Arduino IDE with the arduino-pico core (Earle Philhower).
  Cores : core 0 = motion control (100 Hz), core 1 = the three HC-SR04s, round robin, 30 ms apart.
  Start : press BOOTSEL (or a button from GP11 to GND), or send 'r' over USB serial.
  Tests : open Serial Monitor at 115200 and send a letter (list printed at boot).

  Units : cm, degrees, seconds. Field frame: x from the LEFT wall's inner face, y from the BACK wall's
          inner face, heading 90 = facing up-field. Robot reference point = midpoint of the wheel axle.
*/

#include <Arduino.h>
#include <Servo.h>
#include <pico/time.h>
#include <hardware/gpio.h>
#include "robot_types.h"    // structs live here so the Arduino IDE's auto-prototypes can see them
#include "field_map.h"      // the practice field in cm: walls, T1, T2, line route, start marks

// =====================================================================================
// 1. FIELD AND TARGETS  (measure these on the practice field)
// =====================================================================================
// BOX_W, T1_X, T1_Y, T2 and the line route live in field_map.h
const float SUM_TARGET       = 98.7;   // LEFT+RIGHT when square to the side walls (you measured 98.7)
const float SUM_TOL          = 4.0;    // band used while spinning
const float FRONT_BACK_MAX   = 40.0;   // facing the back wall, FRONT is below this (start region ~44 cm deep)
const float FRONT_UP_MIN     = 48.0;   // facing up-field, FRONT is above this, or no echo

// =====================================================================================
// 2. ROBOT GEOMETRY  (all measured from the wheel-axle midpoint)
// =====================================================================================
const float REAR_TO_AXLE     = 4.5;    // rear bumper face -> axle. Fit a flat rear bumper wider than the wheels.
const float AXLE_TO_FRONT    = 17.5;   // EFFECTIVE axle -> FRONT reading zero. Calibrate: face a wall with the
                                       // axle exactly D cm away (tape), read FRONT, set D - FRONT.
const float SIDE_LAT         = (BOX_W - SUM_TARGET) / 2.0;  // centre line -> side sonar zero (from the sum)
const float DROP_FWD         = 0.0;    // where the item lands, cm ahead of the axle (0 = between the wheels)

// =====================================================================================
// 3. DRIVE CALIBRATION
// =====================================================================================
const float TICKS_PER_CM_L   = 60.5;   // 'd' test: new = old * 100 / measured_cm
const float TICKS_PER_CM_R   = 60.7;
const float TRACK_CM         = 19.3;   // EFFECTIVE track for spins. 's' test: new = old * 1800 / (1800 - shortfall_deg)
const int   ENC_SIGN_L       = -1;     // 'e' test: push the robot forward by hand, both counts must go UP
const int   ENC_SIGN_R       = +1;

// motor power is 0..255 here; the optocouplers invert it in motorWrite()
const float PWM_MIN          = 70;     // smallest power that keeps a wheel turning (the "creep power")
const float PWM_PER_CMS      = 3.0;    // feed-forward power per cm/s (about 255 / 73 cm/s at 12 V)
const float RIGHT_FF         = 0.95;   // right motor is stronger
const float PWM_MAX          = 235;
const float KP_W             = 4.0;    // wheel speed PI: power per cm/s of error
const float KI_W             = 30.0;   //                 power per cm of accumulated error
const float INTEG_MAX        = 4.0;
const float KP_HDG           = 1.5;    // heading hold: wheel speed difference (cm/s) per degree of error
const float KD_HDG           = 0.08;
const float KP_TURN          = 3.0;    // spin: deg/s per degree remaining

const float V_DRIVE          = 25;     // cm/s, the T1 shot
const float V_SLOW           = 12;     // cm/s, short moves and ranging
const float V_MIN            = 4;      // cm/s, final approach speed
const float V_SQUARE         = 10;     // cm/s, reversing toward the wall before contact
const float ACCEL            = 40;     // cm/s^2
const float DECEL            = 35;     // cm/s^2
const float STOP_LEAD_CM     = 0.3;    // stop this early to allow for coasting
const float W_TURN           = 110;    // deg/s
const float W_SCAN           = 35;     // deg/s while looking for the box pattern
const float W_MIN            = 14;     // deg/s
const float TURN_TOL_DEG     = 0.6;

const int   SQUARE_PWM       = 115;    // push power against the wall (enough to pivot flat, not to climb)
const int   SQUARE_PUSH_MS   = 500;    // keep pushing this long after both wheels stall
const float STALL_CMS        = 1.0;    // a wheel slower than this counts as stalled
const int   STALL_MS         = 150;

const bool  FAST_MODE        = false;  // true: skip the slide; one small angled turn instead (~1.5 cm, ~8 s faster)

// =====================================================================================
// 4. LINE AND T2  (measure on the practice field; 't' teaches T2)
// =====================================================================================
const int   IR_BLACK_LEVEL     = HIGH;  // what digitalRead gives over black. Check in the 'm' monitor.
const float CAPTURE_MIN_CM      = 14;    // black is only accepted between these distances after facing right
const float CAPTURE_MAX_CM      = 32;    //   (oval's right side expected ~22 cm out; anything else is a decoy)
const float LINE_V             = 14;    // cm/s on the easy part
const float LINE_V_SLOW        = 7;     // cm/s in SLOW route sections
const float LINE_STEER         = 1.1;   // steering: wheel speed difference as a multiple of speed
const int   TURN_HOLD_MS       = 70;    // keep turning this long after the sensor clears (sharp corners)
const float JUNCTION_CM        = 4;     // both sensors black (crossing): hold heading this far
const float LINE_MAX_ROUTE     = 245;   // safety: stop following after this route distance
const float LINE_END_FRONT     = 20;    // FRONT reading when the IRs reach the line's end ('t' there)
const float T2_TURN_DEG        = RUNG_NORMAL_HDG - LINE_END_HDG;   // -52: turn to face the rung
const float T2_FRONT_TARGET    = 5.0;   // FRONT with the pointer held on T2 ('t' teach)

// route table: positions are cm of line from the pickup point, measured at the IR sensors
// (= encoder distance + AXLE_TO_FRONT). The print-out at the end of a run shows the real numbers.
const RouteRule ROUTE[] = {
  { RT_OVAL_END - 12, RT_OVAL_END + 13, IGNORE_LEFT,  0, 0 },  // oval's loose end ~9 cm left of the line
  { RT_PASSAGE_IN,    RT_PASSAGE_OUT,   SLOW,         0, 0 },  // narrow passage
  { RT_BRANCH - 10,   RT_BRANCH + 15,   IGNORE_RIGHT, 0, 0 },  // branch to the T-wall (decoy)
  { RT_TOP + 10,      RT_END + 20,      SLOW,         0, 0 },  // top curve, hairpin, last leg
  // hairpin: when the IRs reach the vertex, drive 8 cm on, pivot 124 deg left. The IRs land on the
  // return leg ~24 cm past the vertex, 19 deg off its direction, and the follower takes it from there.
  // CHECK on the field that the pivot clears the top of the ladder's left panel; if it stalls the
  // code backs off 4 cm and finishes the turn.
  { RT_HAIRPIN,       RT_HAIRPIN + 1,   SCRIPT_TURN,  8, 124 },
};
const int N_ROUTE = sizeof(ROUTE) / sizeof(ROUTE[0]);

// =====================================================================================
// 5. PINS
// =====================================================================================
const uint8_t PIN_ENA = 0, PIN_IN1 = 1, PIN_IN2 = 2;   // right motor (L298N OUT1/OUT2)
const uint8_t PIN_IN3 = 3, PIN_IN4 = 4, PIN_ENB = 5;   // left motor  (L298N OUT3/OUT4)
const uint8_t TRIG[3] = {8, 6, 18};                     // FRONT, RIGHT, LEFT
const uint8_t ECHO[3] = {9, 7, 19};
enum { S_F = 0, S_R = 1, S_L = 2 };
const uint8_t ENC_R_A = 12, ENC_R_B = 13, ENC_L_A = 14, ENC_L_B = 15;
const uint8_t IR_L = 16, IR_R = 17;
const int     IR_C = -1;                                // optional centre IR (e.g. 22); -1 = not fitted
const uint8_t PIN_DROP_SERVO = 10;                      // SG90 trap door
const int     PIN_POINTER_SERVO = -1;                   // optional fold-out pointer; -1 = fixed pointer
const uint8_t PIN_START = 11;                           // optional start button to GND
const int DROP_CLOSED_DEG = 10, DROP_OPEN_DEG = 100;
const int POINTER_IN_DEG = 0, POINTER_OUT_DEG = 90;

const uint32_t RUN_LIMIT_MS   = 89000;  // hard stop 1 s before the 90 s limit
const float    SOUND_CM_PER_US = 0.0346; // 346 m/s at ~25 C. Recalibrate on the day (0.17 % per degree C).
const uint32_t SONAR_TIMEOUT_US = 20000;
const uint32_t SONAR_GAP_MS    = 30;    // between any two pings (crosstalk)

// =====================================================================================
// sonar on core 1
// =====================================================================================
volatile float    sonarVal[3] = {NAN, NAN, NAN};
volatile uint32_t sonarSeq[3] = {0, 0, 0};
volatile uint32_t sonarCycles = 0;
volatile bool     sonarOn = true;

static float pingOnce(uint8_t i) {
  uint32_t t0 = millis();                       // a module that lost its echo can hold ECHO high for ~40-200 ms
  while (digitalRead(ECHO[i]) == HIGH && millis() - t0 < 60) {}
  digitalWrite(TRIG[i], LOW);  delayMicroseconds(3);
  digitalWrite(TRIG[i], HIGH); delayMicroseconds(10);
  digitalWrite(TRIG[i], LOW);
  unsigned long us = pulseIn(ECHO[i], HIGH, SONAR_TIMEOUT_US);
  if (us == 0) return NAN;                      // no echo is "unknown", never 0 cm
  float cm = us * SOUND_CM_PER_US * 0.5f;
  return (cm < 2.5f || cm > 300.0f) ? NAN : cm;
}

void setup1() {
  for (int i = 0; i < 3; i++) { pinMode(TRIG[i], OUTPUT); digitalWrite(TRIG[i], LOW); pinMode(ECHO[i], INPUT); }
}

void loop1() {
  for (int i = 0; i < 3; i++) {
    if (!sonarOn) { delay(10); return; }
    uint32_t t0 = millis();
    float d = pingOnce(i);
    sonarVal[i] = d;
    sonarSeq[i] = sonarSeq[i] + 1;
    while (millis() - t0 < SONAR_GAP_MS) {}
  }
  sonarCycles = sonarCycles + 1;
}

// =====================================================================================
// encoders (4x quadrature by interrupts; ~4 400 edges/s per wheel at full speed is easy)
// =====================================================================================
static const int8_t QDEC[16] = {0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};
volatile int32_t encRaw[2] = {0, 0};    // 0 = left, 1 = right
volatile uint8_t encState[2] = {0, 0};

void isrLeft()  { uint8_t s = (gpio_get(ENC_L_A) << 1) | gpio_get(ENC_L_B); encRaw[0] += QDEC[(encState[0] << 2) | s]; encState[0] = s; }
void isrRight() { uint8_t s = (gpio_get(ENC_R_A) << 1) | gpio_get(ENC_R_B); encRaw[1] += QDEC[(encState[1] << 2) | s]; encState[1] = s; }

float cmL() { noInterrupts(); int32_t c = encRaw[0]; interrupts(); return ENC_SIGN_L * c / TICKS_PER_CM_L; }
float cmR() { noInterrupts(); int32_t c = encRaw[1]; interrupts(); return ENC_SIGN_R * c / TICKS_PER_CM_R; }
float avgCm() { return 0.5f * (cmL() + cmR()); }
float hdgDeg() { return (cmR() - cmL()) / TRACK_CM * 57.29578f; }   // CCW positive, continuous

// =====================================================================================
// motors (inverted PWM through the PC817s)
// =====================================================================================
volatile bool timeUp = false;
bool runActive = false;

void motorWrite(int pL, int pR) {               // -255..255, + = forward
  if (timeUp) return;
  pL = constrain(pL, -255, 255); pR = constrain(pR, -255, 255);
  digitalWrite(PIN_IN1, pR >= 0 ? LOW : HIGH);  // right forward: IN1 LOW, IN2 HIGH
  digitalWrite(PIN_IN2, pR >= 0 ? HIGH : LOW);
  analogWrite(PIN_ENA, 255 - abs(pR));
  digitalWrite(PIN_IN3, pL >= 0 ? HIGH : LOW);  // left forward: IN3 HIGH, IN4 LOW
  digitalWrite(PIN_IN4, pL >= 0 ? LOW : HIGH);
  analogWrite(PIN_ENB, 255 - abs(pL));
}
void motorsCoast() { if (timeUp) return; analogWrite(PIN_ENA, 255); analogWrite(PIN_ENB, 255); }
void motorsBrake(uint16_t ms) {                 // IN pins equal + enable full = short-circuit brake
  if (timeUp) return;
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW); digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENA, 0); analogWrite(PIN_ENB, 0);
  uint32_t t = millis(); while (millis() - t < ms) {}
  motorsCoast();
}

// 90 s guard: a hardware alarm forces both enable lines to "off", whatever the main loop is doing
int64_t onTimeUp(alarm_id_t, void *) {
  timeUp = true;
  gpio_set_function(PIN_ENA, GPIO_FUNC_SIO); gpio_set_dir(PIN_ENA, true); gpio_put(PIN_ENA, 1);
  gpio_set_function(PIN_ENB, GPIO_FUNC_SIO); gpio_set_dir(PIN_ENB, true); gpio_put(PIN_ENB, 1);
  return 0;
}

Servo dropServo, pointerServo;

void haltForever() {
  timeUp = true; sonarOn = false;
  gpio_set_function(PIN_ENA, GPIO_FUNC_SIO); gpio_set_dir(PIN_ENA, true); gpio_put(PIN_ENA, 1);
  gpio_set_function(PIN_ENB, GPIO_FUNC_SIO); gpio_set_dir(PIN_ENB, true); gpio_put(PIN_ENB, 1);
  if (dropServo.attached()) dropServo.detach();
  if (pointerServo.attached()) pointerServo.detach();
  Serial.println("STOPPED.");
  while (true) { delay(1000); }
}

// =====================================================================================
// 100 Hz control tick: wheel speeds, wheel speed PI, odometry, stall watch
// =====================================================================================
Wheel wl, wr;
bool  speedMode = false;
uint32_t lastTickUs = 0;
uint32_t stallSinceMs = 0;
float hdgOffset = 0;            // field heading = hdgDeg() + hdgOffset (valid after squaring)
float odoX = 61, odoY = 20;     // field position of the axle (valid after squaring)

float worldHdg() { return hdgDeg() + hdgOffset; }
float wrap180(float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
float nz(float v) { return isnan(v) ? 250.0f : v; }

static int wheelPower(Wheel &w, float dt, float ffScale) {
  if (fabsf(w.target) < 0.3f) { w.integ = 0; return 0; }
  float e = w.target - w.vel;
  w.integ = constrain(w.integ + e * dt, -INTEG_MAX, INTEG_MAX);
  float ff = (w.target > 0 ? 1 : -1) * (PWM_MIN + PWM_PER_CMS * fabsf(w.target)) * ffScale;
  return (int)constrain(ff + KP_W * e + KI_W * w.integ, -PWM_MAX, PWM_MAX);
}

bool tick() {                                  // returns true once per 10 ms
  if (timeUp) haltForever();
  uint32_t now = micros();
  if (now - lastTickUs < 10000) return false;
  float dt = (now - lastTickUs) * 1e-6f;
  lastTickUs = now;
  if (dt > 0.05f) dt = 0.01f;
  float L = cmL(), R = cmR();
  float dL = L - wl.last, dR = R - wr.last;
  wl.last = L; wr.last = R;
  wl.vel = 0.5f * wl.vel + 0.5f * dL / dt;
  wr.vel = 0.5f * wr.vel + 0.5f * dR / dt;
  float ds = 0.5f * (dL + dR), th = worldHdg() * 0.0174533f;
  odoX += ds * cosf(th); odoY += ds * sinf(th);
  if (speedMode) motorWrite(wheelPower(wl, dt, 1.0f), wheelPower(wr, dt, RIGHT_FF));
  bool commanded = fabsf(wl.target) > 2 || fabsf(wr.target) > 2;
  bool still = fabsf(wl.vel) < STALL_CMS && fabsf(wr.vel) < STALL_CMS;
  if (!(commanded && still)) stallSinceMs = millis();
  return true;
}
bool stalled(uint32_t ms) { return millis() - stallSinceMs > ms; }
void resetStall() { stallSinceMs = millis(); }
void waitMs(uint32_t ms) { uint32_t t = millis(); while (millis() - t < ms) tick(); }
void setWheels(float vL, float vR) { wl.target = vL; wr.target = vR; speedMode = true; }
void stopAll() { speedMode = false; wl.target = wr.target = 0; wl.integ = wr.integ = 0; motorsBrake(80); waitMs(150); }

// =====================================================================================
// sonar helpers (core 0 side)
// =====================================================================================
float sonarMedian(uint8_t i, int n) {          // n fresh samples; NAN unless most of them echoed
  float buf[11]; int k = 0, got = 0; n = min(n, 11);
  uint32_t seq = sonarSeq[i], t0 = millis();
  while (got < n && millis() - t0 < (uint32_t)(120 * n + 200)) {
    tick();
    if (sonarSeq[i] != seq) { seq = sonarSeq[i]; got++; float v = sonarVal[i]; if (!isnan(v)) buf[k++] = v; }
  }
  if (k * 2 <= got || k == 0) return NAN;
  for (int a = 1; a < k; a++) { float v = buf[a]; int b = a - 1; while (b >= 0 && buf[b] > v) { buf[b + 1] = buf[b]; b--; } buf[b + 1] = v; }
  return buf[k / 2];
}

// =====================================================================================
// motion primitives
// =====================================================================================
bool driveStraight(float cm, float vmax) {     // heading hold on encoders, trapezoid speed
  float sL = cmL(), sR = cmR(), h0 = hdgDeg(), sg = cm >= 0 ? 1 : -1, goal = fabsf(cm), v = 0, prevE = 0;
  uint32_t tStart = millis(), tLimit = (uint32_t)(goal / max(vmax, 3.0f) * 2500 + 3000);
  resetStall();
  while (true) {
    if (!tick()) continue;
    float done = sg * ((cmL() - sL) + (cmR() - sR)) * 0.5f, rem = goal - done;
    if (rem <= STOP_LEAD_CM) break;
    float vCap = sqrtf(2 * DECEL * max(rem, 0.0f)) + V_MIN;
    v = min(min(vmax, vCap), v + ACCEL * 0.01f);
    float e = h0 - hdgDeg(), c = KP_HDG * e + KD_HDG * (e - prevE) / 0.01f;
    prevE = e;
    setWheels(sg * v - c, sg * v + c);
    if (stalled(400) || millis() - tStart > tLimit) { stopAll(); return false; }
  }
  stopAll();
  return true;
}

bool turnBy(float deg) {                        // spin in place; + = counter-clockwise (left)
  float target = hdgDeg() + deg;
  for (int pass = 0; pass < 3; pass++) {
    uint32_t t0 = millis(); resetStall();
    while (true) {
      if (!tick()) continue;
      float rem = target - hdgDeg();
      if (fabsf(rem) < TURN_TOL_DEG) break;
      float w = constrain(KP_TURN * fabsf(rem), W_MIN, W_TURN) * (rem > 0 ? 1 : -1);
      float v = w * 0.0174533f * TRACK_CM * 0.5f;
      setWheels(-v, v);
      if (stalled(300)) { stopAll(); return false; }
      if (millis() - t0 > 6000) break;
    }
    stopAll();
    if (fabsf(target - hdgDeg()) < 1.0f) return true;
  }
  return true;
}
bool turnToWorld(float h) { return turnBy(wrap180(h - worldHdg())); }

// reverse into the back wall: heading hold until first contact, then push until both wheels stall
bool squareBack(float maxCm) {
  float s0 = avgCm(), h0 = hdgDeg(); bool contact = false; uint32_t tContact = 0, tStill = 0;
  resetStall();
  while (true) {
    if (!tick()) continue;
    float moved = s0 - avgCm();
    if (!contact) {
      if (moved > maxCm) { stopAll(); return false; }
      float e = h0 - hdgDeg(), c = KP_HDG * e;
      setWheels(-V_SQUARE - c, -V_SQUARE + c);
      bool slowL = fabsf(wl.vel) < 0.3f * V_SQUARE, slowR = fabsf(wr.vel) < 0.3f * V_SQUARE;
      if ((slowL || slowR) && moved > 1.0f) { if (!tStill) tStill = millis(); if (millis() - tStill > 100) { contact = true; tContact = millis(); tStill = 0; } }
      else tStill = 0;
    } else {
      speedMode = false;
      motorWrite(-SQUARE_PWM, -SQUARE_PWM);
      bool bothStill = fabsf(wl.vel) < STALL_CMS && fabsf(wr.vel) < STALL_CMS;
      if (bothStill) { if (!tStill) tStill = millis(); } else tStill = 0;
      if ((tStill && millis() - tStill > (uint32_t)SQUARE_PUSH_MS) || millis() - tContact > 1800) break;
    }
  }
  motorsCoast(); waitMs(250);
  hdgOffset = 90.0f - hdgDeg();                // the wall defines heading 90
  odoY = REAR_TO_AXLE;
  return true;
}

void approachFront(float target, int tries) {   // range to a wall straight ahead
  for (int k = 0; k < tries; k++) {
    float f = sonarMedian(S_F, 5);
    if (isnan(f)) return;
    float e = f - target;
    if (fabsf(e) < 0.4f) return;
    driveStraight(constrain(e, -40.0f, 40.0f), fabsf(e) > 6 ? V_SLOW : V_MIN + 2);
  }
}

float measureX() {                              // both side walls are square to the sensors now
  float L = sonarMedian(S_L, 7), R = sonarMedian(S_R, 7);
  float xl = L + SIDE_LAT, xr = BOX_W - R - SIDE_LAT;
  float x;
  if (!isnan(L) && !isnan(R)) x = fabsf(xl - xr) < 3 ? 0.5f * (xl + xr) : (L < R ? xl : xr);
  else if (!isnan(L)) x = xl; else if (!isnan(R)) x = xr; else x = BOX_W / 2;
  Serial.printf("  x: L=%.1f R=%.1f -> x=%.1f\n", L, R, x);
  odoX = x;
  return x;
}

// =====================================================================================
// localisation: spin until the box pattern, take its middle
// =====================================================================================

Sig scanForSignature() {
  Sig out;
  float L0 = sonarMedian(S_L, 3), R0 = sonarMedian(S_R, 3);
  int dir = nz(L0) < nz(R0) ? -1 : +1;        // turn away from the nearer side wall
  float openHdg = hdgDeg(), openF = nz(sonarMedian(S_F, 3));
  float turned = 0, lastH = hdgDeg();
  int runCls = 0, runN = 0, blocks = 0, flips = 0;
  float runA = 0, runB = 0, sMin = 0, sMax = 0, runL = NAN, runR = NAN, runF = NAN;
  uint32_t cyc = sonarCycles;
  resetStall();
  while (turned < 720) {
    if (!tick()) continue;
    float v = dir * W_SCAN * 0.0174533f * TRACK_CM * 0.5f;
    setWheels(-v, v);
    float h = hdgDeg(); turned += fabsf(h - lastH); lastH = h;
    if (stalled(300) || nz(sonarVal[S_F]) < 5) {          // blocked: a corner touched something
      stopAll();
      if (runN >= 2) {                                      // blocked inside a pattern: good enough,
        out.found = true; out.back = runCls == 1;           // squaring fixes the heading anyway
        out.center = 0.5f * (runA + runB); out.L = runL; out.R = runR; out.F = runF;
        Serial.println("  pattern (cut short by a wall)");
        return out;
      }
      if (turned < 30 && flips == 0) { dir = -dir; flips++; resetStall(); lastH = hdgDeg(); continue; }  // spin the other way
      blocks++; runN = 0;
      turnBy(openHdg - hdgDeg());                          // back to the most open heading (already swept)
      float f = sonarMedian(S_F, 3), go = isnan(f) ? 12 : constrain(f - 20, -6.0f, 12.0f);
      if (fabsf(go) > 1) driveStraight(go, V_SLOW);
      if (blocks >= 3) { dir = -dir; blocks = 0; flips++; }
      if (flips >= 3) break;
      cyc = sonarCycles; lastH = hdgDeg(); resetStall();
      continue;
    }
    if (sonarCycles == cyc) continue;
    cyc = sonarCycles;
    float F = sonarVal[S_F], L = sonarVal[S_L], R = sonarVal[S_R];
    if (nz(F) > openF) { openF = nz(F); openHdg = h; }
    int cls = 0; float sum = NAN;
    if (!isnan(L) && !isnan(R)) {
      sum = L + R;
      if (fabsf(sum - SUM_TARGET) < SUM_TOL) {
        if (!isnan(F) && F < FRONT_BACK_MAX) cls = 1;     // facing the back wall
        else if (isnan(F) || F >= FRONT_UP_MIN) cls = 2;  // facing up-field
      }
    }
    if (cls && (runN == 0 || cls == runCls)) {
      if (runN == 0) { runCls = cls; runA = h; sMin = sMax = sum; }
      runB = h; runN++; sMin = min(sMin, sum); sMax = max(sMax, sum);
      if (runN == 2) { runL = L; runR = R; runF = F; }
    } else {
      if (runN >= 4 && sMax - sMin < 3) {                 // a real flat zone: take its middle
        stopAll();
        out.found = true; out.back = runCls == 1; out.center = 0.5f * (runA + runB);
        out.L = runL; out.R = runR; out.F = runF;
        Serial.printf("  pattern %s, %d samples, width %.1f deg\n", out.back ? "BACK" : "UP", runN, fabsf(runB - runA));
        return out;
      }
      runN = 0;
      if (cls) { runCls = cls; runA = runB = h; sMin = sMax = sum; runN = 1; }
    }
  }
  stopAll();
  Serial.println("  pattern not found");
  return out;
}

bool localiseAndSquare() {
  float F0 = sonarMedian(S_F, 3);
  if (!isnan(F0) && F0 < 15) driveStraight(-(18 - F0), V_SLOW);   // room for the long front end to swing
  for (int attempt = 0; attempt < 3; attempt++) {
    Sig s = scanForSignature();
    if (s.found) {
      turnBy(s.center - hdgDeg());
      if (s.back) {
        float f = sonarMedian(S_F, 3);
        if (!isnan(f) && f < 8) driveStraight(-(10 - f), V_SLOW);
        // facing the back wall, RIGHT points at the LEFT wall: swing the front toward the farther wall
        turnBy(nz(s.R) < nz(s.L) ? 180 : -180);
      }
    }
    // hugging a side wall? step ~5 cm away first, or the reverse scrapes the wall and that side sonar
    // sits under its 2.5 cm minimum range
    float Ls = sonarMedian(S_L, 3), Rs = sonarMedian(S_R, 3);
    int side = (!isnan(Ls) && Ls < 6) ? -1 : (!isnan(Rs) && Rs < 6) ? 1 : 0;
    if (side) { float a = side < 0 ? -25 : 25; turnBy(a); driveStraight(12, V_SLOW); turnBy(-a); }
    if (!squareBack(60)) { driveStraight(8, V_SLOW); continue; }
    float L = sonarMedian(S_L, 3), R = sonarMedian(S_R, 3), F = sonarMedian(S_F, 3);
    bool ok = !isnan(L) && !isnan(R) && fabsf(L + R - SUM_TARGET) < SUM_TOL && (isnan(F) || F >= FRONT_UP_MIN);
    Serial.printf("  squared: L=%.1f R=%.1f sum=%.1f F=%.1f -> %s\n", L, R, L + R, F, ok ? "OK" : "retry");
    if (ok) return true;
    driveStraight(20, V_SLOW);
  }
  return false;
}

// =====================================================================================
// drop, line, T2
// =====================================================================================
void dropperClose() { dropServo.attach(PIN_DROP_SERVO, 540, 2400); dropServo.write(DROP_CLOSED_DEG); delay(350); dropServo.detach(); }
void dropItem() {                               // motors are stopped: the servo current spike can't stack on theirs
  dropServo.attach(PIN_DROP_SERVO, 540, 2400);
  dropServo.write(DROP_OPEN_DEG); waitMs(450);
  dropServo.write(DROP_CLOSED_DEG); waitMs(300);
  dropServo.detach();
}
void deployPointer() {
  if (PIN_POINTER_SERVO < 0) return;
  pointerServo.attach(PIN_POINTER_SERVO, 540, 2400);
  pointerServo.write(POINTER_OUT_DEG); waitMs(400);
  pointerServo.detach();
}

bool irBlack(int pin) { return pin >= 0 && digitalRead(pin) == IR_BLACK_LEVEL; }

bool driveUntilBlack(float minCm, float maxCm, float v) {   // heading hold; black only counts in the window
  float s0 = avgCm(), h0 = hdgDeg();
  resetStall();
  while (true) {
    if (!tick()) continue;
    float d = avgCm() - s0;
    if (d >= minCm && (irBlack(IR_L) || irBlack(IR_R))) { stopAll(); Serial.printf("  line found at %.1f cm\n", d); return true; }
    if (d >= maxCm || stalled(400)) { stopAll(); Serial.printf("  no line by %.1f cm\n", d); return false; }
    float e = h0 - hdgDeg(); setWheels(v - KP_HDG * e, v + KP_HDG * e);
  }
}

bool pickUpLine() {                             // from T1: the oval's right side is straight to the right
  turnToWorld(0);
  bool ok = driveUntilBlack(CAPTURE_MIN_CM, CAPTURE_MAX_CM, V_SLOW);
  driveStraight(AXLE_TO_FRONT - 1.0f, V_SLOW);  // roll on until the axle is over the line
  turnToWorld(LINE_PICKUP_HDG);                 // line direction there: the IRs now straddle it
  return ok;
}

float followLine() {
  float s0 = avgCm(), hHold = hdgDeg(), junctionEnd = -1;
  int lastSide = 0, holdSide = 0; uint32_t holdUntil = 0;
  bool scriptDone[N_ROUTE] = {false};
  float route = 0;
  while (true) {
    if (!tick()) continue;
    float along = avgCm() - s0;
    route = along + AXLE_TO_FRONT;              // where the IR sensors are along the route
    if (route >= LINE_MAX_ROUTE) break;
    float F = sonarVal[S_F];
    if (route >= RT_END - 15 && !isnan(F) && F <= LINE_END_FRONT) break;   // end of the line: ladder ahead
    bool ignL = false, ignR = false, slow = false;
    for (int i = 0; i < N_ROUTE; i++) {
      const RouteRule &r = ROUTE[i];
      if (route < r.from || route > r.to) continue;
      if (r.act == IGNORE_LEFT) ignL = true;
      else if (r.act == IGNORE_RIGHT) ignR = true;
      else if (r.act == SLOW) slow = true;
      else if (r.act == SCRIPT_TURN && !scriptDone[i]) {
        scriptDone[i] = true;
        stopAll();
        if (r.preCm != 0) driveStraight(r.preCm, V_SLOW);
        float hBefore = hdgDeg();
        if (!turnBy(r.turnDeg)) {               // stalled on a wall: back off and finish the turn
          driveStraight(-4, V_SLOW);
          turnBy(r.turnDeg - (hdgDeg() - hBefore));
        }
        hHold = hdgDeg(); holdUntil = 0; junctionEnd = -1;
        Serial.printf("  scripted turn at route %.1f\n", route);
      }
    }
    float v = slow ? LINE_V_SLOW : LINE_V;
    bool l = irBlack(IR_L) && !ignL, r = irBlack(IR_R) && !ignR;
    if (along < junctionEnd) {                   // crossing a decoy or junction: hold heading
      float e = hHold - hdgDeg(); setWheels(v - KP_HDG * e, v + KP_HDG * e); continue;
    }
    if (l && r) { junctionEnd = along + JUNCTION_CM; hHold = hdgDeg(); continue; }
    uint32_t now = millis();
    if (l) { lastSide = holdSide = -1; holdUntil = now + TURN_HOLD_MS; }
    else if (r) { lastSide = holdSide = +1; holdUntil = now + TURN_HOLD_MS; }
    int steer = l ? -1 : r ? +1 : (now < holdUntil ? holdSide : 0);
    if (steer == 0 && IR_C >= 0 && !irBlack(IR_C)) steer = lastSide;   // centre lost: keep turning to the last side
    float s = LINE_STEER * v;
    if (steer < 0) setWheels(v - s, v + s);       // line under the LEFT sensor: turn left
    else if (steer > 0) setWheels(v + s, v - s);
    else setWheels(v, v);
    hHold = hdgDeg();
  }
  stopAll();
  Serial.printf("  line followed to route %.1f cm (map says the end is at %.0f)\n", route, RT_END);
  return route;
}

void approachT2() {
  if (T2_TURN_DEG != 0) turnBy(T2_TURN_DEG);
  approachFront(T2_FRONT_TARGET, 5);
}

// =====================================================================================
// the run
// =====================================================================================
void runCompetition() {
  Serial.println("RUN");
  runActive = true;
  add_alarm_in_ms(RUN_LIMIT_MS, onTimeUp, nullptr, true);
  uint32_t t0 = millis();
  auto mark = [&](const char *s) { Serial.printf("[%5.1f s] %s\n", (millis() - t0) / 1000.0f, s); };

  mark("localise and square");
  localiseAndSquare();
  float x = measureX();

  if (FAST_MODE) {
    mark("fast: one angled shot");
    driveStraight(10, V_SLOW);
    float dx = T1_X - x, dy = T1_Y - DROP_FWD - (REAR_TO_AXLE + 10);
    turnToWorld(90.0f - atan2f(dx, dy) * 57.29578f);
    driveStraight(sqrtf(dx * dx + dy * dy), V_DRIVE);
  } else {
    if (fabsf(x - T1_X) > 0.8f) {
      mark("slide under T1");
      driveStraight(22, V_SLOW);
      bool right = x < T1_X;
      turnToWorld(right ? 0 : 180);
      approachFront(right ? (BOX_W - T1_X - AXLE_TO_FRONT) : (T1_X - AXLE_TO_FRONT), 5);
      turnToWorld(90);
      squareBack(40);
      measureX();
    }
    mark("straight shot to T1");
    driveStraight(T1_Y - REAR_TO_AXLE - DROP_FWD, V_DRIVE);
  }
  mark("drop");
  waitMs(300);
  dropItem();

  mark("pick up the line");
  pickUpLine();
  mark("follow the route");
  followLine();
  mark("T2");
  approachT2();
  deployPointer();
  mark("done");
  haltForever();
}

// =====================================================================================
// setup, start, bench tests
// =====================================================================================
void printHelp() {
  Serial.println("\nIM rover. Press BOOTSEL (or GP11 button) to RUN, or send:");
  Serial.println("  r run | m monitor on/off | e encoder counts | s spin test (5 turns) | d drive 100 cm");
  Serial.println("  w drive 200 cm (drift) | 9 four 90-degree turns | q square on the wall behind | t teach T2");
  Serial.println("  o test the trap door");
}

void setup() {
  Serial.begin(115200);
  analogWriteFreq(1000);                       // PC817 + 1 k pull-up switches in ~10 us: keep PWM low
  analogWriteRange(255);
  for (uint8_t p : {PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4}) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
  motorsCoast();
  for (uint8_t p : {ENC_L_A, ENC_L_B, ENC_R_A, ENC_R_B}) pinMode(p, INPUT_PULLUP);
  encState[0] = (gpio_get(ENC_L_A) << 1) | gpio_get(ENC_L_B);
  encState[1] = (gpio_get(ENC_R_A) << 1) | gpio_get(ENC_R_B);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), isrLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), isrRight, CHANGE);
  pinMode(IR_L, INPUT); pinMode(IR_R, INPUT); if (IR_C >= 0) pinMode(IR_C, INPUT);
  pinMode(PIN_START, INPUT_PULLUP);
  dropperClose();
  if (PIN_POINTER_SERVO >= 0) { pointerServo.attach(PIN_POINTER_SERVO, 540, 2400); pointerServo.write(POINTER_IN_DEG); delay(300); pointerServo.detach(); }
  lastTickUs = micros();
  delay(500);
  printHelp();
}

bool monitorOn = false;
uint32_t lastMon = 0;

void loop() {
  tick();
  bool start = BOOTSEL || digitalRead(PIN_START) == LOW;
  if (start) { while (BOOTSEL || digitalRead(PIN_START) == LOW) delay(5); delay(400); runCompetition(); }
  if (monitorOn && millis() - lastMon > 200) {
    lastMon = millis();
    float F = sonarVal[S_F], L = sonarVal[S_L], R = sonarVal[S_R];
    Serial.printf("F %6.1f  L %6.1f  R %6.1f  L+R %6.1f | cmL %7.1f cmR %7.1f hdg %7.1f | IR %d%d%d\n",
                  F, L, R, L + R, cmL(), cmR(), hdgDeg(), irBlack(IR_L), irBlack(IR_C), irBlack(IR_R));
  }
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 'r': runCompetition(); break;
    case 'm': monitorOn = !monitorOn; break;
    case 'e': Serial.printf("raw L %ld  R %ld  (push forward: both must rise)\n", (long)(ENC_SIGN_L * encRaw[0]), (long)(ENC_SIGN_R * encRaw[1])); break;
    case 's': Serial.println("spin 1800 deg CCW. Measure the shortfall in degrees."); turnBy(1800); break;
    case 'd': Serial.println("drive 100 cm. Measure the real distance."); driveStraight(100, V_DRIVE); break;
    case 'w': Serial.println("drive 200 cm. Each 1 cm drift to the LEFT: lower TICKS_PER_CM_R by 0.1 %."); driveStraight(200, V_DRIVE); break;
    case '9': for (int i = 0; i < 4; i++) { turnBy(90); waitMs(300); } Serial.println("4 x 90 done: should face the start heading"); break;
    case 'q': if (squareBack(40)) measureX(); break;
    case 't': {
      float F = sonarMedian(S_F, 9), L = sonarMedian(S_L, 9), R = sonarMedian(S_R, 9);
      Serial.printf("TEACH  F %.1f  L %.1f  R %.1f   -> set T2_FRONT_TARGET = %.1f\n", F, L, R, F);
      break; }
    case 'o': dropItem(); break;
    default: break;
  }
}
