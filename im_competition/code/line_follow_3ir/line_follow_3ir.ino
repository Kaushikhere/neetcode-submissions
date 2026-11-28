// ============================================================================
// 3-IR LINE FOLLOWER - your pins + your FRIEND'S turning method
//   IR: L = GP17, C = GP22, R = GP16 (HIGH = black)
//   Motors: R ENA GP0 IN1 GP1 IN2 GP2 | L IN3 GP3 IN4 GP4 ENB GP5 (inverted PWM)
//   Encoders: R GP12/GP13, L GP14/GP15 (quadrature)
//   Ultrasonics: F 8/9 (front stop at the line end), R 6/7, L 18/19 (log only)
//   No gyro.
//
//   Steering = the friend's code, exactly:
//     each wheel has its own SPEED controller (PI on encoder speed, every 20 ms)
//     centre BLACK : 110 -> gentle left  (L slow,  R fast)
//                    011 -> gentle right (L fast,  R slow)
//                    010 / 111 -> straight
//     centre WHITE : 100 -> SHARP left   (L REVERSES, R fast)
//                    001 -> SHARP right  (L fast, R REVERSES)
//                    000 -> pivot IMMEDIATELY toward the side the line was last seen
//   No wall slow-down / no "never reverse near a wall" (that is what made the
//   old code too weak in the C turn, so it slid into the wall).
//
//   Line end (kept from the old code, no U-turn): after the sharp V turn, a lost
//   line is checked AHEAD only (+/-45, creep 5 cm, +/-45) -> nothing = stop.
//
//   Starts 3 s after power-up. Serial 115200: + faster, - slower, v log on/off,
//   any other key = STOP.
// ============================================================================
#include <Arduino.h>
#include <math.h>

#define SWAP_LR_IR        0   // 1 if 'L' and 'R' IR are swapped
#define KEEP_LEFT_AT_FORK 1   // 101 (two lines, centre white) = take the LEFT one
#define USE_FRONT_STOP    1   // stop if something is right in front (line end F)
#define STOP_AT_LINE_END  1   // after the sharp V, line lost = end of line

const int R_ENA = 0, R_IN1 = 1, R_IN2 = 2;
const int L_IN3 = 3, L_IN4 = 4, L_ENB = 5;
const int F_TRIG = 8,  F_ECHO = 9;
const int R_TRIG = 6,  R_ECHO = 7;
const int L_TRIG = 18, L_ECHO = 19;
const int ENC_R_C1 = 12, ENC_R_C2 = 13;
const int ENC_L_C1 = 14, ENC_L_C2 = 15;
const int PIN_IR_L = 17;
const int PIN_IR_R = 16;
const int PIN_IR_C = 22;
const int IR_BLACK_STATE = HIGH;

#define INVERT_MOTOR_DIR 0
const int8_t ENC_R_SIGN = -1;
const int8_t ENC_L_SIGN =  1;
const float TICKS_PER_CM_L = 60.5f;
const float TICKS_PER_CM_R = 60.7f;
const float WHEELBASE_CM   = 19.0f;

// ---- Friend's wheel speeds, in cm/s ----
// (friend: 600 / 750 / 250 / -250 / 450 counts/s with ~30 counts/cm)
// '+' / '-' on Serial scale all of them together (speedScale).
const float SPD_CRUISE    = 20.0f;   // straight
const float SPD_FAST      = 25.0f;   // outer wheel (gentle and sharp)
const float SPD_SLOW      =  8.3f;   // inner wheel, gentle turn
const float SPD_SHARP_IN  = -8.3f;   // inner wheel, SHARP turn: runs BACKWARDS
const float SPD_PIVOT     = 15.0f;   // in-place pivot when all 3 see white

// ---- Friend's per-wheel PI (PWM 0..255, inverted on the pins) ----
const float SPD_BASE_PWM  = 135.0f;
const float SPD_MIN_PWM   = 80.0f;
const float SPD_MAX_PWM   = 230.0f;
const float SPD_KP        = 1.2f;    // PWM per cm/s of speed error   (friend 0.04 per count/s)
const float SPD_KI        = 3.6f;    // PWM per cm of summed error     (friend 0.12)
const float SPD_I_MAX     = 66.0f;   // integral clamp, cm              (friend 2000 counts)
const unsigned long SPD_DT_MS = 20UL;
float speedScale = 1.0f;

// ---- Sharp V (after the C turn) & line end ----
const float LF_V_TURN_DEG    = 115.0f; // heading change this big ...
const int   LF_V_SLOTS       = 8;      // ... within 8 x 1.5 = 12 cm of travel = sharp V done
const float LF_V_SWING_DEG   = 80.0f;  // OR a lost-line pivot found the line after this much LEFT
const float LF_END_TURN_DEG  = 195.0f; // backup: total left turn since the start (front stop / search fail)
const float LF_SEARCH_DEG    = 150.0f; // max pivot each way (the line BEHIND, ~180, is never reached)
const float LF_END_CHECK_DEG = 45.0f;  // end check: look this far left and right (line ahead only)
const float LF_END_CREEP_CM  = 5.0f;   // ... then creep straight this far and look again
const float LF_END_WALL_CM   = 12.0f;  // line lost with a wall this close ahead = end
const float LF_FRONT_STOP_CM = 6.0f;

const unsigned long LF_MAX_STANDALONE_MS = 120000UL;
const unsigned long LF_LOG_MS            = 150UL;

bool timeUp() { return false; }
float gLineHdgAtEnd = 0.0f, gEndOvershootCm = 0.0f;   // kept for the full firmware's T2 step
bool  gEndByLoss = false;

// ============================================================================
// ENCODERS
// ============================================================================
volatile long leftCount  = 0;
volatile long rightCount = 0;
volatile uint8_t prevStateL = 0, prevStateR = 0;

const int8_t QEM[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void isrEncR() {
  uint8_t s = ((uint8_t)digitalRead(ENC_R_C1) << 1) | (uint8_t)digitalRead(ENC_R_C2);
  rightCount += (long)QEM[(prevStateR << 2) | s] * ENC_R_SIGN;
  prevStateR = s;
}

void isrEncL() {
  uint8_t s = ((uint8_t)digitalRead(ENC_L_C1) << 1) | (uint8_t)digitalRead(ENC_L_C2);
  leftCount += (long)QEM[(prevStateL << 2) | s] * ENC_L_SIGN;
  prevStateL = s;
}

void resetEncoders() {
  noInterrupts();
  leftCount  = 0;
  rightCount = 0;
  interrupts();
}

void readEncoders(long &l, long &r) {
  noInterrupts();
  l = leftCount;
  r = rightCount;
  interrupts();
}

float getEncoderHeadingDeg() {
  long l, r;
  readEncoders(l, r);
  float dR = (float)r / TICKS_PER_CM_R;
  float dL = (float)l / TICKS_PER_CM_L;
  return ((dR - dL) / WHEELBASE_CM) * (180.0f / (float)M_PI);
}

float getEncoderDistCm() {
  long l, r;
  readEncoders(l, r);
  return 0.5f * ((float)l / TICKS_PER_CM_L + (float)r / TICKS_PER_CM_R);
}

float getHeadingDeg() { return getEncoderHeadingDeg(); }   // + = left

// ============================================================================
// MOTORS
// ============================================================================
void setLeftDir(int dir) {
  if (INVERT_MOTOR_DIR) dir = -dir;
  if (dir > 0)      { digitalWrite(L_IN3, HIGH); digitalWrite(L_IN4, LOW);  }
  else if (dir < 0) { digitalWrite(L_IN3, LOW);  digitalWrite(L_IN4, HIGH); }
  else              { digitalWrite(L_IN3, LOW);  digitalWrite(L_IN4, LOW);  }
}

void setRightDir(int dir) {
  if (INVERT_MOTOR_DIR) dir = -dir;
  if (dir > 0)      { digitalWrite(R_IN1, LOW);  digitalWrite(R_IN2, HIGH); }
  else if (dir < 0) { digitalWrite(R_IN1, HIGH); digitalWrite(R_IN2, LOW);  }
  else              { digitalWrite(R_IN1, LOW);  digitalWrite(R_IN2, LOW);  }
}

void releaseMotors() {
  analogWrite(L_ENB, 255);
  analogWrite(R_ENA, 255);
  digitalWrite(L_IN3, LOW); digitalWrite(L_IN4, LOW);
  digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
}

void stopMotors(int brakeMs = 35) {
  digitalWrite(L_IN3, LOW); digitalWrite(L_IN4, LOW);
  digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
  analogWrite(L_ENB, 0);          // EN full on + both IN low = brake
  analogWrite(R_ENA, 0);
  delay(brakeMs);
  releaseMotors();
}

// ---- Friend's per-wheel speed PI (targets in cm/s, sign = direction) ----
float spTgtL = 0.0f, spTgtR = 0.0f;
float spPwmL = SPD_BASE_PWM, spPwmR = SPD_BASE_PWM;
float spIntL = 0.0f, spIntR = 0.0f;
float spMeasL = 0.0f, spMeasR = 0.0f;           // measured |speed| cm/s (for the log)
long  spPrevL = 0, spPrevR = 0;
unsigned long spPrevMs = 0;

void speedReset() {
  readEncoders(spPrevL, spPrevR);
  spPrevMs = millis();
  spIntL = spIntR = 0.0f;
  spPwmL = spPwmR = SPD_BASE_PWM;
}

void applyWheel(bool left, float tgt, float pwm) {
  if (tgt == 0.0f) {
    if (left) { setLeftDir(0);  analogWrite(L_ENB, 255); }
    else      { setRightDir(0); analogWrite(R_ENA, 255); }
    return;
  }
  if (left) { setLeftDir(tgt > 0.0f ? 1 : -1);  analogWrite(L_ENB, 255 - (int)pwm); }
  else      { setRightDir(tgt > 0.0f ? 1 : -1); analogWrite(R_ENA, 255 - (int)pwm); }
}

// call every loop: direction changes at once, PI updates every 20 ms
void speedDrive(float tL, float tR) {
  tL *= speedScale; tR *= speedScale;
  spTgtL = tL; spTgtR = tR;
  unsigned long now = millis();
  if (now - spPrevMs >= SPD_DT_MS) {
    float dt = (float)(now - spPrevMs) * 0.001f;
    spPrevMs = now;
    long l, r;
    readEncoders(l, r);
    spMeasL = fabsf((float)(l - spPrevL) / TICKS_PER_CM_L) / dt;
    spMeasR = fabsf((float)(r - spPrevR) / TICKS_PER_CM_R) / dt;
    spPrevL = l; spPrevR = r;

    if (tL == 0.0f) { spIntL = 0.0f; spPwmL = 0.0f; }
    else {
      float err = fabsf(tL) - spMeasL;
      spIntL = constrain(spIntL + err * dt, -SPD_I_MAX, SPD_I_MAX);
      spPwmL = constrain(SPD_BASE_PWM + SPD_KP * err + SPD_KI * spIntL, SPD_MIN_PWM, SPD_MAX_PWM);
    }
    if (tR == 0.0f) { spIntR = 0.0f; spPwmR = 0.0f; }
    else {
      float err = fabsf(tR) - spMeasR;
      spIntR = constrain(spIntR + err * dt, -SPD_I_MAX, SPD_I_MAX);
      spPwmR = constrain(SPD_BASE_PWM + SPD_KP * err + SPD_KI * spIntR, SPD_MIN_PWM, SPD_MAX_PWM);
    }
  }
  applyWheel(true,  tL, spPwmL < SPD_MIN_PWM ? SPD_BASE_PWM : spPwmL);
  applyWheel(false, tR, spPwmR < SPD_MIN_PWM ? SPD_BASE_PWM : spPwmR);
}

// ============================================================================
// SENSORS
// ============================================================================
// Majority-of-3 debounced IR read
bool irRaw(int pin) {
  int b = 0;
  for (int i = 0; i < 3; i++) {
    if (digitalRead(pin) == IR_BLACK_STATE) b++;
    delayMicroseconds(40);
  }
  return (b >= 2);
}

void readIR3(bool &l, bool &c, bool &r) {
  c = irRaw(PIN_IR_C);
#if SWAP_LR_IR
  l = irRaw(PIN_IR_R);
  r = irRaw(PIN_IR_L);
#else
  l = irRaw(PIN_IR_L);
  r = irRaw(PIN_IR_R);
#endif
}

static inline float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// Background ultrasonics: one sensor every 15 ms (L, R, L, R, F)
float lfBufL[3] = { 99.0f, 99.0f, 99.0f };
float lfBufR[3] = { 99.0f, 99.0f, 99.0f };
int   lfIdxL = 0, lfIdxR = 0;
float lfSideL = 99.0f, lfSideR = 99.0f, lfFrontF = 99.0f;
int   lfFrontHits = 0;
int   lfUsSlot = 0;
unsigned long lfLastUsMs = 0;

float lfUsRead(int trig, int echo, unsigned long toUs) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, toUs);
  if (us == 0) return 99.0f;
  float cm = (float)us * 0.01715f;
  return (cm < 2.0f) ? 1.5f : cm;
}

void resetLineUltrasonics() {
  for (int i = 0; i < 3; i++) { lfBufL[i] = 99.0f; lfBufR[i] = 99.0f; }
  lfIdxL = 0; lfIdxR = 0;
  lfSideL = 99.0f; lfSideR = 99.0f; lfFrontF = 99.0f;
  lfFrontHits = 0;
  lfUsSlot = 0;
  lfLastUsMs = 0;
}

void updateLineUltrasonics(unsigned long now) {
  if (lfLastUsMs != 0 && (now - lfLastUsMs < 15UL)) return;
  lfLastUsMs = now;
  if (lfUsSlot == 0 || lfUsSlot == 2) {
    lfBufL[lfIdxL] = lfUsRead(L_TRIG, L_ECHO, 2500UL);
    lfIdxL = (lfIdxL + 1) % 3;
    lfSideL = median3(lfBufL[0], lfBufL[1], lfBufL[2]);
  } else if (lfUsSlot == 1 || lfUsSlot == 3) {
    lfBufR[lfIdxR] = lfUsRead(R_TRIG, R_ECHO, 2500UL);
    lfIdxR = (lfIdxR + 1) % 3;
    lfSideR = median3(lfBufR[0], lfBufR[1], lfBufR[2]);
  } else {
    lfFrontF = lfUsRead(F_TRIG, F_ECHO, 2500UL);
    lfFrontHits = (lfFrontF < LF_FRONT_STOP_CM) ? (lfFrontHits + 1) : 0;
  }
  lfUsSlot = (lfUsSlot + 1) % 5;
}

// ============================================================================
// LINE FOLLOWER
// ============================================================================
bool followLine(bool standaloneTest = true) {
  if (timeUp()) return false;

  if (standaloneTest) {
    Serial.println(F("\n[FOLLOW] starting in 3 s... (+ faster, - slower, v log, any key STOP)"));
    for (int i = 3; i > 0; i--) { Serial.print(i); Serial.println(F("...")); delay(1000); }
    while (Serial.available()) Serial.read();
  }

  resetLineUltrasonics();
  for (int i = 0; i < 10; i++) { updateLineUltrasonics(millis()); delay(16); }

  int   lastTurnDir = -1;               // friend's memory: -1 = line last seen LEFT, +1 = RIGHT
  unsigned long lastJunctionMs = 0;     // last 111 / 101

  // heading vs forward distance (10 slots x 1.5 cm) -> sharp V detection
  const int DRING = 10;
  float dRing[DRING];
  float initHdg = getHeadingDeg();
  for (int i = 0; i < DRING; i++) dRing[i] = initHdg;
  int   dIdx = 0;
  float fwdOdo = 0.0f, lastOdoDist = getEncoderDistCm(), lastRingOdo = 0.0f;
  bool  vDone = false;                  // sharp V turned -> next line loss = end
  bool  bigTurnLogged = false;
  float bigTurnMax = 0.0f;
  float lastOnLineHdg = initHdg;

  // lost-line state
  unsigned long lostStartMs = 0;
  bool  endZoneNow = false;
  int   searchDir = -1, searchPhase = 0;
  float lostHdg0 = 0.0f, lostDist0 = 0.0f;
  float srchStuckHdg = 0.0f; unsigned long srchStuckMs = 0;
  int   endChk = 0;
  float endCreep0 = 0.0f;
  int   endStepNo = -1; float endStepRel = 0.0f; unsigned long endStepMs = 0;

  bool  logOn = true;
  unsigned long lastLogMs = 0;
  unsigned long t0 = millis();
  const char *why = "unknown";
  bool reachedPointF = false;
  bool manualAbort   = false;

  speedReset();

  while (true) {
    unsigned long now = millis();
    if (timeUp()) { why = "race time limit"; break; }
    if (standaloneTest && (now - t0 > LF_MAX_STANDALONE_MS)) { why = "2-min safety limit"; break; }

    if (standaloneTest && Serial.available()) {
      char k = Serial.read();
      if (k == '+')      { speedScale = min(speedScale + 0.1f, 1.8f); Serial.print(F("[speed x]")); Serial.println(speedScale, 1); }
      else if (k == '-') { speedScale = max(speedScale - 0.1f, 0.5f); Serial.print(F("[speed x]")); Serial.println(speedScale, 1); }
      else if (k == 'v' || k == 'V') logOn = !logOn;
      else if (k != '\r' && k != '\n') { why = "key pressed"; manualAbort = true; break; }
    }

    // ---- front ultrasonic: wall right in front = stop (line end F) ----
    updateLineUltrasonics(now);
#if USE_FRONT_STOP
    if (lfFrontHits >= 2) {
      why = "obstacle in front";
      if (vDone || (lastOnLineHdg - initHdg >= LF_END_TURN_DEG)) reachedPointF = true;
      break;
    }
#endif

    // ---- heading vs forward distance ----
    float hdgNow = getHeadingDeg();
    {
      float d = getEncoderDistCm();
      float ds = d - lastOdoDist;
      lastOdoDist = d;
      if (ds > 0.0f) fwdOdo += ds;
    }
    if (fwdOdo - lastRingOdo >= 1.5f) {
      lastRingOdo = fwdOdo;
      dIdx = (dIdx + 1) % DRING;
      dRing[dIdx] = hdgNow;
    }
    float turnV = hdgNow - dRing[(dIdx + DRING - LF_V_SLOTS) % DRING];   // + = left, last ~12 cm

    // ---- IR ----
    bool l, c, r;
    readIR3(l, c, r);
    int code = ((int)l << 2) | ((int)c << 1) | (int)r;
    if (code == 0b111 || code == 0b101) lastJunctionMs = now;
    bool lost = (code == 0b000);
#if !KEEP_LEFT_AT_FORK
    if (code == 0b101) lost = true;     // friend: centre white + not one side only = pivot
#endif

    float tL = 0.0f, tR = 0.0f;
    const char *mode = "";

    if (!lost) {
      // ================= FRIEND'S LOGIC =================
      if (lostStartMs != 0) {                     // line back after a pivot
        float leftSwing = hdgNow - lostHdg0;
        if (endChk != 0) Serial.println(F("  [END CHECK] more line ahead -> following on"));
        else { Serial.print(F("  [FOUND] after ")); Serial.print(leftSwing, 0); Serial.println(F(" deg pivot (+ = left)")); }
        if (!vDone && endChk == 0 && leftSwing >= LF_V_SWING_DEG) {
          vDone = true;
          Serial.println(F("  [SHARP V DONE] big left pivot found the line -> next line loss = END"));
        }
        lostStartMs = 0; endChk = 0;
      }
      lastOnLineHdg = hdgNow;

      if (c) {                                    // ---- centre BLACK: follow ----
        if (l && !r)      { tL = SPD_SLOW;  tR = SPD_FAST;  lastTurnDir = -1; mode = "GENTLE-L"; }
        else if (!l && r) { tL = SPD_FAST;  tR = SPD_SLOW;  lastTurnDir =  1; mode = "GENTLE-R"; }
        else              { tL = SPD_CRUISE; tR = SPD_CRUISE;                  mode = "STRAIGHT"; }
      } else {                                    // ---- centre WHITE: sharp turn ----
        if (l && !r)      { tL = SPD_SHARP_IN; tR = SPD_FAST;    lastTurnDir = -1; mode = "SHARP-L"; }
        else if (!l && r) { tL = SPD_FAST;     tR = SPD_SHARP_IN; lastTurnDir =  1; mode = "SHARP-R"; }
        else              { tL = SPD_SHARP_IN; tR = SPD_FAST;    lastTurnDir = -1; mode = "FORK-L"; }   // 101
      }

      // sharp V detection (big left turn in a short distance)
      if (!vDone) {
        if (turnV > bigTurnMax) bigTurnMax = turnV;
        if (turnV >= LF_V_TURN_DEG) {
          vDone = true;
          Serial.print(F("  [SHARP V DONE] turned ")); Serial.print(turnV, 0);
          Serial.println(F(" deg left in 12 cm -> next line loss = END"));
        } else if (turnV >= 60.0f && !bigTurnLogged) {
          bigTurnLogged = true;
          Serial.print(F("  [TURN] ")); Serial.print(turnV, 0); Serial.println(F(" deg left in 12 cm (V needs 115)"));
        } else if (turnV < 30.0f) bigTurnLogged = false;
      }
    } else {
      // ================= ALL WHITE (000) =================
      if (lostStartMs == 0) {
        lostStartMs = now;
        lostHdg0    = hdgNow;
        lostDist0   = getEncoderDistCm();
        endZoneNow  = vDone;
        searchPhase = 0;
        srchStuckMs = 0;
        { float sum = 0.0f;
          for (int k = 0; k < 6; k++) sum += dRing[(dIdx - k + DRING) % DRING];
          gLineHdgAtEnd = sum / 6.0f; }
        // friend: pivot toward the side the line was last seen (fork / V tip: LEFT)
        searchDir = (KEEP_LEFT_AT_FORK && lastJunctionMs != 0 && now - lastJunctionMs < 500UL) ? -1 : lastTurnDir;
      }
      float rel = hdgNow - lostHdg0;            // + = turned left since the line was lost

#if STOP_AT_LINE_END
      if (endZoneNow) {
        // ---- END CHECK: any line AHEAD? (+/-45, creep 5 cm, +/-45) - never behind ----
        if (endChk == 0) {
          stopMotors(60); speedReset();
          rel = getHeadingDeg() - lostHdg0;
          if (lfFrontF < LF_END_WALL_CM) { endChk = 7; Serial.println(F("  [END CHECK] wall ahead -> end of line")); }
          else { endChk = 1; Serial.println(F("  [END CHECK] line lost after the V -> looking AHEAD only")); }
          endStepNo = endChk; endStepRel = rel; endStepMs = millis();
        }
        // a turn that is not turning (blocked) gives up after 0.6 s
        if (endChk != endStepNo || fabsf(rel - endStepRel) > 1.0f) { endStepNo = endChk; endStepRel = rel; endStepMs = now; }
        else if (endChk != 4 && now - endStepMs > 600UL) {
          if (endChk == 3 || endChk == 7) rel = 0.0f; else endChk++;
          endStepNo = -1;
        }
        float pv = SPD_PIVOT * 0.8f;
        mode = "ENDCHK";
        if (endChk == 1 || endChk == 5) {                 // look left
          if (rel >= LF_END_CHECK_DEG) endChk++; else { tL = -pv; tR = pv; }
        }
        if (endChk == 2 || endChk == 6) {                 // look right
          if (rel <= -LF_END_CHECK_DEG) endChk++; else { tL = pv; tR = -pv; }
        }
        if (endChk == 3 || endChk == 7) {                 // back to the line heading
          if (fabsf(rel) <= 3.0f) {
            if (endChk == 3) { endChk = 4; endCreep0 = getEncoderDistCm(); }
            else {
              why = "END OF LINE (nothing ahead)";
              reachedPointF   = true;
              gEndByLoss      = true;
              gEndOvershootCm = getEncoderDistCm() - lostDist0;
              break;
            }
          } else {
            float sp = (fabsf(rel) < 12.0f) ? 6.0f : pv;
            if (rel > 0.0f) { tL = sp; tR = -sp; } else { tL = -sp; tR = sp; }
          }
        }
        if (endChk == 4) {                                // creep straight, heading held
          if (getEncoderDistCm() - endCreep0 >= LF_END_CREEP_CM || lfFrontF < LF_FRONT_STOP_CM + 4.0f) endChk = 5;
          else {
            float k = constrain(0.4f * rel, -4.0f, 4.0f);  // turned left -> left wheel faster
            tL = 10.0f + k; tR = 10.0f - k;
          }
        }
      } else
#endif
      {
        // ---- friend: pivot IMMEDIATELY toward lastTurnDir (max 150 deg, then the other way) ----
        float progress = (searchDir < 0) ? rel : -rel;
        bool blocked = false;                              // pivot not turning for 0.6 s (wall)
        if (srchStuckMs == 0 || fabsf(hdgNow - srchStuckHdg) > 1.0f) { srchStuckHdg = hdgNow; srchStuckMs = now; }
        else if (now - srchStuckMs > 600UL) { blocked = true; srchStuckMs = 0; }
        if (progress >= LF_SEARCH_DEG || blocked) {
          if (searchPhase == 0) { searchPhase = 1; searchDir = -searchDir; }
          else {
            why = blocked ? "pivot blocked" : "line not found";
            if (vDone || (lastOnLineHdg - initHdg >= LF_END_TURN_DEG)) reachedPointF = true;
            break;
          }
        }
        tL = (searchDir < 0) ? -SPD_PIVOT :  SPD_PIVOT;
        tR = (searchDir < 0) ?  SPD_PIVOT : -SPD_PIVOT;
        mode = (searchDir < 0) ? "PIVOT-L" : "PIVOT-R";
      }
    }

    speedDrive(tL, tR);

    if (logOn && (now - lastLogMs >= LF_LOG_MS)) {
      lastLogMs = now;
      Serial.print(F("IR=")); Serial.print(l); Serial.print(c); Serial.print(r);
      Serial.print(F(" ")); Serial.print(mode);
      Serial.print(F(" | tgt ")); Serial.print(spTgtL, 1); Serial.print(F("/")); Serial.print(spTgtR, 1);
      Serial.print(F(" cm/s | meas ")); Serial.print(spMeasL, 1); Serial.print(F("/")); Serial.print(spMeasR, 1);
      Serial.print(F(" | pwm ")); Serial.print((int)spPwmL); Serial.print(F("/")); Serial.print((int)spPwmR);
      Serial.print(F(" | US L=")); Serial.print(lfSideL, 1);
      Serial.print(F(" F="));      Serial.print(lfFrontF, 1);
      Serial.print(F(" R="));      Serial.println(lfSideR, 1);
    }
    delay(2);
  }

  stopMotors(60);
  if (!gEndByLoss) {
    float sum = 0.0f;
    for (int k = 0; k < 6; k++) sum += dRing[(dIdx - k + DRING) % DRING];
    gLineHdgAtEnd = sum / 6.0f;
    gEndOvershootCm = 0.0f;
  }
  Serial.print(F("[FOLLOW] stopped: ")); Serial.print(why);
  Serial.print(F(" | time "));     Serial.print((millis() - t0) * 0.001f, 1);
  Serial.print(F(" s | fwdOdo ")); Serial.print(fwdOdo, 0);
  Serial.print(F(" cm | V done ")); Serial.print(vDone ? F("yes") : F("NO"));
  Serial.print(F(" | biggest turn before V ")); Serial.print(bigTurnMax, 0); Serial.println(F(" deg/12cm"));
  while (Serial.available()) Serial.read();

  return (!manualAbort && !timeUp() && reachedPointF);
}

// ============================================================================
// SETUP / LOOP
// ============================================================================
void setup() {
  digitalWrite(R_ENA, HIGH);
  digitalWrite(L_ENB, HIGH);
  pinMode(R_ENA, OUTPUT);
  pinMode(L_ENB, OUTPUT);
  analogWriteFreq(250);
  analogWriteRange(255);
  pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  pinMode(L_IN3, OUTPUT); pinMode(L_IN4, OUTPUT);
  releaseMotors();

  pinMode(F_TRIG, OUTPUT); pinMode(F_ECHO, INPUT);
  pinMode(R_TRIG, OUTPUT); pinMode(R_ECHO, INPUT);
  pinMode(L_TRIG, OUTPUT); pinMode(L_ECHO, INPUT);
  digitalWrite(F_TRIG, LOW); digitalWrite(R_TRIG, LOW); digitalWrite(L_TRIG, LOW);

  pinMode(ENC_R_C1, INPUT_PULLUP); pinMode(ENC_R_C2, INPUT_PULLUP);
  pinMode(ENC_L_C1, INPUT_PULLUP); pinMode(ENC_L_C2, INPUT_PULLUP);
  prevStateR = ((uint8_t)digitalRead(ENC_R_C1) << 1) | (uint8_t)digitalRead(ENC_R_C2);
  prevStateL = ((uint8_t)digitalRead(ENC_L_C1) << 1) | (uint8_t)digitalRead(ENC_L_C2);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C1), isrEncR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C2), isrEncR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_C1), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_C2), isrEncL, CHANGE);

  pinMode(PIN_IR_L, INPUT);
  pinMode(PIN_IR_C, INPUT);
  pinMode(PIN_IR_R, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  delay(500);
  resetEncoders();
  followLine(true);          // 3 s countdown, then follows until the line end / a key
  releaseMotors();
}

void loop() {
  digitalWrite(LED_BUILTIN, (millis() / 500) % 2);   // done: blinking
  delay(50);
}
