// ============================================================================
// 3-IR LINE FOLLOWER - standalone (taken from the full-race firmware, Phase 5)
//   IR: L = GP17, C = GP22, R = GP16 (HIGH = black)
//   Motors: R ENA GP0 IN1 GP1 IN2 GP2 | L IN3 GP3 IN4 GP4 ENB GP5 (inverted PWM)
//   Encoders: R GP12/GP13, L GP14/GP15 (quadrature)
//   Ultrasonics: F 8/9, R 6/7, L 18/19 (wall assist + front stop)
//   No gyro: heading from the encoders.
//   Starts 3 s after power-up. Serial (115200): + faster, - slower, v log on/off,
//   any other key = STOP.
//   Line end: once the hairpin is done (>= 195 deg total left turn on the line, or
//   85 deg within 15 cm) a lost line is checked AHEAD only (+/-45 deg, creep 5 cm,
//   +/-45 deg) - the line behind is never picked up, so no U-turn. Nothing ahead
//   (or a wall < 12 cm ahead) = end of line: it stops there.
// ============================================================================
#include <Arduino.h>
#include <math.h>

#define SWAP_LR_IR        0   // 1 if 'L' and 'R' IR are swapped
#define KEEP_LEFT_AT_FORK 1   // fork: main line goes LEFT
#define USE_WALL_ASSIST   1   // side ultrasonics keep the wheels off walls
#define USE_FRONT_STOP    1   // stop if something is right in front
#define STOP_AT_LINE_END  1   // after the hairpin, line lost = end of line

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
const float RIGHT_FF       = 0.86f;

int         lfBasePwm            = 135;    // Straight-line speed (0..255); '+'/'-' adjust live
const int   LF_BASE_MIN          = 105;
const int   LF_BASE_MAX          = 190;
const int   LF_MIN_PWM_L         = 80;
const int   LF_MIN_PWM_R         = 70;

const float LF_KP                = 40.0f;  // Steering PWM per error step
const float LF_KD                = 70.0f;  // Kick when error changes (damps wobble)
const float LF_KD_TAU_MS         = 40.0f;  // Decay constant for derivative kick
const float LF_SLOW_PER_ERR      = 18.0f;  // Slow down per error step
const unsigned long LF_HARD_SPIN_MS    = 120UL;
const int   LF_SPIN_FWD          = 138;
const int   LF_SPIN_REV          = 120;
const unsigned long LF_SEARCH_BRAKE_MS = 90UL;
const unsigned long LF_FOUND_BRAKE_MS  = 45UL;

// Hairpin (E) & Line End (F) detection
const float LF_HAIRPIN_DEG       = 85.0f;  // Left turn this big...
const float LF_HAIRPIN_WINDOW_CM = 15.0f;  // ...within 15 cm forward travel = hairpin done
const float LF_LINE_END_MIN_CM   = 6.0f;   // Min travel after hairpin before Point F counts
const unsigned long LF_LINE_END_LOST_MS = 150UL;
const float LF_SETTLE_CM         = 12.0f;  // Slow settle distance after big pivot
const int   LF_SETTLE_PWM        = 112;
const float LF_CURVE_DEG         = 25.0f;  // Heading change in last ~7.5 cm -> curve
const float LF_CURVE_SPEED       = 0.80f;
const unsigned long LF_LOST_GRACE_MS = 40UL;
const float LF_SEARCH_DEG        = 150.0f; // max pivot each way: the line BEHIND (~180) is never reached

// End of line: nothing behind is ever accepted near the end
const float LF_END_TURN_DEG      = 195.0f; // total LEFT turn since the line start that means "hairpin done"
                                           // (A -> F turns ~260 deg left; before the hairpin at most ~130)
const float LF_END_CHECK_DEG     = 45.0f;  // end check: look this far left and right (line ahead only)
const float LF_END_CREEP_CM      = 5.0f;   // ... then creep straight this far and look again
const float LF_END_WALL_CM       = 12.0f;  // line ends with a wall this close ahead = end, no pivoting

// Wall safety (raw ultrasonic cm)
const float LF_NEAR_WALL_CM      = 9.0f;
const float LF_PASSAGE_CM        = 12.0f;
const float LF_WALL_PAD_CM       = 4.0f;
const float LF_WALL_PAD_K        = 14.0f;
const int   LF_WALL_PAD_MAX      = 40;
const float LF_WALL_CENTER_K     = 5.0f;
const int   LF_WALL_CENTER_MAX   = 30;
const int   LF_NEAR_WALL_TURN_MAX= 55;
const float LF_NEAR_WALL_SPEED   = 0.85f;
const float LF_LOST_CREEP_CM     = 10.0f;
const float LF_FRONT_STOP_CM     = 6.0f;
const float LF_SPIN_WALL_CM      = 7.0f;

const unsigned long LF_MAX_STANDALONE_MS = 120000UL;
const unsigned long LF_LOG_MS            = 150UL;


int lastSignedL = 0, lastSignedR = 0;
bool timeUp() { return false; }
float gLineHdgAtEnd = 0.0f, gEndOvershootCm = 0.0f;   // kept for the full firmware's T2 step
bool  gEndByLoss = false;

static inline float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

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

float getHeadingDeg()    { return getEncoderHeadingDeg(); }   // no gyro: encoder heading

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

void setMotorPWM(int leftPwm, int rightPwm) {
  analogWrite(L_ENB, 255 - constrain(leftPwm,  0, 255));
  analogWrite(R_ENA, 255 - constrain(rightPwm, 0, 255));
}

void setSignedMotors(int lCmd, int rCmd) {
  lastSignedL = lCmd;
  lastSignedR = rCmd;

  if (abs(lCmd) < 20) {
    digitalWrite(L_IN3, LOW); digitalWrite(L_IN4, LOW);
    analogWrite(L_ENB, 0);
  } else {
    setLeftDir(lCmd > 0 ? 1 : -1);
    analogWrite(L_ENB, 255 - constrain(abs(lCmd), 75, 245));
  }

  if (abs(rCmd) < 20) {
    digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
    analogWrite(R_ENA, 0);
  } else {
    setRightDir(rCmd > 0 ? 1 : -1);
    int pwmR = constrain((int)roundf((float)abs(rCmd) * RIGHT_FF), 68, 245);
    analogWrite(R_ENA, 255 - pwmR);
  }
}

// Exact motor drive curve from your working 3-IR line follower (MIN_PWM 80 / 70, MAX 245 / 240)
void driveLineMotors(int lCmd, int rCmd) {
  lastSignedL = lCmd;
  lastSignedR = rCmd;

  if (abs(lCmd) < 20) {
    digitalWrite(L_IN3, LOW); digitalWrite(L_IN4, LOW);
    analogWrite(L_ENB, 0); // active brake
  } else {
    setLeftDir(lCmd > 0 ? 1 : -1);
    analogWrite(L_ENB, 255 - constrain(abs(lCmd), LF_MIN_PWM_L, 245));
  }

  if (abs(rCmd) < 20) {
    digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
    analogWrite(R_ENA, 0); // active brake
  } else {
    setRightDir(rCmd > 0 ? 1 : -1);
    int pwmR = constrain((int)roundf((float)abs(rCmd) * RIGHT_FF), LF_MIN_PWM_R, 240);
    analogWrite(R_ENA, 255 - pwmR);
  }
}

void releaseMotors() {
  lastSignedL = 0; lastSignedR = 0;
  analogWrite(L_ENB, 255);
  analogWrite(R_ENA, 255);
  digitalWrite(L_IN3, LOW); digitalWrite(L_IN4, LOW);
  digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
}

void stopMotors(int brakeMs = 35) {
  lastSignedL = 0; lastSignedR = 0;
  digitalWrite(L_IN3, LOW); digitalWrite(L_IN4, LOW);
  digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
  analogWrite(L_ENB, 0);
  analogWrite(R_ENA, 0);
  delay(brakeMs);
  releaseMotors();
}

// Majority-of-3 debounced IR read (zero single-sample glitches)
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

inline bool irL() {
#if SWAP_LR_IR
  return irRaw(PIN_IR_R);
#else
  return irRaw(PIN_IR_L);
#endif
}
inline bool irR() {
#if SWAP_LR_IR
  return irRaw(PIN_IR_L);
#else
  return irRaw(PIN_IR_R);
#endif
}
inline bool irC() { return irRaw(PIN_IR_C); }
inline bool anyIrBlack() {
  bool l, c, r;
  readIR3(l, c, r);
  return (l || c || r);
}

// Phase 5 background ultrasonic scanner (exact 15 ms L, R, L, R, F cadence)
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

bool followLine(bool standaloneTest = true) {
  if (timeUp()) return false;

  if (standaloneTest) {
    Serial.println(F("\n[Phase 5 FOLLOW] starting in 3 s... (+ faster, - slower, v log, any key STOP)"));
    for (int i = 3; i > 0; i--) { Serial.print(i); Serial.println(F("...")); delay(1000); }
    while (Serial.available()) Serial.read();
  } else {
    Serial.println(F("[Phase 5] Working 3-IR Line Follower + Hairpin E + Wall Safety ON!"));
  }

  // Fresh ultrasonic ring warm-up (10 clean pings before moving)
  resetLineUltrasonics();
  for (int i = 0; i < 10; i++) {
    updateLineUltrasonics(millis());
    delay(16);
  }

  float prevE = 0.0f, dTerm = 0.0f;
  int   lastSide = -1;                 // -1 = line last seen LEFT, +1 = RIGHT
  unsigned long lastSideMs     = 0;
  unsigned long lastJunctionMs = 0;    // last time 111 or 101 was seen
  int   hardSide = 0;
  unsigned long hardStartMs = 0;

  unsigned long lostStartMs = 0;
  int   searchDir = -1, searchPhase = 0;
  float lostHdg0 = 0.0f, lostDist0 = 0.0f;
  bool  lostNearWall = false;

  bool  inPassage = false;
  int   passIn = 0, passOut = 0;

  // Heading history (uses Core 1 6-axis Gyro with automatic encoder failover; + = LEFT)
  const int RING = 16;
  float hRing[RING];
  float initHdg = getHeadingDeg();
  for (int i = 0; i < RING; i++) hRing[i] = initHdg;
  int hIdx = 0;
  unsigned long lastRingMs = 0;

  // Heading vs FORWARD distance (10 slots x 1.5 cm = 15 cm window) -> Hairpin E / curve detection
  const int DRING = 10;
  float dRing[DRING];
  for (int i = 0; i < DRING; i++) dRing[i] = initHdg;
  int   dIdx = 0;
  float fwdOdo = 0.0f, lastOdoDist = getEncoderDistCm(), lastRingOdo = 0.0f;
  bool  hairpinDone = false;
  float hairpinOdo  = 0.0f;
  float settleUntilOdo = -1.0f;

  float searchHdg0 = 0.0f;
  int   endChk = 0;                    // end-of-line check step (0 = not checking)
  float endCreep0 = 0.0f;
  int   endStepNo = -1; float endStepRel = 0.0f; unsigned long endStepMs = 0;
  float srchStuckHdg = 0.0f; unsigned long srchStuckMs = 0;
  float lastOnLineHdg = 0.0f;          // heading the last time an IR saw the line
  bool  endZoneNow = false;            // decided once, when the line is lost
  bool  searching  = false;
  unsigned long searchStartMs = 0;

  bool  logOn = true;
  unsigned long lastLogMs  = 0;
  unsigned long t0         = millis();
  unsigned long lastLoopMs = t0;

  const char *why = "unknown";
  bool reachedPointF = false;
  bool manualAbort   = false;

  while (true) {
    unsigned long now = millis();
    float dtMs = (float)(now - lastLoopMs);
    lastLoopMs = now;

    if (timeUp()) { why = "race time limit"; break; }
    if (standaloneTest && (now - t0 > LF_MAX_STANDALONE_MS)) { why = "2-min safety limit"; break; }

    // Live serial controls (only active in USB standalone line-follow test mode)
    if (standaloneTest && Serial.available()) {
      char k = Serial.read();
      if (k == '+') {
        lfBasePwm = min(lfBasePwm + 5, LF_BASE_MAX);
        Serial.print(F("[lfBasePwm] ")); Serial.println(lfBasePwm);
      } else if (k == '-') {
        lfBasePwm = max(lfBasePwm - 5, LF_BASE_MIN);
        Serial.print(F("[lfBasePwm] ")); Serial.println(lfBasePwm);
      } else if (k == 'v' || k == 'V') {
        logOn = !logOn;
      } else if (k != '\r' && k != '\n') {
        why = "key pressed";
        manualAbort = true;
        break;
      }
    }

    // ---- Background ultrasonics ----
    updateLineUltrasonics(now);
#if USE_FRONT_STOP
    if (lfFrontHits >= 2) {
      why = "obstacle in front";
      if (hairpinDone || (lastOnLineHdg - initHdg >= LF_END_TURN_DEG)) reachedPointF = true;
      break;
    }
#endif

    // ---- Heading & forward-distance rings ----
    float hdgNow = getHeadingDeg();
    if (now - lastRingMs >= 10UL) {
      lastRingMs = now;
      hIdx = (hIdx + 1) % RING;
      hRing[hIdx] = hdgNow;
    }
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

    float turn15 = hdgNow - dRing[(dIdx + 1) % DRING];          // heading change over last ~15 cm (+ = left)
    float turn7  = hdgNow - dRing[(dIdx + DRING - 4) % DRING];  // heading change over last ~7.5 cm
    if (!hairpinDone && turn15 >= LF_HAIRPIN_DEG) {
      hairpinDone    = true;
      hairpinOdo     = fwdOdo;
      settleUntilOdo = fwdOdo + LF_SETTLE_CM;
      Serial.print(F("  [HAIRPIN DONE] turned ")); Serial.print(turn15, 0);
      Serial.println(F(" deg left -> next line loss = END (Point F)"));
    }

    // ---- Debounced 3-IR read ----
    bool l, c, r;
    readIR3(l, c, r);
    int code = ((int)l << 2) | ((int)c << 1) | (int)r;

    // Passage detection with hysteresis
    if (lfSideL < LF_PASSAGE_CM && lfSideR < LF_PASSAGE_CM) {
      passOut = 0;
      if (++passIn >= 3) inPassage = true;
    } else {
      passIn = 0;
      if (lfSideL > LF_PASSAGE_CM + 4.0f || lfSideR > LF_PASSAGE_CM + 4.0f) {
        if (++passOut >= 4) inPassage = false;
      }
    }
    bool nearWall = inPassage || (lfSideL < LF_NEAR_WALL_CM) || (lfSideR < LF_NEAR_WALL_CM);

    // ---- Wall assist (+ = steer RIGHT, - = steer LEFT) ----
    float wallTurn = 0.0f;
#if USE_WALL_ASSIST
    if (inPassage) {
      wallTurn = constrain(LF_WALL_CENTER_K * (lfSideR - lfSideL),
                           -(float)LF_WALL_CENTER_MAX, (float)LF_WALL_CENTER_MAX);
    }
    if (lfSideL < LF_WALL_PAD_CM && lfSideL <= lfSideR) {
      wallTurn += constrain(LF_WALL_PAD_K * (LF_WALL_PAD_CM - lfSideL), 0.0f, (float)LF_WALL_PAD_MAX);
    } else if (lfSideR < LF_WALL_PAD_CM && lfSideR < lfSideL) {
      wallTurn -= constrain(LF_WALL_PAD_K * (LF_WALL_PAD_CM - lfSideR), 0.0f, (float)LF_WALL_PAD_MAX);
    }
#endif

    // ---- Line error: negative = line is to the LEFT (turn left) ----
    bool  lost = false;
    float e    = 0.0f;
    switch (code) {
      case 0b010: e =  0.0f; break;
      case 0b110: e = -1.0f; break;
      case 0b100: e = -2.0f; break;
      case 0b011: e =  1.0f; break;
      case 0b001: e =  2.0f; break;
#if KEEP_LEFT_AT_FORK
      case 0b111: e = -0.7f; break; // Junction / fork D: lean left
      case 0b101: e = -2.0f; break; // Split lines around centre: take left
#else
      case 0b111: e =  0.0f; break;
      case 0b101: e =  0.0f; break;
#endif
      default: lost = true; break;  // 000
    }
    if (!lost && e != 0.0f) { lastSide = (e < 0.0f) ? -1 : 1; lastSideMs = now; }
    if (code == 0b111 || code == 0b101) lastJunctionMs = now;

    int cmdL = 0, cmdR = 0;
    const char *mode = "LINE";

    if (!lost) {
      lastOnLineHdg = hdgNow;
      if (lostStartMs != 0) {
        lostStartMs = 0;
        if (endChk != 0) { Serial.println(F("  [END CHECK] more line ahead -> following on")); endChk = 0; }
        prevE = e;
        dTerm = 0.0f;
        hardStartMs = now;
        if (searching) {
          searching = false;
          float swung = fabsf(getHeadingDeg() - searchHdg0);
          if (now - searchStartMs > 150UL) {
            driveLineMotors(0, 0);
            delay(LF_FOUND_BRAKE_MS);
          }
          if (swung > 45.0f) settleUntilOdo = fwdOdo + LF_SETTLE_CM;
          Serial.print(F("  [FOUND] after ")); Serial.print(swung, 0); Serial.println(F(" deg pivot"));
        }
      }

      if (e != prevE) dTerm = LF_KD * (e - prevE);
      else            dTerm *= expf(-dtMs / LF_KD_TAU_MS);
      prevE = e;

      float base = (float)lfBasePwm - LF_SLOW_PER_ERR * fabsf(e);
      if (fabsf(turn7) > LF_CURVE_DEG) base *= LF_CURVE_SPEED;
      if (fwdOdo < settleUntilOdo)     base = min(base, (float)LF_SETTLE_PWM);
      float turn = LF_KP * e + dTerm;

      if (nearWall) {
        base *= LF_NEAR_WALL_SPEED;
        turn  = constrain(turn, -(float)LF_NEAR_WALL_TURN_MAX, (float)LF_NEAR_WALL_TURN_MAX);
        turn += wallTurn;
        cmdL  = max((int)(base + turn), 0);
        cmdR  = max((int)(base - turn), 0); // never pivot backwards near a wall
        hardSide = 0;
        mode = inPassage ? "PASSAGE" : "NEARWALL";
      } else {
        turn += wallTurn;
        int side = (fabsf(e) >= 2.0f) ? ((e < 0.0f) ? -1 : 1) : 0;
        if (side != hardSide) { hardSide = side; hardStartMs = now; }
        if (side != 0 && (now - hardStartMs > LF_HARD_SPIN_MS)) {
          cmdL = (side < 0) ? -LF_SPIN_REV :  LF_SPIN_FWD;
          cmdR = (side < 0) ?  LF_SPIN_FWD : -LF_SPIN_REV;
          mode = "PIVOT";
        } else {
          cmdL = (int)(base + turn);
          cmdR = (int)(base - turn);
          if (fabsf(e) < 2.0f) { cmdL = max(cmdL, 0);   cmdR = max(cmdR, 0);   }
          else                 { cmdL = max(cmdL, -60); cmdR = max(cmdR, -60); }
        }
      }
    } else {
      // ---------------- LINE LOST (000) ----------------
      if (lostStartMs == 0) {
        lostStartMs  = now;
        lostHdg0     = hdgNow;
        lostDist0    = getEncoderDistCm();
        // end of the line only if the robot had ALREADY turned the hairpin while
        // on the line (search pivots do not count)
        endZoneNow   = hairpinDone || (lastOnLineHdg - initHdg >= LF_END_TURN_DEG);
        {   // line direction = average heading over the last ~9 cm ON the line
          float sum = 0.0f;
          for (int k = 0; k < 6; k++) sum += dRing[(dIdx - k + DRING) % DRING];
          gLineHdgAtEnd = sum / 6.0f;
        }
        searchPhase  = 0;
        lostNearWall = nearWall;
        if (KEEP_LEFT_AT_FORK && lastJunctionMs != 0 && (now - lastJunctionMs < 500UL)) {
          searchDir = -1; // V tip / fork: always LEFT
        } else if (now - lastSideMs < 400UL) {
          searchDir = lastSide;
        } else {
          float rot = hRing[hIdx] - hRing[(hIdx + 1) % RING];
          if (rot > 1.5f)       searchDir =  1;
          else if (rot < -1.5f) searchDir = -1;
          else                  searchDir = lastSide;
        }
      }

#if STOP_AT_LINE_END
      // ---- END CHECK (after the hairpin): is there any line AHEAD? ----
      // look +/-45 deg, creep 5 cm straight, look +/-45 again. The line behind
      // (180 deg away) can never be found this way, so no U-turn.
      if (endZoneNow && (now - lostStartMs >= LF_LOST_GRACE_MS)) {
        if (endChk == 0) {
          endChk = 1;
          driveLineMotors(0, 0);
          delay(LF_SEARCH_BRAKE_MS);
          if (lfFrontF < LF_END_WALL_CM) {
            endChk = 7;                                   // wall right ahead: this IS the end, no pivoting
            Serial.println(F("  [END CHECK] line lost with a wall ahead -> end of line"));
          } else {
            Serial.println(F("  [END CHECK] line lost after the hairpin -> looking AHEAD only"));
          }
          endStepMs = millis(); endStepRel = getHeadingDeg() - lostHdg0; endStepNo = endChk;
        }
        float rel = getHeadingDeg() - lostHdg0;          // + = turned left of the line heading
        // a turn that is not turning (blocked by a wall) gives up after 0.5 s
        if (endChk != endStepNo || fabsf(rel - endStepRel) > 1.0f) { endStepNo = endChk; endStepRel = rel; endStepMs = millis(); }
        else if (endChk != 4 && millis() - endStepMs > 500UL) {
          if (endChk == 3 || endChk == 7) rel = 0.0f;     // cannot turn back: accept where it is
          else endChk++;
          endStepNo = -1;
        }
        int pf = (int)(LF_SPIN_FWD * 0.85f), pr = (int)(LF_SPIN_REV * 0.85f);
        mode = "ENDCHK";
        cmdL = 0; cmdR = 0;
        if (endChk == 1 || endChk == 5) {                 // look left
          if (rel >= LF_END_CHECK_DEG || lfSideL < LF_SPIN_WALL_CM) endChk++;
          else { cmdL = -pr; cmdR = pf; }
        }
        if (endChk == 2 || endChk == 6) {                 // look right
          if (rel <= -LF_END_CHECK_DEG || lfSideR < LF_SPIN_WALL_CM) endChk++;
          else { cmdL = pf; cmdR = -pr; }
        }
        if (endChk == 3 || endChk == 7) {                 // back to the line heading
          if (fabsf(rel) <= 3.0f) {
            if (endChk == 3) { endChk = 4; endCreep0 = getEncoderDistCm(); }
            else {
              driveLineMotors(0, 0);
              why = "END OF LINE (nothing ahead) -> T2";
              reachedPointF   = true;
              gEndByLoss      = true;
              gEndOvershootCm = getEncoderDistCm() - lostDist0;
              break;
            }
          } else {
            int sp = (fabsf(rel) < 12.0f) ? 95 : pf;
            if (rel > 0.0f) { cmdL = sp; cmdR = -sp; } else { cmdL = -sp; cmdR = sp; }
          }
        }
        if (endChk == 4) {                                // creep straight ahead, heading held
          if (getEncoderDistCm() - endCreep0 >= LF_END_CREEP_CM || lfFrontF < LF_FRONT_STOP_CM + 4.0f) endChk = 5;
          else {
            float c = constrain(4.0f * rel, -30.0f, 30.0f); // turned left -> left wheel faster
            cmdL = (int)(118.0f + c);
            cmdR = (int)(118.0f - c);
          }
        }
      } else
#endif
      if (now - lostStartMs < LF_LOST_GRACE_MS) {
        cmdL = lastSignedL; cmdR = lastSignedR; mode = "GRACE";
      } else if (!searching && !(lostNearWall || nearWall) && (now - lostStartMs < LF_LOST_GRACE_MS + LF_SEARCH_BRAKE_MS)) {
        cmdL = 0; cmdR = 0; mode = "BRAKE";
      } else if (lostNearWall || nearWall) {
        float turn = (float)searchDir * 35.0f + wallTurn;
        int   base = 112;
        cmdL = max((int)(base + turn), 0);
        cmdR = max((int)(base - turn), 0);
        mode = "CREEP";
        if (getEncoderDistCm() - lostDist0 > LF_LOST_CREEP_CM) {
          why = "line lost near a wall (stopped for safety)";
          break;
        }
      } else {
        if (!searching) {
          searching     = true;
          searchHdg0    = hdgNow;
          searchStartMs = now;
          lostHdg0      = searchHdg0;
        }
        float rel      = hdgNow - lostHdg0; // + = turned left
        float progress = (searchDir < 0) ? rel : -rel;
        bool wallAhead = (searchDir < 0) ? (lfSideL < LF_SPIN_WALL_CM) : (lfSideR < LF_SPIN_WALL_CM);
        // pivot blocked (not turning for 0.6 s) counts like a wall on that side
        if (fabsf(hdgNow - srchStuckHdg) > 1.0f || srchStuckMs == 0) { srchStuckHdg = hdgNow; srchStuckMs = now; }
        else if (now - srchStuckMs > 600UL) { wallAhead = true; srchStuckMs = 0; }
        if (progress >= LF_SEARCH_DEG || wallAhead) {
          if (searchPhase == 0) {
            searchPhase = 1;
            searchDir   = -searchDir;
          } else {
            why = wallAhead ? "search blocked by wall" : "line not found (end of line?)";
            if (hairpinDone) reachedPointF = true;
            break;
          }
        }
        cmdL = (searchDir < 0) ? -LF_SPIN_REV :  LF_SPIN_FWD;
        cmdR = (searchDir < 0) ?  LF_SPIN_FWD : -LF_SPIN_REV;
        mode = (searchDir < 0) ? "SEARCH-L" : "SEARCH-R";
      }
    }

    driveLineMotors(cmdL, cmdR);

    if (logOn && (now - lastLogMs >= LF_LOG_MS)) {
      lastLogMs = now;
      Serial.print(F("IR=")); Serial.print(l); Serial.print(c); Serial.print(r);
      Serial.print(F(" e=")); Serial.print(lost ? 9.0f : e, 1);
      Serial.print(F(" "));   Serial.print(mode);
      Serial.print(F(" | L=")); Serial.print(cmdL); Serial.print(F(" R=")); Serial.print(cmdR);
      Serial.print(F(" | US L=")); Serial.print(lfSideL, 1);
      Serial.print(F(" F="));      Serial.print(lfFrontF, 1);
      Serial.print(F(" R="));      Serial.print(lfSideR, 1);
      Serial.print(F(" wall="));   Serial.println(wallTurn, 0);
    }

    delay(2);
  }

  stopMotors(60);
  if (!gEndByLoss) {                       // ended another way: line direction from the last ~9 cm
    float sum = 0.0f;
    for (int k = 0; k < 6; k++) sum += dRing[(dIdx - k + DRING) % DRING];
    gLineHdgAtEnd = sum / 6.0f;
    gEndOvershootCm = 0.0f;
  }
  Serial.print(F("[Phase 5] stopped: ")); Serial.print(why);
  Serial.print(F(" | time "));            Serial.print((millis() - t0) * 0.001f, 1);
  Serial.print(F(" s | fwdOdo "));        Serial.print(fwdOdo, 0); Serial.println(F(" cm"));
  while (Serial.available()) Serial.read();

  return (!manualAbort && !timeUp() && (reachedPointF || hairpinDone));
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
