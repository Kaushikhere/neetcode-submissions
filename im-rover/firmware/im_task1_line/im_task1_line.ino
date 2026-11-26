// ============================================================================
// IM Competition — TASK 1 (Back-Wall Localization -> T1) + LINE & WALL FOLLOW -> T2
// Task 1 part is your tested code (unchanged except 3 marked [ADDED] lines).
// Section 10 (bottom) = line + wall follower to T2.
//   Menu:  f / w / l / t (as before)   n = line-follow test from T1   i = IR/sonar monitor
// ============================================================================

#include <Arduino.h>
#include <math.h>

// 1 = Serial Menu (f / w / l / t / n / i) | 0 = Standalone battery mode (3 s countdown)
#define INTERACTIVE_MENU 1

// ============================================================================
// 1. CONFIRMED PINS (RASPBERRY PI PICO)
// ============================================================================
const int R_ENA = 0, R_IN1 = 1, R_IN2 = 2;
const int L_IN3 = 3, L_IN4 = 4, L_ENB = 5;

const int F_TRIG = 8,  F_ECHO = 9;
const int R_TRIG = 6,  R_ECHO = 7;
const int L_TRIG = 18, L_ECHO = 19;

const int ENC_R_C1 = 12, ENC_R_C2 = 13;
const int ENC_L_C1 = 14, ENC_L_C2 = 15;

// Confirmed working polarity from your Mode 3 + 'f'/'w' tests:
#define INVERT_MOTOR_DIR 0
const int8_t ENC_R_SIGN = -1;
const int8_t ENC_L_SIGN =  1;

// ============================================================================
// 2. ROBOT & FIELD MEASUREMENTS (UPDATE T1 WITH TAPE MEASURE)
// ============================================================================
const float TICKS_PER_CM_L   = 60.5f;
const float TICKS_PER_CM_R   = 60.7f;
const float WHEELBASE_CM     = 19.0f; // Adjust if 'w' visually over/under-turns 90 deg

const float FRONT_OFFSET_CM  = 17.0f; // Rear axle midpoint -> Front US face
const float SIDE_OFFSET_CM   = 10.7f; // Centerline -> Left/Right US face
const float DROP_OFFSET_CM   = 0.0f;  // Rear axle midpoint -> drop spot (+forward)

const float BOX_WIDTH_CM     = 122.0f;
const float T1_X_CM          = 58.4f;
const float T1_Y_CM          = 97.0f;

// ============================================================================
// 3. TUNED MOTOR, SYNC & BRAKE SETTINGS
// ============================================================================
// From your 'f' test (L=1220, R=1442): right motor is ~16% faster at equal PWM
const float RIGHT_FF         = 0.86f;

// Turning (200 Hz proportional ramp-down)
const int   TURN_FAST_PWM    = 175;
const int   TURN_CREEP_PWM   = 130;
const float TURN_LEAD_DEG    = 2.2f;
const float TURN_SYNC_KP     = 0.45f; // Keeps pivot centered on axle

// Straight driving (100 Hz heading-lock PD)
const int   DRIVE_CRUISE_PWM = 180;
const int   DRIVE_SLOW_PWM   = 145;
const float DRIVE_SYNC_KP    = 1.40f; // Strong tick-lock keeps |R-L| < 10 ticks
const float DRIVE_SYNC_KD    = 0.08f;
const float DRIVE_MAX_CORR   = 75.0f;

// Anti-stall boost (if floor friction holds a wheel)
const unsigned long STALL_MS = 100;
const int   STALL_STEP_PWM   = 12;
const int   STALL_MAX_PWM    = 65;

const unsigned long HARD_STOP_MS = 88000UL;

float g_backHdg = NAN;  // [ADDED] encoder heading that faces the back wall (set in Phase 3)

// ============================================================================
// 4. CONTINUOUS 4x QUADRATURE ENCODERS & GLOBAL HEADING
// ============================================================================
volatile long leftCount  = 0;
volatile long rightCount = 0;
volatile uint8_t prevL = 0, prevR = 0;

const int8_t QEM[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void isrEncR() {
  uint8_t s = ((uint8_t)digitalRead(ENC_R_C1) << 1) | (uint8_t)digitalRead(ENC_R_C2);
  rightCount += (long)QEM[(prevR << 2) | s] * ENC_R_SIGN;
  prevR = s;
}

void isrEncL() {
  uint8_t s = ((uint8_t)digitalRead(ENC_L_C1) << 1) | (uint8_t)digitalRead(ENC_L_C2);
  leftCount += (long)QEM[(prevL << 2) | s] * ENC_L_SIGN;
  prevL = s;
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

// Continuous unwrapped heading in degrees (+ = CCW / Left)
float getHeadingDeg() {
  long l, r;
  readEncoders(l, r);
  float dR_cm = (float)r / TICKS_PER_CM_R;
  float dL_cm = (float)l / TICKS_PER_CM_L;
  return ((dR_cm - dL_cm) / WHEELBASE_CM) * (180.0f / (float)M_PI);
}

static inline float wrapAngle180(float deg) {
  while (deg >  180.0f) deg -= 360.0f;
  while (deg <= -180.0f) deg += 360.0f;
  return deg;
}

unsigned long runStartMs = 0;

bool timeUp() {
  if (INTERACTIVE_MENU) return false;
  return (millis() - runStartMs >= HARD_STOP_MS);
}

// ============================================================================
// 5. MOTOR HELPERS (250 Hz INVERTED PWM + ACTIVE L298N BRAKE)
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

void setMotorPWM(int leftPwm, int rightPwm) {
  analogWrite(L_ENB, 255 - constrain(leftPwm,  0, 255));
  analogWrite(R_ENA, 255 - constrain(rightPwm, 0, 255));
}

void releaseMotors() {
  analogWrite(L_ENB, 255); // Inverted PWM: 255 = OFF
  analogWrite(R_ENA, 255);
  digitalWrite(L_IN3, LOW); digitalWrite(L_IN4, LOW);
  digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
}

void stopMotors() {
  // Active L298N dynamic brake (both inputs pulled HIGH + ENA/ENB HIGH via PC817)
  digitalWrite(L_IN3, LOW); digitalWrite(L_IN4, LOW);
  digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
  analogWrite(L_ENB, 0);
  analogWrite(R_ENA, 0);
  delay(45);
  releaseMotors();
}

// ============================================================================
// 6. ULTRASONICS (MEDIAN FILTER + CLEAN ECHO GAP)
// ============================================================================
float readUltrasonicSingle(int trigPin, int echoPin, unsigned long toUs = 15000UL) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long us = pulseIn(echoPin, HIGH, toUs);
  if (us == 0) return 250.0f;
  float cm = (float)us * 0.01715f;
  return (cm < 2.0f) ? 250.0f : cm;
}

float readUltrasonicMedian(int trigPin, int echoPin, int samples = 3) {
  float vals[5];
  if (samples > 5) samples = 5;
  for (int i = 0; i < samples; i++) {
    vals[i] = readUltrasonicSingle(trigPin, echoPin);
    delay(14);
  }
  for (int i = 0; i < samples - 1; i++) {
    for (int j = i + 1; j < samples; j++) {
      if (vals[j] < vals[i]) {
        float tmp = vals[i]; vals[i] = vals[j]; vals[j] = tmp;
      }
    }
  }
  return vals[samples / 2];
}

void readAllUltrasonics(float &dF, float &dL, float &dR, int samples = 3) {
  dF = readUltrasonicMedian(F_TRIG, F_ECHO, samples);
  delay(10);
  dL = readUltrasonicMedian(L_TRIG, L_ECHO, samples);
  delay(10);
  dR = readUltrasonicMedian(R_TRIG, R_ECHO, samples);
}

// ============================================================================
// 7. ABSOLUTE-HEADING PRECISION TURN & HEADING-LOCKED STRAIGHT DRIVE
// ============================================================================
bool turnToHeadingRaw(float targetHdgDeg) {
  float err0 = targetHdgDeg - getHeadingDeg();
  if (fabsf(err0) < 1.5f) return true;

  int dir = (err0 > 0.0f) ? 1 : -1; // +1 = CCW (L-, R+)
  long l0, r0;
  readEncoders(l0, r0);

  setLeftDir(-dir);
  setRightDir(dir);

  long stallL = l0, stallR = r0;
  int boost = 0;
  unsigned long lastMoveMs = millis();
  unsigned long tStart     = lastMoveMs;

  while (!timeUp()) {
    unsigned long now = millis();
    if (now - tStart > 5500UL) {
      stopMotors();
      Serial.println(F("  [turnToHeading] Timeout!"));
      return false;
    }

    float curHdg = getHeadingDeg();
    float remDeg = (targetHdgDeg - curHdg) * (float)dir;

    // Catch wrong direction immediately
    if (remDeg > fabsf(err0) + 15.0f) {
      stopMotors();
      Serial.println(F("  [turnToHeading] ABORT: Spinning wrong direction!"));
      return false;
    }

    float lead = min(TURN_LEAD_DEG, fabsf(err0) * 0.30f);
    if (remDeg <= lead) break;

    long l, r;
    readEncoders(l, r);
    if (labs(l - stallL) + labs(r - stallR) >= 4) {
      stallL = l; stallR = r;
      lastMoveMs = now;
      boost = 0;
    } else if (now - lastMoveMs > STALL_MS) {
      boost = min(boost + STALL_STEP_PWM, STALL_MAX_PWM);
      lastMoveMs = now;
    }

    // Proportional speed ramp from 130 (creep) up to 175 (fast)
    float frac = constrain((remDeg - 8.0f) / 28.0f, 0.0f, 1.0f);
    float base = (float)TURN_CREEP_PWM + frac * (float)(TURN_FAST_PWM - TURN_CREEP_PWM) + (float)boost;

    // Balance L and R wheel travel so robot spins in place
    long dL = labs(l - l0);
    long dR = labs(r - r0);
    float syncCorr = constrain(TURN_SYNC_KP * (float)(dR - dL), -40.0f, 40.0f);

    int lPWM = (int)roundf(base + syncCorr);
    int rPWM = (int)roundf(base * RIGHT_FF - syncCorr);
    setMotorPWM(constrain(lPWM, 100, 240), constrain(rPWM, 90, 235));
    delay(5);
  }

  stopMotors();
  delay(65);
  return true;
}

// Turn to an exact global heading (with optional single micro-trim if > 2.5 deg off)
bool turnToHeading(float targetHdgDeg, bool doTrim = true) {
  if (!turnToHeadingRaw(targetHdgDeg)) return false;
  if (doTrim) {
    float rem = targetHdgDeg - getHeadingDeg();
    if (fabsf(rem) > 2.5f && fabsf(rem) < 25.0f) {
      turnToHeadingRaw(targetHdgDeg);
    }
  }
  return true;
}

// Turn by a relative angle (+ = CCW/Left, - = CW/Right)
bool pivotRelative(float deltaDeg, bool doTrim = true) {
  return turnToHeading(getHeadingDeg() + deltaDeg, doTrim);
}

// Dead-straight drive using feedforward + 100 Hz tick-difference PD lock
bool driveStraightCm(float cm, bool checkFrontObstacle = true) {
  if (fabsf(cm) < 0.5f) return true;

  int dir = (cm > 0.0f) ? 1 : -1;
  float targetCm = fabsf(cm);

  long l0, r0;
  readEncoders(l0, r0);
  long stallL = l0, stallR = r0;
  int boost = 0;

  setLeftDir(dir);
  setRightDir(dir);

  float prevErr = 0.0f;
  unsigned long prevMs      = millis();
  unsigned long tStart      = prevMs;
  unsigned long lastMoveMs  = prevMs;
  unsigned long lastFrontMs = prevMs;
  int wallHits = 0;

  while (!timeUp()) {
    unsigned long now = millis();
    if (dir < 0 && (now - tStart > 1500UL)) break; // Max 1.5 s reverse
    if (now - tStart > 10000UL) {
      stopMotors();
      Serial.println(F("  [driveStraightCm] Timeout!"));
      return false;
    }

    long l, r;
    readEncoders(l, r);
    float dL_cm = (float)(l - l0) * (float)dir / TICKS_PER_CM_L;
    float dR_cm = (float)(r - r0) * (float)dir / TICKS_PER_CM_R;
    float avgCm = 0.5f * (dL_cm + dR_cm);
    float remCm = targetCm - avgCm;

    if (avgCm < -4.0f) {
      stopMotors();
      Serial.println(F("  [driveStraightCm] ABORT: Driving wrong direction!"));
      return false;
    }
    if (remCm <= 0.6f) break;

    // Non-blocking emergency front obstacle guard every 160 ms
    if (checkFrontObstacle && dir > 0 && (now - lastFrontMs >= 160UL)) {
      lastFrontMs = now;
      float dF = readUltrasonicSingle(F_TRIG, F_ECHO, 2500UL);
      if (dF < 7.0f) {
        wallHits++;
        if (wallHits >= 2) {
          stopMotors();
          Serial.print(F("  [Safety] Stopped: obstacle "));
          Serial.print(dF, 1); Serial.println(F(" cm ahead."));
          return false;
        }
      } else {
        wallHits = 0;
      }
    }

    if (now - prevMs >= 10UL) {
      float dt = (float)(now - prevMs) * 0.001f;
      prevMs = now;

      if (labs(l - stallL) + labs(r - stallR) >= 5) {
        stallL = l; stallR = r;
        lastMoveMs = now;
        boost = 0;
      } else if (now - lastMoveMs > STALL_MS) {
        boost = min(boost + STALL_STEP_PWM, STALL_MAX_PWM);
        lastMoveMs = now;
      }

      // Tick error in drive direction: positive = right ahead -> boost left, trim right
      float errTicks = (float)((r - r0) - (l - l0)) * (float)dir;
      float deriv    = (errTicks - prevErr) / dt;
      prevErr = errTicks;

      float corr = constrain(DRIVE_SYNC_KP * errTicks + DRIVE_SYNC_KD * deriv,
                             -DRIVE_MAX_CORR, DRIVE_MAX_CORR);

      int base = (remCm < 12.0f || dir < 0) ? DRIVE_SLOW_PWM : DRIVE_CRUISE_PWM;
      base = min(base + boost, 235);

      int lPWM = (int)roundf((float)base + corr);
      int rPWM = (int)roundf((float)base * RIGHT_FF - corr);
      setMotorPWM(constrain(lPWM, 90, 245), constrain(rPWM, 80, 240));
    }
  }

  stopMotors();
  delay(80);
  return true;
}

// ============================================================================
// 8. LOCALIZE IN GREEN BOX & DRIVE TO T1
// ============================================================================
void localizeAndGoToT1(bool driveToTarget) {
  runStartMs = millis();
  resetEncoders(); // Single clean origin for the ENTIRE Task 1 run!

  Serial.println(F("\n--- PHASE 1: 360-DEG START-BOX SCAN (TRUE HEADINGS) ---"));

  // Safety clearance if front starts right up against a wall
  if (readUltrasonicMedian(F_TRIG, F_ECHO, 3) < 9.0f) {
    Serial.println(F("  Front < 9 cm -> backing up 7 cm..."));
    driveStraightCm(-7.0f, false);
  }

  const int NUM_STEPS = 12;
  float scanHdg[NUM_STEPS], scanF[NUM_STEPS], scanL[NUM_STEPS], scanR[NUM_STEPS];
  float startHdg = getHeadingDeg();

  for (int i = 0; i < NUM_STEPS; i++) {
    if (timeUp()) return;
    if (i > 0) {
      // Aim for ABSOLUTE target heading (no cumulative overshoot!)
      if (!turnToHeading(startHdg + (float)(i * 30), false)) return;
    }

    scanHdg[i] = getHeadingDeg();
    readAllUltrasonics(scanF[i], scanL[i], scanR[i], 3);

    float wEst = scanL[i] + scanR[i] + 2.0f * SIDE_OFFSET_CM;
    Serial.print(F("  [")); Serial.print(i);
    Serial.print(F("] Hdg:")); Serial.print(scanHdg[i] - startHdg, 1);
    Serial.print(F(" | F:"));  Serial.print(scanF[i], 1);
    Serial.print(F(" L:"));    Serial.print(scanL[i], 1);
    Serial.print(F(" R:"));    Serial.print(scanR[i], 1);
    Serial.print(F(" | W:"));  Serial.println(wEst, 1);

    if (scanF[i] < 6.5f) {
      driveStraightCm(-6.0f, false);
    }
  }

  // Pick the back-wall angle using ACTUAL measured headings:
  //   (a) Front sees close wall (< 60 cm)
  //   (b) Left + Right + width matches BOX_WIDTH_CM (122 cm)
  //   (c) Sample closest to +180 deg opposite sees OPEN field (> 85 cm)
  int   bestIdx   = 0;
  int   bestTier  = 99;
  float bestScore = 9999.0f;

  for (int i = 0; i < NUM_STEPS; i++) {
    // Find the actual sample closest to 180 deg opposite scanHdg[i]
    int oppIdx = -1;
    float bestOppDiff = 999.0f;
    for (int j = 0; j < NUM_STEPS; j++) {
      float diff = fabsf(wrapAngle180(scanHdg[j] - (scanHdg[i] + 180.0f)));
      if (diff < bestOppDiff) { bestOppDiff = diff; oppIdx = j; }
    }
    bool oppOpen = (oppIdx >= 0 && bestOppDiff < 35.0f && scanF[oppIdx] > 85.0f);

    float wEst = scanL[i] + scanR[i] + 2.0f * SIDE_OFFSET_CM;
    float wErr = fabsf(wEst - BOX_WIDTH_CM);

    int tier = 2;
    float score = scanF[i] + wErr;
    if (scanF[i] < 60.0f && wErr < 16.0f && oppOpen) {
      tier = 0;
      score = scanF[i] + 0.5f * wErr;
    } else if (scanF[i] < 60.0f && wErr < 20.0f) {
      tier = 1;
      score = scanF[i] + 0.5f * wErr;
    }

    if (tier < bestTier || (tier == bestTier && score < bestScore)) {
      bestTier  = tier;
      bestScore = score;
      bestIdx   = i;
    }
  }

  // Shortest turn from current heading to the exact recorded back-wall heading
  float curHdg = getHeadingDeg();
  float backTargetHdg = curHdg + wrapAngle180(scanHdg[bestIdx] - curHdg);
  Serial.print(F("  Best back-wall sample [")); Serial.print(bestIdx);
  Serial.print(F("] (tier ")); Serial.print(bestTier);
  Serial.print(F(") -> turning by ")); Serial.print(backTargetHdg - curHdg, 1);
  Serial.println(F(" deg"));

  if (!turnToHeading(backTargetHdg, true)) return;

  // --- PHASE 2: FINE ALIGNMENT TO BACK WALL (-18..+18 deg in 6 deg absolute steps) ---
  Serial.println(F("--- PHASE 2: FINE ALIGNMENT TO BACK WALL ---"));
  const int FINE_N = 7;
  float fineHdg[FINE_N], fineF[FINE_N];
  float minF = 999.0f;

  for (int k = 0; k < FINE_N; k++) {
    if (timeUp()) return;
    float target = backTargetHdg - 18.0f + 6.0f * (float)k;
    if (!turnToHeading(target, false)) return;

    fineHdg[k] = getHeadingDeg();
    fineF[k]   = readUltrasonicMedian(F_TRIG, F_ECHO, 3);
    if (fineF[k] < minF) minF = fineF[k];

    Serial.print(F("  Fine[")); Serial.print(k);
    Serial.print(F("] off=")); Serial.print(fineHdg[k] - backTargetHdg, 1);
    Serial.print(F(" deg | F=")); Serial.println(fineF[k], 1);
  }

  // Average the TRUE measured headings within minF + 0.8 cm (kills cone flatness)
  float sumHdg = 0.0f;
  int   cntHdg = 0;
  for (int k = 0; k < FINE_N; k++) {
    if (fineF[k] <= minF + 0.8f) {
      sumHdg += fineHdg[k];
      cntHdg++;
    }
  }
  float exactBackHdg = (cntHdg > 0) ? (sumHdg / (float)cntHdg) : backTargetHdg;
  g_backHdg = exactBackHdg;   // [ADDED] line follow needs to know where the back wall is
  Serial.print(F("  Squaring to exact back-wall heading (offset="));
  Serial.print(exactBackHdg - backTargetHdg, 1); Serial.println(F(" deg)..."));

  if (!turnToHeading(exactBackHdg, true)) return;
  delay(120);

  // --- PHASE 3: DIRECT (X, Y) READOUT FACING BACK WALL (-90 DEG) ---
  float dF, dL, dR;
  readAllUltrasonics(dF, dL, dR, 5);

  float wCheck     = dL + dR + 2.0f * SIDE_OFFSET_CM;
  float robotY     = dF + FRONT_OFFSET_CM;
  float xFromRight = dR + SIDE_OFFSET_CM;
  float xFromLeft  = BOX_WIDTH_CM - (dL + SIDE_OFFSET_CM);
  float robotX     = 0.5f * (xFromRight + xFromLeft);

  Serial.print(F("--- PHASE 3: POSE (Facing Back Wall = -90 deg) ---\n  X="));
  Serial.print(robotX, 1);
  Serial.print(F(" (fromR=")); Serial.print(xFromRight, 1);
  Serial.print(F(", fromL=")); Serial.print(xFromLeft, 1);
  Serial.print(F(") | Y="));   Serial.print(robotY, 1);
  Serial.print(F(" cm | L+R+W=")); Serial.println(wCheck, 1);

  // Sanity guard: refuse to drive blindly if readings don't match the start box!
  if (robotY > 70.0f || fabsf(wCheck - BOX_WIDTH_CM) > 16.0f) {
    Serial.println(F("[SAFETY ABORT] Readings not square to start box (Y>70 or W off >16 cm)!"));
    return;
  }

  // --- PHASE 4: TURN & DRIVE TO T1 ---
  float dx = T1_X_CM - robotX;
  float dy = T1_Y_CM - robotY;
  float distToT1    = max(0.0f, hypotf(dx, dy) - DROP_OFFSET_CM);
  float targetAngle = atan2f(dy, dx) * (180.0f / (float)M_PI);
  float turnToT1    = wrapAngle180(targetAngle - (-90.0f));

  Serial.print(F("--- PHASE 4: PLAN TO T1 ("));
  Serial.print(T1_X_CM, 1); Serial.print(F(", ")); Serial.print(T1_Y_CM, 1);
  Serial.print(F(") ---\n  Turn=")); Serial.print(turnToT1, 1);
  Serial.print(F(" deg | Drive=")); Serial.print(distToT1, 1); Serial.println(F(" cm"));

  if (!driveToTarget) {
    Serial.println(F("[Localize-Only Done] Check squareness to back wall & X, Y with tape!"));
    return;
  }

  // Turn directly to exactBackHdg + turnToT1 (includes any tiny residual trim!)
  if (!turnToHeading(exactBackHdg + turnToT1, true)) return;
  delay(120);
  if (!driveStraightCm(distToT1, true)) return;

  stopMotors();
  Serial.println(F(">>> ARRIVED AT T1 & STOPPED! <<<"));
  // dropItem();      // [ADDED] put your drop call here
  lineToT2();         // [ADDED] line + wall follow -> T2
}

// ============================================================================
// 9. SETUP & INTERACTIVE MENU
// ============================================================================
void printMenu() {
  Serial.println(F("\n=== TASK 1 + LINE FOLLOW READY ==="));
  Serial.println(F("  f = Drive FORWARD 20 cm (check L and R ticks match!)"));
  Serial.println(F("  w = Pivot LEFT +90 deg (check heading = 89..91 deg!)"));
  Serial.println(F("  l = Localize ONLY (squares to back wall & checks W ~ 122 cm)"));
  Serial.println(F("  t = FULL RUN (Localize -> T1 -> line + walls -> T2)"));
  Serial.println(F("  n = LINE TEST (place robot ON T1 facing up-field)"));
  Serial.println(F("  i = IR + sonar monitor (5 s) | k = teach FRONT reading"));
}

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

  pinMode(16, INPUT); pinMode(17, INPUT);   // [ADDED] IR line sensors (left GP16, right GP17)

  pinMode(ENC_R_C1, INPUT_PULLUP); pinMode(ENC_R_C2, INPUT_PULLUP);
  pinMode(ENC_L_C1, INPUT_PULLUP); pinMode(ENC_L_C2, INPUT_PULLUP);
  prevR = ((uint8_t)digitalRead(ENC_R_C1) << 1) | (uint8_t)digitalRead(ENC_R_C2);
  prevL = ((uint8_t)digitalRead(ENC_L_C1) << 1) | (uint8_t)digitalRead(ENC_L_C2);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C1), isrEncR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C2), isrEncR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_C1), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_C2), isrEncL, CHANGE);

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

#if INTERACTIVE_MENU == 0
  for (int i = 0; i < 15; i++) { digitalWrite(LED_BUILTIN, i % 2); delay(200); }
  localizeAndGoToT1(true);
  releaseMotors();
#endif
}

unsigned long lastMenuMs = 0;

void loop() {
#if INTERACTIVE_MENU == 1
  digitalWrite(LED_BUILTIN, (millis() / 300) % 2);
  if (millis() - lastMenuMs >= 4500UL) {
    lastMenuMs = millis();
    printMenu();
  }
  if (Serial.available() > 0) {
    char c = Serial.read();
    while (Serial.available() > 0) Serial.read();

    if (c == 'f' || c == 'F') {
      resetEncoders();
      Serial.println(F("[Test 'f'] Driving FORWARD 20 cm..."));
      driveStraightCm(20.0f, true);
      long l, r; readEncoders(l, r);
      Serial.print(F("  Ticks L=")); Serial.print(l);
      Serial.print(F(" R=")); Serial.print(r);
      Serial.print(F(" | Diff=")); Serial.println(r - l);
      printMenu();
    } else if (c == 'w' || c == 'W') {
      resetEncoders();
      Serial.println(F("[Test 'w'] Pivoting LEFT +90.0 deg..."));
      pivotRelative(90.0f, true);
      long l, r; readEncoders(l, r);
      Serial.print(F("  Ticks L=")); Serial.print(l);
      Serial.print(F(" R=")); Serial.print(r);
      Serial.print(F(" | Hdg=")); Serial.print(getHeadingDeg(), 1); Serial.println(F(" deg"));
      printMenu();
    } else if (c == 'l' || c == 'L') {
      localizeAndGoToT1(false);
      printMenu();
    } else if (c == 't' || c == 'T') {
      localizeAndGoToT1(true);
      printMenu();
    } else if (c == 'n' || c == 'N') {        // [ADDED] line test from T1
      g_backHdg = NAN;
      lineToT2();
      printMenu();
    } else if (c == 'i' || c == 'I') {        // [ADDED] sensor monitor
      unsigned long t0 = millis();
      while (millis() - t0 < 5000UL) {
        float dF, dL, dR;
        readAllUltrasonics(dF, dL, dR, 1);
        Serial.print(F("IR L=")); Serial.print(digitalRead(16));
        Serial.print(F(" R=")); Serial.print(digitalRead(17));
        Serial.print(F(" | F=")); Serial.print(dF, 1);
        Serial.print(F(" L=")); Serial.print(dL, 1);
        Serial.print(F(" R=")); Serial.println(dR, 1);
        delay(150);
      }
      printMenu();
    } else if (c == 'k' || c == 'K') {        // [ADDED] teach
      float dF, dL, dR;
      readAllUltrasonics(dF, dL, dR, 5);
      Serial.print(F("[TEACH] FRONT=")); Serial.print(dF, 1);
      Serial.print(F(" LEFT=")); Serial.print(dL, 1);
      Serial.print(F(" RIGHT=")); Serial.println(dR, 1);
      printMenu();
    }
    lastMenuMs = millis();
  }
#else
  releaseMotors();
  digitalWrite(LED_BUILTIN, (millis() / 600) % 2);
  delay(100);
#endif
}

// ============================================================================
// 10. LINE FOLLOW (T1 -> line -> passage -> hairpin -> T2)
//     Paste this BELOW loop(). Uses only the Task 1 helpers above.
// ============================================================================
const int   IR_L_PIN        = 16;      // left IR  (straddles the line)
const int   IR_R_PIN        = 17;      // right IR
const int   IR_BLACK        = HIGH;    // what the IR reads over black ('i' to check; set LOW if reversed)

// --- practice-field route (cm of line from the pickup point, measured at the IRs) ---
const float CAP_MIN_CM      = 13.0f;   // pickup: black only counts between 13..32 cm (decoys ignored)
const float CAP_MAX_CM      = 32.0f;
const float LINE_PICKUP_HDG = 74.0f;   // line direction at pickup (deg, 0 = facing right wall, 90 = up-field)
const float RT_OVAL_END     = 17.0f;   // oval's loose end ~9 cm LEFT of the line   -> ignore LEFT IR
const float RT_PASSAGE_IN   = 62.0f;   // passage ladder <-> T-wall                  -> slow + wall centring
const float RT_BRANCH       = 120.0f;  // branch to the RIGHT toward the T-wall      -> ignore RIGHT IR
const float RT_PASSAGE_OUT  = 125.0f;
const float RT_TOP          = 138.0f;
const float RT_HAIRPIN      = 188.0f;  // vertex: line doubles back LEFT            -> scripted pivot
const float RT_END          = 220.0f;  // end of line, ladder ahead
const float LINE_MAX_ROUTE  = 245.0f;

const int   LF_BASE_PWM     = 150;
const int   LF_SLOW_PWM     = 132;
const int   LF_OUTER_ADD    = 30;      // outer wheel boost when correcting
const unsigned long LF_HOLD_MS = 70;   // keep turning after the sensor clears (sharp bends)
const float WALL_KP         = 6.0f;    // PWM per cm of (LEFT-RIGHT) in the passage
const float WALL_MAX_CM     = 15.0f;
const float HAIRPIN_PRE_CM  = 8.0f;
const float HAIRPIN_TURN    = 124.0f;  // pivot LEFT at the hairpin
const float LINE_END_FRONT  = 20.0f;   // FRONT when the IRs reach the line end ('k' to teach)
const float T2_TURN         = -52.0f;  // from the last leg to facing the upper rung square-on
const float T2_FRONT_TARGET = 5.0f;    // FRONT with the pointer on T2 ('k' to teach)


bool irBlack(int pin) { return digitalRead(pin) == IR_BLACK; }

// encoder heading for a field heading (0 = right wall, 90 = up-field, -90 = back wall)
float encForField(float fieldDeg) {
  float cur = getHeadingDeg();
  return cur + wrapAngle180((g_backHdg + 90.0f + fieldDeg) - cur);
}

// one wheel per sign: + forward, - reverse, 0 stop (RIGHT_FF applied)
void lfDrive(int lp, int rp) {
  setLeftDir(lp > 0 ? 1 : (lp < 0 ? -1 : 0));
  setRightDir(rp > 0 ? 1 : (rp < 0 ? -1 : 0));
  setMotorPWM(abs(lp), (int)roundf((float)abs(rp) * RIGHT_FF));
}

float cmSince(long l0, long r0) {
  long l, r; readEncoders(l, r);
  return 0.5f * ((float)(l - l0) / TICKS_PER_CM_L + (float)(r - r0) / TICKS_PER_CM_R);
}

// ---- pick up the line: face the right wall, creep until an IR sees black in the window ----
bool pickUpLine() {
  Serial.println(F("--- PICK UP LINE ---"));
  turnToHeading(encForField(0.0f), true);
  long l0, r0; readEncoders(l0, r0);
  setLeftDir(1); setRightDir(1);
  bool found = false;
  unsigned long t0 = millis();
  while (!timeUp() && millis() - t0 < 6000UL) {
    float d = cmSince(l0, r0);
    if (d >= CAP_MIN_CM && (irBlack(IR_L_PIN) || irBlack(IR_R_PIN))) { found = true; break; }
    if (d >= CAP_MAX_CM) break;
    long l, r; readEncoders(l, r);
    float corr = constrain(DRIVE_SYNC_KP * (float)((r - r0) - (l - l0)), -DRIVE_MAX_CORR, DRIVE_MAX_CORR);
    setMotorPWM(constrain((int)roundf(DRIVE_SLOW_PWM + corr), 90, 245),
                constrain((int)roundf(DRIVE_SLOW_PWM * RIGHT_FF - corr), 80, 240));
    delay(5);
  }
  stopMotors();
  Serial.print(found ? F("  line found at ") : F("  no line, using distance: "));
  Serial.println(cmSince(l0, r0), 1);
  driveStraightCm(FRONT_OFFSET_CM - 1.0f, false);       // axle over the line
  turnToHeading(encForField(LINE_PICKUP_HDG), true);    // IRs now straddle it
  return found;
}

// ---- follow the line with the route table ----
float followLine() {
  Serial.println(F("--- FOLLOW LINE ---"));
  long l0, r0; readEncoders(l0, r0);
  int holdSide = 0;
  unsigned long holdUntil = 0, lastSonar = 0, lastFront = 0;
  bool hairpinDone = false, pingLeft = true;
  float sideL = 250.0f, sideR = 250.0f, junctionEnd = -1.0f, route = 0.0f;
  float hHold = getHeadingDeg();

  while (!timeUp()) {
    float along = cmSince(l0, r0);
    route = along + FRONT_OFFSET_CM;                   // IRs are at the front
    unsigned long now = millis();
    if (route >= LINE_MAX_ROUTE) break;

    // end of line: ladder in front
    if (route >= RT_END - 15.0f && now - lastFront >= 60UL) {
      lastFront = now;
      if (readUltrasonicSingle(F_TRIG, F_ECHO, 3000UL) <= LINE_END_FRONT) break;
    }

    // hairpin: drive on a bit, pivot left (back off once if it hits the ladder)
    if (!hairpinDone && route >= RT_HAIRPIN) {
      hairpinDone = true;
      Serial.println(F("  hairpin"));
      stopMotors();
      driveStraightCm(HAIRPIN_PRE_CM, false);
      float h0 = getHeadingDeg();
      if (!pivotRelative(HAIRPIN_TURN, true)) { driveStraightCm(-4.0f, false); turnToHeading(h0 + HAIRPIN_TURN, true); }
      hHold = getHeadingDeg(); holdUntil = 0; junctionEnd = -1.0f;
      continue;
    }

    bool ignL   = (route > RT_OVAL_END - 12.0f && route < RT_OVAL_END + 13.0f);
    bool ignR   = (route > RT_BRANCH - 10.0f && route < RT_BRANCH + 15.0f);
    bool inPass = (route > RT_PASSAGE_IN - 5.0f && route < RT_PASSAGE_OUT);
    bool slow   = inPass || route > RT_TOP + 10.0f;
    int  base   = slow ? LF_SLOW_PWM : LF_BASE_PWM;

    // passage: alternate short LEFT/RIGHT pings (1.8 ms timeout so the loop never stalls)
    if (inPass && now - lastSonar >= 25UL) {
      lastSonar = now;
      if (pingLeft) sideL = readUltrasonicSingle(L_TRIG, L_ECHO, 1800UL);
      else          sideR = readUltrasonicSingle(R_TRIG, R_ECHO, 1800UL);
      pingLeft = !pingLeft;
    }

    bool bl = irBlack(IR_L_PIN) && !ignL;
    bool br = irBlack(IR_R_PIN) && !ignR;

    // crossing a decoy (both black): hold heading straight for 4 cm
    if (along < junctionEnd) {
      int c = (int)constrain((hHold - getHeadingDeg()) * 4.0f, -40.0f, 40.0f);
      lfDrive(base - c, base + c);
      continue;
    }
    if (bl && br) { junctionEnd = along + 4.0f; hHold = getHeadingDeg(); continue; }

    if (bl)      { holdSide = -1; holdUntil = now + LF_HOLD_MS; }
    else if (br) { holdSide =  1; holdUntil = now + LF_HOLD_MS; }
    int steer = bl ? -1 : (br ? 1 : (now < holdUntil ? holdSide : 0));

    if (steer < 0)      lfDrive(0, base + LF_OUTER_ADD);          // line under LEFT IR -> turn left
    else if (steer > 0) lfDrive(base + LF_OUTER_ADD, 0);          // line under RIGHT IR -> turn right
    else if (inPass && sideL < WALL_MAX_CM && sideR < WALL_MAX_CM) {
      int c = (int)constrain(WALL_KP * (sideL - sideR), -35.0f, 35.0f);   // centre between the walls
      lfDrive(base - c, base + c);
    } else {
      lfDrive(base, base);
    }
    hHold = getHeadingDeg();
    delay(2);
  }
  stopMotors();
  Serial.print(F("  line ended at route ")); Serial.print(route, 1);
  Serial.print(F(" (map: ")); Serial.print(RT_END, 0); Serial.println(F(")"));
  return route;
}

// ---- T2: face the upper rung square-on, creep to the taught FRONT ----
void approachT2() {
  Serial.println(F("--- T2 ---"));
  pivotRelative(T2_TURN, true);
  for (int k = 0; k < 6 && !timeUp(); k++) {
    float dF = readUltrasonicMedian(F_TRIG, F_ECHO, 5);
    if (dF > 200.0f) { driveStraightCm(3.0f, false); continue; }
    float e = dF - T2_FRONT_TARGET;
    Serial.print(F("  FRONT=")); Serial.println(dF, 1);
    if (fabsf(e) < 0.5f) break;
    driveStraightCm(constrain(e, -10.0f, 30.0f), false);
  }
  stopMotors();
  Serial.println(F(">>> AT T2 <<<"));
}

// ---- call this after the drop at T1 ----
void lineToT2() {
  if (isnan(g_backHdg)) g_backHdg = getHeadingDeg() - 180.0f;   // test mode: robot placed on T1 facing up-field
  pickUpLine();
  followLine();
  approachT2();
  releaseMotors();
}
