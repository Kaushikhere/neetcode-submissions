// ============================================================================
// IM Competition: FULL RUN  (Task 1 -> drop at T1 -> line + wall follow -> T2)
//
// Sections 1-8 are your working Task 1 code (same pins, tuning, scan, squaring,
// turnToHeading, driveStraightCm). Additions are marked  [NEW].
//
//   Task 1 : 360 scan -> square to back wall -> (X,Y) -> turn -> drive to T1
//   [NEW] 9  : field heading frame + odometry (dead reckoning) after Phase 3
//   [NEW] 10 : drop at T1 (SG90 trap door on GP10)
//   [NEW] 11 : pick up the line on the oval's right side (gated: decoys ignored)
//   [NEW] 12 : follow the line with a ROUTE TABLE keyed to encoder distance:
//              ignore the oval's loose end, ignore the branch, centre between
//              the passage walls with LEFT/RIGHT, scripted hairpin pivot
//   [NEW] 13 : end of line -> face the upper rung -> stop at the taught FRONT
//
// Field frame (same as the map): x = cm from LEFT wall, y = cm from BACK wall,
// field heading 0 = facing the right wall, 90 = facing up-field.
// ============================================================================

#include <Arduino.h>
#include <math.h>
#include <Servo.h>

// 1 = Serial Menu | 0 = Standalone battery mode (3 s countdown, then FULL run)
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

const int IR_L_PIN = 16, IR_R_PIN = 17;      // [NEW] line sensors (straddle the line)
const int DROP_SERVO_PIN = 10;               // [NEW] SG90 trap door (spare GP10)

#define INVERT_MOTOR_DIR 0
const int8_t ENC_R_SIGN = -1;
const int8_t ENC_L_SIGN =  1;

// ============================================================================
// 2. ROBOT & FIELD MEASUREMENTS (UPDATE WITH TAPE MEASURE)
// ============================================================================
const float TICKS_PER_CM_L   = 60.5f;
const float TICKS_PER_CM_R   = 60.7f;
const float WHEELBASE_CM     = 19.0f;

const float FRONT_OFFSET_CM  = 17.0f; // Rear axle midpoint -> Front US face (IRs are at the front too)
const float SIDE_OFFSET_CM   = 10.7f;
const float DROP_OFFSET_CM   = 0.0f;

const float BOX_WIDTH_CM     = 122.0f;
const float T1_X_CM          = 58.4f;
const float T1_Y_CM          = 97.0f;

// ---- [NEW] practice-field map (traced from photos, +-2-4 cm; tape-check the starred ones) ----
const float LINE_PICKUP_X    = 96.5f;   // * oval's right side at the height of T1
const float LINE_PICKUP_HDG  = 74.0f;   //   line direction there (field deg)
// Route = cm of line from the pickup point, measured at the IR sensors.
const float RT_OVAL_END      = 17.0f;   // oval's loose end comes ~9 cm LEFT of the line
const float RT_PASSAGE_IN    = 62.0f;   // passage between ladder right panel and T-wall (~30 cm wide)
const float RT_BRANCH        = 120.0f;  // branch splits RIGHT toward the T-wall (decoy)
const float RT_PASSAGE_OUT   = 125.0f;
const float RT_TOP           = 138.0f;  // top curve
const float RT_HAIRPIN       = 188.0f;  // vertex: line doubles back ~143 deg LEFT
const float RT_END           = 220.0f;  // end of the line, ~14 cm above the upper rung near T2
const float LINE_END_HDG     = -18.0f;  // line direction on the last leg (field deg)
const float RUNG_NORMAL_HDG  = -70.0f;  // facing the upper rung square-on (field deg)

// ============================================================================
// 3. TUNED MOTOR, SYNC & BRAKE SETTINGS
// ============================================================================
const float RIGHT_FF         = 0.86f;

const int   TURN_FAST_PWM    = 175;
const int   TURN_CREEP_PWM   = 130;
const float TURN_LEAD_DEG    = 2.2f;
const float TURN_SYNC_KP     = 0.45f;

const int   DRIVE_CRUISE_PWM = 180;
const int   DRIVE_SLOW_PWM   = 145;
const float DRIVE_SYNC_KP    = 1.40f;
const float DRIVE_SYNC_KD    = 0.08f;
const float DRIVE_MAX_CORR   = 75.0f;

const unsigned long STALL_MS = 100;
const int   STALL_STEP_PWM   = 12;
const int   STALL_MAX_PWM    = 65;

const unsigned long HARD_STOP_MS = 88000UL;

// ---- [NEW] line / wall following ----
const int   IR_BLACK         = HIGH;    // digitalRead over black ('i' monitor to check)
const float CAP_MIN_CM       = 13.0f;   // black only accepted this far into the pickup drive...
const float CAP_MAX_CM       = 32.0f;   // ...and before this (expected ~21 cm). Outside = decoy.
const int   LF_BASE_PWM      = 150;     // easy sections
const int   LF_SLOW_PWM      = 132;     // passage, top, hairpin, last leg
const int   LF_OUTER_ADD     = 30;      // outer wheel boost while correcting
const int   LF_INNER_PWM     = 0;       // inner wheel while correcting (0 = stopped -> pivot on it)
const unsigned long LF_HOLD_MS = 70;    // keep turning after the sensor clears (sharp bends)
const float LF_JUNCTION_CM   = 4.0f;    // both IRs black (crossing): drive straight this far
const float WALL_KP          = 6.0f;    // PWM per cm of (LEFT - RIGHT) when centring in the passage
const float WALL_MAX_CM      = 15.0f;   // side reading above this = no wall on that side
const float LINE_MAX_ROUTE   = 245.0f;  // safety stop
const float LINE_END_FRONT   = 20.0f;   // FRONT when the IRs reach the line's end ('k' teach)
const float T2_FRONT_TARGET  = 5.0f;    // FRONT with the pointer held on T2 ('k' teach)
const float HAIRPIN_PRE_CM   = 8.0f;    // drive on past the vertex trigger...
const float HAIRPIN_TURN_DEG = 124.0f;  // ...then pivot left. IRs land on the return leg.

const int   DROP_CLOSED_DEG  = 10;
const int   DROP_OPEN_DEG    = 100;

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

void readEncoders(long &l, long &r) {
  noInterrupts();
  l = leftCount;
  r = rightCount;
  interrupts();
}

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

// ============================================================================
// [NEW] 9. FIELD HEADING FRAME + ODOMETRY
// ============================================================================
float fieldOffset = 0.0f;   // field heading = getHeadingDeg() - fieldOffset
bool  fieldKnown  = false;
float odoX = 0.0f, odoY = 0.0f;
long  odoL = 0, odoR = 0;

float fieldHdg() { return getHeadingDeg() - fieldOffset; }
float encHdgFor(float fieldDeg) {          // encoder heading for a field heading, nearest turn
  float cur = getHeadingDeg();
  return cur + wrapAngle180((fieldDeg + fieldOffset) - cur);
}

void odoUpdate() {                          // dead reckoning: call often while moving
  long l, r;
  readEncoders(l, r);
  float dL = (float)(l - odoL) / TICKS_PER_CM_L;
  float dR = (float)(r - odoR) / TICKS_PER_CM_R;
  odoL = l; odoR = r;
  if (!fieldKnown) return;
  float ds = 0.5f * (dL + dR);
  float th = fieldHdg() * (float)M_PI / 180.0f;
  odoX += ds * cosf(th);
  odoY += ds * sinf(th);
}

void setPose(float x, float y, float fieldHeadingNow) {
  fieldOffset = getHeadingDeg() - fieldHeadingNow;
  fieldKnown  = true;
  odoX = x; odoY = y;
  readEncoders(odoL, odoR);
}

void printPose(const __FlashStringHelper *tag) {
  Serial.print(F("  [pose] ")); Serial.print(tag);
  Serial.print(F(" x=")); Serial.print(odoX, 1);
  Serial.print(F(" y=")); Serial.print(odoY, 1);
  Serial.print(F(" hdg=")); Serial.println(fieldHdg(), 1);
}

void resetEncoders() {
  noInterrupts();
  leftCount  = 0;
  rightCount = 0;
  interrupts();
  odoL = 0; odoR = 0;
}

unsigned long runStartMs = 0;
bool fullRunActive = false;

bool timeUp() {
  if (INTERACTIVE_MENU && !fullRunActive) return false;
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
  analogWrite(L_ENB, 255);
  analogWrite(R_ENA, 255);
  digitalWrite(L_IN3, LOW); digitalWrite(L_IN4, LOW);
  digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
}

void stopMotors() {
  digitalWrite(L_IN3, LOW); digitalWrite(L_IN4, LOW);
  digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
  analogWrite(L_ENB, 0);
  analogWrite(R_ENA, 0);
  delay(45);
  releaseMotors();
}

// [NEW] signed power per wheel (-255..255, + = forward); RIGHT_FF applied to the right motor
void driveSigned(int lp, int rp) {
  setLeftDir(lp > 0 ? 1 : (lp < 0 ? -1 : 0));
  setRightDir(rp > 0 ? 1 : (rp < 0 ? -1 : 0));
  setMotorPWM(abs(lp), (int)roundf((float)abs(rp) * RIGHT_FF));
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

  int dir = (err0 > 0.0f) ? 1 : -1;
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
    odoUpdate();
    if (now - tStart > 5500UL) {
      stopMotors();
      Serial.println(F("  [turnToHeading] Timeout!"));
      return false;
    }

    float curHdg = getHeadingDeg();
    float remDeg = (targetHdgDeg - curHdg) * (float)dir;

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

    float frac = constrain((remDeg - 8.0f) / 28.0f, 0.0f, 1.0f);
    float base = (float)TURN_CREEP_PWM + frac * (float)(TURN_FAST_PWM - TURN_CREEP_PWM) + (float)boost;

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
  odoUpdate();
  return true;
}

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

bool pivotRelative(float deltaDeg, bool doTrim = true) {
  return turnToHeading(getHeadingDeg() + deltaDeg, doTrim);
}

bool turnToField(float fieldDeg) {          // [NEW]
  return turnToHeading(encHdgFor(fieldDeg), true);
}

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
    odoUpdate();
    if (dir < 0 && (now - tStart > 1500UL)) break;
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
  odoUpdate();
  return true;
}

// ============================================================================
// 8. LOCALIZE IN GREEN BOX & DRIVE TO T1   (your Task 1, returns true on arrival)
// ============================================================================
bool localizeAndGoToT1(bool driveToTarget) {
  runStartMs = millis();
  resetEncoders();
  fieldKnown = false;

  Serial.println(F("\n--- PHASE 1: 360-DEG START-BOX SCAN (TRUE HEADINGS) ---"));

  if (readUltrasonicMedian(F_TRIG, F_ECHO, 3) < 9.0f) {
    Serial.println(F("  Front < 9 cm -> backing up 7 cm..."));
    driveStraightCm(-7.0f, false);
  }

  const int NUM_STEPS = 12;
  float scanHdg[NUM_STEPS], scanF[NUM_STEPS], scanL[NUM_STEPS], scanR[NUM_STEPS];
  float startHdg = getHeadingDeg();

  for (int i = 0; i < NUM_STEPS; i++) {
    if (timeUp()) return false;
    if (i > 0) {
      if (!turnToHeading(startHdg + (float)(i * 30), false)) return false;
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

  int   bestIdx   = 0;
  int   bestTier  = 99;
  float bestScore = 9999.0f;

  for (int i = 0; i < NUM_STEPS; i++) {
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

  float curHdg = getHeadingDeg();
  float backTargetHdg = curHdg + wrapAngle180(scanHdg[bestIdx] - curHdg);
  Serial.print(F("  Best back-wall sample [")); Serial.print(bestIdx);
  Serial.print(F("] (tier ")); Serial.print(bestTier);
  Serial.print(F(") -> turning by ")); Serial.print(backTargetHdg - curHdg, 1);
  Serial.println(F(" deg"));

  if (!turnToHeading(backTargetHdg, true)) return false;

  Serial.println(F("--- PHASE 2: FINE ALIGNMENT TO BACK WALL ---"));
  const int FINE_N = 7;
  float fineHdg[FINE_N], fineF[FINE_N];
  float minF = 999.0f;

  for (int k = 0; k < FINE_N; k++) {
    if (timeUp()) return false;
    float target = backTargetHdg - 18.0f + 6.0f * (float)k;
    if (!turnToHeading(target, false)) return false;

    fineHdg[k] = getHeadingDeg();
    fineF[k]   = readUltrasonicMedian(F_TRIG, F_ECHO, 3);
    if (fineF[k] < minF) minF = fineF[k];

    Serial.print(F("  Fine[")); Serial.print(k);
    Serial.print(F("] off=")); Serial.print(fineHdg[k] - backTargetHdg, 1);
    Serial.print(F(" deg | F=")); Serial.println(fineF[k], 1);
  }

  float sumHdg = 0.0f;
  int   cntHdg = 0;
  for (int k = 0; k < FINE_N; k++) {
    if (fineF[k] <= minF + 0.8f) {
      sumHdg += fineHdg[k];
      cntHdg++;
    }
  }
  float exactBackHdg = (cntHdg > 0) ? (sumHdg / (float)cntHdg) : backTargetHdg;
  Serial.print(F("  Squaring to exact back-wall heading (offset="));
  Serial.print(exactBackHdg - backTargetHdg, 1); Serial.println(F(" deg)..."));

  if (!turnToHeading(exactBackHdg, true)) return false;
  delay(120);

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

  if (robotY > 70.0f || fabsf(wCheck - BOX_WIDTH_CM) > 16.0f) {
    Serial.println(F("[SAFETY ABORT] Readings not square to start box (Y>70 or W off >16 cm)!"));
    return false;
  }

  // [NEW] from here on the robot knows its field pose: exactBackHdg is field heading -90
  fieldOffset = exactBackHdg - (-90.0f);
  fieldKnown  = true;
  odoX = robotX; odoY = robotY;
  readEncoders(odoL, odoR);

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
    return false;
  }

  if (!turnToHeading(exactBackHdg + turnToT1, true)) return false;
  delay(120);
  if (!driveStraightCm(distToT1, true)) return false;

  stopMotors();
  printPose(F("at T1"));
  Serial.println(F(">>> ARRIVED AT T1 & STOPPED! <<<"));
  return true;
}

// ============================================================================
// [NEW] 10. DROP AT T1
// ============================================================================
Servo dropServo;

void dropperClose() {
  dropServo.attach(DROP_SERVO_PIN, 540, 2400);
  dropServo.write(DROP_CLOSED_DEG);
  delay(350);
  dropServo.detach();
}

void dropItem() {                     // motors are stopped first: no servo + motor current spike together
  stopMotors();
  delay(300);                         // let the robot settle so the item falls straight down
  dropServo.attach(DROP_SERVO_PIN, 540, 2400);
  dropServo.write(DROP_OPEN_DEG);
  delay(450);
  dropServo.write(DROP_CLOSED_DEG);
  delay(300);
  dropServo.detach();
  Serial.println(F(">>> ITEM DROPPED <<<"));
}

// ============================================================================
// [NEW] 11. PICK UP THE LINE (oval's right side, straight right of T1)
// ============================================================================
bool irBlack(int pin) { return digitalRead(pin) == IR_BLACK; }

// heading-locked creep; black only counts between minCm and maxCm (everything else is a decoy)
bool driveUntilLine(float minCm, float maxCm) {
  long l0, r0;
  readEncoders(l0, r0);
  setLeftDir(1); setRightDir(1);
  float prevErr = 0.0f;
  unsigned long prevMs = millis(), tStart = prevMs;
  while (!timeUp()) {
    unsigned long now = millis();
    odoUpdate();
    long l, r;
    readEncoders(l, r);
    float d = 0.5f * ((float)(l - l0) / TICKS_PER_CM_L + (float)(r - r0) / TICKS_PER_CM_R);
    if (d >= minCm && (irBlack(IR_L_PIN) || irBlack(IR_R_PIN))) {
      stopMotors();
      Serial.print(F("  line found at ")); Serial.print(d, 1); Serial.println(F(" cm"));
      return true;
    }
    if (d >= maxCm || now - tStart > 6000UL) break;
    if (now - prevMs >= 10UL) {
      float dt = (float)(now - prevMs) * 0.001f; prevMs = now;
      float errTicks = (float)((r - r0) - (l - l0));
      float corr = constrain(DRIVE_SYNC_KP * errTicks + DRIVE_SYNC_KD * (errTicks - prevErr) / dt,
                             -DRIVE_MAX_CORR, DRIVE_MAX_CORR);
      prevErr = errTicks;
      setMotorPWM(constrain((int)roundf(DRIVE_SLOW_PWM + corr), 90, 245),
                  constrain((int)roundf(DRIVE_SLOW_PWM * RIGHT_FF - corr), 80, 240));
    }
  }
  stopMotors();
  Serial.println(F("  no line in the window: carrying on by dead reckoning"));
  return false;
}

bool pickUpLine() {
  Serial.println(F("--- PICK UP LINE (oval's right side) ---"));
  turnToField(0.0f);                                   // face the right wall
  bool ok = driveUntilLine(CAP_MIN_CM, CAP_MAX_CM);
  if (!ok) {                                           // fall back: drive the remaining planned distance
    float rem = (LINE_PICKUP_X - FRONT_OFFSET_CM) - odoX;
    if (rem > 0.5f && rem < 20.0f) driveStraightCm(rem, false);
  }
  driveStraightCm(FRONT_OFFSET_CM - 1.0f, false);      // roll on until the axle is over the line
  turnToField(LINE_PICKUP_HDG);                        // line direction: the IRs now straddle it
  printPose(F("on line"));
  return ok;
}

// ============================================================================
// [NEW] 12. LINE + WALL FOLLOW WITH A ROUTE TABLE
// ============================================================================
#define ACT_IGNORE_LEFT   1
#define ACT_IGNORE_RIGHT  2
#define ACT_SLOW          4
#define ACT_WALLS         8     // centre between the walls with LEFT/RIGHT when the IRs see white
#define ACT_HAIRPIN      16

const float ROUTE_FROM[] = { RT_OVAL_END - 12, RT_PASSAGE_IN, RT_BRANCH - 10, RT_PASSAGE_IN - 5, RT_TOP + 10, RT_HAIRPIN };
const float ROUTE_TO[]   = { RT_OVAL_END + 13, RT_PASSAGE_OUT, RT_BRANCH + 15, RT_PASSAGE_OUT, RT_END + 20, RT_HAIRPIN + 1 };
const int   ROUTE_ACT[]  = { ACT_IGNORE_LEFT, ACT_SLOW, ACT_IGNORE_RIGHT, ACT_WALLS, ACT_SLOW, ACT_HAIRPIN };
const int   N_ROUTE      = sizeof(ROUTE_ACT) / sizeof(ROUTE_ACT[0]);

int routeFlags(float route) {
  int f = 0;
  for (int i = 0; i < N_ROUTE; i++)
    if (route >= ROUTE_FROM[i] && route <= ROUTE_TO[i]) f |= ROUTE_ACT[i];
  return f;
}

// short side ping for use while moving (timeout ~1.8 ms = 30 cm, so the loop never stalls)
float quickSide(int trig, int echo) { return readUltrasonicSingle(trig, echo, 1800UL); }

void doHairpin() {
  Serial.println(F("  hairpin: scripted pivot"));
  stopMotors();
  driveStraightCm(HAIRPIN_PRE_CM, false);
  float h0 = getHeadingDeg();
  if (!pivotRelative(HAIRPIN_TURN_DEG, true)) {       // stalled on the ladder: back off, finish the turn
    driveStraightCm(-4.0f, false);
    turnToHeading(h0 + HAIRPIN_TURN_DEG, true);
  }
}

float followLine() {
  Serial.println(F("--- FOLLOW LINE + WALLS ---"));
  long l0, r0;
  readEncoders(l0, r0);
  int lastSide = 0, holdSide = 0;
  unsigned long holdUntil = 0, lastSonarMs = 0, lastFrontMs = 0;
  bool hairpinDone = false, pingLeft = true;
  float junctionEnd = -1.0f, sideL = 250.0f, sideR = 250.0f, route = 0.0f;
  float hHold = getHeadingDeg();

  while (!timeUp()) {
    odoUpdate();
    long l, r;
    readEncoders(l, r);
    float along = 0.5f * ((float)(l - l0) / TICKS_PER_CM_L + (float)(r - r0) / TICKS_PER_CM_R);
    route = along + FRONT_OFFSET_CM;                   // where the IRs are on the route
    unsigned long now = millis();

    if (route >= LINE_MAX_ROUTE) break;
    if (route >= RT_END - 15.0f && now - lastFrontMs >= 60UL) {   // end of line: ladder ahead
      lastFrontMs = now;
      float dF = readUltrasonicSingle(F_TRIG, F_ECHO, 3000UL);
      if (dF <= LINE_END_FRONT) break;
    }

    int flags = routeFlags(route);
    if ((flags & ACT_HAIRPIN) && !hairpinDone) {
      hairpinDone = true;
      doHairpin();
      hHold = getHeadingDeg(); holdUntil = 0; junctionEnd = -1.0f;
      continue;
    }
    if ((flags & ACT_WALLS) && now - lastSonarMs >= 25UL) {        // alternate LEFT / RIGHT pings
      lastSonarMs = now;
      if (pingLeft) sideL = quickSide(L_TRIG, L_ECHO); else sideR = quickSide(R_TRIG, R_ECHO);
      pingLeft = !pingLeft;
    }

    int base = (flags & ACT_SLOW) ? LF_SLOW_PWM : LF_BASE_PWM;
    bool bl = irBlack(IR_L_PIN) && !(flags & ACT_IGNORE_LEFT);
    bool br = irBlack(IR_R_PIN) && !(flags & ACT_IGNORE_RIGHT);

    if (along < junctionEnd) {                         // crossing a decoy: hold heading straight
      float e = hHold - getHeadingDeg();
      int c = (int)constrain(e * 4.0f, -40.0f, 40.0f);
      driveSigned(base - c, base + c);
      continue;
    }
    if (bl && br) { junctionEnd = along + LF_JUNCTION_CM; hHold = getHeadingDeg(); continue; }

    if (bl) { lastSide = holdSide = -1; holdUntil = now + LF_HOLD_MS; }
    else if (br) { lastSide = holdSide = 1; holdUntil = now + LF_HOLD_MS; }
    int steer = bl ? -1 : (br ? 1 : (now < holdUntil ? holdSide : 0));
    (void)lastSide;

    if (steer < 0)      driveSigned(LF_INNER_PWM, base + LF_OUTER_ADD);   // line under LEFT IR: turn left
    else if (steer > 0) driveSigned(base + LF_OUTER_ADD, LF_INNER_PWM);   // line under RIGHT IR: turn right
    else if ((flags & ACT_WALLS) && sideL < WALL_MAX_CM && sideR < WALL_MAX_CM) {
      // line between the IRs and both walls in range: centre between the walls
      int c = (int)constrain(WALL_KP * (sideL - sideR), -35.0f, 35.0f);   // L > R: too close to the right wall
      driveSigned(base - c, base + c);
    } else {
      driveSigned(base, base);
    }
    hHold = getHeadingDeg();
    delay(2);
  }
  stopMotors();
  Serial.print(F("  line followed to route ")); Serial.print(route, 1);
  Serial.print(F(" cm (map says end at ")); Serial.print(RT_END, 0); Serial.println(F(")"));
  printPose(F("end of line"));
  return route;
}

// ============================================================================
// [NEW] 13. T2: face the upper rung, stop at the taught FRONT reading
// ============================================================================
void approachT2() {
  Serial.println(F("--- T2 APPROACH ---"));
  pivotRelative(RUNG_NORMAL_HDG - LINE_END_HDG, true);   // ~ -52 deg: from the last leg to square-on
  for (int k = 0; k < 6 && !timeUp(); k++) {
    float dF = readUltrasonicMedian(F_TRIG, F_ECHO, 5);
    if (dF > 200.0f) { driveStraightCm(3.0f, false); continue; }
    float e = dF - T2_FRONT_TARGET;
    Serial.print(F("  FRONT=")); Serial.print(dF, 1); Serial.print(F(" err=")); Serial.println(e, 1);
    if (fabsf(e) < 0.5f) break;
    driveStraightCm(constrain(e, -10.0f, 30.0f), false);
  }
  stopMotors();
  printPose(F("at T2"));
  Serial.println(F(">>> AT T2: POINTER ON TARGET <<<"));
}

// ============================================================================
// [NEW] FULL RUN + TASK-2-ONLY TEST
// ============================================================================
void secondHalf() {
  dropItem();
  pickUpLine();
  followLine();
  approachT2();
}

void fullRun() {
  fullRunActive = true;
  if (localizeAndGoToT1(true)) secondHalf();
  stopMotors();
  releaseMotors();
  Serial.print(F("=== RUN OVER at ")); Serial.print((millis() - runStartMs) / 1000.0f, 1);
  Serial.println(F(" s ==="));
  fullRunActive = false;
}

// robot placed by hand on T1, facing up-field (tests the line/T2 part without Task 1)
void fromT1Test() {
  runStartMs = millis();
  fullRunActive = true;
  setPose(T1_X_CM, T1_Y_CM, 90.0f);
  secondHalf();
  stopMotors(); releaseMotors();
  fullRunActive = false;
}

// ============================================================================
// 9b. SETUP & INTERACTIVE MENU
// ============================================================================
void printMenu() {
  Serial.println(F("\n=== IM ROVER READY ==="));
  Serial.println(F("  f = Drive FORWARD 20 cm       w = Pivot LEFT +90 deg"));
  Serial.println(F("  l = Localize ONLY             t = TASK 1 only (to T1)"));
  Serial.println(F("  a = FULL RUN (T1, drop, line, T2)"));
  Serial.println(F("  g = from T1 onward (place robot on T1 facing up-field)"));
  Serial.println(F("  d = test drop      i = IR + sonar monitor (5 s)      k = teach (FRONT/LEFT/RIGHT)"));
}

void monitorSensors() {
  unsigned long t0 = millis();
  while (millis() - t0 < 5000UL) {
    float dF, dL, dR;
    readAllUltrasonics(dF, dL, dR, 1);
    Serial.print(F("IR L=")); Serial.print(irBlack(IR_L_PIN));
    Serial.print(F(" R=")); Serial.print(irBlack(IR_R_PIN));
    Serial.print(F(" | F=")); Serial.print(dF, 1);
    Serial.print(F(" L=")); Serial.print(dL, 1);
    Serial.print(F(" R=")); Serial.println(dR, 1);
    delay(150);
  }
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

  pinMode(IR_L_PIN, INPUT); pinMode(IR_R_PIN, INPUT);

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
  dropperClose();

#if INTERACTIVE_MENU == 0
  for (int i = 0; i < 15; i++) { digitalWrite(LED_BUILTIN, i % 2); delay(200); }
  fullRun();
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
    } else if (c == 'w' || c == 'W') {
      resetEncoders();
      Serial.println(F("[Test 'w'] Pivoting LEFT +90.0 deg..."));
      pivotRelative(90.0f, true);
      Serial.print(F("  Hdg=")); Serial.print(getHeadingDeg(), 1); Serial.println(F(" deg"));
    } else if (c == 'l' || c == 'L') {
      localizeAndGoToT1(false);
    } else if (c == 't' || c == 'T') {
      localizeAndGoToT1(true);
    } else if (c == 'a' || c == 'A') {
      fullRun();
    } else if (c == 'g' || c == 'G') {
      fromT1Test();
    } else if (c == 'd' || c == 'D') {
      dropItem();
    } else if (c == 'i' || c == 'I') {
      monitorSensors();
    } else if (c == 'k' || c == 'K') {
      float dF, dL, dR;
      readAllUltrasonics(dF, dL, dR, 5);
      Serial.print(F("[TEACH] FRONT=")); Serial.print(dF, 1);
      Serial.print(F(" LEFT=")); Serial.print(dL, 1);
      Serial.print(F(" RIGHT=")); Serial.println(dR, 1);
      Serial.println(F("  pointer on T2 -> T2_FRONT_TARGET | IRs at line end -> LINE_END_FRONT"));
    }
    printMenu();
    lastMenuMs = millis();
  }
#else
  releaseMotors();
  digitalWrite(LED_BUILTIN, (millis() / 600) % 2);
  delay(100);
#endif
}
