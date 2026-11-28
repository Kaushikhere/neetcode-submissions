// ============================================================================
// FAST 3-IR LINE FOLLOWER (NEVER STOPS BETWEEN WALLS) - NEW CHASSIS (+3 cm FRONT BLOCK)
//   New block: IR row + front US are 3 cm further forward (LF_IR_EXTRA_CM).
//     * line lost -> creep the extra 3 cm before braking/pivoting, so the wheels
//       pivot exactly where they did on the old chassis (V tip -> same 87 deg path)
//     * search: the 2nd direction now sweeps the full 150 deg past the start
//       (old code only got back to ~+5 deg -> never found the V's next line)
//     * T2 wall hit armed after 13 cm (nose is 3 cm longer -> wall comes 3 cm earlier)
// + GYRO >97 -> 90 + 3 DEG CW (87 DEG) + PURE GYRO FAST STRAIGHT + 9s PUSH -> STOP
//   - Standalone Line-Following Code ONLY (ready to test & integrate later)
//   - Fixes passage stop bug:
//       * REMOVED front-US stop break, 10 cm near-wall lost break, and
//         search wall-blocked break -> robot NEVER stops between walls!
//   - Higher speed across ALL parts:
//       * Line follow: lfBasePwm = 215 (up to 245 via '+'/'-')
//       * Passage C  : ~189 PWM with wall-assist centering
//       * Final T2   : 190 PWM pure gyro-locked straight drive
//   - At Hairpin E (> 97 deg on Gyro):
//       1. Stops line following permanently (IR OFF)
//       2. Turns accurately using MPU6050 gyro to 90 deg left + 3 deg CW (87.0 deg)
//       3. NO ultrasonic wall adjust, NO distance correction — PURE 200 Hz GYRO ONLY!
//       4. Detects end-wall hit via MPU6050 Accel Shock OR Encoder Stall ->
//          pushes straight for 9.0 seconds -> STOPS permanently!
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define SWAP_LR_IR        0   // 1 if 'L' and 'R' IR are swapped
#define KEEP_LEFT_AT_FORK 1   // Fork D: main line goes LEFT
#define USE_WALL_ASSIST   1   // Side US keeps wheels off walls in Passage C (never stops!)
#define USE_GYRO          1   // Core 1 MPU6050 6-axis IMU on GP20/GP21

// ---------------- CONFIRMED PINS (RASPBERRY PI PICO) ----------------
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
const int PIN_I2C_SDA = 20;
const int PIN_I2C_SCL = 21;
const int IR_BLACK_STATE = HIGH;

#define INVERT_MOTOR_DIR 0
const int8_t ENC_R_SIGN = -1;
const int8_t ENC_L_SIGN =  1;
const float TICKS_PER_CM_L = 60.5f;
const float TICKS_PER_CM_R = 60.7f;
const float WHEELBASE_CM   = 19.0f;
const float RIGHT_FF       = 0.86f;

// Calibrated 6-Axis MPU6050 Constants (CCW / LEFT = +, CW / RIGHT = -)
const float GYRO_SIGN         = 1.0f;
const float GYRO_SCALE        = 0.9907f;
const float GYRO_SENS_500DPS  = 65.5f;
const float ACCEL_SENS_4G     = 8192.0f; // LSB per g at +/-4g
const float GYRO_DEADBAND_DPS = 0.08f;

// ---------------- MAX-SPEED LINE-FOLLOW TUNING (215+ PWM!) ----------------
int         lfBasePwm            = 215;    // Fast straight speed (130..245; '+'/'-' live)
const int   LF_BASE_MIN          = 130;
const int   LF_BASE_MAX          = 245;
const int   LF_MIN_PWM_L         = 80;
const int   LF_MIN_PWM_R         = 70;

const float LF_KP                = 58.0f;  // Scaled steering PWM per error step for 215 PWM
const float LF_KD                = 105.0f; // Strong derivative kick to kill high-speed wobble
const float LF_KD_TAU_MS         = 35.0f;
const float LF_SLOW_PER_ERR      = 24.0f;  // Full 215 on straight, controlled on bends
const unsigned long LF_HARD_SPIN_MS    = 80UL;   // Snappy sharp-turn pivot at high speed
const int   LF_SPIN_FWD          = 185;
const int   LF_SPIN_REV          = 158;
const unsigned long LF_SEARCH_BRAKE_MS = 95UL;
const unsigned long LF_FOUND_BRAKE_MS  = 40UL;

const float LF_SETTLE_CM         = 8.0f;
const int   LF_SETTLE_PWM        = 155;
const float LF_CURVE_DEG         = 25.0f;
const float LF_CURVE_SPEED       = 0.88f;
const unsigned long LF_LOST_GRACE_MS = 35UL;
const float LF_SEARCH_DEG        = 150.0f;

// NEW CHASSIS: wooden block in front -> IR row & front US are this much further forward
const float LF_IR_EXTRA_CM       = 3.0f;   // set 0 for the old chassis
const int   LF_IR_EXTRA_PWM      = 150;    // creep speed for that extra distance
const float LF_RIGHT_FIRST_DEG   = 60.0f;  // all turns on this course are LEFT: a right-first search
                                           // only fixes a small overshoot, then sweeps LEFT

// Passage C wall assist (keeps wheels centered & NEVER stops between walls!)
const float LF_NEAR_WALL_CM      = 9.0f;
const float LF_PASSAGE_CM        = 12.0f;
const float LF_WALL_PAD_CM       = 4.0f;
const float LF_WALL_PAD_K        = 16.0f;
const int   LF_WALL_PAD_MAX      = 45;
const float LF_WALL_CENTER_K     = 5.5f;
const int   LF_WALL_CENTER_MAX   = 34;
const int   LF_NEAR_WALL_TURN_MAX= 65;
const float LF_NEAR_WALL_SPEED   = 0.88f;  // ~189 PWM fast & clean through Passage C!
const float LF_SPIN_WALL_CM      = 7.0f;

// ---------------- W2 CLEAR, >97 -> 90 + 3 CW (87 DEG) & PURE GYRO T2 ----------------
// 1. Go straight past W2 tip at D so back wheel clears side obstacle before turning left
const float LF_WALL_CLEAR_FWD_CM = 7.0f;
const int   LF_WALL_CLEAR_PWM    = 185;

// 2. Sharp Hairpin E (> 97 deg) -> Gyro accurately turns back to 90 deg left + 3 deg CW (87.0 deg)
const float SHARP_TRIG_DEG       = 97.0f;
const float SHARP_CW_OFFSET_DEG  =  3.0f;  // 3 degrees clockwise from 90 deg
const float SHARP_TARGET_DEG     = 90.0f - SHARP_CW_OFFSET_DEG; // = +87.0 deg!
const float GYRO_TURN_TOL_DEG    = 0.45f;
const float TURN_SYNC_KP         = 0.40f;

// 3. Final T2 Straight Drive: PURE GYRO ONLY (NO US / NO WALL ADJUST / NO DISTANCE CORRECTING!)
const int   T2_BASE_PWM          = 190;    // Fast straight gyro-locked drive into T2
const float T2_GYRO_KP           = 6.8f;   // 200 Hz gyro straight-hold P gain
const float T2_GYRO_KI           = 4.0f;   // 200 Hz gyro straight-hold I gain
const float T2_GYRO_KD           = 0.42f;  // 200 Hz gyro yaw-rate damping
const float T2_MAX_CORR          = 65.0f;

// Wall-Hit Detection using IMU + Encoders ONLY (NO ultrasonics!)
const float         T2_MIN_HIT_CM        = 16.0f - LF_IR_EXTRA_CM; // 13 cm: nose is 3 cm longer
const unsigned long T2_MIN_HIT_MS        = 450UL;  // Ignore launch accel during first 450 ms
const int32_t       T2_HIT_JERK_MG       = 1100;   // MPU6050 accel impact spike (mg)
const unsigned long T2_STALL_HIT_MS      = 240UL;  // Encoders < 6 ticks for 240 ms = hit wall!
const unsigned long T2_MAX_SEEK_MS       = 9000UL; // Safety cap before starting 9s push

// After wall hit: drive STRAIGHT for 9.0 seconds, then STOP!
const int           T2_PUSH_PWM          = 165;
const unsigned long T2_PUSH_AFTER_HIT_MS = 9000UL; // 9.0 seconds straight push -> STOP!

const unsigned long LF_MAX_STANDALONE_MS = 120000UL;
const unsigned long LF_LOG_MS            = 150UL;

int   lastSignedL = 0, lastSignedR = 0;
bool  gSharpTurn97Hit = false;
float gTargetGyroHdg  = 0.0f;

static inline float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// ============================================================================
// CORE 1 MPU6050 6-AXIS IMU ENGINE (GYRO ANGLE + ACCEL WALL-HIT DETECTOR)
// ============================================================================
volatile int32_t  gyroHeadingMdeg = 0;
volatile int32_t  gyroRateMdps    = 0;
volatile int32_t  imuJerkPeakMg   = 0;
volatile uint32_t gyroHeartbeat   = 0;
volatile bool     gyroReady       = false;
volatile bool     gyroHwOk        = false;
volatile bool     gyroZeroReq     = false;

#if USE_GYRO
static uint8_t c1MpuAddr      = 0x68;
static float   c1GyroBias     = 0.0f;
static float   c1AxBias       = 0.0f;
static float   c1AyBias       = 0.0f;
static float   c1Heading      = 0.0f;
static float   c1PrevDps      = 0.0f;
static float   c1PrevAxMg     = 0.0f;
static float   c1PrevAyMg     = 0.0f;
static float   c1JerkPeak     = 0.0f;
static unsigned long c1PrevUs = 0;
static int     c1I2cErrCount  = 0;

bool c1WriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(c1MpuAddr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

bool c1ReadBurst6Axis(int16_t &rawAx, int16_t &rawAy, int16_t &rawGz) {
  uint8_t b[14];
  Wire.beginTransmission(c1MpuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(c1MpuAddr, (uint8_t)14) != 14) return false;
  for (uint8_t i = 0; i < 14; i++) b[i] = Wire.read();
  rawAx = (int16_t)((b[0]  << 8) | b[1]);
  rawAy = (int16_t)((b[2]  << 8) | b[3]);
  rawGz = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

bool c1InitMPU() {
  Wire.beginTransmission(0x68);
  if (Wire.endTransmission() == 0) c1MpuAddr = 0x68;
  else {
    Wire.beginTransmission(0x69);
    if (Wire.endTransmission() == 0) c1MpuAddr = 0x69;
    else return false;
  }
  if (!c1WriteReg(0x6B, 0x01)) return false; // Wake + PLL X gyro
  delay(25);
  if (!c1WriteReg(0x1A, 0x03)) return false; // DLPF 42 Hz
  if (!c1WriteReg(0x19, 0x01)) return false; // 500 Hz
  if (!c1WriteReg(0x1B, 0x08)) return false; // Gyro +/- 500 dps
  if (!c1WriteReg(0x1C, 0x08)) return false; // Accel +/- 4g (wall-hit detection)
  delay(20);
  return true;
}

void c1RecoverI2C() {
  Wire.end();
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(PIN_I2C_SCL, LOW);  delayMicroseconds(10);
    digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(10);
  }
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();
  Wire.setClock(100000);
  c1InitMPU();
  c1PrevUs = micros();
}

void setup1() {
  delay(200);
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();
  Wire.setClock(100000);

  if (c1InitMPU()) {
    double sumG = 0.0, sumAx = 0.0, sumAy = 0.0;
    int valid = 0;
    for (int i = 0; i < 380; i++) {
      int16_t ax, ay, gz;
      if (c1ReadBurst6Axis(ax, ay, gz)) {
        sumG  += (double)gz;
        sumAx += (double)ax;
        sumAy += (double)ay;
        valid++;
        gyroHeartbeat++;
      }
      delay(2);
    }
    if (valid >= 240) {
      c1GyroBias      = (float)(sumG  / (double)valid);
      c1AxBias        = (float)(sumAx / (double)valid);
      c1AyBias        = (float)(sumAy / (double)valid);
      c1Heading       = 0.0f;
      c1PrevDps       = 0.0f;
      c1PrevAxMg      = 0.0f;
      c1PrevAyMg      = 0.0f;
      c1JerkPeak      = 0.0f;
      gyroHeadingMdeg = 0;
      gyroRateMdps    = 0;
      imuJerkPeakMg   = 0;
      c1PrevUs        = micros();
      gyroHwOk        = true;
    }
  }
  gyroReady = true;
}

void loop1() {
  if (!gyroHwOk) { delay(50); return; }

  if (gyroZeroReq) {
    c1Heading       = 0.0f;
    c1PrevDps       = 0.0f;
    c1JerkPeak      = 0.0f;
    gyroHeadingMdeg = 0;
    gyroRateMdps    = 0;
    imuJerkPeakMg   = 0;
    gyroZeroReq     = false;
  }

  unsigned long nowUs = micros();
  float dt = (float)(nowUs - c1PrevUs) * 1e-6f;
  if (dt < 0.003f) return;
  if (dt > 0.05f)  dt = 0.003f;
  c1PrevUs = nowUs;

  int16_t rawAx, rawAy, rawGz;
  if (!c1ReadBurst6Axis(rawAx, rawAy, rawGz)) {
    if (++c1I2cErrCount >= 5) {
      c1I2cErrCount = 0;
      c1RecoverI2C();
    }
    return;
  }
  c1I2cErrCount = 0;
  gyroHeartbeat++;

  float dps = (((float)rawGz - c1GyroBias) / GYRO_SENS_500DPS) * GYRO_SIGN * GYRO_SCALE;
  if (fabsf(dps) < GYRO_DEADBAND_DPS) dps = 0.0f;

  c1Heading += 0.5f * (dps + c1PrevDps) * dt;
  c1PrevDps = dps;

  float axMg = (((float)rawAx - c1AxBias) / ACCEL_SENS_4G) * 1000.0f;
  float ayMg = (((float)rawAy - c1AyBias) / ACCEL_SENS_4G) * 1000.0f;
  float jerkMg = hypotf(axMg - c1PrevAxMg, ayMg - c1PrevAyMg);
  c1PrevAxMg = axMg;
  c1PrevAyMg = ayMg;
  c1JerkPeak = max(jerkMg, c1JerkPeak * 0.88f);

  gyroHeadingMdeg = (int32_t)lroundf(c1Heading * 1000.0f);
  gyroRateMdps    = (int32_t)lroundf(dps * 1000.0f);
  imuJerkPeakMg   = (int32_t)lroundf(c1JerkPeak);
}
#endif

// ============================================================================
// ENCODERS & IMU ACCESSORS
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

float getHeadingDeg() { return getEncoderHeadingDeg(); }

bool          gyroFailover      = false;
uint32_t      lastSeenHeartbeat = 0;
unsigned long lastHeartbeatMs   = 0;
float         lastGoodGyroHdg   = 0.0f;
float         encFallbackOffDeg = 0.0f;

bool isGyroHealthy() {
#if USE_GYRO
  if (!gyroHwOk) return false;
  uint32_t hb = gyroHeartbeat;
  unsigned long now = millis();
  if (hb != lastSeenHeartbeat) {
    lastSeenHeartbeat = hb;
    lastHeartbeatMs   = now;
    if (!gyroFailover) {
      lastGoodGyroHdg = (float)gyroHeadingMdeg * 0.001f;
      return true;
    }
    return false;
  }
  if (!gyroFailover && (now - lastHeartbeatMs > 200UL)) {
    gyroFailover      = true;
    encFallbackOffDeg = lastGoodGyroHdg - getEncoderHeadingDeg();
    return false;
  }
  return !gyroFailover;
#else
  return false;
#endif
}

float getGyroHeadingDeg() {
  if (isGyroHealthy()) {
    lastGoodGyroHdg = (float)gyroHeadingMdeg * 0.001f;
    return lastGoodGyroHdg;
  }
  return getEncoderHeadingDeg() + encFallbackOffDeg;
}

float getGyroRateDps() {
  if (isGyroHealthy()) return (float)gyroRateMdps * 0.001f;
  return 0.0f;
}

int32_t getImuJerkPeakMg() {
  if (isGyroHealthy()) return imuJerkPeakMg;
  return 0;
}

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

void setMotorPWM(int leftPwm, int rightPwm) {
  analogWrite(L_ENB, 255 - constrain(leftPwm,  0, 255));
  analogWrite(R_ENA, 255 - constrain(rightPwm, 0, 255));
}

void driveLineMotors(int lCmd, int rCmd) {
  lastSignedL = lCmd;
  lastSignedR = rCmd;

  if (abs(lCmd) < 20) {
    digitalWrite(L_IN3, LOW); digitalWrite(L_IN4, LOW);
    analogWrite(L_ENB, 0); // active brake
  } else {
    setLeftDir(lCmd > 0 ? 1 : -1);
    analogWrite(L_ENB, 255 - constrain(abs(lCmd), LF_MIN_PWM_L, 252));
  }

  if (abs(rCmd) < 20) {
    digitalWrite(R_IN1, LOW); digitalWrite(R_IN2, LOW);
    analogWrite(R_ENA, 0); // active brake
  } else {
    setRightDir(rCmd > 0 ? 1 : -1);
    int pwmR = constrain((int)roundf((float)abs(rCmd) * RIGHT_FF), LF_MIN_PWM_R, 250);
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

// ============================================================================
// IR & SIDE ULTRASONIC SENSORS (PASSAGE C ASSIST ONLY — NEVER STOPS ROBOT!)
// ============================================================================
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

float lfBufL[3] = { 99.0f, 99.0f, 99.0f };
float lfBufR[3] = { 99.0f, 99.0f, 99.0f };
int   lfIdxL = 0, lfIdxR = 0;
float lfSideL = 99.0f, lfSideR = 99.0f;
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
  lfSideL = 99.0f; lfSideR = 99.0f;
  lfUsSlot = 0;
  lfLastUsMs = 0;
}

// Only pings Left & Right side US during line follow (Front US NEVER stops passage!)
void updateLineUltrasonics(unsigned long now) {
  if (lfLastUsMs != 0 && (now - lfLastUsMs < 15UL)) return;
  lfLastUsMs = now;
  if (lfUsSlot == 0) {
    lfBufL[lfIdxL] = lfUsRead(L_TRIG, L_ECHO, 2500UL);
    lfIdxL = (lfIdxL + 1) % 3;
    lfSideL = median3(lfBufL[0], lfBufL[1], lfBufL[2]);
  } else {
    lfBufR[lfIdxR] = lfUsRead(R_TRIG, R_ECHO, 2500UL);
    lfIdxR = (lfIdxR + 1) % 3;
    lfSideR = median3(lfBufR[0], lfBufR[1], lfBufR[2]);
  }
  lfUsSlot = (lfUsSlot + 1) % 2;
}

// ============================================================================
// PRECISION VELOCITY-PROFILED GYRO TURN TO EXACT 90 DEG + 3 DEG CW (+87.0 DEG)
// ============================================================================
void turnBackToGyroHeading(float targetGyroHdg) {
  Serial.print(F("  [GYRO -> 90 + 3 CW (87)] Start=")); Serial.print(getGyroHeadingDeg(), 2);
  Serial.print(F(" deg -> Target="));                   Serial.print(targetGyroHdg, 2); Serial.println(F(" deg"));

  float err0 = targetGyroHdg - getGyroHeadingDeg();
  if (fabsf(err0) > 2.5f) {
    int dir = (err0 > 0.0f) ? 1 : -1;
    long l0, r0;
    readEncoders(l0, r0);
    setLeftDir(-dir);
    setRightDir(dir);

    unsigned long t0 = millis();
    while (millis() - t0 < 2600UL) {
      long l, r;
      readEncoders(l, r);
      float cur     = getGyroHeadingDeg();
      float rem     = (targetGyroHdg - cur) * (float)dir;
      float rateDps = fabsf(getGyroRateDps());
      float lead    = constrain(0.50f + 0.022f * rateDps, 0.65f, 7.5f);
      if (rem <= lead) break;

      float targetDps = constrain(rem * 3.8f, 24.0f, 145.0f);
      float ffPwm     = 110.0f + ((targetDps - 24.0f) / 121.0f) * 55.0f;
      float velTrim   = isGyroHealthy() ? constrain((targetDps - rateDps) * 0.40f, -22.0f, 22.0f) : 0.0f;
      float base      = ffPwm + velTrim;

      long dL = labs(l - l0), dR = labs(r - r0);
      float sync = constrain(TURN_SYNC_KP * (float)(dR - dL), -28.0f, 28.0f);
      setMotorPWM(constrain((int)roundf(base + sync), 88, 220),
                  constrain((int)roundf((base - sync) * RIGHT_FF), 78, 215));
      delay(4);
    }
    stopMotors(45);
    delay(55);
  }

  // Up to 4 micro-trims to nail exact +/- 0.45 deg!
  int trimPwm = 122;
  for (int pulse = 0; pulse < 4; pulse++) {
    float before = getGyroHeadingDeg();
    float rem    = targetGyroHdg - before;
    if (fabsf(rem) <= GYRO_TURN_TOL_DEG || fabsf(rem) > 25.0f) break;

    int dir     = (rem > 0.0f) ? 1 : -1;
    int pulseMs = constrain((int)lroundf(8.0f + fabsf(rem) * 4.2f), 10, 34);
    setLeftDir(-dir);
    setRightDir(dir);
    setMotorPWM(trimPwm, (int)roundf((float)trimPwm * RIGHT_FF));
    delay(pulseMs);
    stopMotors(35);
    delay(50);

    float after = getGyroHeadingDeg();
    if ((targetGyroHdg - after) * rem <= 0.0f) {
      if (fabsf(targetGyroHdg - after) <= GYRO_TURN_TOL_DEG) break;
    } else if (fabsf(after - before) < 0.20f) {
      trimPwm = min(trimPwm + 22, 195);
    }
  }

  Serial.print(F("  [GYRO LOCKED] Final=")); Serial.print(getGyroHeadingDeg(), 2);
  Serial.print(F(" deg (err="));            Serial.print(getGyroHeadingDeg() - targetGyroHdg, 2);
  Serial.println(F(" deg)"));
}

// ============================================================================
// PURE GYRO FAST STRAIGHT DRIVE (ZERO US / ZERO WALL ADJUST!) -> HIT -> 9s PUSH -> STOP
// ============================================================================
void driveGyroStraightThenPush9s(float targetGyroHdg) {
  Serial.println(F("[T2 GYRO-ONLY STRAIGHT] NO US / NO wall adjust -> Driving straight fast!"));

  long l0, r0;
  readEncoders(l0, r0);
  long stallRefL = l0, stallRefR = r0;

  setLeftDir(1);
  setRightDir(1);

  float gyroInteg = 0.0f;
  unsigned long tStart     = millis();
  unsigned long prevMs     = tStart;
  unsigned long lastMoveMs = tStart;
  unsigned long lastLogMs  = 0;

  bool        abortedKey = false;
  const char *hitReason  = "max seek time";

  // -------- STAGE 1: FAST PURE-GYRO STRAIGHT UNTIL END WALL HIT --------
  while (millis() - tStart < T2_MAX_SEEK_MS) {
    unsigned long now = millis();

    if (Serial.available()) {
      char k = Serial.read();
      if (k != '\r' && k != '\n') { abortedKey = true; break; }
    }

    long curL, curR;
    readEncoders(curL, curR);
    float movedCm = 0.5f * ((float)(curL - l0) / TICKS_PER_CM_L + (float)(curR - r0) / TICKS_PER_CM_R);

    // Hit Detector #1: MPU6050 Accelerometer impact spike (armed after launch ramp & 16 cm)
    if ((now - tStart >= T2_MIN_HIT_MS) && (movedCm >= T2_MIN_HIT_CM)) {
      if (getImuJerkPeakMg() >= T2_HIT_JERK_MG) {
        hitReason = "MPU6050 Accel Impact Spike";
        break;
      }
    }

    // Hit Detector #2: Encoder stall against end rung
    if (labs(curL - stallRefL) + labs(curR - stallRefR) >= 6) {
      stallRefL  = curL;
      stallRefR  = curR;
      lastMoveMs = now;
    } else if ((now - tStart >= T2_MIN_HIT_MS) && (movedCm >= T2_MIN_HIT_CM) && (now - lastMoveMs >= T2_STALL_HIT_MS)) {
      hitReason = "Encoder Stall on End Wall";
      break;
    }

    // 200 Hz (5 ms) Pure Gyro Heading Hold
    if (now - prevMs >= 5UL) {
      float dt = min((float)(now - prevMs) * 0.001f, 0.04f);
      prevMs = now;

      float hdgErr = getGyroHeadingDeg() - targetGyroHdg; // + = drifted LEFT -> speed up Left (+corr)
      gyroInteg = constrain(gyroInteg + hdgErr * dt, -14.0f, 14.0f);
      float corr = constrain(T2_GYRO_KP * hdgErr + T2_GYRO_KI * gyroInteg + T2_GYRO_KD * getGyroRateDps(),
                             -T2_MAX_CORR, T2_MAX_CORR);

      // Brief 180 ms soft launch ramp (135 -> 190 PWM) so accel doesn't false-trigger at start
      unsigned long el = now - tStart;
      int base = T2_BASE_PWM;
      if (el < 180UL) base = 135 + (int)((long)(T2_BASE_PWM - 135) * (long)el / 180L);

      int lPwm = constrain((int)roundf((float)base + corr), 90, 250);
      int rPwm = constrain((int)roundf(((float)base - corr) * RIGHT_FF), 80, 248);
      setMotorPWM(lPwm, rPwm);
    }

    if (now - lastLogMs >= LF_LOG_MS) {
      lastLogMs = now;
      Serial.print(F("[T2 GYRO] d=")); Serial.print(movedCm, 1);
      Serial.print(F(" | HdgErr="));   Serial.print(getGyroHeadingDeg() - targetGyroHdg, 2);
      Serial.print(F(" | Jerk="));     Serial.println(getImuJerkPeakMg());
    }
  }

  if (abortedKey) {
    stopMotors(60);
    Serial.println(F("[T2 ABORT] Key pressed."));
    return;
  }

  // -------- STAGE 2: WALL HIT DETECTED -> STRAIGHT GYRO PUSH FOR 9.0 SECONDS -> STOP! --------
  Serial.print(F(">>> [WALL HIT: ")); Serial.print(hitReason);
  Serial.println(F("] Pushing STRAIGHT for 9.0 s then STOPPING! <<<"));

  setLeftDir(1);
  setRightDir(1);
  unsigned long pushStart = millis();
  prevMs = pushStart;
  while (millis() - pushStart < T2_PUSH_AFTER_HIT_MS) {
    unsigned long now = millis();
    if (Serial.available()) {
      char k = Serial.read();
      if (k != '\r' && k != '\n') break;
    }
    if (now - prevMs >= 10UL) {
      prevMs = now;
      float hdgErr = getGyroHeadingDeg() - targetGyroHdg;
      float corr   = constrain(3.5f * hdgErr + 0.30f * getGyroRateDps(), -30.0f, 30.0f);
      setMotorPWM(constrain((int)roundf((float)T2_PUSH_PWM + corr), 90, 235),
                  constrain((int)roundf(((float)T2_PUSH_PWM - corr) * RIGHT_FF), 80, 230));
    }
  }

  stopMotors(80);
  Serial.println(F(">>> [T2 COMPLETE — 9.0 s PUSH DONE & STOPPED!] <<<"));
}

// ============================================================================
// MAIN HIGH-SPEED 3-IR LINE FOLLOWER (ZERO PASSAGE STOPS!)
// ============================================================================
bool followLine(bool standaloneTest = true) {
  gSharpTurn97Hit = false;

  if (standaloneTest) {
    Serial.println(F("\n[MAX-SPEED LINE FOLLOW] starting in 3 s... (+ faster, - slower, v log, any key STOP)"));
    for (int i = 3; i > 0; i--) { Serial.print(i); Serial.println(F("...")); delay(1000); }
    while (Serial.available()) Serial.read();
  }

  resetLineUltrasonics();
  for (int i = 0; i < 8; i++) {
    updateLineUltrasonics(millis());
    delay(16);
  }

  float prevE = 0.0f, dTerm = 0.0f;
  int   lastSide = -1;
  unsigned long lastSideMs     = 0;
  unsigned long lastJunctionMs = 0;
  int   hardSide = 0;
  unsigned long hardStartMs = 0;

  unsigned long lostStartMs = 0;
  int   searchDir = -1, searchPhase = 0;
  float lostHdg0 = 0.0f;
  bool  lostNearWall = false;
  bool  inExtraCreep = false;
  float extraDist0 = 0.0f;               // creep start (new-chassis extra 3 cm)
  unsigned long extraDoneMs = 0;         // when the creep finished (brake starts)

  bool  inPassage = false;
  int   passIn = 0, passOut = 0;

  // Back-wheel clearance state at W2 exit (D bend before Hairpin E)
  bool  seenLeftWall    = false;
  int   leftWallHits    = 0;
  bool  didWallClearFwd = false;
  float wallClearEndOdo = -1.0f;

  const int RING = 16;
  float hRing[RING];
  float initHdg = getHeadingDeg();
  for (int i = 0; i < RING; i++) hRing[i] = initHdg;
  int hIdx = 0;
  unsigned long lastRingMs = 0;

  // 14-slot x 1.5 cm heading ring; gRing freezes during pivots/lost for exact incoming straight ref
  const int DRING = 14;
  float dRing[DRING];
  float gRing[DRING];
  float initGyroHdg = getGyroHeadingDeg();
  for (int i = 0; i < DRING; i++) { dRing[i] = initHdg; gRing[i] = initGyroHdg; }
  int   dIdx = 0;
  float fwdOdo = 0.0f, lastOdoDist = getEncoderDistCm(), lastRingOdo = 0.0f;
  float settleUntilOdo = -1.0f;

  float searchHdg0 = 0.0f;
  float srchStuckHdg = 0.0f; unsigned long srchStuckMs = 0;
  bool  searching = false;
  unsigned long searchStartMs = 0;

  bool  logOn = true;
  unsigned long lastLogMs  = 0;
  unsigned long t0         = millis();
  unsigned long lastLoopMs = t0;

  const char *why = "unknown";
  bool manualAbort = false;

  while (true) {
    unsigned long now = millis();
    float dtMs = (float)(now - lastLoopMs);
    lastLoopMs = now;

    if (standaloneTest && (now - t0 > LF_MAX_STANDALONE_MS)) { why = "2-min safety limit"; break; }

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

    // ---- Background side ultrasonics (L & R only — NEVER stops between walls!) ----
    updateLineUltrasonics(now);

    // ---- Debounced 3-IR read ----
    bool l, c, r;
    readIR3(l, c, r);
    int code = ((int)l << 2) | ((int)c << 1) | (int)r;

    // ---- Heading & forward-distance rings ----
    float hdgNow  = getHeadingDeg();
    float gyroNow = getGyroHeadingDeg();
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
    bool onLineMoving = (code != 0b000) && !searching;
    if (inExtraCreep) lastRingOdo = fwdOdo;   // extra-3cm creep: keep the incoming-heading ring as it was
    if (fwdOdo - lastRingOdo >= 1.5f) {
      lastRingOdo = fwdOdo;
      dIdx = (dIdx + 1) % DRING;
      dRing[dIdx] = hdgNow;
      gRing[dIdx] = onLineMoving ? gyroNow : gRing[(dIdx + DRING - 1) % DRING];
    }

    // ---- >97 DEG SHARP TURN RULE -> GYRO BACK TO 90 + 3 CW (+87.0 DEG) -> PURE GYRO STRAIGHT ----
    float entryGyroHdg = 0.25f * (gRing[(dIdx + DRING - 5) % DRING] +
                                  gRing[(dIdx + DRING - 6) % DRING] +
                                  gRing[(dIdx + DRING - 7) % DRING] +
                                  gRing[(dIdx + DRING - 8) % DRING]);
    float gyroTurn = gyroNow - entryGyroHdg; // + = LEFT turn

    if (gyroTurn > SHARP_TRIG_DEG) {
      gSharpTurn97Hit = true;
      gTargetGyroHdg  = entryGyroHdg + SHARP_TARGET_DEG; // 90 deg left then 3 deg CW = +87.0 deg!
      why = "SHARP TURN > 97 deg -> Gyro back to 87 deg (90 + 3 CW) -> Pure Gyro Straight!";
      Serial.print(F("  [>97 TRIGGERED] gyroTurn=")); Serial.print(gyroTurn, 1);
      Serial.print(F(" deg | entry="));               Serial.print(entryGyroHdg, 1);
      Serial.print(F(" -> target87="));                Serial.println(gTargetGyroHdg, 1);
      break;
    }

    float turn7 = hdgNow - dRing[(dIdx + DRING - 5) % DRING];

    // ---- Passage detection with hysteresis ----
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

    // ---- Back-wheel clearance past W2 tip at D (go 7 cm straight before turning left) ----
    if (inPassage) {
      seenLeftWall = true;
    } else if (!didWallClearFwd && lfSideL < LF_NEAR_WALL_CM) {
      if (++leftWallHits >= 15) seenLeftWall = true;
    } else if (!seenLeftWall) {
      leftWallHits = 0;
    }

    if (seenLeftWall && !didWallClearFwd && !inPassage && lfSideL > LF_PASSAGE_CM + 4.0f) {
      didWallClearFwd = true;
      wallClearEndOdo = fwdOdo + LF_WALL_CLEAR_FWD_CM;
      Serial.print(F("  [W2 CLEARED] Straight +")); Serial.print(LF_WALL_CLEAR_FWD_CM, 1);
      Serial.println(F(" cm so back wheel clears wall!"));
    }

    if (didWallClearFwd && fwdOdo < wallClearEndOdo) {
      driveLineMotors(LF_WALL_CLEAR_PWM, LF_WALL_CLEAR_PWM);
      hardSide    = 0;
      lostStartMs = 0;
      lastSide    = -1;
      lastSideMs  = now;
      delay(2);
      continue;
    }

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
      case 0b111: e = -0.7f; break;
      case 0b101: e = -2.0f; break;
#else
      case 0b111: e =  0.0f; break;
      case 0b101: e =  0.0f; break;
#endif
      default: lost = true; break;
    }
    if (!lost && e != 0.0f) { lastSide = (e < 0.0f) ? -1 : 1; lastSideMs = now; }
    if (code == 0b111 || code == 0b101) lastJunctionMs = now;

    int cmdL = 0, cmdR = 0;
    const char *mode = "LINE";
    inExtraCreep = false;

    if (!lost) {
      if (lostStartMs != 0) {
        lostStartMs = 0;
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
        cmdR  = max((int)(base - turn), 0);
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
          else                 { cmdL = max(cmdL, -75); cmdR = max(cmdR, -75); }
        }
      }
    } else {
      // ---------------- LINE LOST (000) — NEVER STOPS BETWEEN WALLS! ----------------
      if (lostStartMs == 0) {
        lostStartMs  = now;
        lostHdg0     = hdgNow;
        searchPhase  = 0;
        lostNearWall = nearWall;
        extraDist0   = -1.0f;
        extraDoneMs  = 0;
        // right after the W2-clear straight the IRs are already past the corner: no extra creep
        if (didWallClearFwd && fwdOdo < wallClearEndOdo + 8.0f) extraDoneMs = now;
        if (KEEP_LEFT_AT_FORK && lastJunctionMs != 0 && (now - lastJunctionMs < 500UL)) {
          searchDir = -1;
        } else if (now - lastSideMs < 400UL) {
          searchDir = lastSide;
        } else {
          float rot = hRing[hIdx] - hRing[(hIdx + 1) % RING];
          if (rot > 1.5f)       searchDir =  1;
          else if (rot < -1.5f) searchDir = -1;
          else                  searchDir = lastSide;
        }
      }

      bool openArea = !(lostNearWall || nearWall);
      if (now - lostStartMs < LF_LOST_GRACE_MS) {
        cmdL = lastSignedL; cmdR = lastSignedR; mode = "GRACE";
      } else if (!searching && openArea && extraDoneMs == 0) {
        // NEW CHASSIS: IRs are 3 cm further ahead -> creep those 3 cm straight first
        if (extraDist0 < 0.0f) extraDist0 = getEncoderDistCm();
        if (getEncoderDistCm() - extraDist0 >= LF_IR_EXTRA_CM || now - lostStartMs > 450UL) {
          extraDoneMs = now;
          cmdL = 0; cmdR = 0; mode = "BRAKE";
        } else {
          cmdL = LF_IR_EXTRA_PWM; cmdR = LF_IR_EXTRA_PWM; mode = "EXTRA3"; inExtraCreep = true;
        }
      } else if (!searching && openArea && (now - extraDoneMs < LF_SEARCH_BRAKE_MS)) {
        cmdL = 0; cmdR = 0; mode = "BRAKE";
      } else if (lostNearWall || nearWall) {
        // NEVER stop between walls! Keep driving through centered by wallTurn until IR re-locks!
        float turn = inPassage ? wallTurn : ((float)searchDir * 30.0f + wallTurn);
        int   base = (int)((float)lfBasePwm * 0.72f); // ~155 PWM steady drive through gap
        cmdL = max((int)(base + turn), 80);
        cmdR = max((int)(base - turn), 80);
        mode = "PASS-FWD";
      } else {
        if (!searching) {
          searching     = true;
          searchHdg0    = hdgNow;
          searchStartMs = now;
          lostHdg0      = searchHdg0;
        }
        float rel      = hdgNow - lostHdg0;
        float progress = (searchDir < 0) ? rel : -rel;
        bool wallAhead = (searchDir < 0) ? (lfSideL < LF_SPIN_WALL_CM) : (lfSideR < LF_SPIN_WALL_CM);
        if (fabsf(hdgNow - srchStuckHdg) > 1.0f || srchStuckMs == 0) { srchStuckHdg = hdgNow; srchStuckMs = now; }
        else if (now - srchStuckMs > 600UL) { wallAhead = true; srchStuckMs = 0; }
        float sweepMax = (searchPhase == 0 && searchDir > 0) ? LF_RIGHT_FIRST_DEG : LF_SEARCH_DEG;
        if (progress >= sweepMax || wallAhead) {
          if (searchPhase == 0) {
            searchPhase = 1;
            searchDir   = -searchDir;   // sweep back THROUGH the start to 150 deg on the other side
          } else {
            // Do NOT stop dead if blocked by walls — drive forward centered to clear the gap!
            searching   = false;
            searchPhase = 0;
            lostNearWall = true;
            cmdL = 140; cmdR = 140;
            mode = "RECOVER";
          }
        }
        if (searching) {
          cmdL = (searchDir < 0) ? -LF_SPIN_REV :  LF_SPIN_FWD;
          cmdR = (searchDir < 0) ?  LF_SPIN_FWD : -LF_SPIN_REV;
          mode = (searchDir < 0) ? "SEARCH-L" : "SEARCH-R";
        }
      }
    }

    driveLineMotors(cmdL, cmdR);

    if (logOn && (now - lastLogMs >= LF_LOG_MS)) {
      lastLogMs = now;
      Serial.print(F("IR=")); Serial.print(l); Serial.print(c); Serial.print(r);
      Serial.print(F(" "));   Serial.print(mode);
      Serial.print(F(" | gT="));   Serial.print(gyroTurn, 0);
      Serial.print(F(" | US L=")); Serial.print(lfSideL, 1);
      Serial.print(F(" R="));      Serial.println(lfSideR, 1);
    }

    delay(2);
  }

  stopMotors(75);
  Serial.print(F("[Line Follow] stopped: ")); Serial.print(why);
  Serial.print(F(" | time "));               Serial.print((millis() - t0) * 0.001f, 1);
  Serial.print(F(" s | fwdOdo "));           Serial.print(fwdOdo, 0); Serial.println(F(" cm"));
  while (Serial.available()) Serial.read();

  // ---- >97 HIT -> GYRO TO 90 + 3 CW (87 DEG) -> PURE GYRO FAST STRAIGHT -> 9s PUSH -> STOP! ----
  if (gSharpTurn97Hit && !manualAbort) {
    delay(60);
    turnBackToGyroHeading(gTargetGyroHdg);
    delay(50);
    driveGyroStraightThenPush9s(gTargetGyroHdg);
    return true;
  }

  return !manualAbort;
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

#if USE_GYRO
  // Wait for Core 1 MPU6050 6-axis calibration (~1 s — keep robot still!)
  unsigned long tWait = millis();
  while (!gyroReady && (millis() - tWait < 5000UL)) {
    digitalWrite(LED_BUILTIN, (millis() / 100) % 2);
    delay(20);
  }
  lastSeenHeartbeat = gyroHeartbeat;
  lastHeartbeatMs   = millis();
#endif

  delay(300);
  resetEncoders();
  followLine(true);
  releaseMotors();
}

void loop() {
  digitalWrite(LED_BUILTIN, (millis() / 500) % 2);
  delay(50);
}
