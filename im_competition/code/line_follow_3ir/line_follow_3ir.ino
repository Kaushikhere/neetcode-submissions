// ============================================================================
// 3-IR LINE FOLLOWER + GYRO + T2  (standalone, from the full-race firmware)
//   IR: L = GP17, C = GP22, R = GP16 (HIGH = black)
//   Motors: R ENA GP0 IN1 GP1 IN2 GP2 | L IN3 GP3 IN4 GP4 ENB GP5 (inverted PWM)
//   Encoders: R GP12/GP13, L GP14/GP15 (quadrature)
//   Ultrasonics: F 8/9, R 6/7, L 18/19 (wall assist + front stop + T2 rails)
//   MPU6050: SDA GP20, SCL GP21 (read on core 1; encoders take over if it fails)
//
//   Power on and keep the robot STILL for 2 s (gyro calibration). It starts
//   3 s later. Serial 115200: + faster, - slower, v log on/off, other key = STOP.
//
//   SHARP V TURN (after the C turn) is detected with the GYRO only:
//     heading change >= 115 deg within 12 cm of travel, or one search pivot that
//     finds the line after >= 80 deg to the LEFT. The C turn only reaches ~90 deg
//     in 12 cm, so it can no longer be mistaken for the V (that is why the old
//     code stopped at the V: encoder heading over-counts turns when wheels slip).
//   After the V, a lost line is checked AHEAD only (+/-45, creep 5 cm, +/-45):
//     nothing ahead = line end F -> (GO_TO_T2 1) rail-lock drive into the T2 pocket.
// ============================================================================
#include <Arduino.h>
#include <math.h>
#include <Wire.h>

#define SWAP_LR_IR        0   // 1 if 'L' and 'R' IR are swapped
#define KEEP_LEFT_AT_FORK 1   // fork: main line goes LEFT
#define USE_WALL_ASSIST   1   // side ultrasonics keep the wheels off walls
#define USE_FRONT_STOP    1   // stop if something is right in front
#define STOP_AT_LINE_END  1   // after the sharp V, line lost = end of line
#define USE_GYRO          1   // MPU6050 heading (0 = encoders only)
#define GO_TO_T2          1   // after the line end, drive into the T2 pocket (0 = just stop)

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
const int PIN_I2C_SDA = 20, PIN_I2C_SCL = 21;   // GY-521 / MPU6050

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

// Sharp V turn (after the C turn) & Line End (F) detection - GYRO heading
const float LF_V_TURN_DEG        = 115.0f; // heading change this big ...
const int   LF_V_SLOTS           = 8;      // ... within 8 x 1.5 = 12 cm of travel = V done (C turn: ~90)
const float LF_V_SWING_DEG       = 80.0f;  // OR a search pivot found the line after this much LEFT swing
const float LF_LINE_END_MIN_CM   = 6.0f;   // Min travel after hairpin before Point F counts
const unsigned long LF_LINE_END_LOST_MS = 150UL;
const float LF_SETTLE_CM         = 12.0f;  // Slow settle distance after big pivot
const int   LF_SETTLE_PWM        = 112;
const float LF_CURVE_DEG         = 25.0f;  // Heading change in last ~7.5 cm -> curve
const float LF_CURVE_SPEED       = 0.80f;
const unsigned long LF_LOST_GRACE_MS = 40UL;
const float LF_SEARCH_DEG        = 150.0f; // max pivot each way: the line BEHIND (~180) is never reached

// End of line: nothing behind is ever accepted near the end
const float LF_END_TURN_DEG      = 195.0f; // backup only: total LEFT turn since A that counts as "past the V"
                                           // (used if the line is never found again / front wall stop)
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

// MPU6050 (same calibration as the full firmware)
const float GYRO_SIGN         = 1.0f;    // CCW (left) = positive
const float GYRO_SCALE        = 0.9907f; // 360.0 / 363.39
const float GYRO_SENS_500DPS  = 65.5f;
const float ACCEL_SENS_4G     = 8192.0f;
const float GYRO_DEADBAND_DPS = 0.08f;
const int32_t IMPACT_JERK_MG  = 1400;

// Gyro turns (T2 step)
const int   TURN_FAST_PWM    = 168;
const int   TURN_CREEP_PWM   = 112;
const float TURN_VEL_LEAD_S  = 0.022f;
const float TURN_TRIM_TOL    = 0.40f;
const float TURN_SYNC_KP     = 0.42f;
const unsigned long STALL_MS = 100;
const int   STALL_STEP_PWM   = 12;
const int   STALL_MAX_PWM    = 65;

// T2 rail-lock docking (geometry relative to the line end F, from the field map:
// last line piece heading -22 deg, pocket axis -62.7 deg, T2 9.0 cm ahead / 20.6 cm right of F)
const float SIDE_OFFSET_L        = 10.80f; // left  US to robot centre
const float SIDE_OFFSET_R        = 11.30f; // right US to robot centre
const float T2_PARK_STOP_CM      = 5.5f;   // front US reading where it parks
const float LINE_END_TO_AXIS_DEG = -40.7f;
const float T2_U_CM              =   9.0f;
const float T2_V_CM              = -20.6f;
const float AIM_BEFORE_T2_CM     =   8.0f;
const float IR_FWD_CM            =   7.5f;
const float RAIL_SEEN_CM         =  30.0f;
const float RAIL_LAT_GAIN        =   1.8f;
const float RAIL_LAT_MAX_DEG     =  14.0f;
const float RAIL_CENTER_TOL_CM   =   1.5f;
const float RAIL_HALF_GAP_CM     =  19.6f;


int lastSignedL = 0, lastSignedR = 0;
bool timeUp() { return false; }
float raceSecondsLeft() { return 999.0f; }
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


// ============================================================================
// 5. CORE 1 MPU6050 6-AXIS BURST ENGINE (333 Hz GYRO + ACCEL SHOCK DETECTOR)
// ============================================================================
volatile int32_t  gyroHeadingMdeg = 0;
volatile int32_t  gyroRateMdps    = 0;
volatile int32_t  imuHorizMg      = 0;
volatile int32_t  imuJerkPeakMg   = 0;
volatile uint32_t gyroHeartbeat   = 0;
volatile bool     gyroReady       = false;
volatile bool     gyroHwOk        = false;
volatile bool     gyroZeroReq     = false;
volatile bool     gyroQuickCalReq = false;
volatile bool     core1PauseReq   = false;
volatile bool     core1Paused     = false;

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

bool c1ReadRegs(uint8_t reg, uint8_t cnt, uint8_t *buf) {
  Wire.beginTransmission(c1MpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(c1MpuAddr, cnt) != cnt) return false;
  for (uint8_t i = 0; i < cnt; i++) buf[i] = Wire.read();
  return true;
}

bool c1ReadBurst6Axis(int16_t &rawAx, int16_t &rawAy, int16_t &rawGz) {
  uint8_t b[14];
  if (!c1ReadRegs(0x3B, 14, b)) return false;
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
  if (!c1WriteReg(0x6B, 0x01)) return false; // Wake + PLL X gyro clock
  delay(25);
  if (!c1WriteReg(0x1A, 0x03)) return false; // DLPF = 42 Hz
  if (!c1WriteReg(0x19, 0x01)) return false; // 500 Hz sample rate
  if (!c1WriteReg(0x1B, 0x08)) return false; // Gyro +/- 500 dps
  if (!c1WriteReg(0x1C, 0x08)) return false; // Accel +/- 4g (8192 LSB/g)
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

bool c1CalibrateBias(int samples, int maxTries) {
  for (int attempt = 0; attempt < maxTries; attempt++) {
    double sumG = 0.0, sumAx = 0.0, sumAy = 0.0;
    int16_t minR = 32767, maxR = -32768;
    int valid = 0;
    for (int i = 0; i < samples; i++) {
      int16_t ax, ay, gz;
      if (c1ReadBurst6Axis(ax, ay, gz)) {
        sumG  += (double)gz;
        sumAx += (double)ax;
        sumAy += (double)ay;
        if (gz < minR) minR = gz;
        if (gz > maxR) maxR = gz;
        valid++;
        gyroHeartbeat++;
      }
      delay(2);
    }
    if (valid >= (samples * 3) / 4 && (maxR - minR) < 330) {
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
      imuHorizMg      = 0;
      imuJerkPeakMg   = 0;
      c1PrevUs        = micros();
      return true;
    }
    delay(60);
  }
  return false;
}

void setup1() {
  delay(200);
  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();
  Wire.setClock(100000);

  if (c1InitMPU()) {
    c1CalibrateBias(450, 6);
    gyroHwOk = true;
  } else {
    gyroHwOk = false;
  }
  gyroReady = true;
}

void loop1() {
  if (core1PauseReq) {
    core1Paused = true;
    while (core1PauseReq) { delay(5); }
    core1Paused = false;
    c1PrevUs = micros();
    return;
  }
  if (!gyroHwOk) { delay(50); return; }

  if (gyroQuickCalReq) {
    c1CalibrateBias(220, 3);
    gyroQuickCalReq = false;
    return;
  }
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
  if (dt < 0.003f) return; // ~333 Hz 14-byte burst
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
  float horizMg = hypotf(axMg, ayMg);
  float jerkMg  = hypotf(axMg - c1PrevAxMg, ayMg - c1PrevAyMg);
  c1PrevAxMg = axMg;
  c1PrevAyMg = ayMg;

  c1JerkPeak = max(jerkMg, c1JerkPeak * 0.88f);

  gyroHeadingMdeg = (int32_t)lroundf(c1Heading * 1000.0f);
  gyroRateMdps    = (int32_t)lroundf(dps * 1000.0f);
  imuHorizMg      = (int32_t)lroundf(horizMg);
  imuJerkPeakMg   = (int32_t)lroundf(c1JerkPeak);
}
#endif

bool          gyroFailover         = false;
uint32_t      lastSeenHeartbeat    = 0;
unsigned long lastHeartbeatMs      = 0;
float         lastGoodGyroHdg      = 0.0f;
float         encFallbackOffsetDeg = 0.0f;

void triggerGyroFailover(const __FlashStringHelper *reason) {
  if (gyroFailover) return;
  gyroFailover = true;
  encFallbackOffsetDeg = lastGoodGyroHdg - getEncoderHeadingDeg();
  Serial.print(F("  [FAILOVER] Gyro -> Encoders ("));
  Serial.print(reason); Serial.println(F(")"));
}

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
  if (!gyroFailover && (now - lastHeartbeatMs > 180UL)) {
    triggerGyroFailover(F("I2C heartbeat stalled"));
    return false;
  }
  return !gyroFailover;
#else
  return false;
#endif
}

float getHeadingDeg() {
  if (isGyroHealthy()) {
    lastGoodGyroHdg = (float)gyroHeadingMdeg * 0.001f;
    return lastGoodGyroHdg;
  }
  return getEncoderHeadingDeg() + encFallbackOffsetDeg;
}

float getGyroRateDps() {
  if (isGyroHealthy()) return (float)gyroRateMdps * 0.001f;
  return 0.0f;
}

int32_t getImuJerkPeakMg() {
  if (isGyroHealthy()) return imuJerkPeakMg;
  return 0;
}

void resetHeadingAndEncoders(bool quickRecalGyro = false) {
  resetEncoders();
  encFallbackOffsetDeg = 0.0f;
  lastGoodGyroHdg      = 0.0f;
#if USE_GYRO
  if (gyroHwOk) {
    gyroFailover = false;
    if (quickRecalGyro) {
      gyroQuickCalReq = true;
      unsigned long t0 = millis();
      while (gyroQuickCalReq && (millis() - t0 < 1800UL)) delay(5);
    } else {
      gyroZeroReq = true;
      unsigned long t0 = millis();
      while (gyroZeroReq && (millis() - t0 < 80UL)) delay(1);
    }
    lastSeenHeartbeat = gyroHeartbeat;
    lastHeartbeatMs   = millis();
  }
#endif
}

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
  bool  hairpinDone = false;          // true once the sharp V has been turned
  float bigTurnMax  = 0.0f;            // biggest 12 cm left turn before the V (for tuning)
  bool  bigTurnLogged = false;
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

    float turn7  = hdgNow - dRing[(dIdx + DRING - 4) % DRING];  // heading change over last ~7.5 cm
    float turnV  = hdgNow - dRing[(dIdx + DRING - LF_V_SLOTS) % DRING]; // over the last ~12 cm (+ = left)

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

    // ---- SHARP V detection (gyro): big left turn in a short distance, while ON the line ----
    if (!lost && !hairpinDone) {
      if (turnV > bigTurnMax) bigTurnMax = turnV;
      if (turnV >= LF_V_TURN_DEG) {
        hairpinDone    = true;
        hairpinOdo     = fwdOdo;
        settleUntilOdo = fwdOdo + LF_SETTLE_CM;
        Serial.print(F("  [SHARP V DONE] turned ")); Serial.print(turnV, 0);
        Serial.println(F(" deg left in 12 cm -> next line loss = END (Point F)"));
      } else if (turnV >= 60.0f && !bigTurnLogged) {
        bigTurnLogged = true;
        Serial.print(F("  [TURN] ")); Serial.print(turnV, 0);
        Serial.println(F(" deg left in 12 cm (C turn? V needs 115)"));
      } else if (turnV < 30.0f) bigTurnLogged = false;
    }

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
          float leftSwing = getHeadingDeg() - searchHdg0;      // + = found to the LEFT
          float swung = fabsf(leftSwing);
          if (now - searchStartMs > 150UL) {
            driveLineMotors(0, 0);
            delay(LF_FOUND_BRAKE_MS);
          }
          if (swung > 45.0f) settleUntilOdo = fwdOdo + LF_SETTLE_CM;
          Serial.print(F("  [FOUND] after ")); Serial.print(leftSwing, 0); Serial.println(F(" deg pivot (+ = left)"));
          if (!hairpinDone && leftSwing >= LF_V_SWING_DEG) {
            hairpinDone = true;
            hairpinOdo  = fwdOdo;
            Serial.println(F("  [SHARP V DONE] big left pivot found the line -> next line loss = END (Point F)"));
          }
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
        // end of the line only once the sharp V is done (gyro)
        endZoneNow   = hairpinDone;
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
            if (hairpinDone || (lastOnLineHdg - initHdg >= LF_END_TURN_DEG)) reachedPointF = true;
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
  Serial.print(F(" s | fwdOdo "));        Serial.print(fwdOdo, 0);
  Serial.print(F(" cm | V done "));       Serial.print(hairpinDone ? F("yes") : F("NO"));
  Serial.print(F(" | biggest turn before V ")); Serial.print(bigTurnMax, 0); Serial.println(F(" deg/12cm"));
  while (Serial.available()) Serial.read();

  return (!manualAbort && !timeUp() && (reachedPointF || hairpinDone));
}


// ============================================================================
// T2: rail-lock docking (from the full firmware, Phase 6)
// ============================================================================
float readUltrasonicSingle(int trigPin, int echoPin, unsigned long toUs = 15000UL, bool clampClose = true) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long us = pulseIn(echoPin, HIGH, toUs);
  if (us == 0) return (toUs * 0.01715f);
  float cm = (float)us * 0.01715f;
  if (clampClose) return (cm < 2.5f) ? 1.5f : cm;
  return (cm < 2.0f) ? 250.0f : cm;
}

float readUltrasonicMedian(int trigPin, int echoPin, int samples = 3, unsigned long toUs = 15000UL) {
  float vals[11];
  if (samples > 11) samples = 11;
  for (int i = 0; i < samples; i++) {
    vals[i] = readUltrasonicSingle(trigPin, echoPin, toUs, true);
    delay(12);
  }
  for (int i = 0; i < samples - 1; i++) {
    for (int j = i + 1; j < samples; j++) {
      if (vals[j] < vals[i]) { float t = vals[i]; vals[i] = vals[j]; vals[j] = t; }
    }
  }
  return vals[samples / 2];
}

bool fitQuadraticMin(const float *x, const float *y, int n, float &xVertOut) {
  if (n < 3) return false;
  double s0 = (double)n, s1 = 0, s2 = 0, s3 = 0, s4 = 0;
  double t0 = 0, t1 = 0, t2 = 0;
  for (int i = 0; i < n; i++) {
    double xi = (double)x[i], xi2 = xi * xi, yi = (double)y[i];
    s1 += xi; s2 += xi2; s3 += xi2 * xi; s4 += xi2 * xi2;
    t0 += yi; t1 += xi * yi; t2 += xi2 * yi;
  }
  double det  = s4 * (s2 * s0 - s1 * s1) - s3 * (s3 * s0 - s1 * s2) + s2 * (s3 * s1 - s2 * s2);
  if (fabs(det) < 1e-7) return false;
  double detA = t2 * (s2 * s0 - s1 * s1) - s3 * (t1 * s0 - s1 * t0) + s2 * (t1 * s1 - s2 * t0);
  double detB = s4 * (t1 * s0 - s1 * t0) - t2 * (s3 * s0 - s1 * s2) + s2 * (s3 * t0 - t1 * s2);
  double a = detA / det, b = detB / det;
  if (a <= 1e-4) return false;
  xVertOut = (float)(-b / (2.0 * a));
  return true;
}

bool turnToHeadingRaw(float targetHdgDeg) {
  float hdg0    = getHeadingDeg();
  float encHdg0 = getEncoderHeadingDeg();
  float err0    = targetHdgDeg - hdg0;
  if (fabsf(err0) < 0.45f) return true;

  int dir = (err0 > 0.0f) ? 1 : -1;
  long l0, r0;
  readEncoders(l0, r0);

  setLeftDir(-dir);
  setRightDir(dir);

  long stallL = l0, stallR = r0;
  int boost = 0;
  unsigned long lastMoveMs = millis();
  unsigned long tStart     = lastMoveMs;

  float prevVelHdg   = hdg0;
  float smoothVelDps = 0.0f;
  unsigned long prevVelMs = lastMoveMs;

  while (!timeUp()) {
    unsigned long now = millis();
    if (now - tStart > 4500UL) { stopMotors(); Serial.println(F("  [Turn] Timeout!")); return false; }

    long l, r;
    readEncoders(l, r);

    if ((now - tStart > 1200UL) && (labs(l - l0) + labs(r - r0) < 10)) {
      stopMotors();
      Serial.println(F("  [ERROR] Wheels/encoders not moving!"));
      return false;
    }

    if (isGyroHealthy()) {
      float encMoved  = fabsf(getEncoderHeadingDeg() - encHdg0);
      float gyroMoved = fabsf(getHeadingDeg() - hdg0);
      if (encMoved > 28.0f && gyroMoved < 4.0f) {
        triggerGyroFailover(F("gyro stuck vs encoders"));
      }
    }

    float curHdg = getHeadingDeg();
    float remDeg = (targetHdgDeg - curHdg) * (float)dir;
    if (remDeg > fabsf(err0) + 18.0f) {
      stopMotors();
      Serial.println(F("  [ERROR] Wrong spin direction!"));
      return false;
    }

    float rateDps = 0.0f;
    bool  gyroOk  = isGyroHealthy();
    if (gyroOk) {
      rateDps = fabsf(getGyroRateDps());
    } else {
      if (now - prevVelMs >= 16UL) {
        float dtS = (float)(now - prevVelMs) * 0.001f;
        smoothVelDps = 0.5f * smoothVelDps + 0.5f * (fabsf(curHdg - prevVelHdg) / dtS);
        prevVelHdg = curHdg;
        prevVelMs  = now;
      }
      rateDps = smoothVelDps;
    }

    float dynLead = constrain(0.55f + TURN_VEL_LEAD_S * rateDps, 0.75f, 8.5f);
    float lead    = min(dynLead, fabsf(err0) * 0.45f);
    if (remDeg <= lead) break;

    if (labs(l - stallL) + labs(r - stallR) >= 3) {
      stallL = l; stallR = r; lastMoveMs = now; boost = 0;
    } else if (now - lastMoveMs > STALL_MS) {
      boost = min(boost + STALL_STEP_PWM, STALL_MAX_PWM);
      lastMoveMs = now;
    }

    float base = 0.0f;
    if (gyroOk) {
      float targetDps = constrain(remDeg * 3.8f, 26.0f, 155.0f);
      float ffPwm     = (float)TURN_CREEP_PWM + ((targetDps - 26.0f) / 129.0f) * (float)(TURN_FAST_PWM - TURN_CREEP_PWM);
      float velTrim   = constrain((targetDps - rateDps) * 0.42f, -26.0f, 26.0f);
      base = ffPwm + velTrim + (float)boost;
    } else {
      float frac = constrain((remDeg - 8.0f) / 34.0f, 0.0f, 1.0f);
      base = (float)TURN_CREEP_PWM + frac * (float)(TURN_FAST_PWM - TURN_CREEP_PWM) + (float)boost;
    }

    long dL = labs(l - l0), dR = labs(r - r0);
    float syncCorr = constrain(TURN_SYNC_KP * (float)(dR - dL), -35.0f, 35.0f);

    int lPWM = (int)roundf(base + syncCorr);
    int rPWM = (int)roundf((base - syncCorr) * RIGHT_FF);
    setMotorPWM(constrain(lPWM, 92, 235), constrain(rPWM, 82, 230));
    delay(4);
  }

  stopMotors(35);
  delay(45);
  return true;
}

bool turnToHeading(float targetHdgDeg, bool doTrim = true, float tolDeg = TURN_TRIM_TOL) {
  float initRem = targetHdgDeg - getHeadingDeg();
  if (fabsf(initRem) > 3.5f) {
    if (!turnToHeadingRaw(targetHdgDeg)) return false;
  }
  if (!doTrim) return true;

  int trimPwm = 132;
  for (int pulse = 0; pulse < 2 && !timeUp(); pulse++) {
    float before = getHeadingDeg();
    float rem    = targetHdgDeg - before;
    if (fabsf(rem) <= tolDeg || fabsf(rem) > 18.0f) break;

    int dir     = (rem > 0.0f) ? 1 : -1;
    int pulseMs = constrain((int)lroundf(9.0f + fabsf(rem) * 4.2f), 11, 36);

    setLeftDir(-dir);
    setRightDir(dir);
    setMotorPWM(trimPwm, (int)roundf((float)trimPwm * RIGHT_FF));
    delay(pulseMs);
    stopMotors(30);
    delay(45);

    float after    = getHeadingDeg();
    float newRem   = targetHdgDeg - after;
    float movedDeg = (after - before) * (float)dir;
    if (newRem * rem <= 0.0f) break;
    if (movedDeg < 0.18f) trimPwm = min(trimPwm + 24, 200);
  }
  return true;
}

// Straight drive holding a gyro heading (cm < 0 = reverse); optional front stop
void driveStraightHold(float cm, float holdHdg, float frontStopCm = 0.0f) {
  int   dir    = (cm >= 0.0f) ? 1 : -1;
  float target = fabsf(cm);
  float d0     = getEncoderDistCm();
  setLeftDir(dir); setRightDir(dir);
  int hits = 0;
  unsigned long t0 = millis(), lastUs = 0;
  while (!timeUp() && millis() - t0 < 6000UL) {
    float done = fabsf(getEncoderDistCm() - d0);
    if (done >= target - 0.3f) break;
    if (dir > 0 && frontStopCm > 0.0f && millis() - lastUs >= 25UL) {
      lastUs = millis();
      float f = readUltrasonicSingle(F_TRIG, F_ECHO, 6000UL, true);
      if (f <= frontStopCm) { if (++hits >= 2) break; } else hits = 0;
    }
    float err  = getHeadingDeg() - holdHdg;                 // + = pointing left
    float corr = constrain(5.0f * err + 0.3f * getGyroRateDps(), -40.0f, 40.0f) * (float)dir;
    int   base = (target - done < 4.0f) ? 108 : 130;
    setMotorPWM(constrain((int)roundf((float)base + corr), 85, 200),
                constrain((int)roundf(((float)base - corr) * RIGHT_FF), 75, 190));
    delay(3);
  }
  stopMotors(50);
  delay(40);
}

float railLockDrive(float aimHdg, float &axisHdg, float maxCm) {
  const float D2R = (float)M_PI / 180.0f;
  long l0, r0; readEncoders(l0, r0);
  setLeftDir(1); setRightDir(1);
  bool  railsSeen = false;
  float posLeft = 99.0f, winS = 0.0f, winPos = 0.0f; bool winSet = false;
  int   stopHits = 0, slot = 0;
  float dL = 99.0f, dR = 99.0f, dF = 99.0f;
  unsigned long t0 = millis(), lastUs = 0;
  while (!timeUp() && millis() - t0 < 9000UL) {
    unsigned long now = millis();
    long l, r; readEncoders(l, r);
    float s = 0.5f * ((float)(l - l0) / TICKS_PER_CM_L + (float)(r - r0) / TICKS_PER_CM_R);
    if (s >= maxCm) break;
    if (now - t0 > 300UL && getImuJerkPeakMg() > IMPACT_JERK_MG) { Serial.println(F("  [Rail-Lock] bump -> stop")); break; }

    if (now - lastUs >= 22UL) {               // L, F, R, F ... one sensor per slot
      lastUs = now;
      if      (slot == 0) dL = readUltrasonicSingle(L_TRIG, L_ECHO, 3500UL, true);
      else if (slot == 2) dR = readUltrasonicSingle(R_TRIG, R_ECHO, 3500UL, true);
      else {
        dF = readUltrasonicSingle(F_TRIG, F_ECHO, 6000UL, true);
        if (dF <= T2_PARK_STOP_CM) { if (++stopHits >= 2) break; } else stopHits = 0;
      }
      slot = (slot + 1) % 4;
      bool seeL = dL < RAIL_SEEN_CM, seeR = dR < RAIL_SEEN_CM;
      bool both = seeL && seeR;
      if (seeL || seeR) {                     // one rail is enough: the pocket width is known
        float fromL = RAIL_HALF_GAP_CM - (dL + SIDE_OFFSET_L);   // + = left of centre
        float fromR = (dR + SIDE_OFFSET_R) - RAIL_HALF_GAP_CM;
        posLeft = both ? 0.5f * (fromL + fromR) : (seeL ? fromL : fromR);
        if (!railsSeen) { railsSeen = true; Serial.print(F("  [Rail-Lock] rails seen, offset ")); Serial.println(posLeft, 1); }
        // learn the rails' direction: offset change per cm driven
        if (!both) { winSet = false; }
        else if (!winSet) { winSet = true; winS = s; winPos = posLeft; }
        else if (s - winS >= 4.0f) {
          float slope = (posLeft - winPos) / (s - winS);          // + = drifting left
          if (fabsf(slope) < 0.5f) {
            float railsHdg = getHeadingDeg() - asinf(slope) / D2R;
            axisHdg = 0.7f * axisHdg + 0.3f * railsHdg;
          }
          winS = s; winPos = posLeft;
        }
      }
    }

    float target = railsSeen ? axisHdg - constrain(RAIL_LAT_GAIN * posLeft, -RAIL_LAT_MAX_DEG, RAIL_LAT_MAX_DEG)
                             : aimHdg;
    float err  = getHeadingDeg() - target;                   // + = pointing left of target
    float corr = constrain(6.0f * err + 0.4f * getGyroRateDps(), -45.0f, 45.0f);
    int base = (dF < T2_PARK_STOP_CM + 9.0f) ? 112 : 130;
    setMotorPWM(constrain((int)roundf((float)base + corr), 88, 200),
                constrain((int)roundf(((float)base - corr) * RIGHT_FF), 78, 190));
    delay(3);
  }
  stopMotors(60);
  Serial.print(F("  [Rail-Lock] stop: L=")); Serial.print(dL, 1); Serial.print(F(" R=")); Serial.print(dR, 1);
  Serial.print(F(" F=")); Serial.println(dF, 1);
  return railsSeen ? posLeft : 99.0f;
}


void runPhase6_PinpointT2() {
  if (timeUp()) return;
  const float D2R = (float)M_PI / 180.0f;
  Serial.println(F("[Phase 6] RAIL-LOCK T2: exact line-end pose -> aim on centre line -> rail lock -> front stop"));
  stopMotors(40);
  delay(60);

  // 1. Back up the overshoot: IRs back on the line end F
  if (gEndByLoss && gEndOvershootCm > 0.8f && gEndOvershootCm < 25.0f) {
    Serial.print(F("  [Line End] backing ")); Serial.print(gEndOvershootCm, 1); Serial.println(F(" cm onto F"));
    driveStraightHold(-gEndOvershootCm, getHeadingDeg());
  }

  // 2. Pose in the line frame: IRs on F, robot angle to the line from the gyro
  float hLine  = gLineHdgAtEnd;
  float axisHdg = hLine + LINE_END_TO_AXIS_DEG;
  float rel    = (getHeadingDeg() - hLine) * D2R;
  float axU    = -IR_FWD_CM * cosf(rel), axV = -IR_FWD_CM * sinf(rel);
  float aDir   = LINE_END_TO_AXIS_DEG * D2R;
  float pU     = T2_U_CM - AIM_BEFORE_T2_CM * cosf(aDir);
  float pV     = T2_V_CM - AIM_BEFORE_T2_CM * sinf(aDir);
  float aimHdg = hLine + atan2f(pV - axV, pU - axU) / D2R;
  Serial.print(F("  [Aim] line hdg ")); Serial.print(hLine, 1);
  Serial.print(F(" | pocket axis ")); Serial.print(axisHdg, 1);
  Serial.print(F(" | aim ")); Serial.println(aimHdg, 1);
  if (!turnToHeading(aimHdg, true)) return;

  // 3. Drive in: aim, then rail lock, stop with the front ultrasonic
  float off = railLockDrive(aimHdg, axisHdg, 45.0f);

  // 4. Still off centre? Back out 10 cm along the axis and come in again (once)
  if (!timeUp() && raceSecondsLeft() > 9.0f && off != 99.0f && fabsf(off) > RAIL_CENTER_TOL_CM) {
    Serial.print(F("  [Rail-Lock] parked ")); Serial.print(off, 1); Serial.println(F(" cm off centre -> re-entry"));
    turnToHeading(axisHdg, true);
    driveStraightHold(-16.0f, axisHdg);                                           // back out along the axis
    float a = constrain(atan2f(off, 10.0f) / D2R, -35.0f, 35.0f);                // diagonal that cancels the offset
    turnToHeading(axisHdg - a, true);
    driveStraightHold(10.0f / cosf(a * D2R), axisHdg - a, T2_PARK_STOP_CM + 3.0f);
    turnToHeading(axisHdg, true);                                                 // straighten, then dock again
    off = railLockDrive(axisHdg, axisHdg, 14.0f);
  }
  if (!timeUp()) turnToHeading(axisHdg, true);           // square to the rails

  // 5. Square to the back wall: it is perpendicular to the pocket, so the
  //    shortest front reading in a +/-8 deg sweep is the exact pocket heading
  if (!timeUp() && raceSecondsLeft() > 5.0f) {
    float sx[5], sy[5]; int n = 0;
    for (int k = -2; k <= 2 && !timeUp(); k++) {
      if (!turnToHeading(axisHdg + 4.0f * k, true)) break;
      delay(40);
      float f = readUltrasonicMedian(F_TRIG, F_ECHO, 5, 6000UL);
      if (f < 40.0f) { sx[n] = getHeadingDeg() - axisHdg; sy[n] = f; n++; }
    }
    float v = 0.0f;
    if (n >= 3 && fitQuadraticMin(sx, sy, n, v)) axisHdg += constrain(v, -8.0f, 8.0f);
    turnToHeading(axisHdg, true);
  }
  stopMotors();
  Serial.print(F(">>> [TARGET 2 PARKED!] centre offset ")); Serial.print(off, 1);
  Serial.println(F(" cm. MISSION COMPLETE! <<<"));
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
#if USE_GYRO
  unsigned long tWait = millis();                // core 1 calibrates the gyro (keep still!)
  while (!gyroReady && (millis() - tWait < 7000UL)) {
    digitalWrite(LED_BUILTIN, (millis() / 100) % 2);
    delay(20);
  }
  lastSeenHeartbeat = gyroHeartbeat;
  lastHeartbeatMs   = millis();
  Serial.println(gyroHwOk ? F("[GYRO] MPU6050 OK - heading from the gyro")
                          : F("[GYRO] MPU6050 NOT FOUND - using encoders (check SDA GP20 / SCL GP21)"));
#endif
  resetHeadingAndEncoders(true);                 // zero heading (quick re-cal while still)
  bool atEnd = followLine(true);                 // 3 s countdown, then follows until the line end / a key
#if GO_TO_T2
  if (atEnd) runPhase6_PinpointT2();
  else Serial.println(F("[T2] skipped: line end not confirmed"));
#endif
  (void)atEnd;
  releaseMotors();
}

void loop() {
  digitalWrite(LED_BUILTIN, (millis() / 500) % 2);   // done: blinking
  delay(50);
}
