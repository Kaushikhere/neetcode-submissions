// ============================================================================
// IM Competition — STANDALONE BATTERY FULL-RACE FIRMWARE (CALIBRATED VALUES BAKED IN)
//   Command 'a': In-Situ T1 Self-Calibration!
//               Place drop box on T1 facing Back Wall -> dead-straight 0.00 deg
//               drive to F = 16.5 cm -> measures exact F_T1, L_back, R_back,
//               F_left, F_right & sensor offsets -> auto-feeds RAM + saves Flash!
//   Phases 1-3: Ultimate Fusion (Translate-First -> 6-Axis 360 -> Pull 15.5 cm ->
//               Distance-First L/R Match -> Unconditional +/-15 Gyro/Sum Lock ->
//               Pure 0.00-deg Dead-Straight Reverse -> Full 180-deg Servo Drop)
//   Phase 4   : Wall-Safe Clearance -> Outer Highway Seek & Lock
//   Phase 5   : Integrated 3-IR Line Follower (L=GP17, C=GP22, R=GP16)
//               - Majority-of-3 debounced 3-IR + decaying derivative kick
//               - Brake -> Slow Pivot -> Brake -> Settle on Hairpin E
//               - Junction memory (111/101 -> search LEFT) + Keep-Left Fork D
//               - >= 85 deg left in 15 cm = Hairpin Done -> next loss = Point F
//               - Side-US wall safety (no spin near walls, passage centering,
//                 wall pad, front stop)
//   Phase 6   : PINPOINT_T2 Pocket Entry & Rail-Centered Park
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <EEPROM.h>

// 0 = USB Serial Test Menu (a/v/z/1/2/0/p/g/w/i/c/t/f/k/d/r)
// 1 = Standalone battery mode (calibrates when still -> waits for 12V switch -> Full Race)
#define AUTO_START_ON_MOTOR_SWITCH 1

// 1 = Use Core 1 MPU6050 6-axis IMU with auto I2C recovery & encoder failover
#define USE_GYRO 1

// 1 = MG90S servo on GP27
#define USE_SERVO 1

// 1 = Persist auto-calibrated T1/box values to RP2040 Flash (survives reboot & battery switch!)
#define USE_EEPROM_CAL 1

// ---------------- Phase 5 Line Follower Options ----------------
#define SWAP_LR_IR        0   // Set 1 if 'i' shows LEFT (GP17) and RIGHT (GP16) IR swapped
#define KEEP_LEFT_AT_FORK 1   // Fork D: main line goes LEFT, right branch is a trap
#define USE_WALL_ASSIST   1   // Side ultrasonics keep wheels off inner walls
#define USE_FRONT_STOP    1   // Stop if an obstacle is right in front during Phase 5
#define STOP_AT_LINE_END  1   // After hairpin E, line lost = end of line (Point F) -> handoff to T2

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

// Matched directly to your working 3-IR line follower!
const int PIN_IR_L = 17; // Left   IR (GP17, physical pin 22)
const int PIN_IR_R = 16; // Right  IR (GP16, physical pin 21)
const int PIN_IR_C = 22; // Centre IR (GP22, physical pin 29)

const int PIN_I2C_SDA = 20; // GY-521 SDA
const int PIN_I2C_SCL = 21; // GY-521 SCL
const int PIN_SERVO   = 27; // MG90S signal on GP27 (Pin 32, PWM Slice 5B)

const int IR_BLACK_STATE = HIGH;

#define INVERT_MOTOR_DIR 0
const int8_t ENC_R_SIGN = -1;
const int8_t ENC_L_SIGN =  1;

// ============================================================================
// 2. SINGLE-SOURCE-OF-TRUTH FIELD, CENTER & OUTWARD-DROP SERVO CONSTANTS
// ============================================================================
const float TICKS_PER_CM_L   = 60.5f;
const float TICKS_PER_CM_R   = 60.7f;
const float WHEELBASE_CM     = 19.0f;

// Baked-in verified sensor auto-calibrated values (guaranteed active on standalone battery!)
const float DEF_BOX_WIDTH_CM     = 119.53f; // Measured: L(50.49) + R(46.94) + 10.80 + 11.30
const float DEF_T1_X_CM          =  58.24f; // Measured exact T1 drop X (already includes cup shift)
const float DEF_T1_REV_FRONT_CM  =  71.24f; // Measured exact Front US reading on T1
const float T1_REV_TOL_CM        =   1.0f;  // Verify within +/- 1.0 cm

const float DEF_FRONT_OFFSET_CM  =  18.00f; // Anchor Front US offset
const float DEF_SIDE_OFFSET_L    =  10.80f; // Verified Left  US offset
const float DEF_SIDE_OFFSET_R    =  11.30f; // Verified Right US offset
const float DEF_DROP_SHIFT_X_CM  =   0.00f; // Already baked into DEF_T1_X_CM = 58.24 cm!

// In-Situ Auto-Calibration Target Stop Distance (inside 10..20 cm range, safe for +/-90 probe)
const float CAL_STOP_F_CM        =  16.5f;

// --- LIVE AUTO-CALIBRATED GEOMETRY STRUCT (FEEDS DIRECTLY INTO 'r' AND 't'!) ---
const uint32_t CAL_MAGIC   = 0x54314341UL; // "T1CA"
const uint16_t CAL_VERSION = 2;             // Bumped to 2 so baked values load cleanly on flash!

struct CalData {
  uint32_t magic;
  uint16_t version;
  uint16_t fromSensorCal;      // 1 = measured by 'a' command, 0 = factory defaults
  float boxWidthCm;
  float t1RevFrontCm;          // Exact Front US reading measured on T1!
  float frontOffsetCm;         // Anchor Front US offset
  float sideOffsetL;           // Sensor-calibrated Left US effective offset
  float sideOffsetR;           // Sensor-calibrated Right US effective offset
  float targetLrSumCm;         // Exact L + R measured facing Back Wall
  float targetXCm;             // Exact X of T1 drop line in sensor frame
  float targetYCm;             // t1RevFrontCm + frontOffsetCm
  float targetFrontLeftCm;     // Exact Front US reading facing Left Wall on T1 line
  float targetFrontRightCm;    // Exact Front US reading facing Right Wall on T1 line
  float targetBackLCm;         // Exact Left  US reading facing Back Wall on T1 line
  float targetBackRCm;         // Exact Right US reading facing Back Wall on T1 line
  float sideFrontSumCm;        // targetFrontLeftCm + targetFrontRightCm (for 360 scorer!)
  uint32_t checksum;
};

CalData cal;

uint32_t computeCalChecksum(const CalData &c) {
  const uint8_t *p = (const uint8_t *)&c;
  size_t len = offsetof(CalData, checksum);
  uint32_t h = 2166136261UL;
  for (size_t i = 0; i < len; i++) {
    h ^= p[i];
    h *= 16777619UL;
  }
  return h;
}

void loadDefaultCal(CalData &c) {
  c.magic              = CAL_MAGIC;
  c.version            = CAL_VERSION;
  c.fromSensorCal      = 1; // Baked sensor-calibrated values
  c.boxWidthCm         = DEF_BOX_WIDTH_CM;
  c.t1RevFrontCm       = DEF_T1_REV_FRONT_CM;
  c.frontOffsetCm      = DEF_FRONT_OFFSET_CM;
  c.sideOffsetL        = DEF_SIDE_OFFSET_L;
  c.sideOffsetR        = DEF_SIDE_OFFSET_R;
  c.targetLrSumCm      = DEF_BOX_WIDTH_CM - (DEF_SIDE_OFFSET_L + DEF_SIDE_OFFSET_R);
  c.targetXCm          = DEF_T1_X_CM + DEF_DROP_SHIFT_X_CM;
  c.targetYCm          = DEF_T1_REV_FRONT_CM + DEF_FRONT_OFFSET_CM;
  c.targetFrontLeftCm  = c.targetXCm - DEF_FRONT_OFFSET_CM;
  c.targetFrontRightCm = (DEF_BOX_WIDTH_CM - c.targetXCm) - DEF_FRONT_OFFSET_CM;
  c.targetBackLCm      = (DEF_BOX_WIDTH_CM - c.targetXCm) - DEF_SIDE_OFFSET_L;
  c.targetBackRCm      = c.targetXCm - DEF_SIDE_OFFSET_R;
  c.sideFrontSumCm     = c.targetFrontLeftCm + c.targetFrontRightCm;
  c.checksum           = computeCalChecksum(c);
}

// --- TRANSLATE-FIRST CLEARANCE & DUAL-SLIP / ACCEL BUMPER CONSTANTS ---
const float ALIGN_BACK_DF_CM      = 15.5f; // Deep inside box (Y~33.5 cm) so +/-15 deg beams ALWAYS hit Y<58 walls!
const float SPIN_GUARD_F_CM       = 12.5f;
const float SPIN_GUARD_SIDE_CM    = 9.5f;
const float GYRO_TOUCH_DPS        = 12.0f;
const unsigned long GYRO_TOUCH_MS = 90UL;
const int32_t IMPACT_JERK_MG      = 1400;  // Raised to avoid brushed-motor vibration false triggers
const float FRONT_SAFE_STOP_CM    = 14.0f;
const float BALL_CLEAR_FWD_CM     = 7.0f;

// --- FULL 180-DEG OUTWARD SERVO DROP + RELEASE TAP (GP27, 50.00 Hz HW PWM) ---
const int   SERVO_HOLD_DEG         = 45;
const int   SERVO_CRACK_DEG        = 75;
const int   SERVO_DRIBBLE_DEG      = 145;
const int   SERVO_DUMP_DEG         = 180;  // Full 180-deg flip so object ALWAYS falls cleanly!
const int   SERVO_TAP_BACK_DEG     = 162;  // Brief shake tap at bottom to release any sticky edge
const int   SERVO_FAST_MS_PER_DEG  = 5;
const int   SERVO_DRIBBLE_MS_DEG   = 18;
const int   SERVO_FINISH_MS_DEG    = 7;
const int   SERVO_RET_MS_PER_DEG   = 6;
const unsigned long T1_SETTLE_MS    = 450UL;
const unsigned long T1_DUMP_HOLD_MS = 450UL;

// Calibrated 6-Axis MPU6050 Constants
const float GYRO_SIGN         = 1.0f;    // CW = negative, CCW = positive
const float GYRO_SCALE        = 0.9907f; // 360.0 / 363.39
const float GYRO_SENS_500DPS  = 65.5f;
const float ACCEL_SENS_4G     = 8192.0f; // LSB per g at +/-4g
const float GYRO_DEADBAND_DPS = 0.08f;

// Phase 4 & Phase 6 Landmarks
const float HIGHWAY_SEEK_HDG_DEG = 4.0f;
const float IR_AXLE_PULL_MAX_CM  = 7.5f;
const float MAX_HIGHWAY_SEEK_CM  = 58.0f;
const float T2_PIVOT_RIGHT_DEG   = 40.0f;
const float T2_PARK_STOP_CM      = 5.5f;

// --- RAIL-LOCK T2 DOCKING (Phase 6) ---
// Map geometry relative to the LINE END F (line frame: u = along the last line
// segment, v = to its left). From the field map: last segment heading -22 deg,
// pocket axis -62.7 deg, T2 9.0 cm ahead and 20.6 cm right of F.
const float LINE_END_TO_AXIS_DEG = -40.7f; // pocket axis relative to the last line segment
const float T2_U_CM              =   9.0f;
const float T2_V_CM              = -20.6f;
const float AIM_BEFORE_T2_CM     =   8.0f; // first aim at this point on the centre line
const float IR_FWD_CM            =   7.5f; // IR row ahead of the axle
const float RAIL_SEEN_CM         =  30.0f; // side reading below this = a pocket rail
const float RAIL_LAT_GAIN        =   1.8f; // deg of steer per cm off centre
const float RAIL_LAT_MAX_DEG     =  14.0f;
const float RAIL_CENTER_TOL_CM   =   1.5f; // re-entry if parked further off centre
const float RAIL_HALF_GAP_CM     =  19.6f; // pocket centre line to the inner face of either rail
float gLineHdgAtEnd   = 0.0f;              // gyro heading of the last line segment
float gEndOvershootCm = 0.0f;              // driven past the line end before stopping
bool  gEndByLoss      = false;

// ============================================================================
// 3. GYRO-REGULATED 360 SCAN, VELOCITY-PROFILED TURNS & 200 Hz PID TUNING
// ============================================================================
const float RIGHT_FF         = 0.86f;

const float TARGET_SCAN_DPS  = 72.0f;
const int   SWEEP_SPIN_PWM   = 124;
const int   TURN_FAST_PWM    = 168;
const int   TURN_CREEP_PWM   = 112;
const float TURN_VEL_LEAD_S  = 0.022f;
const float TURN_TRIM_TOL    = 0.40f;
const float TURN_SYNC_KP     = 0.42f;

const int   DRIVE_CRUISE_PWM = 185;
const int   DRIVE_SLOW_PWM   = 140;
const int   REV_CRUISE_PWM   = 166;
const float DRIVE_MAX_CORR   = 84.0f;
const int   SPEED_SWEEP      = 150;

// 200 Hz (5 ms) Gyro PID + Pure Dead-Straight Reverse Lock (ZERO curving!)
const float GYRO_DRV_KP       = 7.80f;
const float GYRO_DRV_KI       = 6.20f;
const float GYRO_DRV_KD       = 0.48f;
const float LAT_KP_DEG_CM     = 1.80f;
const float LAT_MAX_DEG       = 2.00f; // Pure straight-line hold only (never curves!)
const float DIST_MATCH_TOL_CM = 1.60f; // Prioritize L & R within +/-1.6 cm of calibrated values!

// Fallback Encoder PID (only used if gyro fails over)
const float ENC_DRV_KP       = 1.40f;
const float ENC_DRV_KI       = 2.40f;
const float ENC_DRV_KD       = 0.08f;

const unsigned long STALL_MS = 100;
const int   STALL_STEP_PWM   = 12;
const int   STALL_MAX_PWM    = 65;

// ============================================================================
// 4. PHASE 5 WORKING 3-IR LINE FOLLOWER TUNING (EXACT INTEGRATED PARAMETERS)
// ============================================================================
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
const float LF_SEARCH_DEG        = 170.0f;

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

const unsigned long HARD_STOP_MS         = 87500UL;
const unsigned long HAND_CLEAR_DELAY_MS  = 800UL;

bool driveStraightCm(float cm, bool checkFrontWall = true, bool *hitLineOut = nullptr,
                     bool slowOnly = false, float *drivenCmOut = nullptr,
                     bool useExplicitHdg = false, float explicitHdgDeg = 0.0f,
                     float frontSafeCm = FRONT_SAFE_STOP_CM);

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

// ============================================================================
// 6. DEDICATED 50.00 Hz HW PWM SERVO DRIVER — 2-STAGE CRACK & DRIBBLE OUTWARD
// ============================================================================
bool servoInitDone   = false;
int  currentServoDeg = SERVO_HOLD_DEG;

void writeServoDeg(int angle) {
#if USE_SERVO
  angle = constrain(angle, 0, 180);
  if (!servoInitDone) {
    gpio_set_function(PIN_SERVO, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PIN_SERVO);
    float sysHz = (float)clock_get_hz(clk_sys);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, sysHz / 1000000.0f);
    pwm_config_set_wrap(&cfg, 19999);
    pwm_init(slice, &cfg, true);
    servoInitDone = true;
  }
  uint16_t pulseUs = (uint16_t)map(angle, 0, 180, 500, 2500); // Full mechanical 180-deg throw
  pwm_set_gpio_level(PIN_SERVO, pulseUs);
  currentServoDeg = angle;
#else
  (void)angle;
#endif
}

void moveServoSmooth(int targetAngle, int msPerDeg) {
#if USE_SERVO
  targetAngle = constrain(targetAngle, 0, 180);
  if (!servoInitDone) { writeServoDeg(targetAngle); return; }
  int step = (targetAngle >= currentServoDeg) ? 1 : -1;
  while (currentServoDeg != targetAngle) {
    writeServoDeg(currentServoDeg + step);
    delay(msPerDeg);
  }
#else
  (void)targetAngle; (void)msPerDeg;
#endif
}

void relaxServo() {
#if USE_SERVO
  if (servoInitDone) pwm_set_gpio_level(PIN_SERVO, 0);
#endif
}

void performOutwardBallDrop() {
#if USE_SERVO
  writeServoDeg(currentServoDeg);
  delay(T1_SETTLE_MS);

  int lo = min(currentServoDeg, SERVO_DUMP_DEG);
  int hi = max(currentServoDeg, SERVO_DUMP_DEG);

  if (SERVO_CRACK_DEG > lo && SERVO_CRACK_DEG < hi) {
    moveServoSmooth(SERVO_CRACK_DEG, SERVO_FAST_MS_PER_DEG);
  }
  if (SERVO_DRIBBLE_DEG > lo && SERVO_DRIBBLE_DEG < hi) {
    moveServoSmooth(SERVO_DRIBBLE_DEG, SERVO_DRIBBLE_MS_DEG);
  }
  // Full 180-deg dump + brief release tap so object ALWAYS drops cleanly!
  moveServoSmooth(SERVO_DUMP_DEG, SERVO_FINISH_MS_DEG);
  delay(T1_DUMP_HOLD_MS);
  moveServoSmooth(SERVO_TAP_BACK_DEG, 4);
  delay(80);
  moveServoSmooth(SERVO_DUMP_DEG, 4);
  delay(300);

  moveServoSmooth(SERVO_HOLD_DEG, SERVO_RET_MS_PER_DEG);
  delay(120);
  relaxServo();
#else
  delay(1000);
#endif
}

// ============================================================================
// 7. ENCODERS, AUTO-RECOVERING IMU ABSTRACTION & RACE TIMER
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

void printCalValues() {
  Serial.print(F("\n--- ACTIVE T1 CALIBRATION ["));
  Serial.print(cal.fromSensorCal ? F("SENSOR AUTO-CALIBRATED") : F("FACTORY DEFAULTS"));
  Serial.println(F("] ---"));
  Serial.print(F("  T1 Reverse Stop F   = ")); Serial.print(cal.t1RevFrontCm, 2);       Serial.println(F(" cm"));
  Serial.print(F("  Back-Wall Target L  = ")); Serial.print(cal.targetBackLCm, 2);      Serial.println(F(" cm"));
  Serial.print(F("  Back-Wall Target R  = ")); Serial.print(cal.targetBackRCm, 2);      Serial.println(F(" cm"));
  Serial.print(F("  Back-Wall L+R Sum   = ")); Serial.print(cal.targetLrSumCm, 2);      Serial.println(F(" cm"));
  Serial.print(F("  Face-Left  Target F = ")); Serial.print(cal.targetFrontLeftCm, 2);  Serial.println(F(" cm"));
  Serial.print(F("  Face-Right Target F = ")); Serial.print(cal.targetFrontRightCm, 2); Serial.println(F(" cm"));
  Serial.print(F("  Side Front Sum (L+R)= ")); Serial.print(cal.sideFrontSumCm, 2);     Serial.println(F(" cm"));
  Serial.print(F("  Cal Box Width / X   = ")); Serial.print(cal.boxWidthCm, 2);
  Serial.print(F(" / "));                      Serial.print(cal.targetXCm, 2);          Serial.println(F(" cm"));
  Serial.print(F("  Sensor Offsets F/L/R= ")); Serial.print(cal.frontOffsetCm, 1);
  Serial.print(F(" / "));                      Serial.print(cal.sideOffsetL, 2);
  Serial.print(F(" / "));                      Serial.print(cal.sideOffsetR, 2);        Serial.println(F(" cm"));
}

bool saveCalToFlash() {
  cal.magic    = CAL_MAGIC;
  cal.version  = CAL_VERSION;
  cal.checksum = computeCalChecksum(cal);
#if USE_EEPROM_CAL
  // Pause Core 1 safely (interrupts enabled) so RP2040 Flash commit never clashes with I2C
  core1PauseReq = true;
  unsigned long tP = millis();
  while (!core1Paused && (millis() - tP < 200UL)) delay(2);

  EEPROM.put(0, cal);
  bool ok = EEPROM.commit();

  core1PauseReq = false;
  delay(10);
  lastSeenHeartbeat = gyroHeartbeat;
  lastHeartbeatMs   = millis();
  return ok;
#else
  return true;
#endif
}

bool loadCalFromFlash() {
  loadDefaultCal(cal);
#if USE_EEPROM_CAL
  CalData tmp;
  EEPROM.get(0, tmp);
  if (tmp.magic == CAL_MAGIC && tmp.version == CAL_VERSION &&
      tmp.checksum == computeCalChecksum(tmp) &&
      tmp.t1RevFrontCm >= 35.0f && tmp.t1RevFrontCm <= 125.0f &&
      tmp.targetLrSumCm >= 70.0f && tmp.targetLrSumCm <= 125.0f) {
    cal = tmp;
    return true;
  }
#endif
  return false;
}

static inline float wrapAngle180(float deg) {
  while (deg >  180.0f) deg -= 360.0f;
  while (deg <= -180.0f) deg += 360.0f;
  return deg;
}

static inline float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

int findNearestScanIdx(const float *scanHdg, int n, float targetHdg, float *errOut = nullptr) {
  int best = 0;
  float bestDiff = 999.0f;
  for (int j = 0; j < n; j++) {
    float diff = fabsf(wrapAngle180(scanHdg[j] - targetHdg));
    if (diff < bestDiff) { bestDiff = diff; best = j; }
  }
  if (errOut) *errOut = bestDiff;
  return best;
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

float exactBackHdg = 0.0f;
unsigned long runStartMs = 0;
bool raceTimerActive = false;
int lastSignedL = 0, lastSignedR = 0;

float getFieldHeadingDeg() {
  return wrapAngle180(-90.0f + (getHeadingDeg() - exactBackHdg));
}

bool timeUp() {
  if (!raceTimerActive) return false;
  return (millis() - runStartMs >= HARD_STOP_MS);
}

// ============================================================================
// 8. MOTORS (250 Hz INVERTED PWM + LINEAR SIGNED DRIVE + ACTIVE BRAKE)
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

// ============================================================================
// 9. MOTOR-SWITCH AUTO-START DETECTOR
// ============================================================================
void waitForMotorSwitchOn() {
  Serial.println(F("[ARMED] Set robot still in box & flip 12V switch ON..."));
  bool ledState = false;
  bool biasFresh = false;
  unsigned long lastLedMs = millis();

  while (true) {
    long l0, r0, l1, r1;
    readEncoders(l0, r0);
    unsigned long waitStart = millis();
    while (millis() - waitStart < 350UL) {
      if (millis() - lastLedMs >= 600UL) {
        lastLedMs = millis();
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
      }
      delay(20);
    }
    readEncoders(l1, r1);
    if (labs(l1 - l0) + labs(r1 - r0) > 4) { biasFresh = false; continue; }

    if (!biasFresh) { resetHeadingAndEncoders(true); biasFresh = true; }

    long p1StartL, p1StartR, p1EndL, p1EndR;
    readEncoders(p1StartL, p1StartR);
    setLeftDir(-1); setRightDir(1);
    setMotorPWM(205, (int)roundf(205.0f * RIGHT_FF));
    delay(85);
    stopMotors();
    delay(45);
    readEncoders(p1EndL, p1EndR);

    long dL1 = p1EndL - p1StartL;
    long dR1 = p1EndR - p1StartR;

    if ((dR1 - dL1) >= 18 && dR1 >= -2 && dL1 <= 2) {
      long p2EndL, p2EndR;
      setLeftDir(1); setRightDir(-1);
      setMotorPWM(205, (int)roundf(205.0f * RIGHT_FF));
      delay(85);
      stopMotors();
      delay(45);
      readEncoders(p2EndL, p2EndR);

      long dL2 = p2EndL - p1EndL;
      long dR2 = p2EndR - p1EndR;

      if ((dL2 - dR2) >= 12 && dL2 >= -2 && dR2 <= 2) {
        digitalWrite(LED_BUILTIN, HIGH);
        runStartMs      = millis();
        raceTimerActive = true;
        Serial.println(F("[GO!] 12V motor switch ON -> starting race!"));
        delay(HAND_CLEAR_DELAY_MS);
        return;
      }
    }
  }
}

// ============================================================================
// 10. WALL-SAFE ULTRASONICS & DEBOUNCED 3-IR SENSORS (L=GP17, C=GP22, R=GP16)
// ============================================================================
float readUltrasonicSingle(int trigPin, int echoPin,
                           unsigned long toUs = 15000UL, bool clampClose = true) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
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

void readAllUltrasonics(float &dF, float &dL, float &dR, int samples = 3) {
  dL = readUltrasonicMedian(L_TRIG, L_ECHO, samples);
  delay(10);
  dF = readUltrasonicMedian(F_TRIG, F_ECHO, samples);
  delay(10);
  dR = readUltrasonicMedian(R_TRIG, R_ECHO, samples);
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

// ============================================================================
// 11. VELOCITY-PROFILED GYRO TURNS, TRANSLATE-FIRST PRE-CLEAR & HYBRID REVERSE
// ============================================================================
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

bool safeGentlePivot(float deltaDeg) {
  if (fabsf(deltaDeg) < 2.0f) return true;
  float startHdg  = getHeadingDeg();
  float targetHdg = startHdg + deltaDeg;
  int   dir       = (deltaDeg > 0.0f) ? 1 : -1;

  setLeftDir(-dir);
  setRightDir(dir);
  setMotorPWM(132, (int)roundf(132.0f * RIGHT_FF));

  unsigned long tStart = millis();
  unsigned long lowRateSince = 0;

  while (!timeUp() && (millis() - tStart < 1400UL)) {
    unsigned long now = millis();
    float rem = (targetHdg - getHeadingDeg()) * (float)dir;
    if (rem <= 2.5f) break;

    if (now - tStart > 120UL && isGyroHealthy()) {
      if (fabsf(getGyroRateDps()) < GYRO_TOUCH_DPS || getImuJerkPeakMg() > IMPACT_JERK_MG) {
        if (lowRateSince == 0) lowRateSince = now;
        if (now - lowRateSince >= 90UL || getImuJerkPeakMg() > IMPACT_JERK_MG) {
          stopMotors(30);
          return false;
        }
      } else {
        lowRateSince = 0;
      }
    }
    delay(6);
  }
  stopMotors(35);
  delay(35);
  return true;
}

bool pivotRelative(float deltaDeg, bool doTrim = true) {
  return turnToHeading(getHeadingDeg() + deltaDeg, doTrim);
}

bool turnToFieldHeading(float fieldDeg) {
  float delta = wrapAngle180(fieldDeg - getFieldHeadingDeg());
  return turnToHeading(getHeadingDeg() + delta, true);
}

bool sweepSearchLine(float maxDeg, int dir, float *sweptDegOut = nullptr) {
  float startHdg = getHeadingDeg();
  unsigned long startMs = millis();
  unsigned long maxMs   = (unsigned long)(fabsf(maxDeg) * 32.0f + 700.0f);
  bool found = false;

  while (!timeUp()) {
    if (anyIrBlack()) { found = true; break; }
    float progress = (dir < 0) ? (getHeadingDeg() - startHdg) : (startHdg - getHeadingDeg());
    unsigned long elapsed = millis() - startMs;
    if (progress >= maxDeg || elapsed >= maxMs) break;

    int swPwm = SPEED_SWEEP;
    if (elapsed > 280UL && progress < (float)elapsed * 0.025f) {
      swPwm = min(225, SPEED_SWEEP + 38);
    }
    if (dir < 0) setSignedMotors(-(swPwm - 10),  swPwm);
    else         setSignedMotors(  swPwm,       -(swPwm - 10));
    delay(3);
  }

  stopMotors();
  delay(45);
  if (sweptDegOut) *sweptDegOut = fabsf(getHeadingDeg() - startHdg);
  return found;
}

void performWallEscapeArc(bool awayRight, bool goForward, unsigned long durMs = 420UL) {
  int dir = goForward ? 1 : -1;
  setLeftDir(dir);
  setRightDir(dir);

  int fastPwm = 176;
  int slowPwm = 104;
  int lPwm, rPwm;

  if (goForward) {
    lPwm = awayRight ? fastPwm : slowPwm;
    rPwm = awayRight ? slowPwm : fastPwm;
  } else {
    lPwm = awayRight ? slowPwm : fastPwm;
    rPwm = awayRight ? fastPwm : slowPwm;
  }

  setMotorPWM(lPwm, (int)roundf((float)rPwm * RIGHT_FF));

  unsigned long t0 = millis();
  long lRef, rRef;
  readEncoders(lRef, rRef);
  unsigned long stallSince = 0;

  while (!timeUp() && (millis() - t0 < durMs)) {
    unsigned long now = millis();
    if (goForward) {
      float fNow = readUltrasonicSingle(F_TRIG, F_ECHO, 4500UL, true);
      if (fNow <= 13.0f) break;
    }
    if (now - t0 > 130UL) {
      if (getImuJerkPeakMg() > IMPACT_JERK_MG) break;
      long l, r; readEncoders(l, r);
      if (labs(l - lRef) + labs(r - rRef) < 3) {
        if (stallSince == 0) stallSince = now;
        if (now - stallSince > 120UL) break;
      } else {
        lRef = l; rRef = r; stallSince = 0;
      }
    }
    delay(20);
  }
  stopMotors(50);
}

enum PreClearStatus { PRE_OK = 0, PRE_FRONT_US = 1, PRE_BOX_EXIT = 2, PRE_BUMP = 3 };

PreClearStatus driveSafePreClearStraight(float cm, bool watchBoxExit, float *movedOut = nullptr) {
  if (movedOut) *movedOut = 0.0f;
  if (fabsf(cm) < 0.5f) return PRE_OK;

  int   dir      = (cm > 0.0f) ? 1 : -1;
  float targetCm = fabsf(cm);
  float holdHdg  = getHeadingDeg();

  long l0, r0;
  readEncoders(l0, r0);
  long stallL = l0, stallR = r0;

  setLeftDir(dir);
  setRightDir(dir);

  float avgCm = 0.0f, gyroInteg = 0.0f;
  unsigned long tStart = millis(), prevMs = tStart, lastUsMs = 0, stallCheckMs = tStart;
  int usSlot = 0;
  float curLUs = 45.0f, curRUs = 45.0f;
  int frontHits = 0;
  PreClearStatus status = PRE_OK;

  while (!timeUp()) {
    unsigned long now = millis();
    if (now - tStart > 2600UL) break;

    long l, r;
    readEncoders(l, r);
    float dL = (float)(l - l0) * (float)dir / TICKS_PER_CM_L;
    float dR = (float)(r - r0) * (float)dir / TICKS_PER_CM_R;
    avgCm = 0.5f * (dL + dR);
    if (avgCm >= targetCm - 0.8f) break;

    if (now - tStart > 130UL) {
      if (isGyroHealthy() && fabsf(getHeadingDeg() - holdHdg) > 6.5f) {
        status = PRE_BUMP;
        Serial.println(F("  [Pre-Clear] Gyro twist > 6.5 deg -> hidden oblique wall detected!"));
        break;
      }
      if (getImuJerkPeakMg() > IMPACT_JERK_MG) {
        status = PRE_BUMP;
        Serial.println(F("  [Pre-Clear] Accel impact spike -> hidden wall detected!"));
        break;
      }
      if (now - stallCheckMs >= 150UL) {
        long movedTicks = labs(l - stallL) + labs(r - stallR);
        stallL = l; stallR = r; stallCheckMs = now;
        if (movedTicks < 45) {
          status = PRE_BUMP;
          Serial.println(F("  [Pre-Clear] Encoder stall -> chassis blocked!"));
          break;
        }
      }
    }

    if (now - lastUsMs >= 26UL) {
      lastUsMs = now;
      if (dir > 0 && (usSlot == 0 || !watchBoxExit)) {
        float dF = readUltrasonicSingle(F_TRIG, F_ECHO, 5500UL, true);
        if (dF <= FRONT_SAFE_STOP_CM) {
          if (++frontHits >= 2 || dF <= 9.5f) { status = PRE_FRONT_US; break; }
        } else {
          frontHits = 0;
        }
      } else if (dir > 0 && watchBoxExit && usSlot == 1) {
        curLUs = readUltrasonicSingle(L_TRIG, L_ECHO, 7500UL, true);
      } else if (dir > 0 && watchBoxExit && usSlot == 2) {
        curRUs = readUltrasonicSingle(R_TRIG, R_ECHO, 7500UL, true);
        if ((curLUs + curRUs) > 130.0f) {
          status = PRE_BOX_EXIT;
          Serial.println(F("  [Pre-Clear] Reached start-box edge -> stopping cleanly inside box!"));
          break;
        }
      }
      usSlot = (usSlot + 1) % 3;
    }

    if (now - prevMs >= 5UL) {
      float dt = min((float)(now - prevMs) * 0.001f, 0.04f);
      prevMs = now;
      float corr = 0.0f;
      if (isGyroHealthy()) {
        float hdgErr = getHeadingDeg() - holdHdg;
        gyroInteg = constrain(gyroInteg + hdgErr * dt, -10.0f, 10.0f);
        corr = (5.5f * hdgErr + 3.5f * gyroInteg + 0.35f * getGyroRateDps()) * (float)dir;
      }
      corr = constrain(corr, -45.0f, 45.0f);
      int base = 134;
      setMotorPWM(constrain((int)roundf((float)base + corr), 92, 185),
                  constrain((int)roundf(((float)base - corr) * RIGHT_FF), 82, 180));
    }
  }

  stopMotors(50);
  if (movedOut) *movedOut = avgCm;
  return status;
}

void translateFirstPreClear() {
  for (int step = 0; step < 2 && !timeUp(); step++) {
    float dF, dL, dR;
    readAllUltrasonics(dF, dL, dR, 3);
    float sumLR = dL + dR;
    bool parallelToWalls = (sumLR >= 88.0f && sumLR <= 112.0f);

    Serial.print(F("  [Translate-First #")); Serial.print(step + 1);
    Serial.print(F("] F=")); Serial.print(dF, 1);
    Serial.print(F(" L="));  Serial.print(dL, 1);
    Serial.print(F(" R="));  Serial.println(dR, 1);

    if (step == 0) {
      if (dF < 24.0f) {
        float backCm = constrain(28.0f - dF, 12.0f, 22.0f);
        Serial.print(F("  [Pre-Clear] Wall ahead -> backing straight -")); Serial.print(backCm, 1); Serial.println(F(" cm"));
        float moved = 0.0f;
        PreClearStatus st = driveSafePreClearStraight(-backCm, false, &moved);
        if (st == PRE_BUMP && moved < 4.0f) {
          performWallEscapeArc((dR >= dL), false, 360UL);
        }
      } else if (parallelToWalls && (dL < 20.0f || dR < 20.0f)) {
        bool awayRight = (dL < dR);
        bool fwdOk     = (dF > 32.0f);
        Serial.println(fwdOk
          ? F("  [Pre-Clear] Tight side wall -> smooth FORWARD arc into center!")
          : F("  [Pre-Clear] Tight side & front -> smooth REVERSE arc into center!"));
        performWallEscapeArc(awayRight, fwdOk, 440UL);
      } else {
        Serial.println(F("  [Pre-Clear] Translating straight +16 cm into open center (0 corner swing)..."));
        float moved = 0.0f;
        PreClearStatus st = driveSafePreClearStraight(16.0f, parallelToWalls, &moved);
        if (st == PRE_BUMP || (st == PRE_FRONT_US && moved < 5.0f)) {
          Serial.println(F("  [Pre-Clear] Hidden wall ahead -> reversing straight -18 cm into center!"));
          driveSafePreClearStraight(-18.0f, false, nullptr);
        }
      }
    } else {
      if (dF < 18.0f) {
        driveSafePreClearStraight(-11.0f, false, nullptr);
      } else if (dL < 18.0f || dR < 18.0f) {
        performWallEscapeArc((dL < dR), (dF > 28.0f), 360UL);
      }
    }
    delay(40);
  }
}

void performGyroTouchPullout(int blockedSpinDir) {
  stopMotors(35);
  Serial.println(F("  [TACTILE REFLEX] Wall contact -> unwinding 20 deg!"));

  safeGentlePivot((float)(-blockedSpinDir) * 20.0f);
  delay(35);

  long l0, r0;
  readEncoders(l0, r0);
  setLeftDir(-1);
  setRightDir(-1);
  setMotorPWM(142, (int)roundf(142.0f * RIGHT_FF));

  unsigned long t0 = millis();
  float revMovedCm = 0.0f;
  while (!timeUp() && (millis() - t0 < 250UL)) {
    long l, r;
    readEncoders(l, r);
    float dL = (float)(l0 - l) / TICKS_PER_CM_L;
    float dR = (float)(r0 - r) / TICKS_PER_CM_R;
    revMovedCm = 0.5f * (dL + dR);
    delay(10);
  }
  stopMotors(40);

  if (revMovedCm >= 1.5f) {
    Serial.print(F("  [Pullout] Front contact (rev=")); Serial.print(revMovedCm, 1);
    Serial.println(F(" cm) -> backing 10 cm more into open center!"));
    driveSafePreClearStraight(-10.0f, false, nullptr);
  } else {
    Serial.println(F("  [Pullout] Rear contact -> driving +12 cm forward into open center!"));
    float fNow = readUltrasonicMedian(F_TRIG, F_ECHO, 3);
    if (fNow > FRONT_SAFE_STOP_CM + 2.0f) {
      driveSafePreClearStraight(12.0f, false, nullptr);
    } else {
      float lNow = readUltrasonicMedian(L_TRIG, L_ECHO, 3);
      float rNow = readUltrasonicMedian(R_TRIG, R_ECHO, 3);
      performWallEscapeArc((rNow >= lNow), true, 360UL);
    }
  }
  delay(45);
}

bool driveStraightCm(float cm, bool checkFrontWall, bool *hitLineOut,
                     bool slowOnly, float *drivenCmOut,
                     bool useExplicitHdg, float explicitHdgDeg,
                     float frontSafeCm) {
  if (hitLineOut)  *hitLineOut  = false;
  if (drivenCmOut) *drivenCmOut = 0.0f;
  if (fabsf(cm) < 0.4f) return true;

  const float D2R = (float)M_PI / 180.0f;
  int dir = (cm > 0.0f) ? 1 : -1;
  float targetCm = fabsf(cm);
  float stopLead = (targetCm < 5.0f) ? 0.3f : 1.1f;

  if (checkFrontWall && dir > 0 && readUltrasonicMedian(F_TRIG, F_ECHO, 3) <= frontSafeCm) {
    return false;
  }

  long l0, r0;
  readEncoders(l0, r0);
  long stallL = l0, stallR = r0;
  int boost = 0;

  float holdHdg = useExplicitHdg ? explicitHdgDeg : getHeadingDeg();

  setLeftDir(dir);
  setRightDir(dir);

  float prevEncErr = 0.0f, encInteg = 0.0f, gyroInteg = 0.0f;
  float avgCm = 0.0f, prevAvgCm = 0.0f, latErrCm = 0.0f;
  unsigned long prevMs = millis(), tStart = prevMs, lastMoveMs = prevMs, lastFrontMs = 0;
  int wallHits = 0;

  while (!timeUp()) {
    unsigned long now = millis();
    if (dir < 0 && (now - tStart > 4500UL)) break;
    if (now - tStart > 10000UL) { stopMotors(90); if (drivenCmOut) *drivenCmOut = avgCm; return false; }

    if (hitLineOut && anyIrBlack()) { *hitLineOut = true; break; }

    long l, r;
    readEncoders(l, r);
    float dL_cm = (float)(l - l0) * (float)dir / TICKS_PER_CM_L;
    float dR_cm = (float)(r - r0) * (float)dir / TICKS_PER_CM_R;

    float maxD = max(dL_cm, dR_cm), minD = min(dL_cm, dR_cm);
    avgCm = (maxD > 5.0f && minD < 0.25f * maxD) ? maxD : (0.5f * (dL_cm + dR_cm));
    float remCm = targetCm - avgCm;

    if (avgCm < -4.0f) { stopMotors(90); if (drivenCmOut) *drivenCmOut = avgCm; return false; }
    if (remCm <= stopLead) break;

    if (checkFrontWall && dir > 0 && (now - lastFrontMs >= 30UL)) {
      lastFrontMs = now;
      float dF = readUltrasonicSingle(F_TRIG, F_ECHO, 5000UL, true);
      if (dF <= frontSafeCm) {
        if (++wallHits >= 2 || dF <= 8.5f) {
          stopMotors(100);
          Serial.print(F("  [FRONT SAFETY STOP] F=")); Serial.println(dF, 1);
          if (drivenCmOut) *drivenCmOut = avgCm;
          return false;
        }
      } else {
        wallHits = 0;
      }
    }

    if (now - prevMs >= 5UL) {
      float dt = min((float)(now - prevMs) * 0.001f, 0.04f);
      prevMs = now;

      if (labs(l - stallL) + labs(r - stallR) >= 4) {
        stallL = l; stallR = r; lastMoveMs = now; boost = 0;
      } else if (now - lastMoveMs > STALL_MS) {
        boost = min(boost + STALL_STEP_PWM, STALL_MAX_PWM);
        lastMoveMs = now;
      }

      float corr = 0.0f;
      if (isGyroHealthy()) {
        float curHdg = getHeadingDeg();
        float ds = avgCm - prevAvgCm;
        prevAvgCm = avgCm;
        if (ds > 0.0f) latErrCm += ds * sinf((curHdg - holdHdg) * D2R);
        float aimHdg = holdHdg - constrain(latErrCm * LAT_KP_DEG_CM, -LAT_MAX_DEG, LAT_MAX_DEG);
        float hdgErr = curHdg - aimHdg;
        gyroInteg = constrain(gyroInteg + hdgErr * dt, -14.0f, 14.0f);
        corr = (GYRO_DRV_KP * hdgErr + GYRO_DRV_KI * gyroInteg + GYRO_DRV_KD * getGyroRateDps()) * (float)dir;
      } else {
        float errTicks = (float)((r - r0) - (l - l0)) * (float)dir;
        encInteg = constrain(encInteg + errTicks * dt, -28.0f, 28.0f);
        float deriv = (errTicks - prevEncErr) / dt;
        prevEncErr = errTicks;
        corr = ENC_DRV_KP * errTicks + ENC_DRV_KI * encInteg + ENC_DRV_KD * deriv;
      }
      corr = constrain(corr, -DRIVE_MAX_CORR, DRIVE_MAX_CORR);

      unsigned long el = now - tStart;
      int base = DRIVE_CRUISE_PWM;
      if (dir < 0 || slowOnly) {
        base = DRIVE_SLOW_PWM;
        if (remCm < 5.0f) base = 114;
      } else {
        if (el < 250UL) base = 148 + (int)((long)(DRIVE_CRUISE_PWM - 148) * (long)el / 250L);
        if (remCm < 5.0f)       base = 116;
        else if (remCm < 15.0f) base = min(base, DRIVE_SLOW_PWM);
      }
      base = min(base + boost, 235);

      int lPWM = (int)roundf((float)base + corr);
      int rPWM = (int)roundf(((float)base - corr) * RIGHT_FF);
      setMotorPWM(constrain(lPWM, 90, 245), constrain(rPWM, 80, 240));
    }
  }

  stopMotors(100);
  if (drivenCmOut) *drivenCmOut = avgCm;
  return true;
}

bool driveToExactCenterFrontDistCm(float targetFrontCm, float holdHdg) {
  float curF = readUltrasonicMedian(F_TRIG, F_ECHO, 5);

  if (curF < targetFrontCm - 1.0f) {
    float backCm = min(targetFrontCm - curF, 48.0f);
    driveStraightCm(-backCm, false, nullptr, true, nullptr, true, holdHdg);
  } else if (curF > targetFrontCm + 1.0f) {
    float maxEncCm = min((curF - targetFrontCm) + 14.0f, 80.0f);
    const float D2R = (float)M_PI / 180.0f;

    long l0, r0;
    readEncoders(l0, r0);
    long stallL = l0, stallR = r0;
    int boost = 0;

    setLeftDir(1);
    setRightDir(1);

    float prevEncErr = 0.0f, encInteg = 0.0f, gyroInteg = 0.0f;
    float prevAvgCm = 0.0f, latErrCm = 0.0f;
    float fBuf[3] = { curF, curF, curF };
    int   fIdx = 0;
    float fMed = curF;
    int   targetHits = 0, wallHits = 0;
    unsigned long prevMs = millis(), tStart = prevMs, lastMoveMs = prevMs, lastFrontMs = 0;

    while (!timeUp()) {
      unsigned long now = millis();
      if (now - tStart > 8500UL) break;

      long l, r;
      readEncoders(l, r);
      float dL_cm = (float)(l - l0) / TICKS_PER_CM_L;
      float dR_cm = (float)(r - r0) / TICKS_PER_CM_R;
      float maxD = max(dL_cm, dR_cm), minD = min(dL_cm, dR_cm);
      float avgCm = (maxD > 5.0f && minD < 0.25f * maxD) ? maxD : (0.5f * (dL_cm + dR_cm));

      if (avgCm >= maxEncCm) break;

      if (now - lastFrontMs >= 30UL) {
        lastFrontMs = now;
        float rawF = readUltrasonicSingle(F_TRIG, F_ECHO, 9500UL, true);
        if (rawF < 130.0f) {
          fBuf[fIdx] = rawF;
          fIdx = (fIdx + 1) % 3;
          fMed = median3(fBuf[0], fBuf[1], fBuf[2]);
        }
        if (rawF <= FRONT_SAFE_STOP_CM) {
          if (++wallHits >= 2) break;
        } else {
          wallHits = 0;
        }
        if (fMed <= targetFrontCm + 0.8f) {
          if (++targetHits >= 2) break;
        } else {
          targetHits = 0;
        }
      }

      if (now - prevMs >= 5UL) {
        float dt = min((float)(now - prevMs) * 0.001f, 0.04f);
        prevMs = now;

        if (labs(l - stallL) + labs(r - stallR) >= 4) {
          stallL = l; stallR = r; lastMoveMs = now; boost = 0;
        } else if (now - lastMoveMs > STALL_MS) {
          boost = min(boost + STALL_STEP_PWM, STALL_MAX_PWM);
          lastMoveMs = now;
        }

        float corr = 0.0f;
        if (isGyroHealthy()) {
          float curHdg = getHeadingDeg();
          float ds = avgCm - prevAvgCm;
          prevAvgCm = avgCm;
          if (ds > 0.0f) latErrCm += ds * sinf((curHdg - holdHdg) * D2R);
          float aimHdg = holdHdg - constrain(latErrCm * LAT_KP_DEG_CM, -LAT_MAX_DEG, LAT_MAX_DEG);
          float hdgErr = curHdg - aimHdg;
          gyroInteg = constrain(gyroInteg + hdgErr * dt, -14.0f, 14.0f);
          corr = GYRO_DRV_KP * hdgErr + GYRO_DRV_KI * gyroInteg + GYRO_DRV_KD * getGyroRateDps();
        } else {
          float errTicks = (float)((r - r0) - (l - l0));
          encInteg = constrain(encInteg + errTicks * dt, -28.0f, 28.0f);
          float deriv = (errTicks - prevEncErr) / dt;
          prevEncErr = errTicks;
          corr = ENC_DRV_KP * errTicks + ENC_DRV_KI * encInteg + ENC_DRV_KD * deriv;
        }
        corr = constrain(corr, -DRIVE_MAX_CORR, DRIVE_MAX_CORR);

        float remCm = max(0.0f, fMed - targetFrontCm);
        int base = (remCm < 5.5f) ? 115 : DRIVE_SLOW_PWM;
        base = min(base + boost, 225);

        int lPWM = (int)roundf((float)base + corr);
        int rPWM = (int)roundf(((float)base - corr) * RIGHT_FF);
        setMotorPWM(constrain(lPWM, 90, 245), constrain(rPWM, 80, 240));
      }
    }
    stopMotors(100);
  }

  delay(55);
  float fChk = readUltrasonicMedian(F_TRIG, F_ECHO, 5);
  float err  = fChk - targetFrontCm;
  if (fChk < 120.0f && fabsf(err) > 1.5f && fabsf(err) <= 18.0f) {
    driveStraightCm(err, (err > 0.0f), nullptr, true, nullptr, true, holdHdg, FRONT_SAFE_STOP_CM);
  }
  return true;
}

// ============================================================================
// PURE DEAD-STRAIGHT 0.00-DEG GYRO-LOCKED REVERSE TO T1 (ZERO ANGLE ROTATIONS!)
//   - Holds exact holdHdg (0.00 deg) from start to finish at 200 Hz (5 ms)
//   - Constantly checks gyro heading & keeps rotation locked near 0.00 deg
//   - Stops when Front US reaches cal.t1RevFrontCm +/- 1.0 cm (with up to 2
//     gentle straight trims at 0.00 deg)
// ============================================================================
bool drivePureStraightReverseToT1(float targetFrontCm, float startFrontCm, float holdHdg,
                                  float *revDistOut = nullptr, float *finalFrontOut = nullptr) {
  if (revDistOut)    *revDistOut    = 0.0f;
  if (finalFrontOut) *finalFrontOut = startFrontCm;

  if (startFrontCm >= targetFrontCm - 0.8f || startFrontCm > 64.0f || startFrontCm < 5.0f) {
    Serial.print(F("  [TABLE SAFETY ABORT] Invalid start F=")); Serial.println(startFrontCm, 1);
    return false;
  }

  const float D2R = (float)M_PI / 180.0f;
  float estRevCm  = targetFrontCm - startFrontCm;
  float minEncCm  = max(0.0f, estRevCm - 6.0f);
  float maxEncCm  = min(estRevCm + 7.0f, 68.0f);

  long l0, r0;
  readEncoders(l0, r0);
  long stallL = l0, stallR = r0;
  int  boost  = 0;

  setLeftDir(-1);
  setRightDir(-1);

  float prevEncErr = 0.0f, encInteg = 0.0f, gyroInteg = 0.0f;
  float avgCm = 0.0f, prevAvgCm = 0.0f, latErrCm = 0.0f;
  float maxGyroDev = 0.0f;

  float fBuf[3] = { startFrontCm, startFrontCm, startFrontCm };
  int   fIdx = 0;
  float fMed = startFrontCm;
  int   targetHits = 0;

  unsigned long prevMs = millis(), tStart = prevMs, lastMoveMs = prevMs, lastUsMs = 0;

  while (!timeUp()) {
    unsigned long now = millis();
    if (now - tStart > 9500UL) break;

    float curHdg = getHeadingDeg();
    float devDeg = fabsf(curHdg - holdHdg);
    if (devDeg > maxGyroDev) maxGyroDev = devDeg;

    if (isGyroHealthy() && devDeg > 12.0f && (now - tStart > 250UL)) {
      stopMotors(110);
      Serial.print(F("  [TABLE SAFETY STOP] Heading drifted ")); Serial.print(curHdg - holdHdg, 2);
      Serial.println(F(" deg during straight reverse!"));
      if (revDistOut)    *revDistOut    = avgCm;
      if (finalFrontOut) *finalFrontOut = fMed;
      return false;
    }

    long l, r;
    readEncoders(l, r);
    float dL_cm = (float)(l0 - l) / TICKS_PER_CM_L;
    float dR_cm = (float)(r0 - r) / TICKS_PER_CM_R;
    float maxD  = max(dL_cm, dR_cm), minD = min(dL_cm, dR_cm);
    avgCm       = (maxD > 5.0f && minD < 0.25f * maxD) ? maxD : (0.5f * (dL_cm + dR_cm));

    if (avgCm >= maxEncCm) break;

    // Fast Front-US polling every 28 ms (no L/R steering interference during reverse!)
    if (now - lastUsMs >= 28UL) {
      lastUsMs = now;
      float rawF = readUltrasonicSingle(F_TRIG, F_ECHO, 11000UL, true);
      if (rawF < 120.0f || avgCm >= estRevCm - 3.0f) {
        fBuf[fIdx] = rawF;
        fIdx = (fIdx + 1) % 3;
        fMed = median3(fBuf[0], fBuf[1], fBuf[2]);
      }
      if (avgCm >= minEncCm && fMed >= targetFrontCm - 0.7f) {
        if (++targetHits >= 2) break;
      } else {
        targetHits = 0;
      }
    }

    // 200 Hz (5 ms) Pure 0.00-deg Gyro Heading Lock
    if (now - prevMs >= 5UL) {
      float dt = min((float)(now - prevMs) * 0.001f, 0.04f);
      prevMs = now;

      if (labs(l - stallL) + labs(r - stallR) >= 4) {
        stallL = l; stallR = r; lastMoveMs = now; boost = 0;
      } else if (now - lastMoveMs > STALL_MS) {
        boost = min(boost + STALL_STEP_PWM, STALL_MAX_PWM);
        lastMoveMs = now;
      }

      float corr = 0.0f;
      if (isGyroHealthy()) {
        float ds = avgCm - prevAvgCm;
        prevAvgCm = avgCm;
        if (ds > 0.0f) latErrCm += ds * sinf((curHdg - holdHdg) * D2R);
        // Tiny cross-track damper (max +/-1.8 deg) strictly to cancel tiny wheel drift -> stays dead-straight!
        float aimHdg = holdHdg - constrain(latErrCm * LAT_KP_DEG_CM, -LAT_MAX_DEG, LAT_MAX_DEG);
        float hdgErr = curHdg - aimHdg;
        gyroInteg = constrain(gyroInteg + hdgErr * dt, -14.0f, 14.0f);
        corr = -(GYRO_DRV_KP * hdgErr + GYRO_DRV_KI * gyroInteg + GYRO_DRV_KD * getGyroRateDps());
      } else {
        float errTicks = (float)((l - l0) - (r - r0));
        encInteg = constrain(encInteg + errTicks * dt, -28.0f, 28.0f);
        float deriv = (errTicks - prevEncErr) / dt;
        prevEncErr = errTicks;
        corr = ENC_DRV_KP * errTicks + ENC_DRV_KI * encInteg + ENC_DRV_KD * deriv;
      }
      corr = constrain(corr, -DRIVE_MAX_CORR, DRIVE_MAX_CORR);

      float remByUs  = max(0.0f, targetFrontCm - fMed);
      float remByEnc = max(0.0f, estRevCm - avgCm);
      float remCm    = min(remByUs, remByEnc + 2.5f);

      unsigned long el = now - tStart;
      int base = REV_CRUISE_PWM;
      if (el < 280UL)         base = 128 + (int)((long)(REV_CRUISE_PWM - 128) * (long)el / 280L);
      if (remCm < 5.5f)       base = 112;
      else if (remCm < 14.0f) base = min(base, DRIVE_SLOW_PWM);
      base = min(base + boost, 225);

      int lPWM = (int)roundf((float)base + corr);
      int rPWM = (int)roundf(((float)base - corr) * RIGHT_FF);
      setMotorPWM(constrain(lPWM, 90, 245), constrain(rPWM, 80, 240));
    }
  }

  stopMotors(110);
  delay(80);

  // Verify exact cal.t1RevFrontCm +/- 1.0 cm with dead-straight 0.00 deg trims
  float fCheck = readUltrasonicMedian(F_TRIG, F_ECHO, 5);
  for (int trim = 0; trim < 2 && !timeUp(); trim++) {
    float err = fCheck - targetFrontCm;
    if (fCheck >= 115.0f || fabsf(err) <= T1_REV_TOL_CM || fabsf(err) > 8.0f) break;
    Serial.print(F("  [T1 Straight Trim ")); Serial.print(trim + 1);
    Serial.print(F("] F="));                 Serial.print(fCheck, 1);
    Serial.print(F(" -> move "));            Serial.print(err, 1); Serial.println(F(" cm"));
    driveStraightCm(err, (err > 0.0f), nullptr, true, nullptr, true, holdHdg);
    delay(60);
    fCheck = readUltrasonicMedian(F_TRIG, F_ECHO, 5);
  }

  Serial.print(F("  [Straight Reverse Audit] Final F=")); Serial.print(fCheck, 2);
  Serial.print(F(" (target "));                           Serial.print(targetFrontCm, 2);
  Serial.print(F(") | GyroHdg="));                        Serial.print(getHeadingDeg() - holdHdg, 2);
  Serial.print(F(" deg | MaxGyroDev="));                  Serial.print(maxGyroDev, 2);
  Serial.println(F(" deg"));

  if (revDistOut)    *revDistOut    = avgCm;
  if (finalFrontOut) *finalFrontOut = fCheck;
  return true;
}

// ============================================================================
// 11B. GYRO-REGULATED SMOOTH 360 SCAN + 5-POINT WIDE +/-18 SYMMETRY ALIGN
// ============================================================================
int chooseSpecularSafeSpinDir(float lInit, float rInit) {
  bool lTimeout = (lInit >= 155.0f);
  bool rTimeout = (rInit >= 155.0f);
  bool lRealMid = (!lTimeout && lInit >= 32.0f && lInit <= 105.0f);
  bool rRealMid = (!rTimeout && rInit >= 32.0f && rInit <= 105.0f);

  if (lTimeout && rRealMid) return -1;
  if (rTimeout && lRealMid) return  1;
  return (lInit < rInit) ? -1 : 1;
}

bool ultimateSmooth360FindBack(float &backTargetOut, float &mostOpenHdgOut) {
  const int MAX_SAMPLES = 110;
  float sHdg[MAX_SAMPLES], sF[MAX_SAMPLES], sL[MAX_SAMPLES], sR[MAX_SAMPLES], sSum[MAX_SAMPLES];
  int   n = 0;

  translateFirstPreClear();

  float fInit, lInit, rInit;
  readAllUltrasonics(fInit, lInit, rInit, 3);
  int dir = chooseSpecularSafeSpinDir(lInit, rInit);

  float lRing[3] = { lInit, lInit, lInit };
  float fRing[3] = { fInit, fInit, fInit };
  float rRing[3] = { rInit, rInit, rInit };
  int   ringIdx  = 0, rawCnt = 0;

  float startHdg      = getHeadingDeg();
  float lastStoredHdg = startHdg - 99.0f;
  float encHdg0       = getEncoderHeadingDeg();

  long l0, r0;
  readEncoders(l0, r0);
  int pulloutsDone = 0;

  float rateInteg   = 0.0f;
  float slipRefGyro = startHdg;
  float slipRefEnc  = encHdg0;
  unsigned long slipRefMs      = millis();
  unsigned long tStart         = slipRefMs;
  unsigned long spinStartMs    = slipRefMs;
  unsigned long prevLoopMs     = slipRefMs;
  unsigned long gyroTouchSince = 0;

  setLeftDir(-dir);
  setRightDir(dir);

  Serial.print(F("\n[Phase 1A] Gyro-Regulated 360 Scan (72 dps, dir="));
  Serial.print(dir > 0 ? F("CCW") : F("CW")); Serial.println(F(")..."));

  while (!timeUp()) {
    unsigned long now = millis();
    if (now - tStart > 16000UL) break;
    float loopDtS = constrain((float)(now - prevLoopMs) * 0.001f, 0.015f, 0.065f);
    prevLoopMs = now;

    long l, r;
    readEncoders(l, r);

    float curHdg    = getHeadingDeg();
    float curEncHdg = getEncoderHeadingDeg();
    float curDps    = getGyroRateDps();

    if (isGyroHealthy()) {
      float encMoved  = fabsf(curEncHdg - encHdg0);
      float gyroMoved = fabsf(curHdg - startHdg);
      if (encMoved > 35.0f && gyroMoved < 4.0f) {
        triggerGyroFailover(F("gyro stuck vs encoders"));
      }
    }

    float base = (float)SWEEP_SPIN_PWM;
    if (isGyroHealthy() && (now - spinStartMs > 100UL)) {
      float rateErr = TARGET_SCAN_DPS - fabsf(curDps);
      rateInteg = constrain(rateInteg + rateErr * loopDtS, -22.0f, 22.0f);
      base = constrain((float)SWEEP_SPIN_PWM + 0.60f * rateErr + 0.35f * rateInteg, 104.0f, 154.0f);
    }

    long dL = labs(l - l0), dR = labs(r - r0);
    float syncCorr = constrain(TURN_SYNC_KP * (float)(dR - dL), -28.0f, 28.0f);
    setMotorPWM(constrain((int)roundf(base + syncCorr), 95, 175),
                constrain((int)roundf((base - syncCorr) * RIGHT_FF), 85, 170));

    if (now - spinStartMs > 100UL) {
      bool hardStall = isGyroHealthy() ? (fabsf(curDps) < GYRO_TOUCH_DPS)
                                       : (labs(l - l0) + labs(r - r0) < 4);
      bool wallSlip  = false;
      if (now - slipRefMs >= 80UL) {
        float dG = fabsf(curHdg - slipRefGyro);
        float dE = fabsf(curEncHdg - slipRefEnc);
        if (isGyroHealthy() && dE > 4.0f && dG < 0.55f * dE) {
          wallSlip = true;
        }
        slipRefGyro = curHdg;
        slipRefEnc  = curEncHdg;
        slipRefMs   = now;
      }
      bool impactHit = (now - spinStartMs > 150UL) && (getImuJerkPeakMg() > IMPACT_JERK_MG);

      if (hardStall) {
        if (gyroTouchSince == 0) gyroTouchSince = now;
      } else {
        gyroTouchSince = 0;
      }

      if (wallSlip || impactHit || (gyroTouchSince > 0 && (now - gyroTouchSince >= GYRO_TOUCH_MS))) {
        gyroTouchSince = 0;
        if (pulloutsDone < 3) {
          pulloutsDone++;
          Serial.print(F("  [360 Bumper Triggered: "));
          if (impactHit)      Serial.print(F("ACCEL-SHOCK"));
          else if (wallSlip)  Serial.print(F("DUAL-SLIP"));
          else                Serial.print(F("GYRO-STALL"));
          Serial.println(F("]"));
          performGyroTouchPullout(dir);

          readAllUltrasonics(fInit, lInit, rInit, 3);
          dir = chooseSpecularSafeSpinDir(lInit, rInit);
          for (int k = 0; k < 3; k++) { lRing[k] = lInit; fRing[k] = fInit; rRing[k] = rInit; }
          n              = 0;
          rawCnt         = 0;
          rateInteg      = 0.0f;
          startHdg       = getHeadingDeg();
          lastStoredHdg  = startHdg - 99.0f;
          encHdg0        = getEncoderHeadingDeg();
          slipRefGyro    = startHdg;
          slipRefEnc     = encHdg0;
          setLeftDir(-dir);
          setRightDir(dir);
          readEncoders(l0, r0);
          spinStartMs    = millis();
          slipRefMs      = spinStartMs;
          prevLoopMs     = spinStartMs;
          continue;
        } else {
          stopMotors(40);
          break;
        }
      }
    }

    float pL   = readUltrasonicSingle(L_TRIG, L_ECHO, 9500UL, true);
    delay(4);
    float pF   = readUltrasonicSingle(F_TRIG, F_ECHO, 9500UL, true);
    float hMid = getHeadingDeg();
    delay(4);
    float pR   = readUltrasonicSingle(R_TRIG, R_ECHO, 9500UL, true);
    delay(4);

    bool hitF = (pF <= SPIN_GUARD_F_CM);
    bool hitL = (pL <= SPIN_GUARD_SIDE_CM);
    bool hitR = (pR <= SPIN_GUARD_SIDE_CM);
    if (pulloutsDone < 3 && (hitF || hitL || hitR)) {
      delay(10);
      bool confirmed = false;
      if (hitF)      confirmed = (readUltrasonicSingle(F_TRIG, F_ECHO, 5000UL, true) <= SPIN_GUARD_F_CM);
      else if (hitL) confirmed = (readUltrasonicSingle(L_TRIG, L_ECHO, 5000UL, true) <= SPIN_GUARD_SIDE_CM);
      else if (hitR) confirmed = (readUltrasonicSingle(R_TRIG, R_ECHO, 5000UL, true) <= SPIN_GUARD_SIDE_CM);

      if (confirmed) {
        pulloutsDone++;
        stopMotors(35);
        if (hitF)      driveSafePreClearStraight(-9.0f, false, nullptr);
        else if (hitL) performWallEscapeArc(true, (pF > 22.0f), 360UL);
        else           performWallEscapeArc(false, (pF > 22.0f), 360UL);

        readAllUltrasonics(fInit, lInit, rInit, 3);
        for (int k = 0; k < 3; k++) { lRing[k] = lInit; fRing[k] = fInit; rRing[k] = rInit; }
        n              = 0;
        rawCnt         = 0;
        rateInteg      = 0.0f;
        startHdg       = getHeadingDeg();
        lastStoredHdg  = startHdg - 99.0f;
        encHdg0        = getEncoderHeadingDeg();
        slipRefGyro    = startHdg;
        slipRefEnc     = encHdg0;
        setLeftDir(-dir);
        setRightDir(dir);
        readEncoders(l0, r0);
        spinStartMs    = millis();
        slipRefMs      = spinStartMs;
        prevLoopMs     = spinStartMs;
        gyroTouchSince = 0;
        continue;
      }
    }

    lRing[ringIdx] = pL;
    fRing[ringIdx] = pF;
    rRing[ringIdx] = pR;
    ringIdx = (ringIdx + 1) % 3;
    rawCnt++;

    float spun = (hMid - startHdg) * (float)dir;

    if (rawCnt >= 3 && n < MAX_SAMPLES && fabsf(hMid - lastStoredHdg) >= 4.5f) {
      float mL = median3(lRing[0], lRing[1], lRing[2]);
      float mF = median3(fRing[0], fRing[1], fRing[2]);
      float mR = median3(rRing[0], rRing[1], rRing[2]);
      float lagDeg  = isGyroHealthy() ? constrain(curDps * loopDtS, -5.5f, 5.5f) : ((float)dir * 2.5f);
      sHdg[n]       = hMid - lagDeg;
      sF[n]         = mF;
      sL[n]         = mL;
      sR[n]         = mR;
      sSum[n]       = mL + mR;
      lastStoredHdg = hMid;
      n++;
    }

    if (spun >= 362.0f || n >= MAX_SAMPLES) break;
  }

  stopMotors(40);
  delay(45);
  if (n < 12) return false;

  int   bestBackIdx = 0, bestLeftIdx = 0, bestRightIdx = 0, mostOpenIdx = 0, bestTier = 99;
  float bestScore   = 9999.0f, maxOpenF = -1.0f;

  for (int i = 0; i < n; i++) {
    if (sF[i] > maxOpenF) { maxOpenF = sF[i]; mostOpenIdx = i; }

    float errL = 0, errR = 0, errO = 0;
    int leftIdx  = findNearestScanIdx(sHdg, n, sHdg[i] - 90.0f, &errL);
    int rightIdx = findNearestScanIdx(sHdg, n, sHdg[i] + 90.0f, &errR);
    int openIdx  = findNearestScanIdx(sHdg, n, sHdg[i] + 180.0f, &errO);

    bool openStrict  = (errO < 18.0f && sF[openIdx] > max(104.0f, cal.sideFrontSumCm + 18.0f));
    bool openLoose   = (errO < 18.0f && sF[openIdx] > 88.0f);
    bool leftWallOk  = (errL < 18.0f && sF[leftIdx]  < 108.0f && sL[leftIdx]  < 70.0f && sR[leftIdx]  > 105.0f);
    bool rightWallOk = (errR < 18.0f && sF[rightIdx] < 108.0f && sR[rightIdx] < 70.0f && sL[rightIdx] > 105.0f);

    float sumErr   = fabsf(sSum[i] - cal.targetLrSumCm);
    float sideSum  = sF[leftIdx] + sF[rightIdx];
    float wErrSide = fabsf(sideSum - cal.sideFrontSumCm);

    float yFromLeft  = sL[leftIdx]  + cal.sideOffsetL;
    float yFromRight = sR[rightIdx] + cal.sideOffsetR;
    float yBackRef   = (sF[i] < 72.0f) ? (sF[i] + cal.frontOffsetCm)
                                       : (0.5f * (yFromLeft + yFromRight));
    float yAgreeErr  = fabsf(yFromLeft - yBackRef) + fabsf(yFromRight - yBackRef);

    int sideHits = (leftWallOk ? 1 : 0) + (rightWallOk ? 1 : 0);
    int tier = 4;
    if      (leftWallOk && rightWallOk && openStrict && sumErr < 16.0f && wErrSide < 18.0f)      tier = 0;
    else if (sideHits >= 1 && openStrict && sF[i] < 68.0f && sumErr < 18.0f && wErrSide < 22.0f) tier = 1;
    else if (openStrict && sF[i] < 68.0f && sumErr < 20.0f && wErrSide < 26.0f)                  tier = 2;
    else if (openLoose  && sF[i] < 68.0f && sumErr < 22.0f)                                      tier = 3;

    float score = 1.3f * sumErr + 1.0f * wErrSide + 0.5f * yAgreeErr;
    if (tier < bestTier || (tier == bestTier && score < bestScore)) {
      bestTier     = tier;
      bestScore    = score;
      bestBackIdx  = i;
      bestLeftIdx  = leftIdx;
      bestRightIdx = rightIdx;
    }
  }

  float refHdg = sHdg[bestBackIdx];
  float fx[12], fy[12];
  int   fn = 0;
  for (int i = 0; i < n && fn < 12; i++) {
    float dh = wrapAngle180(sHdg[i] - refHdg);
    if (fabsf(dh) <= 18.0f && sSum[i] >= cal.targetLrSumCm - 6.0f && sSum[i] <= cal.targetLrSumCm + 22.0f) {
      fx[fn] = dh;
      fy[fn] = sSum[i];
      fn++;
    }
  }
  float vertOff = 0.0f;
  if (fitQuadraticMin(fx, fy, fn, vertOff)) {
    vertOff = constrain(vertOff, -14.0f, 14.0f);
  }

  float curHdg    = getHeadingDeg();
  backTargetOut   = curHdg + wrapAngle180((refHdg + vertOff) - curHdg);
  mostOpenHdgOut  = curHdg + wrapAngle180(sHdg[mostOpenIdx] - curHdg);

  Serial.print(F("  [360 Lock] pts=")); Serial.print(n);
  Serial.print(F(" | Tier=")); Serial.print(bestTier);
  Serial.print(F(" | F="));    Serial.print(sF[bestBackIdx], 1);
  Serial.print(F(" | L+R="));  Serial.print(sSum[bestBackIdx], 1);
  Serial.print(F(" | SideF=")); Serial.print(sF[bestLeftIdx] + sF[bestRightIdx], 1); Serial.print(F(" (cal ")); Serial.print(cal.sideFrontSumCm, 1); Serial.println(F(")"));
  return true;
}

// Symmetric Wide Angle Lock: sweeps [-halfSpan .. +halfSpan] using Gyro,
// fits ONLY symmetric valid pairs so a missed wall on one edge can NEVER skew the angle!
bool wide5PtSymmetryAlign(float centerGuessHdg, float halfSpanDeg, int numPts, float &alignedHdgOut) {
  if (numPts != 3 && numPts != 5) numPts = 5;
  float fineHdg[5], fineF[5], fineSum[5];
  bool  validS[5];
  float stepDeg = (2.0f * halfSpanDeg) / (float)(numPts - 1);
  int   minSIdx = numPts / 2, minFIdx = numPts / 2;
  float minSVal = 1e9f, minFVal = 1e9f;

  for (int k = 0; k < numPts; k++) {
    if (timeUp()) return false;
    if (!turnToHeading(centerGuessHdg - halfSpanDeg + stepDeg * (float)k, false)) return false;
    delay(40);
    fineHdg[k] = getHeadingDeg();

    float f, l, r;
    readAllUltrasonics(f, l, r, 5);
    fineF[k]   = f;
    fineSum[k] = l + r;
    if (f < minFVal) { minFVal = f; minFIdx = k; }
    if (fineSum[k] >= cal.targetLrSumCm - 5.0f && fineSum[k] <= cal.targetLrSumCm + 22.0f) {
      validS[k] = true;
      if (fineSum[k] < minSVal) { minSVal = fineSum[k]; minSIdx = k; }
    } else {
      validS[k] = false;
    }
  }

  // Enforce symmetric pairs around center index so one-sided outliers NEVER bias the parabola!
  int mid = numPts / 2;
  bool useMask[5] = { false, false, false, false, false };
  int  symValid = 0;
  if (validS[mid]) { useMask[mid] = true; symValid++; }
  for (int d = 1; d <= mid; d++) {
    if (validS[mid - d] && validS[mid + d]) {
      useMask[mid - d] = true;
      useMask[mid + d] = true;
      symValid += 2;
    }
  }

  float fitX[5], fitY[5];
  int   fitN   = 0;
  bool  useSum = (symValid >= 3);
  int   cK     = useSum ? minSIdx : minFIdx;
  float refHdg = fineHdg[cK];

  for (int k = 0; k < numPts; k++) {
    if (useSum && !useMask[k]) continue;
    fitX[fitN] = fineHdg[k] - refHdg;
    fitY[fitN] = useSum ? fineSum[k] : fineF[k];
    fitN++;
  }

  float xVert = 0.0f;
  alignedHdgOut = fitQuadraticMin(fitX, fitY, fitN, xVert)
                  ? (refHdg + constrain(xVert, -halfSpanDeg, halfSpanDeg))
                  : refHdg;

  Serial.print(F("  [Gyro+US Angle Lock] L+R=("));
  for (int k = 0; k < numPts; k++) { Serial.print(fineSum[k], 1); if (k < numPts - 1) Serial.print(','); }
  Serial.print(F(") symPts=")); Serial.print(symValid);
  Serial.print(F(" -> shift ")); Serial.print(alignedHdgOut - centerGuessHdg, 2); Serial.println(F(" deg"));

  return turnToHeading(alignedHdgOut, true, TURN_TRIM_TOL);
}

bool moveLaterallyToCenter(float dxToCenter, float backHdg) {
  if (dxToCenter < 0.0f) {
    float leftWallHdg = backHdg - 90.0f;
    Serial.print(F("\n[Side Shift] Facing LEFT WALL (-90 CW), target F="));
    Serial.print(cal.targetFrontLeftCm, 1); Serial.println(F(" cm -> then +90 CCW to Back!"));
    if (!turnToHeading(leftWallHdg, true, TURN_TRIM_TOL)) return false;
    driveToExactCenterFrontDistCm(cal.targetFrontLeftCm, leftWallHdg);
    return turnToHeading(backHdg, true, TURN_TRIM_TOL);
  } else {
    float rightWallHdg = backHdg + 90.0f;
    Serial.print(F("\n[Side Shift] Facing RIGHT WALL (+90 CCW), target F="));
    Serial.print(cal.targetFrontRightCm, 1); Serial.println(F(" cm -> then -90 CW to Back!"));
    if (!turnToHeading(rightWallHdg, true, TURN_TRIM_TOL)) return false;
    driveToExactCenterFrontDistCm(cal.targetFrontRightCm, rightWallHdg);
    return turnToHeading(backHdg, true, TURN_TRIM_TOL);
  }
}

// ============================================================================
// 11C. IN-SITU T1 SELF-CALIBRATION (COMMAND 'a')
//   1. Place drop box directly on T1 with robot facing dead-straight at Back Wall
//   2. Calibrates 6-axis gyro to 0.00 deg & measures exact F_T1 (11-pt median)
//   3. Drives DEAD-STRAIGHT (0.00 deg, zero turn, 200 Hz gyro + cross-track lock)
//      until Front US hits 16.5 cm (middle of 10..20 cm range)
//   4. Measures exact L_back, F_stop, R_back (11-pt medians)
//   5. Safely pivots -90 -> +90 -> 0 in place at F = 16.5 cm to directly measure
//      F_left, L_toBack, F_right, R_toBack with the robot's OWN ultrasonics!
//   6. Solves exact sensor-consistent offsets & targets -> auto-feeds RAM + Flash!
// ============================================================================
bool driveCalStraightToFrontCm(float stopFrontCm, float startFrontCm,
                               float *drivenCmOut, float *maxHdgDevOut, float *finalLatOut) {
  if (drivenCmOut)  *drivenCmOut  = 0.0f;
  if (maxHdgDevOut) *maxHdgDevOut = 0.0f;
  if (finalLatOut)  *finalLatOut  = 0.0f;

  const float D2R = (float)M_PI / 180.0f;
  float maxEncCm  = min(max(12.0f, (startFrontCm - stopFrontCm) + 12.0f), 80.0f);

  long l0, r0;
  readEncoders(l0, r0);
  long stallL = l0, stallR = r0;
  int  boost  = 0;

  float holdHdg = 0.0f; // Freshly zeroed gyro at T1!
  setLeftDir(1);
  setRightDir(1);

  float prevEncErr = 0.0f, encInteg = 0.0f, gyroInteg = 0.0f;
  float avgCm = 0.0f, prevAvgCm = 0.0f, latErrCm = 0.0f, maxHdgDev = 0.0f;
  float fBuf[3] = { startFrontCm, startFrontCm, startFrontCm };
  int   fIdx = 0;
  float fMed = startFrontCm;
  int   stopHits = 0;

  unsigned long tStart = millis(), prevMs = tStart, lastMoveMs = tStart, lastUsMs = 0;

  while (true) {
    unsigned long now = millis();
    if (now - tStart > 9500UL) { stopMotors(110); Serial.println(F("  [CAL ABORT] Straight-drive timeout!")); return false; }

    // Abort immediately if manually interrupted on Serial
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      stopMotors(110);
      Serial.println(F("  [CAL ABORT] Cancelled by keypress."));
      return false;
    }

    float curHdg = getHeadingDeg();
    float absHdg = fabsf(curHdg - holdHdg);
    if (absHdg > maxHdgDev) maxHdgDev = absHdg;

    // Strict zero-rotation guard: if bumped or wheels slip > 8 deg, abort rather than saving bad cal!
    if (isGyroHealthy() && absHdg > 8.0f) {
      stopMotors(110);
      Serial.print(F("  [CAL ABORT] Heading drifted ")); Serial.print(curHdg, 2); Serial.println(F(" deg!"));
      return false;
    }

    long l, r;
    readEncoders(l, r);
    float dL_cm = (float)(l - l0) / TICKS_PER_CM_L;
    float dR_cm = (float)(r - r0) / TICKS_PER_CM_R;
    avgCm = 0.5f * (dL_cm + dR_cm);
    if (avgCm >= maxEncCm) break;

    if (now - lastUsMs >= 28UL) {
      lastUsMs = now;
      float rawF = readUltrasonicSingle(F_TRIG, F_ECHO, 11000UL, true);
      if (rawF < 130.0f) {
        fBuf[fIdx] = rawF;
        fIdx = (fIdx + 1) % 3;
        fMed = median3(fBuf[0], fBuf[1], fBuf[2]);
      }
      if (rawF <= 10.5f) {
        stopMotors(110);
        break; // Hard 10.5 cm safety floor
      }
      if (fMed <= stopFrontCm + 0.5f) {
        if (++stopHits >= 2) break;
      } else {
        stopHits = 0;
      }
    }

    if (now - prevMs >= 5UL) {
      float dt = min((float)(now - prevMs) * 0.001f, 0.04f);
      prevMs = now;

      if (labs(l - stallL) + labs(r - stallR) >= 4) {
        stallL = l; stallR = r; lastMoveMs = now; boost = 0;
      } else if (now - lastMoveMs > STALL_MS) {
        boost = min(boost + STALL_STEP_PWM, 48);
        lastMoveMs = now;
      }

      float corr = 0.0f;
      if (isGyroHealthy()) {
        float ds = avgCm - prevAvgCm;
        prevAvgCm = avgCm;
        if (ds > 0.0f) latErrCm += ds * sinf((curHdg - holdHdg) * D2R);
        // Ultra-tight cross-track + 0.00 deg heading lock
        float aimHdg = holdHdg - constrain(latErrCm * 2.6f, -3.5f, 3.5f);
        float hdgErr = curHdg - aimHdg;
        gyroInteg = constrain(gyroInteg + hdgErr * dt, -14.0f, 14.0f);
        corr = 7.5f * hdgErr + 6.0f * gyroInteg + 0.45f * getGyroRateDps();
      } else {
        float errTicks = (float)((r - r0) - (l - l0));
        encInteg = constrain(encInteg + errTicks * dt, -28.0f, 28.0f);
        float deriv = (errTicks - prevEncErr) / dt;
        prevEncErr = errTicks;
        corr = ENC_DRV_KP * errTicks + ENC_DRV_KI * encInteg + ENC_DRV_KD * deriv;
      }
      corr = constrain(corr, -DRIVE_MAX_CORR, DRIVE_MAX_CORR);

      // Gentle zero-slip speed profile: soft ramp 120->142 PWM, crawl at 112 PWM below 24 cm
      unsigned long el = now - tStart;
      int base = 142;
      if (el < 300UL)               base = 120 + (int)((22L * (long)el) / 300L);
      if (fMed < stopFrontCm + 7.5f) base = min(base, 112);
      base = min(base + boost, 195);

      int lPWM = (int)roundf((float)base + corr);
      int rPWM = (int)roundf(((float)base - corr) * RIGHT_FF);
      setMotorPWM(constrain(lPWM, 88, 225), constrain(rPWM, 78, 220));
    }
  }

  stopMotors(110);
  delay(150);

  if (drivenCmOut)  *drivenCmOut  = avgCm;
  if (maxHdgDevOut) *maxHdgDevOut = maxHdgDev;
  if (finalLatOut)  *finalLatOut  = latErrCm;
  return true;
}

bool runAutoCalibrateFromT1() {
  raceTimerActive = false;
  Serial.println(F("\n================================================================"));
  Serial.println(F("[AUTO-CALIBRATE FROM T1]"));
  Serial.println(F("  Ensure drop box is on T1 & robot faces DEAD-STRAIGHT at Back Wall!"));
  Serial.println(F("  Hands off! Calibrating 6-axis gyro to 0.00 deg in 2 s..."));
  for (int i = 2; i > 0; i--) { Serial.print(F("  ")); Serial.print(i); Serial.println(F("...")); delay(1000); }
  while (Serial.available()) Serial.read();

  // 1. Fresh stillness gyro calibration -> locks start heading as exact 0.00 deg!
  resetHeadingAndEncoders(true);
  delay(80);

  // 2. Measure exact Front US reading at T1 (11-sample median, 18000 us = 308 cm range)
  float fAtT1 = readUltrasonicMedian(F_TRIG, F_ECHO, 11, 18000UL);
  Serial.print(F("  [Step 1] At T1: Front US = ")); Serial.print(fAtT1, 2); Serial.println(F(" cm"));

  if (fAtT1 < 40.0f || fAtT1 > 115.0f) {
    Serial.println(F("  [CAL ABORT] Front US at T1 out of valid 40..115 cm range! Nothing overwritten."));
    return false;
  }

  // 3. Drive DEAD-STRAIGHT (0.00 deg!) toward Back Wall and stop at F = 16.5 cm (inside 10..20 cm)
  Serial.print(F("  [Step 2] Driving dead-straight (0.00 deg) to F = "));
  Serial.print(CAL_STOP_F_CM, 1); Serial.println(F(" cm..."));

  float drivenCm = 0.0f, maxHdgDev = 0.0f, finalLat = 0.0f;
  if (!driveCalStraightToFrontCm(CAL_STOP_F_CM, fAtT1, &drivenCm, &maxHdgDev, &finalLat)) {
    return false;
  }

  float finalHdg = getHeadingDeg();
  Serial.print(F("  [Straight Audit] Driven=")); Serial.print(drivenCm, 1);
  Serial.print(F(" cm | MaxHdgDev="));          Serial.print(maxHdgDev, 2);
  Serial.print(F(" deg | FinalHdg="));           Serial.print(finalHdg, 2);
  Serial.print(F(" deg | LatDrift="));           Serial.print(finalLat, 2); Serial.println(F(" cm"));

  if (isGyroHealthy() && fabsf(finalHdg) > 3.0f) {
    Serial.println(F("  [CAL ABORT] Final heading error > 3.0 deg! Re-run 'a' cleanly."));
    return false;
  }

  // 4. Measure BOTH side ultrasonics + Front US dead-still at F ~ 16.5 cm (11-sample medians)
  float lBack = readUltrasonicMedian(L_TRIG, L_ECHO, 11, 15000UL);
  delay(15);
  float fStop = readUltrasonicMedian(F_TRIG, F_ECHO, 11, 15000UL);
  delay(15);
  float rBack = readUltrasonicMedian(R_TRIG, R_ECHO, 11, 15000UL);
  float sumLR = lBack + rBack;

  Serial.print(F("  [Step 3] At Stop (0.00 deg): F=")); Serial.print(fStop, 2);
  Serial.print(F(" | L="));                             Serial.print(lBack, 2);
  Serial.print(F(" | R="));                             Serial.print(rBack, 2);
  Serial.print(F(" | L+R="));                           Serial.println(sumLR, 2);
  Serial.print(F("  [Cross-Check] F_T1="));             Serial.print(fAtT1, 2);
  Serial.print(F(" vs (F_stop + Enc)="));               Serial.println(fStop + drivenCm, 2);

  if (fStop < 9.0f || fStop > 26.0f || lBack < 10.0f || lBack > 98.0f ||
      rBack < 10.0f || rBack > 98.0f || sumLR < 74.0f || sumLR > 118.0f) {
    Serial.println(F("  [CAL ABORT] Side/Front stop readings outside valid box range! Nothing overwritten."));
    return false;
  }

  // Build new calibration candidate
  CalData nc;
  loadDefaultCal(nc);
  nc.fromSensorCal = 1;
  nc.frontOffsetCm = DEF_FRONT_OFFSET_CM;
  nc.t1RevFrontCm  = fAtT1;
  nc.targetBackLCm = lBack;
  nc.targetBackRCm = rBack;
  nc.targetLrSumCm = sumLR;

  // 5. Direct Front-US & Side-Offset Probe (-90 -> +90 -> 0 in place at F ~ 16.5 cm)
  //    Eliminates ALL tape-measure & cross-sensor offset errors!
  bool probeOk = false;
  if (fStop >= 13.5f && lBack >= 16.0f && rBack >= 16.0f) {
    Serial.println(F("  [Step 4] Probing Left (-90) & Right (+90) walls directly with Front US..."));

    if (turnToHeading(-90.0f, true, 0.35f)) {
      delay(80);
      float fLeft   = readUltrasonicMedian(F_TRIG, F_ECHO, 9, 15000UL);
      delay(15);
      float lToBack = readUltrasonicMedian(L_TRIG, L_ECHO, 9, 15000UL);

      if (turnToHeading(90.0f, true, 0.35f)) {
        delay(80);
        float fRight  = readUltrasonicMedian(F_TRIG, F_ECHO, 9, 15000UL);
        delay(15);
        float rToBack = readUltrasonicMedian(R_TRIG, R_ECHO, 9, 15000UL);

        // Return neatly to 0.00 deg facing Back Wall
        turnToHeading(0.0f, true, 0.40f);

        Serial.print(F("    At -90 (Left  Wall): F_left="));  Serial.print(fLeft, 2);
        Serial.print(F(" | L_toBack="));                      Serial.println(lToBack, 2);
        Serial.print(F("    At +90 (Right Wall): F_right=")); Serial.print(fRight, 2);
        Serial.print(F(" | R_toBack="));                      Serial.println(rToBack, 2);

        if (fLeft >= 12.0f && fLeft <= 98.0f && fRight >= 12.0f && fRight <= 98.0f &&
            lToBack >= 5.0f && lToBack <= 50.0f && rToBack >= 5.0f && rToBack <= 50.0f) {
          // Two independent measurements of (SIDE_OFFSET_L - FRONT_OFFSET) & (SIDE_OFFSET_R - FRONT_OFFSET)!
          float offL_back = nc.frontOffsetCm + (fStop  - lToBack);
          float offL_side = nc.frontOffsetCm + (fRight - lBack);
          float offR_back = nc.frontOffsetCm + (fStop  - rToBack);
          float offR_side = nc.frontOffsetCm + (fLeft  - rBack);

          float offL = 0.5f * (offL_back + offL_side);
          float offR = 0.5f * (offR_back + offR_side);

          float wFromF  = fLeft + fRight + 2.0f * nc.frontOffsetCm;
          float wFromLR = sumLR + offL + offR;

          Serial.print(F("    Offset Check: offL=(")); Serial.print(offL_back, 1); Serial.print(','); Serial.print(offL_side, 1);
          Serial.print(F(") offR=("));                 Serial.print(offR_back, 1); Serial.print(','); Serial.print(offR_side, 1);
          Serial.print(F(") | W_F="));                 Serial.print(wFromF, 1);
          Serial.print(F(" vs W_LR="));                Serial.println(wFromLR, 1);

          if (offL >= 4.0f && offL <= 20.0f && offR >= 4.0f && offR <= 20.0f && fabsf(wFromF - wFromLR) <= 10.0f) {
            nc.sideOffsetL        = offL;
            nc.sideOffsetR        = offR;
            nc.boxWidthCm         = sumLR + offL + offR;
            nc.targetXCm          = rBack + offR;
            nc.targetYCm          = fAtT1 + nc.frontOffsetCm;
            nc.targetFrontLeftCm  = fLeft;
            nc.targetFrontRightCm = fRight;
            nc.sideFrontSumCm     = fLeft + fRight;
            probeOk               = true;
          }
        }
      }
    }
  }

  if (!probeOk) {
    // Clean algebraic fallback using exact measured lBack, rBack & fAtT1 with default offsets
    Serial.println(F("  [Note] Using direct Back-Wall L/R calibration with default offsets."));
    nc.sideOffsetL        = DEF_SIDE_OFFSET_L;
    nc.sideOffsetR        = DEF_SIDE_OFFSET_R;
    nc.boxWidthCm         = sumLR + nc.sideOffsetL + nc.sideOffsetR;
    nc.targetXCm          = rBack + nc.sideOffsetR;
    nc.targetYCm          = fAtT1 + nc.frontOffsetCm;
    nc.targetFrontLeftCm  = nc.targetXCm - nc.frontOffsetCm;
    nc.targetFrontRightCm = (nc.boxWidthCm - nc.targetXCm) - nc.frontOffsetCm;
    nc.sideFrontSumCm     = nc.targetFrontLeftCm + nc.targetFrontRightCm;
  }

  // 6. Auto-feed into LIVE active RAM AND commit to RP2040 Flash!
  cal = nc;
  bool saved = saveCalToFlash();

  printCalValues();
  Serial.print(F("  [FLASH STATUS] "));
  Serial.println(saved ? F("SAVED to RP2040 Flash (survives reboot & battery mode!)")
                       : F("Loaded in RAM (Flash commit skipped/failed)"));
  Serial.println(F("================================================================"));
  Serial.println(F(">>> READY! Move robot to ANY start pose and type 'r' (Full Race) or 't' (Task 1)! <<<"));
  return true;
}

// ============================================================================
// 12. PHASES 1-3: TRANSLATE-FIRST + GYRO-REGULATED 360 -> 5-PT LOCK (W/ RETRY)
//                 -> SIDE-SHIFT / TRACKED REVERSE TO 71.8 -> OUTWARD DROP
// ============================================================================
// Helper: keeps robot deep inside the 14.5..18.5 cm sweet spot so +/-15 deg beams NEVER miss Y=58 walls!
void settleBackWallSweetSpot(float backHdg) {
  float dFNow = readUltrasonicMedian(F_TRIG, F_ECHO, 5);
  if (dFNow < 13.5f) {
    driveStraightCm(-constrain(ALIGN_BACK_DF_CM - dFNow, 3.0f, 10.0f),
                    false, nullptr, true, nullptr, true, backHdg);
  } else if (dFNow > 19.0f && dFNow < 62.0f) {
    driveStraightCm(constrain(dFNow - ALIGN_BACK_DF_CM, 3.0f, 38.0f),
                    true, nullptr, true, nullptr, true, backHdg, 12.0f);
  }
}

bool runPhases1To3_T1() {
  writeServoDeg(SERVO_HOLD_DEG);
  resetHeadingAndEncoders(true);

  float dF = ALIGN_BACK_DF_CM, dL = cal.targetBackLCm, dR = cal.targetBackRCm, sumLR = cal.targetLrSumCm;
  float robotX = cal.targetXCm;
  bool  localized = false;

  // --- PHASE 1A: 360 SCAN + PULL DEEP TO F = 15.5 cm + INITIAL SQUARING ---
  for (int attempt = 1; attempt <= 2 && !timeUp(); attempt++) {
    Serial.print(F("\n=== [Phase 1] Localization Attempt "));
    Serial.print(attempt); Serial.println(F("/2 ==="));

    float mostOpenHdg = 0.0f;
    if (!ultimateSmooth360FindBack(exactBackHdg, mostOpenHdg)) {
      if (attempt == 1) { driveSafePreClearStraight(10.0f, false, nullptr); continue; }
      return false;
    }

    if (!turnToHeading(exactBackHdg, true, TURN_TRIM_TOL)) return false;
    delay(45);

    // Pull deep into 15.5 cm sweet spot BEFORE sweeping so +/-15 deg hits Y <= 51 cm (< 58 cm wall)!
    settleBackWallSweetSpot(exactBackHdg);

    if (!wide5PtSymmetryAlign(exactBackHdg, 15.0f, 5, exactBackHdg)) return false;
    delay(55);

    readAllUltrasonics(dF, dL, dR, 5);
    sumLR  = dL + dR;
    robotX = 0.5f * ((dR + cal.sideOffsetR) + (cal.boxWidthCm - (dL + cal.sideOffsetL)));

    Serial.print(F("[Phase 1A Back Wall] X=")); Serial.print(robotX, 1);
    Serial.print(F(" (target "));               Serial.print(cal.targetXCm, 1);
    Serial.print(F(") | F="));                  Serial.print(dF, 1);
    Serial.print(F(" L="));                     Serial.print(dL, 1); Serial.print(F(" (cal ")); Serial.print(cal.targetBackLCm, 1); Serial.print(')');
    Serial.print(F(" R="));                     Serial.print(dR, 1); Serial.print(F(" (cal ")); Serial.print(cal.targetBackRCm, 1); Serial.print(')');
    Serial.print(F(" | Sum="));                 Serial.println(sumLR, 1);

    if (dF >= 5.0f && dF <= 64.0f && sumLR >= cal.targetLrSumCm - 12.0f && sumLR <= cal.targetLrSumCm + 12.0f && dL <= 98.0f && dR <= 98.0f) {
      localized = true;
      break;
    }

    if (attempt == 1) {
      Serial.println(F("  [Auto-Retry] Box sanity failed -> stepping 12 cm toward open space!"));
      turnToHeading(mostOpenHdg, false);
      driveSafePreClearStraight(12.0f, false, nullptr);
    }
  }

  if (!localized) {
    stopMotors(50);
    Serial.println(F("!!! [TABLE SAFETY ABORT] Could not verify 3-wall start box!"));
    return false;
  }

  // --- PHASE 1B: DISTANCE FIRST (MATCH CALIBRATED L & R!) -> THEN UNCONDITIONAL ANGLE PERFECTION ---
  for (int pass = 0; pass < 2 && !timeUp(); pass++) {
    float errL  = dL - cal.targetBackLCm;
    float errR  = dR - cal.targetBackRCm;
    float dxRem = cal.targetXCm - robotX;
    float sumErr = sumLR - cal.targetLrSumCm;

    Serial.print(F("[Pre-Reverse Check #")); Serial.print(pass + 1);
    Serial.print(F("] L="));     Serial.print(dL, 1);    Serial.print(F(" (err ")); Serial.print(errL, 1); Serial.print(')');
    Serial.print(F(" | R="));    Serial.print(dR, 1);    Serial.print(F(" (err ")); Serial.print(errR, 1); Serial.print(')');
    Serial.print(F(" | Sum="));  Serial.print(sumLR, 1); Serial.print(F(" (cal ")); Serial.print(cal.targetLrSumCm, 1); Serial.print(')');
    Serial.print(F(" | dx="));   Serial.println(dxRem, 1);

    // 1. Prioritize exact L & R distance FIRST: if off by > 1.6 cm, shift laterally to exact calibrated target!
    bool movedSide = false;
    if (fabsf(dxRem) > DIST_MATCH_TOL_CM || fabsf(errL) > 2.2f || fabsf(errR) > 2.2f) {
      if (fabsf(dxRem) > 0.8f && fabsf(dxRem) < 65.0f) {
        Serial.print(F("  [Distance-First Shift] dx=")); Serial.print(dxRem, 1);
        Serial.println(F(" cm -> moving to exact calibrated L/R line!"));
        if (!moveLaterallyToCenter(dxRem, exactBackHdg)) return false;
        delay(55);
        movedSide = true;
      }
    }

    // Keep nose deep at F ~ 15.5 cm so +/-15 deg angle sweep ALWAYS hits both side walls cleanly!
    settleBackWallSweetSpot(exactBackHdg);

    // 2. UNCONDITIONAL Angle Perfection using Gyro + (L+R) symmetry!
    //    NEVER skipped after a side shift -> kills the +/-7 deg tilt blind spot completely!
    if (movedSide || pass == 0 || fabsf(sumErr) > 0.8f) {
      float span = movedSide ? 15.0f : 12.0f;
      Serial.print(F("  [Angle Perfection] Unconditional +/-")); Serial.print(span, 0);
      Serial.println(F(" deg Gyro + (L+R) symmetry lock..."));
      if (!wide5PtSymmetryAlign(exactBackHdg, span, 5, exactBackHdg)) return false;
      delay(60);
    }

    // 3. Re-read 7-sample medians and verify BOTH exact L/R distances AND L+R sum!
    dL = readUltrasonicMedian(L_TRIG, L_ECHO, 7);
    delay(12);
    dF = readUltrasonicMedian(F_TRIG, F_ECHO, 7);
    delay(12);
    dR = readUltrasonicMedian(R_TRIG, R_ECHO, 7);
    sumLR  = dL + dR;
    robotX = 0.5f * ((dR + cal.sideOffsetR) + (cal.boxWidthCm - (dL + cal.sideOffsetL)));

    errL   = dL - cal.targetBackLCm;
    errR   = dR - cal.targetBackRCm;
    dxRem  = cal.targetXCm - robotX;
    sumErr = sumLR - cal.targetLrSumCm;

    // One tiny +/-10 deg gyro polish if L+R sum is still > 0.8 cm above calibrated perpendicular sum
    if (sumErr > 0.8f && sumErr < 18.0f) {
      Serial.print(F("  [Micro Angle Polish] Sum off by +")); Serial.print(sumErr, 1);
      Serial.println(F(" cm -> tight +/-10 deg 3-pt lock!"));
      wide5PtSymmetryAlign(exactBackHdg, 10.0f, 3, exactBackHdg);
      delay(55);
      dL = readUltrasonicMedian(L_TRIG, L_ECHO, 7); delay(12);
      dF = readUltrasonicMedian(F_TRIG, F_ECHO, 7); delay(12);
      dR = readUltrasonicMedian(R_TRIG, R_ECHO, 7);
      sumLR  = dL + dR;
      robotX = 0.5f * ((dR + cal.sideOffsetR) + (cal.boxWidthCm - (dL + cal.sideOffsetL)));
      errL   = dL - cal.targetBackLCm;
      errR   = dR - cal.targetBackRCm;
      dxRem  = cal.targetXCm - robotX;
    }

    Serial.print(F("  [Verified Launch Pose] F=")); Serial.print(dF, 1);
    Serial.print(F(" | L="));   Serial.print(dL, 1); Serial.print(F("/")); Serial.print(cal.targetBackLCm, 1);
    Serial.print(F(" | R="));   Serial.print(dR, 1); Serial.print(F("/")); Serial.print(cal.targetBackRCm, 1);
    Serial.print(F(" | Sum=")); Serial.print(sumLR, 1); Serial.print(F("/")); Serial.print(cal.targetLrSumCm, 1);
    Serial.print(F(" | dx="));  Serial.println(dxRem, 1);

    if (fabsf(dxRem) <= DIST_MATCH_TOL_CM && fabsf(errL) <= 2.2f && fabsf(errR) <= 2.2f) {
      break; // Both distance AND angle locked dead-on!
    }
  }

  // Final box safety gate before reversing
  if (dF > 64.0f || dF < 5.0f || sumLR < cal.targetLrSumCm - 12.0f || sumLR > cal.targetLrSumCm + 12.0f) {
    stopMotors(50);
    Serial.println(F("!!! [TABLE SAFETY ABORT] Pre-reverse box sanity failed!"));
    return false;
  }

  // Fresh 6-axis stillness reset to 0.00 deg right at the verified perpendicular launch pose!
  stopMotors(40);
  delay(150);
  resetHeadingAndEncoders(true);
  exactBackHdg = 0.0f;

  // --- PHASE 2: PURE DEAD-STRAIGHT 0.00-DEG GYRO REVERSE (ZERO ROTATIONS!) ---
  Serial.print(F("[Phase 2] Launching PURE STRAIGHT 0.00-deg reverse from F="));
  Serial.print(dF, 1); Serial.print(F(" -> Target F=")); Serial.println(cal.t1RevFrontCm, 2);

  float revDist = 0.0f, finalF = dF;
  if (!drivePureStraightReverseToT1(cal.t1RevFrontCm, dF, exactBackHdg, &revDist, &finalF)) {
    stopMotors(50);
    return false;
  }

  // --- PHASE 3: CLEAN DEAD STOP ON T1 -> FULL 180-DEG SERVO OUTWARD DROP ---
  stopMotors(50);
  float endF, endL, endR;
  readAllUltrasonics(endF, endL, endR, 3);
  Serial.print(F("[Phase 3] Reached T1! Final F=")); Serial.print(finalF, 1);
  Serial.print(F(" | L=")); Serial.print(endL, 1);
  Serial.print(F(" R="));   Serial.print(endR, 1);
  Serial.print(F(" | GyroHdg=")); Serial.print(getHeadingDeg(), 2);
  Serial.println(F(" deg -> Executing FULL 180-deg outward drop!"));
  performOutwardBallDrop();
  return true;
}

// ============================================================================
// 13. PHASE 4: WALL-SAFE CLEARANCE -> HIGHWAY SEEK & LOCK (FRONT SAFETY ON!)
// ============================================================================
bool runPhase4_SeekAndLockHighway() {
  if (timeUp()) return false;

  if (readUltrasonicMedian(F_TRIG, F_ECHO, 3) > FRONT_SAFE_STOP_CM + 4.0f) {
    Serial.println(F("[Phase 4] Pulling 7.0 cm clear of T1 ball..."));
    driveStraightCm(BALL_CLEAR_FWD_CM, true, nullptr, true, nullptr, true, exactBackHdg, FRONT_SAFE_STOP_CM);
  }

  Serial.println(F("[Phase 4] Seeking outer right highway (Front Safety ON)..."));
  if (!turnToFieldHeading(HIGHWAY_SEEK_HDG_DEG)) return false;

  bool hitLine = false;
  driveStraightCm(MAX_HIGHWAY_SEEK_CM, true, &hitLine, true, nullptr, false, 0.0f, FRONT_SAFE_STOP_CM);

  float dFWall = readUltrasonicMedian(F_TRIG, F_ECHO, 3);
  float safePull = min(IR_AXLE_PULL_MAX_CM, max(0.0f, dFWall - FRONT_SAFE_STOP_CM));
  if (safePull >= 1.5f) {
    driveStraightCm(safePull, true, nullptr, true, nullptr, false, 0.0f, FRONT_SAFE_STOP_CM);
  } else if (dFWall < FRONT_SAFE_STOP_CM) {
    Serial.println(F("  [Phase 4 Avoid] Near outer wall -> backing 4 cm before turning!"));
    driveStraightCm(-4.0f, false, nullptr, true);
  }

  pivotRelative(70.0f, false);

  bool locked = sweepSearchLine(55.0f, -1);
  if (!locked) locked = sweepSearchLine(95.0f, 1);
  if (!locked) {
    if (readUltrasonicMedian(F_TRIG, F_ECHO, 3) > FRONT_SAFE_STOP_CM + 8.0f) {
      driveStraightCm(7.0f, true, nullptr, true, nullptr, false, 0.0f, FRONT_SAFE_STOP_CM);
    }
    locked = sweepSearchLine(80.0f, -1);
    if (!locked) locked = sweepSearchLine(130.0f, 1);
  }
  if (!locked) turnToFieldHeading(85.0f);

  Serial.println(F("[Phase 4] Locked onto upward highway!"));
  return true;
}

// ============================================================================
// 14. PHASE 5: INTEGRATED WORKING 3-IR LINE FOLLOWER (L=GP17, C=GP22, R=GP16)
//   - standaloneTest = true  -> 3 s countdown, live +/-/v/stop keys, 2 min cap
//   - standaloneTest = false -> instant race handoff, obeys 87.5 s race timer
//   Returns true when Point F (end of line after Hairpin E) is reached cleanly.
// ============================================================================
bool runPhase5_FollowLineAndMaze(bool standaloneTest = false) {
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
      if (hairpinDone) reachedPointF = true;
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
      if (hairpinDone && (fwdOdo - hairpinOdo >= LF_LINE_END_MIN_CM) && (now - lostStartMs >= LF_LINE_END_LOST_MS)) {
        why = "END OF LINE after hairpin (Point F)";
        reachedPointF = true;
        gEndByLoss = true;
        gEndOvershootCm = getEncoderDistCm() - lostDist0;
        break;
      }
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
// 15. PHASE 6: PINPOINT_T2 + RAIL CENTERING
// ============================================================================
// Drive toward aimHdg until both rails are seen, then hold the pocket centre line
// with the side ultrasonics (and learn the rails' true direction from how the
// centre offset changes), stop with the front ultrasonic. Returns the final
// centre offset (+ = left of centre), or 99 if the rails were never seen.
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
        float fromL = RAIL_HALF_GAP_CM - (dL + cal.sideOffsetL);   // + = left of centre
        float fromR = (dR + cal.sideOffsetR) - RAIL_HALF_GAP_CM;
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

float raceSecondsLeft() {
  if (!raceTimerActive) return 999.0f;
  return ((float)HARD_STOP_MS - (float)(millis() - runStartMs)) * 0.001f;
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
    driveStraightCm(-gEndOvershootCm, false, nullptr, true);
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
    driveStraightCm(-16.0f, false, nullptr, true, nullptr, true, axisHdg);       // back out along the axis
    float a = constrain(atan2f(off, 10.0f) / D2R, -35.0f, 35.0f);                // diagonal that cancels the offset
    turnToHeading(axisHdg - a, true);
    driveStraightCm(10.0f / cosf(a * D2R), true, nullptr, true, nullptr, true, axisHdg - a, T2_PARK_STOP_CM + 3.0f);
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
// 16. FULL RACE, SETUP & MENU LOOP
// ============================================================================
void runFullCompetition() {
  if (!raceTimerActive) { runStartMs = millis(); raceTimerActive = true; }
  if (!runPhases1To3_T1())              { stopMotors(); return; }
  if (!runPhase4_SeekAndLockHighway())  { stopMotors(); return; }
  if (runPhase5_FollowLineAndMaze(false)) {
    runPhase6_PinpointT2();
  }
  stopMotors();
  Serial.print(F("=== TOTAL RUN TIME: "));
  Serial.print((millis() - runStartMs) * 0.001f, 1); Serial.println(F(" s ==="));
}

void printMenu() {
  Serial.print(F("\n=== IM SELF-CALIBRATING FULL-RACE BOT (IMU: "));
  Serial.print(isGyroHealthy() ? F("6-AXIS") : F("ENC"));
  Serial.print(F(" | Cal: "));
  Serial.print(cal.fromSensorCal ? F("AUTO-CAL") : F("DEFAULT"));
  Serial.print(F(" | F_T1=")); Serial.print(cal.t1RevFrontCm, 1);
  Serial.print(F(" L/R="));    Serial.print(cal.targetBackLCm, 1);
  Serial.print('/');           Serial.print(cal.targetBackRCm, 1);
  Serial.println(F(") ==="));
  Serial.println(F("  a = AUTO-CALIBRATE FROM T1 (place box on T1 facing Back Wall -> auto-feeds 'r'!)"));
  Serial.println(F("  v = View active calibration values | z = Reset calibration to defaults"));
  Serial.println(F("  1 / 2 = Servo -5 / +5 deg | 0 = Servo HOLD | p = Test 2-Stage Outward Drop"));
  Serial.println(F("  g = Live 6-Axis IMU + HB  | w = Spin test (+90/+180/-90) | c = Recal IMU"));
  Serial.println(F("  i = Sensor Test (IR L17/C22/R16 + US + Enc + Hdg)"));
  Serial.println(F("  t = Task 1 ONLY (uses calibrated values!)"));
  Serial.println(F("  f = Phase 5 Line Follower ONLY (3 s countdown, live +/-/v/STOP)"));
  Serial.println(F("  k = Phase 5 + Phase 6     | d = Phase 6 T2 ONLY | r = FULL RACE (Phases 1-6)"));
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

  writeServoDeg(SERVO_HOLD_DEG);

  Serial.begin(115200);

#if USE_EEPROM_CAL
  EEPROM.begin(256);
#endif
  loadCalFromFlash();

#if USE_GYRO
  unsigned long tWait = millis();
  while (!gyroReady && (millis() - tWait < 7000UL)) {
    digitalWrite(LED_BUILTIN, (millis() / 100) % 2);
    delay(20);
  }
  lastSeenHeartbeat = gyroHeartbeat;
  lastHeartbeatMs   = millis();
#endif

#if AUTO_START_ON_MOTOR_SWITCH == 1
  waitForMotorSwitchOn();
  runFullCompetition();
  releaseMotors();
#endif
}

unsigned long lastMenuMs = 0;

void loop() {
#if AUTO_START_ON_MOTOR_SWITCH == 0
  digitalWrite(LED_BUILTIN, (millis() / 300) % 2);
  if (millis() - lastMenuMs >= 4500UL) { lastMenuMs = millis(); printMenu(); }

  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c == '\n' || c == ' ') return;
    while (Serial.available() > 0) Serial.read();

    if (c == '1') {
      int nextA = constrain(currentServoDeg - 5, 0, 180);
      moveServoSmooth(nextA, 15);
      Serial.print(F("[Servo GP27] -> ")); Serial.print(currentServoDeg); Serial.println(F(" deg"));
    } else if (c == '2') {
      int nextA = constrain(currentServoDeg + 5, 0, 180);
      moveServoSmooth(nextA, 15);
      Serial.print(F("[Servo GP27] -> ")); Serial.print(currentServoDeg); Serial.println(F(" deg"));
    } else if (c == '0') {
      moveServoSmooth(SERVO_HOLD_DEG, 12);
      Serial.print(F("[Servo GP27] HOLD -> ")); Serial.print(currentServoDeg); Serial.println(F(" deg"));
    } else if (c == 'p' || c == 'P') {
      Serial.println(F("[Servo Test] Running 2-stage crack & dribble outward drop..."));
      performOutwardBallDrop();
      Serial.println(F("[Servo Test] Done & relaxed."));
    } else if (c == 'g' || c == 'G') {
      resetHeadingAndEncoders(false);
      Serial.println(F("--- Live 6-Axis IMU vs Encoder + Shock Peak (any key to stop) ---"));
      while (Serial.available() == 0) {
        Serial.print(F("Hdg="));       Serial.print(getHeadingDeg(), 2);
        Serial.print(F(" | Enc="));    Serial.print(getEncoderHeadingDeg(), 2);
        Serial.print(F(" | Rate="));   Serial.print(getGyroRateDps(), 1);
        Serial.print(F(" | JerkMg=")); Serial.print(getImuJerkPeakMg());
        Serial.print(F(" | HB="));     Serial.print(gyroHeartbeat);
        Serial.print(F(" | Mode="));   Serial.println(isGyroHealthy() ? F("6-AXIS IMU") : F("ENC"));
        delay(200);
      }
      while (Serial.available() > 0) Serial.read();
      printMenu();
    } else if (c == 'c' || c == 'C') {
      Serial.println(F("[Calibrating] Keep robot still (~0.5 s)..."));
      resetHeadingAndEncoders(true);
      Serial.println(F("[Done] 6-Axis IMU bias refreshed."));
      printMenu();
    } else if (c == 'w' || c == 'W') {
      resetHeadingAndEncoders(true);
      Serial.println(F("[Spin Test] +90.0 deg LEFT..."));
      pivotRelative(90.0f, true);
      Serial.print(F("  After +90:  Hdg=")); Serial.print(getHeadingDeg(), 2);
      Serial.print(F(" | Enc="));           Serial.println(getEncoderHeadingDeg(), 2);
      delay(600);
      Serial.println(F("[Spin Test] +180.0 deg LEFT..."));
      pivotRelative(180.0f, true);
      Serial.print(F("  After +270: Hdg=")); Serial.print(getHeadingDeg(), 2);
      Serial.print(F(" | Enc="));           Serial.println(getEncoderHeadingDeg(), 2);
      delay(600);
      Serial.println(F("[Spin Test] -90.0 deg RIGHT..."));
      pivotRelative(-90.0f, true);
      Serial.print(F("  After +180: Hdg=")); Serial.print(getHeadingDeg(), 2);
      Serial.print(F(" | Enc="));           Serial.println(getEncoderHeadingDeg(), 2);
      printMenu();
    } else if (c == 'i' || c == 'I') {
      resetEncoders();
      Serial.println(F("--- SENSOR TEST: white floor = 000, centred on tape = 010 (any key stops) ---"));
      while (Serial.available() == 0) {
        bool il, ic, ir;
        readIR3(il, ic, ir);
        long l, r; readEncoders(l, r);
        Serial.print(F("IR(LCR)=")); Serial.print(il); Serial.print(ic); Serial.print(ir);
        Serial.print(F(" raw(17/22/16)="));
        Serial.print(digitalRead(PIN_IR_L)); Serial.print(digitalRead(PIN_IR_C)); Serial.print(digitalRead(PIN_IR_R));
        Serial.print(F(" | Enc L=")); Serial.print(l);
        Serial.print(F(" R="));       Serial.print(r);
        Serial.print(F(" | Hdg="));   Serial.print(getHeadingDeg(), 1);
        Serial.print(F(" | US L="));  Serial.print(lfUsRead(L_TRIG, L_ECHO, 6000UL), 1);
        Serial.print(F(" F="));       Serial.print(lfUsRead(F_TRIG, F_ECHO, 6000UL), 1);
        Serial.print(F(" R="));       Serial.println(lfUsRead(R_TRIG, R_ECHO, 6000UL), 1);
        delay(180);
      }
      while (Serial.available() > 0) Serial.read();
      printMenu();
    } else if (c == 'a' || c == 'A') {
      runAutoCalibrateFromT1(); stopMotors(); printMenu();
    } else if (c == 'v' || c == 'V') {
      printCalValues(); printMenu();
    } else if (c == 'z' || c == 'Z') {
      loadDefaultCal(cal);
      saveCalToFlash();
      Serial.println(F("[CAL RESET] Restored factory default values."));
      printCalValues(); printMenu();
    } else if (c == 't' || c == 'T') {
      raceTimerActive = false; runPhases1To3_T1(); stopMotors(); printMenu();
    } else if (c == 'f' || c == 'F') {
      raceTimerActive = false; resetHeadingAndEncoders(true);
      runPhase5_FollowLineAndMaze(true); stopMotors(); printMenu();
    } else if (c == 'k' || c == 'K') {
      raceTimerActive = false; resetHeadingAndEncoders(true);
      if (runPhase5_FollowLineAndMaze(false)) runPhase6_PinpointT2();
      stopMotors(); printMenu();
    } else if (c == 'd' || c == 'D') {
      raceTimerActive = false; resetHeadingAndEncoders(false);
      runPhase6_PinpointT2(); stopMotors(); printMenu();
    } else if (c == 'r' || c == 'R') {
      raceTimerActive = false; runFullCompetition(); printMenu();
    }
    lastMenuMs = millis();
  }
#else
  releaseMotors();
  digitalWrite(LED_BUILTIN, (millis() / 150) % 2);
  delay(50);
#endif
}
