// =====================================================================
//  COMPETITION 2 - SPEED   (round track ~85 in / 216 cm, 30 s)
//  Raspberry Pi Pico  |  Arduino IDE, board package
//  "Raspberry Pi Pico/RP2040" by Earle Philhower
// =====================================================================
//  HOW IT DRIVES
//   * The car runs laps around the INSIDE of the round wall.
//   * The wheel next to the wall (OUTER wheel) always gets full power.
//   * The inner wheel is speed-controlled with its encoder so the car
//     bends around the circle. The wall-side ultrasonic keeps the car
//     D_TARGET_CM from the wall.
//   * A small estimator mixes the ultrasonic with the encoders, so one bad
//     ultrasonic reading never makes the car swerve.
//   * FRONT ultrasonic = emergency only (car pointing into the wall).
//   * IR line sensors are not used (no line on this track).
//
//  RACE DAY
//   1. 12 V motor switch ON, Pico powered.
//   2. Put the car next to the wall, parallel to it, side sensor ~7 cm
//      from the wall. Best: wall on the car's RIGHT (drives counter-
//      clockwise) because your right motor is the stronger one.
//   3. LED: 1 blink/s = wall seen on the RIGHT, 2 blinks/s = on the LEFT,
//      fast flicker = no wall seen (move the car closer).
//   4. Hold a hand 2-8 cm in front of the FRONT sensor -> LED solid = armed.
//   5. On "GO" pull the hand away -> car starts instantly.
//      (Optional: push button GP22 -> GND, press and release = GO.)
//   6. Car stops by itself after RUN_TIME_MS.
//
//  AFTER A PRACTICE RUN: plug in USB, open Serial Monitor (115200).
//  A summary prints every 3 s. Type L and press Enter for the full log.
//
//  FIRST TIME: set WHEEL_TEST to 1, lift the wheels off the floor,
//  upload, and check both wheels spin FORWARD (see section 2).
// =====================================================================

#include <Arduino.h>
#include <math.h>

// ============== 1. SETTINGS YOU MAY CHANGE ===========================
#define WHEEL_TEST 0  // 1 = bench test (wheels in the air), 0 = race
#define WALL_SIDE  0  // 0 = auto-detect at start, 1 = wall on RIGHT, 2 = wall on LEFT

const uint32_t RUN_TIME_MS  = 32000;  // runs a bit past 30 s so it is still flat out when the clock stops
const float    WALL_DIAM_CM = 216.0f; // inside diameter of the wall (85 in)
const float    D_TARGET_CM  = 7.0f;   // wall-side ultrasonic -> wall (wheel ends up ~6 cm off the wall)

// ============== 2. PINS (Raspberry Pi Pico GPx numbers) ==============
const uint8_t ENA = 0, IN1 = 1, IN2 = 2;   // RIGHT motor
const uint8_t IN3 = 3, IN4 = 4, ENB = 5;   // LEFT motor
const uint8_t FRONT_TRIG = 8,  FRONT_ECHO = 9;
const uint8_t RIGHT_TRIG = 6,  RIGHT_ECHO = 7;
const uint8_t LEFT_TRIG  = 18, LEFT_ECHO  = 19;
const uint8_t ENC_R_C1 = 12, ENC_R_C2 = 13;
const uint8_t ENC_L_C1 = 14, ENC_L_C2 = 15;
const uint8_t START_BTN = 22;              // optional button to GND
const uint8_t LED_PIN = LED_BUILTIN;

// Pico pin levels that make each wheel roll FORWARD.
// Your PC817s invert, so the L298N sees the opposite level.
// Copy these from your Competition-1 code, or run WHEEL_TEST:
// if a wheel turns backwards, swap ITS two values (LOW <-> HIGH).
const bool R_FWD_IN1 = LOW, R_FWD_IN2 = HIGH;
const bool L_FWD_IN3 = LOW, L_FWD_IN4 = HIGH;

// ============== 3. ROBOT (measured) ==================================
const float TRACK_CM       = 19.0f;  // wheel centre to centre (tuned value)
const float TICKS_CM_R     = 60.7f;
const float TICKS_CM_L     = 60.5f;
const float RIGHT_FF       = 0.86f;  // right motor needs 0.86x the power of the left
const float SIDE_OUT_CM    = 9.0f;   // side sensors sideways from the centre line
const float RIGHT_FWD_CM   = 13.0f;  // RIGHT sensor ahead of the wheel axle
const float LEFT_FWD_CM    = 13.5f;  // LEFT sensor ahead of the wheel axle

// ============== 4. TUNING (checked in the simulator) =================
const uint32_t TICK_US     = 20000;  // control loop 50 Hz
const float KP             = 0.0040f;  // wall distance error   -> steering
const float KD             = 0.090f;   // angle to the wall     -> steering
const float KI             = 0.00004f; // slow trim (wall size / wheel errors)
const float I_LIM          = 60.0f;
const float KAP_MIN        = 0.0f;     // never steer toward the wall harder than straight
const float KAP_MAX        = 0.060f;   // tightest turn away from the wall (radius ~17 cm)
const float OBS_A          = 0.45f;    // estimator: trust in each ultrasonic reading
const float OBS_B          = 0.02f;    // estimator: angle correction per reading
const float GATE_CM        = 6.0f;     // readings further than this from the estimate are ignored
const float LOST_NUDGE     = 0.05f;    // rad per tick with no side echo (see estimator)
const float LOST_PSI_MAX   = 0.15f;    // ...unless the car already points away from the wall
const float Z_MIN_CM       = 2.0f, Z_MAX_CM = 60.0f;
const float KPW            = 0.010f;   // inner wheel speed loop
const float KIW            = 0.20f;
const float SPEED_LPF      = 0.5f;
const float START_DUTY     = 0.55f;    // soft start (no wheel spin) ...
const float RAMP_S         = 0.25f;    // ... full power after this many seconds
const float CAP_DOWN       = 0.5f;     // if inner wheel can't keep up, ease the outer wheel
const float CAP_UP         = 1.0f;     // per second back to full power
const float CAP_MIN        = 0.55f;
const float FRONT_PANIC_CM = 15.0f;    // FRONT closer than this = turn hard away from the wall
const uint8_t FRONT_EVERY  = 3;        // FRONT ping every 3rd tick (60 ms)
const uint32_t SIDE_TIMEOUT_US  = 5000;   // ~75 cm
const uint32_t FRONT_TIMEOUT_US = 3000;   // ~40 cm
const float HAND_CM        = 8.0f;

// =====================================================================
// write a pin HIGH/LOW (works with every Arduino core version)
inline void pinOut(uint8_t pin, bool high) { digitalWrite(pin, high ? HIGH : LOW); }

volatile uint32_t ticksR = 0, ticksL = 0;
void isrR() { ticksR++; }   // every edge of C1 and C2 = 1 tick (1320 per wheel turn)
void isrL() { ticksL++; }

// duty 0..1 (1 = full speed). Opto-inverted PWM: 255 = stop, 0 = full.
void setEN(uint8_t pin, float duty) {
  if (duty < 0) duty = 0;
  if (duty > 1) duty = 1;
  analogWrite(pin, 255 - (int)(duty * 255.0f + 0.5f));
}
void setRight(float duty) { pinOut(IN1, R_FWD_IN1); pinOut(IN2, R_FWD_IN2); setEN(ENA, duty); }
void setLeft(float duty)  { pinOut(IN3, L_FWD_IN3); pinOut(IN4, L_FWD_IN4); setEN(ENB, duty); }
void coastAll() { setEN(ENA, 0); setEN(ENB, 0); }
void brakeAll() {  // both IN pins equal + enable = L298N brake
  pinOut(IN1, false); pinOut(IN2, false);
  pinOut(IN3, false); pinOut(IN4, false);
  setEN(ENA, 1); setEN(ENB, 1);
}

// returns cm from the sensor face, or -1 if no echo
float pingCm(uint8_t trig, uint8_t echo, uint32_t timeoutUs) {
  pinOut(trig, false); delayMicroseconds(2);
  pinOut(trig, true);  delayMicroseconds(10);
  pinOut(trig, false);
  uint32_t us = pulseIn(echo, HIGH, timeoutUs);
  return us ? us * 0.01715f : -1.0f;
}

// 1 = wall on the right, 2 = wall on the left, 0 = not sure
int detectWallSide() {
  float r = pingCm(RIGHT_TRIG, RIGHT_ECHO, 25000);
  delay(10);
  float l = pingCm(LEFT_TRIG, LEFT_ECHO, 25000);
  bool rNear = r > 0 && r < 40, lNear = l > 0 && l < 40;
  if (rNear && (!lNear || r <= l)) return 1;
  if (lNear) return 2;
  // car at an angle: the wall-side sensor gets no echo, the other one sees the far wall
  if (r > 60 && l < 0) return 2;
  if (l > 60 && r < 0) return 1;
  return 0;
}

// ---------------- run log + stats (printed after the race) ----------
struct Row { int16_t z, d, psi, kap, dOut, dIn, vOut, vIn; };
const int LOG_N = 1700;
Row logBuf[LOG_N];
int logLen = 0;
float statDist = 0, statTurn = 0, statSumErr = 0, statMinZ = 999;
uint32_t statTicks = 0, statBad = 0, statPanic = 0, statCapped = 0;
bool raceWallRight = true;
bool raceDone = false;

// ---------------- waiting for the start -----------------------------
int waitForStart() {
  int side = (WALL_SIDE == 0) ? detectWallSide() : WALL_SIDE;
  bool armed = false, btnWasDown = false;
  uint32_t nearSince = 0, lastSide = millis();
  int clearCount = 0;
  while (true) {
    uint32_t now = millis();
    if (WALL_SIDE == 0 && !armed && now - lastSide > 300) {
      int s = detectWallSide();
      if (s) side = s;            // keep the last good answer
      lastSide = now;
    }

    float f = pingCm(FRONT_TRIG, FRONT_ECHO, 6000);
    bool hand = (f > 0 && f < HAND_CM);
    if (!armed) {
      if (hand) { if (!nearSince) nearSince = now ? now : 1; if (now - nearSince > 400) armed = true; }
      else nearSince = 0;
    } else {
      if (!hand) { if (++clearCount >= 2) return side ? side : 1; }
      else clearCount = 0;
    }
    bool down = digitalRead(START_BTN) == LOW;
    if (btnWasDown && !down) return side ? side : 1;
    btnWasDown = down;

    // LED: solid = armed, 1 blink = right, 2 blinks = left, flicker = no wall
    bool on;
    uint32_t ph = now % 1000;
    if (armed) on = true;
    else if (side == 1) on = ph < 120;
    else if (side == 2) on = ph < 120 || (ph > 250 && ph < 370);
    else on = (now / 60) % 2;
    pinOut(LED_PIN, on);
    delay(25);
  }
}

// ---------------- the race -----------------------------------------
void race(bool wallRight) {
  const uint8_t sT = wallRight ? RIGHT_TRIG : LEFT_TRIG;
  const uint8_t sE = wallRight ? RIGHT_ECHO : LEFT_ECHO;
  const float   sensFwd = wallRight ? RIGHT_FWD_CM : LEFT_FWD_CM;
  const float   wallR   = WALL_DIAM_CM * 0.5f;
  // path radius of the axle = rEff - (side reading)
  const float   rEff = sqrtf(wallR * wallR - sensFwd * sensFwd) - SIDE_OUT_CM;
  // inner-wheel power needed per unit of (speed ratio x outer power)
  const float   innerScale = wallRight ? 1.0f / RIGHT_FF : RIGHT_FF;

  float z0 = pingCm(sT, sE, SIDE_TIMEOUT_US);
  float dHat = (z0 >= Z_MIN_CM && z0 <= Z_MAX_CM) ? z0 : D_TARGET_CM;
  float psi = 0;          // angle to the wall (rad), + = heading away from the wall
  float integ = 0;
  float vOut = 0, vIn = 0, iW = 0, cap = 1.0f;
  bool  panic = false;
  int   rejects = 0, misses = 0;

  noInterrupts(); uint32_t pR = ticksR, pL = ticksL; interrupts();
  const uint32_t tStart = micros();
  uint32_t tPrev = tStart, tNext = tStart;
  pinOut(LED_PIN, false);

  for (uint32_t n = 0;; n++) {
    while ((int32_t)(micros() - tNext) < 0) { }
    tNext += TICK_US;
    const uint32_t now = micros();
    const float el = (now - tStart) * 1e-6f;
    if (now - tStart >= RUN_TIME_MS * 1000UL) break;
    float dt = (now - tPrev) * 1e-6f;
    tPrev = now;
    if (dt < 0.001f) dt = 0.001f;

    // ---- encoders -> distance, turn, speed
    noInterrupts(); uint32_t cR = ticksR, cL = ticksL; interrupts();
    const float dsR = (cR - pR) / TICKS_CM_R, dsL = (cL - pL) / TICKS_CM_L;
    pR = cR; pL = cL;
    const float dsO = wallRight ? dsR : dsL;
    const float dsI = wallRight ? dsL : dsR;
    const float ds  = 0.5f * (dsO + dsI);
    const float dTh = (dsO - dsI) / TRACK_CM;      // + = turning away from the wall
    vOut += SPEED_LPF * (dsO / dt - vOut);
    vIn  += SPEED_LPF * (dsI / dt - vIn);

    // ---- sensors
    const float z = pingCm(sT, sE, SIDE_TIMEOUT_US);
    if (n % FRONT_EVERY == 0) {
      float f = pingCm(FRONT_TRIG, FRONT_ECHO, FRONT_TIMEOUT_US);
      panic = (f > 0 && f < FRONT_PANIC_CM);
    }

    // ---- estimator: predict with encoders, correct with ultrasonic
    float rc = rEff - dHat;
    if (rc < 40) rc = 40;
    const float psiOld = psi;
    psi  += dTh - ds / rc;
    dHat += psiOld * ds + sensFwd * (psi - psiOld);
    if (z >= Z_MIN_CM && z <= Z_MAX_CM) {
      const float innov = z - dHat;
      if (fabsf(innov) <= GATE_CM) {
        dHat += OBS_A * innov;
        psi  += OBS_B * innov / fmaxf(ds, 1.0f);
        rejects = 0;
      } else if (++rejects >= 4) {   // wall really moved: jump to it
        dHat = z; rejects = 0;
      } else statBad++;
      misses = 0;
    } else {
      // No echo. Inside a round wall this almost always means the wall ahead
      // is turning in toward the car (panel joint, or car angled at the wall),
      // so lean the angle estimate "toward the wall" -> car steers away.
      statBad++;
      if (++misses >= 2 && psi < LOST_PSI_MAX) psi -= LOST_NUDGE;
    }
    psi = constrain(psi, -0.7f, 0.7f);
    if (dHat < 0) dHat = 0;

    // ---- steering: curvature away from the wall (1/cm)
    const float e = dHat - D_TARGET_CM;              // + = too far from the wall
    const float iNew = constrain(integ + e * ds, -I_LIM, I_LIM);
    float kap = 1.0f / rc - (KP * e + KD * psi + KI * iNew);
    if (kap > KAP_MIN && kap < KAP_MAX) integ = iNew; // no wind-up when saturated
    if (panic || dHat < Z_MIN_CM) { kap = KAP_MAX; statPanic++; }
    kap = constrain(kap, KAP_MIN, KAP_MAX);
    const float h = kap * TRACK_CM * 0.5f;
    const float ratio = (1.0f - h) / (1.0f + h);     // inner speed / outer speed

    // ---- wheels: outer flat out, inner follows ratio x outer speed
    float ramp = START_DUTY + (1.0f - START_DUTY) * el / RAMP_S;
    if (ramp > 1) ramp = 1;
    const float dOut = fminf(cap, ramp);
    const float err = ratio * vOut - vIn;
    float dIn = ratio * dOut * innerScale + KPW * err + iW;
    if (dIn > 1.0f) {                 // inner wheel maxed out: ease the outer one
      cap -= CAP_DOWN * (dIn - 1.0f);
      dIn = 1.0f;
      if (err < 0) iW += KIW * err * dt;
      statCapped++;
    } else {
      if (dIn < 0) { dIn = 0; if (err > 0) iW += KIW * err * dt; }
      else iW += KIW * err * dt;
      cap += CAP_UP * dt;
    }
    cap = constrain(cap, CAP_MIN, 1.0f);
    iW  = constrain(iW, -0.4f, 0.4f);
    if (wallRight) { setRight(dOut); setLeft(dIn); }
    else           { setLeft(dOut);  setRight(dIn); }

    // ---- stats + log
    statDist += ds; statTurn += dTh; statTicks++;
    if (z > 0) { statSumErr += fabsf(z - D_TARGET_CM); if (z < statMinZ) statMinZ = z; }
    if (logLen < LOG_N) {
      Row &r = logBuf[logLen++];
      r.z = (int16_t)(z * 10); r.d = (int16_t)(dHat * 10); r.psi = (int16_t)(psi * 1000);
      r.kap = (int16_t)(kap * 10000); r.dOut = (int16_t)(dOut * 1000); r.dIn = (int16_t)(dIn * 1000);
      r.vOut = (int16_t)(vOut * 10); r.vIn = (int16_t)(vIn * 10);
    }
  }
  brakeAll();
  delay(400);
  coastAll();
}

void printSummary() {
  Serial.printf("\n=== RUN SUMMARY (wall on %s) ===\n", raceWallRight ? "RIGHT" : "LEFT");
  Serial.printf("distance   %.0f cm   (%.1f m)\n", statDist, statDist / 100.0f);
  Serial.printf("laps       %.2f  (from wheel turning)\n", statTurn / 6.2832f);
  Serial.printf("avg speed  %.1f cm/s\n", statDist / (RUN_TIME_MS / 1000.0f));
  Serial.printf("avg |side - target| %.2f cm, closest side reading %.1f cm\n",
                statSumErr / (statTicks ? statTicks : 1), statMinZ);
  Serial.printf("bad side readings %lu / %lu, panic ticks %lu, outer eased %lu\n",
                (unsigned long)statBad, (unsigned long)statTicks,
                (unsigned long)statPanic, (unsigned long)statCapped);
  Serial.printf("type L + Enter for the full log\n");
}

void dumpLog() {
  Serial.printf("t_s,side_cm,est_cm,angle_deg,curv_1pcm,outer_pwr,inner_pwr,v_outer,v_inner\n");
  for (int i = 0; i < logLen; i++) {
    Row &r = logBuf[i];
    Serial.printf("%.2f,%.1f,%.1f,%.1f,%.4f,%.3f,%.3f,%.1f,%.1f\n", i * TICK_US * 1e-6f,
                  r.z / 10.0f, r.d / 10.0f, r.psi * 0.0573f, r.kap / 10000.0f,
                  r.dOut / 1000.0f, r.dIn / 1000.0f, r.vOut / 10.0f, r.vIn / 10.0f);
  }
}

// ---------------- bench test (WHEEL_TEST 1) -------------------------
void wheelTest() {
  Serial.printf("\nRIGHT wheel should roll FORWARD now...\n");
  uint32_t a = ticksR; setRight(0.6f); delay(1500); coastAll();
  Serial.printf("  right ticks %lu (expect ~3000+)\n", (unsigned long)(ticksR - a));
  delay(1000);
  Serial.printf("LEFT wheel should roll FORWARD now...\n");
  a = ticksL; setLeft(0.6f); delay(1500); coastAll();
  Serial.printf("  left ticks %lu (expect ~3000+)\n", (unsigned long)(ticksL - a));
  for (int i = 0; i < 6; i++) {
    float f = pingCm(FRONT_TRIG, FRONT_ECHO, 25000); delay(20);
    float r = pingCm(RIGHT_TRIG, RIGHT_ECHO, 25000); delay(20);
    float l = pingCm(LEFT_TRIG, LEFT_ECHO, 25000);
    Serial.printf("FRONT %.1f  RIGHT %.1f  LEFT %.1f  (-1 = no echo)\n", f, r, l);
    delay(400);
  }
}

// =====================================================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  analogWriteFreq(250);
  analogWriteRange(255);
  coastAll();
  pinMode(FRONT_TRIG, OUTPUT); pinMode(RIGHT_TRIG, OUTPUT); pinMode(LEFT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);  pinMode(RIGHT_ECHO, INPUT);  pinMode(LEFT_ECHO, INPUT);
  pinMode(START_BTN, INPUT_PULLUP);
  pinMode(ENC_R_C1, INPUT); pinMode(ENC_R_C2, INPUT);
  pinMode(ENC_L_C1, INPUT); pinMode(ENC_L_C2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C1), isrR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C2), isrR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_C1), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_C2), isrL, CHANGE);
  delay(300);
}

void loop() {
#if WHEEL_TEST
  wheelTest();
  delay(3000);
#else
  if (!raceDone) {
    int side = waitForStart();
    raceWallRight = (side != 2);
    race(raceWallRight);
    raceDone = true;
  }
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 3000) { lastPrint = millis(); printSummary(); }
  pinOut(LED_PIN, (millis() / 500) % 2);
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'L' || c == 'l') dumpLog();
  }
#endif
}
