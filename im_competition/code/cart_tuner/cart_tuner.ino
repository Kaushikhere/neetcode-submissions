/*
  CART v6.6 - motor, encoder and wheel-speed calibration.
  Raspberry Pi Pico, arduino-pico core. Serial monitor: 115200, line ending "Newline".
 
  All tunable values are the variables in the TUNING block below: edit them here, re-flash.
 
  NEW IN v3: CLOSED-LOOP WHEEL SPEED.
    Each wheel now measures its own speed from its encoder and adjusts its own PWM to hit
    a target speed in cm/s. That cancels gearbox friction (cold vs warm), motor mismatch,
    battery droop and carpet, instead of trying to pre-compensate them with fixed trims.
    Set SPEED_CONTROL = 0 to fall back to the old open-loop PWM path with TRIM_L/TRIM_R.
 
  Commands (type  help ):
    show              print every tunable value
    enc               print raw counts for 10 s (turn the wheels by hand)
    zero              reset counts and pose to 0
    where             pose, counts, and revolutions of each wheel
    cpr N             after turning the wheels N revolutions by hand: counts per rev
    m l|r PWM MS      OPEN LOOP: one motor at a fixed PWM (reports steady-state cm/s)
    pwmtest l|r       step that motor's enable through 0/64/128/192/255
    vel CMPS MS       CLOSED LOOP: both wheels at a target speed, prints target vs actual
    vel l|r CMPS MS   closed loop, one wheel only
    ffcal PWM         suggests FF_PWM_PER_CMPS from a run at that PWM
    fwd CM | back CM  straight move
    spin DEG          turn in place, + = right (clockwise)
    arc l|r DEG       pivot around that wheel
    stop              brake both motors
  Any character typed while the cart is moving aborts the move.
 
  FRAME: +X forward, +Y to the right, heading 0 = facing +X, positive = clockwise.
*/
 
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
 
// =====================================================================
// PINS
// =====================================================================
// OUR CONNECTIONS
const uint8_t PIN_ENA = 0, PIN_IN1 = 1, PIN_IN2 = 2;      // RIGHT motor (Motor A, L298N OUT1/OUT2)
const uint8_t PIN_IN3 = 3, PIN_IN4 = 4, PIN_ENB = 5;      // LEFT motor  (Motor B, L298N OUT3/OUT4)
const uint8_t PIN_ENC_L_A = 14, PIN_ENC_L_B = 15;         // LEFT encoder  C1 GP14, C2 GP15
const uint8_t PIN_ENC_R_A = 12, PIN_ENC_R_B = 13;         // RIGHT encoder C1 GP12, C2 GP13
const uint8_t PIN_IR_L = 16, PIN_IR_R = 17;               // IR line sensors
// Each ultrasonic has its OWN trigger pin -> fired one at a time (no cross-talk)
const uint8_t PIN_TRIG_F = 8,  PIN_ECHO_F = 9;            // FRONT ultrasonic
const uint8_t PIN_TRIG_R = 6,  PIN_ECHO_R = 7;            // RIGHT ultrasonic
const uint8_t PIN_TRIG_L = 18, PIN_ECHO_L = 19;           // LEFT ultrasonic (new)
 
// =====================================================================
// TUNING - edit these values, then re-flash
// =====================================================================
 
// --- wiring polarity (all 0 or 1) -----------------------------------
int INVERT_PWM     = 1;   // 1 = optocoupler inverts the enable pin (Pico LOW = L298N HIGH)
int INVERT_DIR     = 1;   // 1 = optocoupler inverts IN1..IN4
int LEFT_FWD_SWAP  = 1;   // 1 = the LEFT motor runs backwards on "forward"
int RIGHT_FWD_SWAP = 0;   // 1 = the RIGHT motor runs backwards on "forward"
int ENC_L_INVERT   = 0;   // 1 = flip the LEFT encoder sign  (forward must count UP)
int ENC_R_INVERT   = 1;   // 1 = flip the RIGHT encoder sign (forward must count UP)
 
// --- enable pin behaviour -------------------------------------------
int USE_PWM       = 1;    // 1 = speed control by PWM; 0 = enable is plain on/off, full speed
int PWM_FREQ_HZ   = 500;  // core default. Lower to ~100 if the optocoupler cannot follow
int BRAKE_ON_STOP = 1;    // 1 = stopping drives both direction pins low (brake)
 
// --- encoder and geometry -------------------------------------------
float COUNTS_PER_REV_L = 1560.0f;  // LEFT  encoder counts per ONE wheel revolution
float COUNTS_PER_REV_R = 975.0f;    // RIGHT encoder counts per ONE wheel revolution
float WHEEL_DIAMETER_L = 7.0f;     // LEFT  wheel diameter, cm
float WHEEL_DIAMETER_R = 7.0f;     // RIGHT wheel diameter, cm
float TRACK_WIDTH_CM   = 20.0f;   // effective distance between the wheels, cm (whatever value
                                   // makes  spin 360  come out right - measure, then verify)
float FRONT_POINT_CM   = 16.0f;    // from the wheel axle FORWARD to the middle of the cart's
                                   // front face. This is the point the main pose tracks.
 
// --- ultrasonic sensors ---------------------------------------------
float US_SCALE       = 0.01715f; // cm per microsecond of echo (343 m/s, halved)
float SIDE_SPAN_CM   = 18.45f;   // between the LEFT and RIGHT sensor faces
float FRONT_OFFSET_CM = 18.0f;   // wheel axle -> front sensor face
int   PING_GAP_MS    = 30;       // minimum time between pings
 
// --- part 1: the start region ---------------------------------------
// The inner boundary (the painted line) is NOT the wall. The wall sits BOUNDARY_GAP_CM
// further out, and the sensors see the wall, so every boundary measurement gets the gap
// added to it before it is compared with a sensor reading.
float START_W_INNER   = 111.0f;  // start region width, inner boundary to inner boundary
float BOUNDARY_GAP_CM = 4.2f;    // inner boundary -> wall, each side
float T1_FROM_LEFT_CM = 56.0f;   // T1 line, from the LEFT inner boundary
float APPROACH_FRONT_CM = 20.0f; // drive until the front sensor reads this off the back wall
// ONE squareness tolerance, used by the search and by the re-check after the approach.
// L+R+span equals the wall-to-wall width only when the cart is square to the side walls;
// off square both readings grow by 1/cos(angle). 5 cm corresponds to about 18 degrees.
// Loose enough and the search stops off square and the sideways measurement is wrong by
// a few cm; tight enough and a cart that cannot reach square simply fails, which is honest.
float SQUARE_TOL_CM   = 8.0f;
// While searching, move away from anything too close. BOTH thresholds are raw SENSOR-to-wall
// readings, the same numbers the log prints. (Axle-to-wall would be the side reading +
// SIDE_SPAN_CM/2, or the front reading + FRONT_OFFSET_CM.)
float ESCAPE_SIDE_CM  = 8.0f;   // side sensor reading that triggers an escape
float ESCAPE_FRONT_CM = 10.0f;   // front sensor reading that triggers reversing
int   ESCAPE_TURN_BACK = 1;      // 1 = after a side escape, turn 90 back to the old heading
int   ESCAPE_MAX      = 4;       // give up after this many escapes
 
// The shift onto the T1 line is a closed loop on the FRONT sensor: after the first 90 deg
// turn the front faces a wall whose target distance is known.
int   SHIFT_MAX_PASSES = 3;      // correction moves allowed
int   P1_SHOW_MOVES   = 0;       // 1 = print every move/turn report inside part 1 (diagnosis)
float FRONT_DETECT_CM = 40.0f;   // while searching, the front sensor must be closer than this
                                 // for the wall ahead to count as the back wall. It must be
                                 // larger than (region depth - FRONT_OFFSET_CM), or a cart
                                 // parked near the open end will never trigger.
float APPROACH_CMPS   = 12.0f;    // slow forward speed while closing on the back wall
// Closing on the back wall, in small steps with a sanity check after each one. An ultrasonic
// sensor that ends up almost touching a wall does not read "very small" - it misses the first
// echo and reports something large, which looks exactly like "lots of room ahead". Taken at
// face value that drives the cart into the wall again, so every reading is checked against
// what it should be before it is believed.
float APPROACH_STEP_CM  = 10.0f; // largest single step while closing in
float FRONT_TRUST_CM    = 6.0f;  // a front reading this far from the prediction is not believed
float APPROACH_MIN_CM   = 4.0f;  // smallest believable front reading (also the in-move limit)
float APPROACH_MAX_CM   = 45.0f; // largest believable front reading inside the start region
float POS_TOL_CM        = 1.0f;  // close enough, for the approach and for the sideways shift
float BACKOFF_CM        = 10.0f; // reverse this far to get clear, escaping or recovering
int   DO_APPROACH       = 1;     // 1 = close on the back wall before turning. 0 = turn from
                                 // wherever the cart found square. The distance from the back
                                 // wall is measured either way, just before the final turn,
                                 // so part 2 still knows how far it has to go.
// Watching the front sensor DURING the approach move. A sensor that ends up against a wall
// misses its echo and reports something large, so a reading that jumps well above what it
// should be is taken as a collision and the move is abandoned on the spot.
float WALL_JUMP_CM      = 8.0f;  // reading this far ABOVE the predicted value = hit the wall
int   WALL_BAD_PINGS    = 2;     // consecutive bad pings before the move is abandoned
int   APPROACH_CHECK    = 1;     // 1 = sanity-check every front reading during the approach
                                 // (believable window, re-ping, odometry fallback, back off
                                 // if the cart may be against the wall). 0 = take every
                                 // reading at face value. The approach is made of short
                                 // steps either way.
float WALL_TOUCH_CM     = 10.0f; // only treat a bad reading as "touching the wall", and back
                                 // off, when odometry says the cart really is this close.
                                 // Further out a bad reading is a sensor artefact, and
                                 // reversing just undoes good progress.
int   APPROACH_MAX_PASSES = 6;   // steps allowed
float SCAN_TURN_CMPS  = 12.0f;    // wheel speed for the SEARCH and ALIGNMENT spins only.
                                 // Slow spins settle the ultrasonic readings and overshoot
                                 // less. The two 90 deg turns still use TURN_CMPS.
// The cart is always placed square to the back wall or at a multiple of 45 degrees to it,
// so the search only has to try those eight headings. It turns RIGHT one step at a time.
float SEARCH_STEP_DEG = 45.0f;   // search step; 45 matches the allowed start orientations
int   DO_ALIGN        = 0;       // 0 = skip squaring up entirely and trust the 45 deg search
                                 // steps. 1 = run the alignment stage chosen below.
float P1_TURN_TOL_DEG = 0.5f;    // a 90 deg turn in part 1 is repeated until the heading has
                                 // actually changed by 90 +/- this
int   P1_TURN_TRIES   = 3;       // how many correcting turns to allow
float ALIGN_DRIVE_CM  = 20.0f;   // length of the measuring drive. Longer = more precise.
// The alignment drive has to stay inside the start region: past the open end the side walls
// stop and the side sensors measure nothing useful. These bound it at both ends.
float SIDE_WALL_DEPTH_CM = 50.0f; // how far the side walls run from the back wall. The axle
                                  // must stay inside this for the side sensors to see them.
float ALIGN_MIN_FRONT_CM = 12.0f; // never drive closer than this to the back wall
float ALIGN_EDGE_MARGIN_CM = 3.0f;// keep this much clear of the open end
float ALIGN_MIN_DRIVE_CM = 10.0f; // below this baseline the angle is too noisy to use
float ALIGN_CONSIST_CM = 3.0f;    // the two side readings must change by equal and opposite
                                  // amounts; if |dL + dR| exceeds this they disagree and the
                                  // measurement is thrown away
float COARSE_MIN_DEG  = 3.0f;    // skip the coarse step when the sum says the error is small
float ALIGN_CONVERGE_DEG = 1.5f; // stop once the measured heading error is under this
int   ALIGN_MAX_PASSES = 2;      // hard cap on alignment passes
 
// --- part 3: following the black line -------------------------------
// Same idea as the earlier line follower: the two sensors straddle the line, and whichever
// one is over it tells the cart which way it has drifted. Seeing the line on the left means
// the cart has wandered right, so the left wheel is slowed to steer back. If that same
// sensor stays on the line for longer than SHARP_TURN_MS the correction was not enough, so
// the inner wheel is reversed and the cart pivots instead.
int   IR_BLACK_HIGH   = 1;       // 1 = the IR output reads HIGH over black (check with  ir )
float LINE_CMPS       = 12.0f;   // forward speed with the line centred
float LINE_SLOW_CMPS  = 0.0f;    // inner wheel during a gentle correction (0 = held still)
float LINE_SHARP_CMPS = -10.0f;   // inner wheel during a sharp correction (negative = pivot)
int   SHARP_TURN_MS   = 150;     // one sensor on the line this long = sharp correction
int   LINE_LOST_MS    = 2000;    // neither sensor sees the line this long = stop (0 = never)
float LINE_RUN_CM     = 0.0f;    // stop after this far along the line (0 = run until stopped)
int   LINE_PRINT_MS   = 300;     // how often to print while following
 
// --- closed-loop wheel speed ----------------------------------------
int   SPEED_CONTROL    = 1;     // 1 = closed loop (recommended), 0 = open loop PWM
float CRUISE_CMPS      = 20.0f; // wheel speed for fwd / back, cm/s
float TURN_CMPS        = 18.0f; // maximum wheel speed during spin / arc, cm/s
float MIN_CMPS         = 5.0f;  // slowest wheel speed used near the end of a turn
float FF_PWM_PER_CMPS_L = 5.0f; // LEFT  feedforward: PWM needed per cm/s (set with  ffcal )
float FF_PWM_PER_CMPS_R = 3.5f; // RIGHT feedforward: the motors differ, so these differ too
// Two dead-band numbers per wheel, both measured by  dband :
//   MIN   = PWM needed to KEEP a moving wheel turning. This is an OFFSET, added to the
//           proportional term:  pwm = FF_PWM_PER_CMPS * speed + FF_MIN_PWM
//   BREAK = PWM needed to get a STOPPED wheel moving. This is a FLOOR, not an offset: while
//           the wheel is stopped the output is raised TO this value if it is below it, and
//           left alone if the proportional term already exceeds it. Adding it on top would
//           launch the wheel far past the target on every start.
float FF_BREAK_PWM_L   = 109.0f; // LEFT  breakaway PWM
float FF_BREAK_PWM_R   = 130.0f; // RIGHT breakaway PWM
float FF_MIN_PWM_L     = 81.0f;  // LEFT  running PWM
float FF_MIN_PWM_R     = 93.0f;  // RIGHT running PWM
float FRICTION_PWM     = 0.0f;   // MANUAL friction trim, PWM, added to both wheels in the
                                 // direction they are driving. Raise it if the cart is
                                 // sluggish everywhere (carpet, tight bearings, cold grease);
                                 // lower or go negative if every move starts with a lurch.
float BREAK_SPEED_CMPS = 0.5f;   // below this measured speed the wheel counts as stopped
float HOLD_DEAD_CMPS   = 1.0f;   // while holding at zero speed, a wheel slower than this is
                                 // left braked instead of being driven. Without this the PI
                                 // winds up against a stopped wheel and jogs it backwards.
float KP_SPEED         = 2.0f;  // PWM per cm/s of error
float KI_SPEED         = 5.0f;  // PWM per (cm/s x second) of accumulated error
int   SPEED_DT_MS      = 30;    // control interval, ms. Longer = more encoder counts per
                                // sample = less jitter at low speed, but a slower loop
float SPEED_FILTER     = 0.6f;  // smoothing of the measured speed, 0..1 (1 = none).
                                // Lower it for the wheel whose encoder has fewer counts
float KP_DIST_CMPS     = 2.5f;  // straight moves: speed = KP_DIST_CMPS * cm remaining.
                                // This is the deceleration ramp - lower it to brake earlier
float STOP_EARLY_CM    = 0.0f;  // stop this far short, to cancel any overshoot left after the ramp
float KP_TURN_CMPS     = 0.40f; // turn speed = KP_TURN_CMPS * degrees remaining
float KI_TURN_CMPS     = 0.25f; // ANTI-STALL only: extra wheel speed per (degree x second),
                                // and it builds ONLY while the cart is barely rotating
float TURN_STALL_DEG_S = 4.0f;  // below this turn rate the cart counts as stalled
float MAX_TURN_BOOST_CMPS = 6.0f; // hard cap on the anti-stall term, so it can never cancel
                                  // the deceleration taper near the target angle
float KP_HEADING_CMPS  = 0.35f; // heading hold: cm/s of wheel difference per degree of error
float MAX_HEADING_CMPS = 8.0f;  // limit on that difference
float KP_CROSS_DEG_PER_CM = 2.0f; // steer this many degrees per cm of sideways drift off the line
float MAX_CROSS_DEG    = 20.0f; // limit on that aim-off angle
float FF_REV_SCALE     = 1.0f;  // feedforward multiplier when a wheel runs BACKWARDS.
                                // Raise if  back  is weaker than  fwd  at the same speed
 
// --- open-loop speeds, used only when SPEED_CONTROL = 0 -------------
int BASE_SPEED = 140;     // straight-line PWM
int TURN_SPEED = 140;     // maximum PWM during spin / arc
int MIN_SPEED  = 95;      // slowest PWM that still moves the cart
float KP_DIST_PWM = 12.0f;// open-loop deceleration ramp: PWM per cm remaining
float TRIM_L   = 1.00f;   // LEFT  motor PWM multiplier
float TRIM_R   = 1.00f;   // RIGHT motor PWM multiplier
float KP_TURN  = 1.5f;    // turn PWM = MIN_SPEED + KP_TURN * degrees remaining
float KP_HEADING = 3.0f;  // heading hold, PWM per degree
 
// --- shared ---------------------------------------------------------
int   SQUARE_UP = 1;          // after fwd/back, spin back to the heading the move started on
float SQUARE_UP_TOL_DEG = 1.5f; // only bother correcting above this. Keep it above TURN_TOL_DEG,
                                // or the correcting turn exits immediately having done nothing
int   HEADING_HOLD = 1;       // 1 = steer to stay ON the starting line (heading + sideways drift)
                              // 0 = no steering; the move is only as straight as the wheels match
int   MAX_STEER    = 50;      // open-loop steering limit, PWM
float TURN_TOL_DEG = 0.5f;    // a turn is finished this close to the target
int   SETTLE_MS    = 200;     // pause after each move before reading the encoders
int   ACTIVE_HOLD_MS = 250;   // after a move, hold the wheels at 0 cm/s with the speed loop
                              // before releasing to the brake. Stops a wheel that coasts or
                              // slips past the target. 0 = straight to the brake
 
// =====================================================================
// STATE
// =====================================================================
volatile int32_t encL = 0, encR = 0;
volatile uint8_t qPrevL = 0, qPrevR = 0;
static const int8_t QDEC[16] = {0, 1, -1, 0,  -1, 0, 0, 1,  1, 0, 0, -1,  0, -1, 1, 0};
 
// TWO poses, one heading. The axle pose is what odometry integrates directly; the front pose
// is derived from it every update, so the two can never drift apart.
float frontX = 16.0f, frontY = 0, poseH = 0;   // MAIN: middle of the cart's front face
float poseX = 0, poseY = 0;                    // axle midpoint, between the two wheels
int32_t odoLastL = 0, odoLastR = 0;
uint32_t lastOdoUs = 0;
bool abortFlag = false;
 
// closed-loop speed controller
bool  spActive = false;
float spTgtL = 0, spTgtR = 0;               // target cm/s, signed
float spMeasL = 0, spMeasR = 0;             // measured cm/s
float spIntL = 0, spIntR = 0;               // integrator
float spOutL = 0, spOutR = 0;               // PWM output, signed
int32_t spCntL = 0, spCntR = 0;
uint32_t spLastMs = 0;
 
// =====================================================================
// ENCODERS + ODOMETRY
// =====================================================================
void isrL() {
  uint8_t cur = ((digitalRead(PIN_ENC_L_A) == HIGH) << 1) | (digitalRead(PIN_ENC_L_B) == HIGH);
  int8_t s = QDEC[(qPrevL << 2) | cur];
  encL += ENC_L_INVERT ? -s : s;
  qPrevL = cur;
}
void isrR() {
  uint8_t cur = ((digitalRead(PIN_ENC_R_A) == HIGH) << 1) | (digitalRead(PIN_ENC_R_B) == HIGH);
  int8_t s = QDEC[(qPrevR << 2) | cur];
  encR += ENC_R_INVERT ? -s : s;
  qPrevR = cur;
}
 
float cmPerCountL() { return 3.14159265f * WHEEL_DIAMETER_L / COUNTS_PER_REV_L; }
float cmPerCountR() { return 3.14159265f * WHEEL_DIAMETER_R / COUNTS_PER_REV_R; }
float wheelDistL()  { return encL * cmPerCountL(); }
float wheelDistR()  { return encR * cmPerCountR(); }
 
void odomUpdate() {
  int32_t l = encL, r = encR;
  float dl = (l - odoLastL) * cmPerCountL();
  float dr = (r - odoLastR) * cmPerCountR();
  odoLastL = l; odoLastR = r;
  float d   = 0.5f * (dl + dr);
  float dth = (dl - dr) / TRACK_WIDTH_CM;                // radians, clockwise +
  float mid = poseH * DEG_TO_RAD + 0.5f * dth;
  poseX += d * cosf(mid);
  poseY += d * sinf(mid);
  poseH += dth * RAD_TO_DEG;
 
  // The front point rides FRONT_POINT_CM ahead of the axle, along the current heading.
  // Deriving it (instead of integrating it separately) is exact: during a spin in place the
  // axle stays put while this point sweeps a circle of radius FRONT_POINT_CM, and this
  // formula gives that for free.
  float th = poseH * DEG_TO_RAD;
  frontX = poseX + FRONT_POINT_CM * cosf(th);
  frontY = poseY + FRONT_POINT_CM * sinf(th);
}
 
// Place the cart: X and Y are where the FRONT POINT is, H is the heading in degrees.
void setPose(float fx, float fy, float hdeg) {
  odomUpdate();                                 // consume any pending counts first
  poseH = hdeg;
  float th = hdeg * DEG_TO_RAD;
  poseX = fx - FRONT_POINT_CM * cosf(th);
  poseY = fy - FRONT_POINT_CM * sinf(th);
  frontX = fx; frontY = fy;
}
 
void printPose() {
  Serial.printf("  front point : x=%.2f y=%.2f  heading=%.2f deg\n", frontX, frontY, poseH);
  Serial.printf("  axle point  : x=%.2f y=%.2f\n", poseX, poseY);
}
 
// =====================================================================
// MOTORS
// =====================================================================
void dirPin(uint8_t p, bool active) { digitalWrite(p, (INVERT_DIR ? !active : active) ? HIGH : LOW); }
 
void enablePin(uint8_t p, int pwm) {
  if (USE_PWM) analogWrite(p, INVERT_PWM ? 255 - pwm : pwm);
  else { bool on = pwm > 0; digitalWrite(p, (INVERT_PWM ? !on : on) ? HIGH : LOW); }
}
 
void setMotor(uint8_t en, uint8_t a, uint8_t b, int speed, int swap) {
  speed = constrain(speed, -255, 255);
  int pwm = abs(speed);
  if (pwm == 0) {
    // Brake: both L298N inputs low, so the motor stops even if the enable pin is stuck high.
    if (BRAKE_ON_STOP) { dirPin(a, false); dirPin(b, false); }
    enablePin(en, 0);
    return;
  }
  bool fwd = (speed > 0) != (swap != 0);
  dirPin(a, fwd);
  dirPin(b, !fwd);
  enablePin(en, pwm);
}
 
void setLeftRaw(int s)  { setMotor(PIN_ENB, PIN_IN3, PIN_IN4, s, LEFT_FWD_SWAP); }   // no trim
void setRightRaw(int s) { setMotor(PIN_ENA, PIN_IN1, PIN_IN2, s, RIGHT_FWD_SWAP); }
void setLeft(int s)  { setLeftRaw((int)(s * TRIM_L)); }                              // trimmed
void setRight(int s) { setRightRaw((int)(s * TRIM_R)); }
void drive(int l, int r) { setLeft(l); setRight(r); }
void stopMotors() { setLeftRaw(0); setRightRaw(0); }
 
// =====================================================================
// CLOSED-LOOP WHEEL SPEED
// =====================================================================
void speedReset() {
  spIntL = spIntR = 0; spOutL = spOutR = 0; spMeasL = spMeasR = 0;
  spCntL = encL; spCntR = encR; spLastMs = millis();
}
void speedSet(float l, float r) {                    // cm/s targets, signed
  if (!spActive) { speedReset(); spActive = true; }
  spTgtL = l; spTgtR = r;
}
void speedStop() { spActive = false; spTgtL = spTgtR = 0; stopMotors(); }
 
// Hold both wheels AT ZERO SPEED under closed loop, then release to the brake. If a wheel
// coasts or slips when the brake alone is used, the loop drives it back to a standstill.
void speedHoldStop();
 
void speedTick() {
  if (!spActive) return;
  uint32_t now = millis();
  if (now - spLastMs < (uint32_t)SPEED_DT_MS) return;
  float dt = (now - spLastMs) / 1000.0f;
  if (dt <= 0.0f) return;
  spLastMs = now;
 
  int32_t cl = encL, cr = encR;
  float rawL = (cl - spCntL) * cmPerCountL() / dt;     // quantised by whole encoder counts
  float rawR = (cr - spCntR) * cmPerCountR() / dt;
  float a = constrain(SPEED_FILTER, 0.05f, 1.0f);
  spMeasL += a * (rawL - spMeasL);                     // low-pass, removes count-quantisation noise
  spMeasR += a * (rawR - spMeasR);
  spCntL = cl; spCntR = cr;
 
  // Holding at a standstill is a damper, not a PI loop: push back only against a wheel that
  // is still rolling, and brake once it has stopped. (An arc, where ONE target is zero and
  // the other is not, still uses the full loop below to hold its pivot wheel.)
  if (fabsf(spTgtL) < 0.1f && fabsf(spTgtR) < 0.1f) {
    spIntL = 0; spIntR = 0;
    float hL = (fabsf(spMeasL) < HOLD_DEAD_CMPS) ? 0.0f : constrain(-KP_SPEED * spMeasL, -120.0f, 120.0f);
    float hR = (fabsf(spMeasR) < HOLD_DEAD_CMPS) ? 0.0f : constrain(-KP_SPEED * spMeasR, -120.0f, 120.0f);
    spOutL = hL; spOutR = hR;
    setLeftRaw((int)hL); setRightRaw((int)hR);
    return;
  }
 
  float eL = spTgtL - spMeasL;
  spIntL += eL * dt;
  float ffL = FF_PWM_PER_CMPS_L * spTgtL * (spTgtL < 0 ? FF_REV_SCALE : 1.0f);
  if (fabsf(spTgtL) > 0.1f) {
    float runL = FF_MIN_PWM_L + FRICTION_PWM;                      // running offset + trim
    float brkL = FF_BREAK_PWM_L + FRICTION_PWM;                    // breakaway floor + trim
    ffL += (spTgtL > 0 ? runL : -runL);
    if (fabsf(spMeasL) < BREAK_SPEED_CMPS && fabsf(ffL) < brkL)
      ffL = (spTgtL > 0 ? brkL : -brkL);                           // stopped: raise to the floor
  }
  float outL = ffL + KP_SPEED * eL + KI_SPEED * spIntL;
  if (outL > 255.0f)  { outL = 255.0f;  spIntL -= eL * dt; }    // anti-windup
  if (outL < -255.0f) { outL = -255.0f; spIntL -= eL * dt; }
 
  float eR = spTgtR - spMeasR;
  spIntR += eR * dt;
  float ffR = FF_PWM_PER_CMPS_R * spTgtR * (spTgtR < 0 ? FF_REV_SCALE : 1.0f);
  if (fabsf(spTgtR) > 0.1f) {
    float runR = FF_MIN_PWM_R + FRICTION_PWM;
    float brkR = FF_BREAK_PWM_R + FRICTION_PWM;
    ffR += (spTgtR > 0 ? runR : -runR);
    if (fabsf(spMeasR) < BREAK_SPEED_CMPS && fabsf(ffR) < brkR)
      ffR = (spTgtR > 0 ? brkR : -brkR);
  }
  float outR = ffR + KP_SPEED * eR + KI_SPEED * spIntR;
  if (outR > 255.0f)  { outR = 255.0f;  spIntR -= eR * dt; }
  if (outR < -255.0f) { outR = -255.0f; spIntR -= eR * dt; }
 
  spOutL = outL; spOutR = outR;
  setLeftRaw((int)outL);                              // trims are not used in closed loop
  setRightRaw((int)outR);
}
 
void tickFwd();                                       // forward declaration
 
void tick() {                                         // call often: odometry + speed loop
  uint32_t n = micros();
  if (n - lastOdoUs >= 1000) { lastOdoUs = n; odomUpdate(); }
  speedTick();
}
 
bool aborted() {
  while (Serial.available()) { int c = Serial.read(); if (c != '\n' && c != '\r') abortFlag = true; }
  return abortFlag;
}
 
void waitMs(uint32_t ms) {                            // delay() that keeps the loops running
  uint32_t t = millis();
  while (millis() - t < ms) { tick(); if (aborted()) return; delayMicroseconds(500); }
}
 
void speedHoldStop() {
  if (SPEED_CONTROL && ACTIVE_HOLD_MS > 0) {
    spIntL = 0; spIntR = 0;                           // drop the cruise windup first
    speedSet(0.0f, 0.0f);
    uint32_t t = millis();
    while (millis() - t < (uint32_t)ACTIVE_HOLD_MS) { tick(); delayMicroseconds(500); }
  }
  speedStop();
}
 
void tickFwd() { tick(); }
 
float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}
 
// =====================================================================
// ULTRASONIC  (separate triggers, fired one after another) -> usF / usL / usR
// =====================================================================
const float BAD = 9999.0f;
float usF = BAD, usL = BAD, usR = BAD;
bool ok(float d) { return d < 400.0f; }
float shown(float d) { return ok(d) ? d : -1.0f; }
 
// NOTE ON THE SHARED TRIGGER: all three sensors are wired to one TRIG pin, so all three
// transmit at the same instant and each receiver can hear the other two. A burst from a side
// sensor that reaches the front receiver by a longer path is reported as a longer front
// distance. Nothing in software can prevent that - the sensors would need separate trigger
// pins to be fired one at a time - so the readings are median-filtered and then checked
// against what the wheels say before they are acted on.
// Separate trigger pins: each sensor is fired on its own and timed alone,
// so one sensor can never hear another's burst.
float pingOne(uint8_t trig, uint8_t echo) {
  uint32_t t0 = micros();
  while (digitalRead(echo) == HIGH) { tick(); if ((uint32_t)(micros() - t0) > 26000UL) return BAD; }
  digitalWrite(trig, HIGH); delayMicroseconds(10); digitalWrite(trig, LOW);
  t0 = micros();
  while (digitalRead(echo) == LOW) { tick(); if ((uint32_t)(micros() - t0) > 5000UL) return BAD; }
  uint32_t rise = micros();
  while (digitalRead(echo) == HIGH) { tick(); if ((uint32_t)(micros() - rise) > 26000UL) return BAD; }
  float cm = (float)(micros() - rise) * US_SCALE;
  return (cm >= 2.0f && cm <= 300.0f) ? cm : BAD;
}

void pingOnce() {
  usF = pingOne(PIN_TRIG_F, PIN_ECHO_F); waitMs(10);
  usL = pingOne(PIN_TRIG_L, PIN_ECHO_L); waitMs(10);
  usR = pingOne(PIN_TRIG_R, PIN_ECHO_R);
}

void sense(int n) {                                // median of n pings into usF/usL/usR
  if (n < 1) n = 1;
  if (n > 5) n = 5;
  float v[3][5];
  for (int k = 0; k < n; k++) {
    pingOnce();
    v[0][k] = usF; v[1][k] = usL; v[2][k] = usR;
    waitMs((uint32_t)PING_GAP_MS);
  }
  float res[3];
  for (int i = 0; i < 3; i++) {
    float t[5]; int m = 0;
    for (int k = 0; k < n; k++) if (ok(v[i][k])) t[m++] = v[i][k];
    if (m * 2 <= n) { res[i] = BAD; continue; }
    for (int a = 1; a < m; a++) { float x = t[a]; int b = a - 1; while (b >= 0 && t[b] > x) { t[b+1] = t[b]; b--; } t[b+1] = x; }
    res[i] = t[m / 2];
  }
  usF = res[0]; usL = res[1]; usR = res[2];
}
 
// =====================================================================
// MOVES
// =====================================================================
void turnBy(float deg, int mode);      // defined below; used by the square-up step
bool turnQuiet = false;                // suppress turnBy's own report during square-up
bool quietMoves = false;               // suppress ALL move reports (used while part1 runs)
float lastMoveAlong = 0;               // signed cm actually travelled by the last straight move
float watchFrontStart = -1.0f;         // >0: watch the front sensor during the move, starting
                                       // from this reading. Set by the approach, cleared after.
bool  moveHitWall = false;             // the last move was abandoned by that watch
 
void moveStraight(float cm) {
  int dir = cm >= 0 ? 1 : -1;
  float target = fabsf(cm);
  odomUpdate();
  float x0 = poseX, y0 = poseY, h0 = poseH;
  float l0 = wheelDistL(), r0 = wheelDistR();
  float ch = cosf(h0 * DEG_TO_RAD), sh = sinf(h0 * DEG_TO_RAD);   // the line we mean to follow
  float cross = 0, along = 0;
  moveHitWall = false;
  int badPings = 0;
  uint32_t lastPing = 0;
  uint32_t t0 = millis(), limit = (uint32_t)(target / 3.0f * 1000.0f) + 3000;
  while (millis() - t0 < limit) {
    tick();
    if (aborted()) break;
 
    // Progress is measured ALONG the intended line, and drift ACROSS it.
    float dx = poseX - x0, dy = poseY - y0;
    along =  (dx * ch + dy * sh) * dir;            // distance made good towards the target
    cross = (-dx * sh + dy * ch);                  // + = the cart is to the RIGHT of the line
    float remaining = target - along - STOP_EARLY_CM;
    if (remaining <= 0.0f) break;
 
    // Front watch. Only forwards, and only when the approach has armed it. The speed loop
    // and the odometry both keep running inside pingOnce, so this does not stall the move.
    if (watchFrontStart > 0 && dir > 0 && millis() - lastPing >= (uint32_t)PING_GAP_MS) {
      lastPing = millis();
      pingOnce();
      float predicted = watchFrontStart - along;
      bool bad = ok(usF) ? (usF > predicted + WALL_JUMP_CM || usF < APPROACH_MIN_CM) : false;
      if (bad) {
        if (++badPings >= WALL_BAD_PINGS) {
          Serial.printf("P1 approach: STOPPED mid-move - front read %.2f with %.2f predicted after %.2f cm\n",
                        shown(usF), predicted, along);
          moveHitWall = true;
          break;
        }
      } else badPings = 0;
    }
 
    float v = dir * constrain(KP_DIST_CMPS * remaining, MIN_CMPS, CRUISE_CMPS);
 
    // Steering: aim off the line direction in proportion to the sideways drift, then drive
    // the heading to that aim. The cross-track term flips sign in reverse, because backing
    // up with the nose turned right moves the cart to the left.
    float aim = h0 - dir * constrain(KP_CROSS_DEG_PER_CM * cross, -MAX_CROSS_DEG, MAX_CROSS_DEG);
 
    if (SPEED_CONTROL) {
      float h = HEADING_HOLD ? constrain(KP_HEADING_CMPS * (aim - poseH), -MAX_HEADING_CMPS, MAX_HEADING_CMPS) : 0.0f;
      speedSet(v + h, v - h);                      // h > 0 turns the cart clockwise, either direction
    } else {
      int base = (int)constrain(MIN_SPEED + KP_DIST_PWM * remaining, (float)MIN_SPEED, (float)BASE_SPEED);
      int steer = HEADING_HOLD ? (int)constrain(KP_HEADING * (aim - poseH), (float)-MAX_STEER, (float)MAX_STEER) : 0;
      drive(dir * base + steer, dir * base - steer);
    }
    delayMicroseconds(1500);
  }
  odomUpdate();
  float runL = wheelDistL() - l0, runR = wheelDistR() - r0;   // at the moment the loop ended
  speedHoldStop();
  waitMs((uint32_t)SETTLE_MS);
  odomUpdate();
  float movedL = wheelDistL() - l0, movedR = wheelDistR() - r0;
  float dx = poseX - x0, dy = poseY - y0;
  along = (dx * ch + dy * sh) * dir;
  cross = (-dx * sh + dy * ch);
  lastMoveAlong = dir * along;                       // signed, for the alignment measurement
  if (!quietMoves) {
    Serial.printf("asked %.1f cm\n", cm);
    Serial.printf("  during the move : L=%.2f  R=%.2f cm\n", runL, runR);
    Serial.printf("  added stopping  : L=%+.2f R=%+.2f cm   <- a big number here means that wheel coasts or slips\n",
                  movedL - runL, movedR - runR);
    Serial.printf("  total           : L=%.2f  R=%.2f cm\n", movedL, movedR);
    Serial.printf("  along the line  : %.2f cm (error %+.2f)   sideways drift: %+.2f cm   heading change %+.2f deg\n",
                  along, along - target, cross, wrap180(poseH - h0));
    printPose();
  }
 
  // Square up: whatever heading the move ended on, turn back to the one it started on, so the
  // cart finishes parallel to its original orientation. The sideways drift is NOT corrected -
  // only the direction the cart is pointing.
  if (SQUARE_UP) {
    float dh = wrap180(poseH - h0);
    if (fabsf(dh) > SQUARE_UP_TOL_DEG) {
      if (!quietMoves) Serial.printf("  squaring up     : turning %+.2f deg\n", -dh);
      turnQuiet = true;
      turnBy(-dh, 0);
      turnQuiet = false;
      odomUpdate();
      if (!quietMoves) Serial.printf("  heading now     : %+.2f deg from the start\n", wrap180(poseH - h0));
    } else if (!quietMoves) {
      Serial.printf("  squaring up     : not needed (%.2f deg, within SQUARE_UP_TOL_DEG)\n", dh);
    }
  }
}
 
// mode 0 = spin in place, 1 = pivot about the LEFT wheel, 2 = pivot about the RIGHT wheel
void turnBy(float deg, int mode) {
  if (fabsf(deg) < 0.5f) return;
  odomUpdate();
  float h0 = poseH, l0 = wheelDistL(), r0 = wheelDistR();
  int dir = deg > 0 ? 1 : -1;
  float iTurn = 0;                                   // anti-stall integral
  float hRate = 0, rateH = h0;                       // measured turn rate, deg/s
  uint32_t lastT = millis(), rateT = millis();
  uint32_t t0 = millis(), limit = (uint32_t)(fabsf(deg) / 20.0f * 1000.0f) + 4000;
  while (millis() - t0 < limit) {
    tick();
    if (aborted()) break;
    uint32_t now = millis();
    float dt = (now - lastT) / 1000.0f; lastT = now;
    float rem = deg - (poseH - h0);
    if (rem * dir <= TURN_TOL_DEG) break;
 
    // Measure how fast the cart is actually rotating, over a 50 ms window.
    if (now - rateT >= 50) {
      hRate = fabsf(poseH - rateH) / ((now - rateT) / 1000.0f);
      rateH = poseH; rateT = now;
    }
 
    // Proportional on the angle remaining gives the deceleration taper. The integral is an
    // ANTI-STALL term only: it builds while the cart is barely moving (small angles, stiction)
    // and decays as soon as it is rotating properly, so it cannot defeat the taper on a large
    // turn and carry full speed into the target angle.
    if (hRate < TURN_STALL_DEG_S) iTurn += fabsf(rem) * dt;
    else                          iTurn -= iTurn * dt * 3.0f;
    if (iTurn < 0) iTurn = 0;
    float boost = constrain(KI_TURN_CMPS * iTurn, 0.0f, MAX_TURN_BOOST_CMPS);
    float mag = constrain(KP_TURN_CMPS * fabsf(rem) + boost, MIN_CMPS, TURN_CMPS);
    float v = dir * mag;
 
    if (SPEED_CONTROL) {
      if (mode == 0)      speedSet(v, -v);
      else if (mode == 1) speedSet(0, -v);           // left wheel actively held still
      else                speedSet(v, 0);
    } else {
      int pwm = (int)constrain(MIN_SPEED + KP_TURN * fabsf(rem), (float)MIN_SPEED, (float)TURN_SPEED);
      if (mode == 0)      drive(dir * pwm, -dir * pwm);
      else if (mode == 1) drive(0, -dir * pwm);
      else                drive(dir * pwm, 0);
    }
    delayMicroseconds(1500);
  }
  odomUpdate();
  float runH = poseH - h0;
  speedHoldStop();
  waitMs((uint32_t)SETTLE_MS);
  odomUpdate();
  if (!turnQuiet && !quietMoves) {
    Serial.printf("asked %.1f deg -> odometry turned %.2f deg (%.2f during the move, %+.2f added stopping)\n",
                  deg, poseH - h0, runH, (poseH - h0) - runH);
    Serial.printf("  wheels L=%.2f R=%.2f cm\n", wheelDistL() - l0, wheelDistR() - r0);
    Serial.println("  Measure the REAL angle. Odometry larger than the real angle -> RAISE TRACK_WIDTH_CM;");
    Serial.println("  if the cart physically turns MORE than asked -> LOWER it, by the ratio asked/real.");
    printPose();
  }
}
 
// =====================================================================
// PART 1 - find the cart in the start region and park it on the T1 line
// =====================================================================
float savedTurnCmps = -1.0f;                         // set while the slow scan speed is in force
float savedCruise   = -1.0f;                         // set while the slow approach speed is on
 
void slowSpins(bool on) {
  if (on) { if (savedTurnCmps < 0) savedTurnCmps = TURN_CMPS; TURN_CMPS = SCAN_TURN_CMPS; }
  else if (savedTurnCmps > 0) { TURN_CMPS = savedTurnCmps; savedTurnCmps = -1.0f; }
}
void slowDrive(bool on) {
  if (on) { if (savedCruise < 0) savedCruise = CRUISE_CMPS; CRUISE_CMPS = APPROACH_CMPS; }
  else if (savedCruise > 0) { CRUISE_CMPS = savedCruise; savedCruise = -1.0f; }
}
 
bool p1fail(const char* why) {
  speedStop();
  slowSpins(false); slowDrive(false);
  quietMoves = false;
  Serial.print("P1 FAIL   : "); Serial.println(why);
  return false;
}
 
// ---------------------------------------------------------------------
// Squaring up to the side walls.
//
// One pair of side readings gives the SIZE of the heading error but not its sign, because
// both readings scale by 1/cos(phi) and cos is symmetric:
//       left + right = (W - span) / cos(phi)
// It is also useless close to square: at 1 deg the total rises by 0.015 cm, well inside the
// sensor noise. So it is only used for a coarse first correction, with the sign resolved by
// a test turn.
//
// The fine work is done by driving instead. A cart with heading error phi that travels d
// along its own heading slides sideways by d*sin(phi), so one side reading grows and the
// other shrinks:
//       phi = asin( (dLeft - dRight) / (2 * d) )       (d signed: + forward, - reverse)
// That is linear in phi and carries its sign. Over 20 cm, 1 deg shows up as 0.7 cm of
// difference - about 45x the signal the sum gives at the same angle.
// ---------------------------------------------------------------------
 
// Turn by deg and keep correcting until the heading really has changed by that much. A
// single turnBy stops as soon as it is within TURN_TOL_DEG of the target, and momentum can
// carry it past, so part 1 checks the result and trims it.
float turnExact(float deg, float tol, int tries) {
  odomUpdate();
  float h0 = poseH;
  turnBy(deg, 0);
  for (int i = 0; i < tries; i++) {
    odomUpdate();
    float err = deg - (poseH - h0);
    if (fabsf(err) <= tol) break;
    Serial.printf("P1 turnfix : off by %+.2f deg, correcting\n", err);
    turnBy(err, 0);
  }
  odomUpdate();
  float done = poseH - h0;
  Serial.printf("P1 turn    : asked %+.0f, got %+.2f deg (error %+.2f)\n", deg, done, done - deg);
  return done;
}
 
// Size of the heading error from the side readings alone. Sign unknown. -1 = no walls seen.
float squareErrorMag(float W) {
  if (!ok(usL) || !ok(usR)) return -1.0f;
  float base = W - SIDE_SPAN_CM;
  float sum = usL + usR;
  if (sum <= base) return 0.0f;                      // already square, or reading short
  return acosf(constrain(base / sum, -1.0f, 1.0f)) * RAD_TO_DEG;
}
 
// Coarse correction: turn by the measured size, then check. If the error roughly doubled we
// guessed the wrong way, and the new reading is exactly the turn needed to come back.
void coarseSquare(float W) {
  sense(5);
  float p0 = squareErrorMag(W);
  if (p0 < 0) { Serial.println("P1 coarse  : no side walls visible, skipped"); return; }
  if (p0 < COARSE_MIN_DEG) { Serial.printf("P1 coarse  : |error| %.1f deg, already close\n", p0); return; }
  turnBy(p0, 0);
  sense(5);
  float p1 = squareErrorMag(W);
  Serial.printf("P1 coarse  : |error| %.1f -> turned %+.1f -> |error| %.1f\n", p0, p0, p1);
  if (p1 > 0 && p1 > 0.8f * p0) {
    turnBy(-p1, 0);
    Serial.printf("P1 coarse  : wrong way, turned back %.1f deg\n", p1);
  }
}
 
// Drive a known distance and read the sideways drift off the walls. Returns the signed
// heading error in degrees, or 999 if a wall was lost.
float driveAlign() {
  sense(5);
  if (!ok(usL) || !ok(usR)) return 999.0f;
  float L0 = usL, R0 = usR;
 
  // Work out how far the cart can actually go each way. Forwards it must not reach the back
  // wall; backwards it must not leave the start region, because the side walls end there and
  // reversing past them is what turns this measurement into nonsense.
  float axleBack = ok(usF) ? (usF + FRONT_OFFSET_CM) : SIDE_WALL_DEPTH_CM;
  float roomFwd  = ok(usF) ? (usF - ALIGN_MIN_FRONT_CM) : 0.0f;
  float roomBack = SIDE_WALL_DEPTH_CM - axleBack - ALIGN_EDGE_MARGIN_CM;
  int   dir  = (roomFwd >= roomBack) ? 1 : -1;
  float room = (dir > 0) ? roomFwd : roomBack;
  float dist = fminf(ALIGN_DRIVE_CM, room);
  if (dist < ALIGN_MIN_DRIVE_CM) {
    Serial.printf("P1 drive   : no room to measure (axle %.1f from the back wall, %.1f cm ahead, %.1f behind) - skipped\n",
                  axleBack, roomFwd, roomBack);
    return 0.0f;                                     // report square rather than guess
  }
 
  slowDrive(true);
  moveStraight(dir * dist);
  slowDrive(false);
  float d = lastMoveAlong;                           // what the wheels actually did
  if (fabsf(d) < 2.0f) return 999.0f;
  sense(5);
  if (!ok(usL) || !ok(usR)) return 999.0f;
  float dL = usL - L0, dR = usR - R0;
 
  // Sliding sideways moves one sensor nearer its wall and the other the same amount further,
  // so dL and dR must be equal and opposite. If they are not, at least one sensor was not
  // watching its own wall the whole way.
  if (fabsf(dL + dR) > ALIGN_CONSIST_CM) {
    Serial.printf("P1 drive   : %+.2f cm  dL=%+.2f dR=%+.2f  -> inconsistent (sum %+.2f), thrown away\n",
                  d, dL, dR, dL + dR);
    return 0.0f;
  }
 
  float sp = constrain((dL - dR) / (2.0f * d), -0.6f, 0.6f);
  float phi = asinf(sp) * RAD_TO_DEG;
  Serial.printf("P1 drive   : %+.2f cm (of %.1f possible %s)  dL=%+.2f dR=%+.2f  ->  heading error %+.2f deg\n",
                d, room, dir > 0 ? "ahead" : "behind", dL, dR, phi);
  return phi;
}
 
float p1BackWallCm  = -1.0f;     // axle to back wall, MEASURED before the second turn
float p1RightWallCm = -1.0f;     // axle to the right-hand wall, from the final readings
 
bool part1() {
  const float W = START_W_INNER + 2.0f * BOUNDARY_GAP_CM;        // wall to wall
  const float TARGET = T1_FROM_LEFT_CM + BOUNDARY_GAP_CM;        // axle distance from the wall
                                                                 // the RIGHT sensor sees
  const float S2 = SIDE_SPAN_CM / 2.0f;
  quietMoves = !P1_SHOW_MOVES;                                   // move reports off unless asked
 
  Serial.println("P1 ======== PART 1 ========");
  Serial.printf("P1 config  : W=%.2f target=%.2f span=%.2f Foff=%.2f approach=%.1f detect=%.1f step=%.0f\n",
                W, TARGET, SIDE_SPAN_CM, FRONT_OFFSET_CM, APPROACH_FRONT_CM, FRONT_DETECT_CM, SEARCH_STEP_DEG);
  Serial.printf("P1 tol     : square=%.1f pos=%.2f fronttrust=%.1f backoff=%.1f scanspeed=%.1f appspeed=%.1f\n",
                SQUARE_TOL_CM, POS_TOL_CM, FRONT_TRUST_CM, BACKOFF_CM, SCAN_TURN_CMPS, APPROACH_CMPS);
  Serial.printf("P1 escape  : side<%.1f front<%.1f (sensor to wall) move %.1f turnback %d max %d\n",
                ESCAPE_SIDE_CM, ESCAPE_FRONT_CM, BACKOFF_CM, ESCAPE_TURN_BACK, ESCAPE_MAX);
  Serial.printf("P1 shift   : passes %d | show moves %d\n", SHIFT_MAX_PASSES, P1_SHOW_MOVES);
  Serial.printf("P1 approach: do=%d check=%d step %.1f window %.1f-%.1f touch %.1f passes %d | watch jump %.1f, %d bad pings\n",
                DO_APPROACH, APPROACH_CHECK, APPROACH_STEP_CM, APPROACH_MIN_CM, APPROACH_MAX_CM,
                WALL_TOUCH_CM, APPROACH_MAX_PASSES, WALL_JUMP_CM, WALL_BAD_PINGS);
  Serial.printf("P1 align   : do=%d drive=%.0fcm converge=%.1f passes=%d coarsemin=%.1f friction=%.1f\n",
                DO_ALIGN, ALIGN_DRIVE_CM, ALIGN_CONVERGE_DEG, ALIGN_MAX_PASSES, COARSE_MIN_DEG, FRICTION_PWM);
  Serial.printf("P1 align   : region depth %.0f, keep %.0f from the back wall and %.0f from the open end, min baseline %.0f\n",
                SIDE_WALL_DEPTH_CM, ALIGN_MIN_FRONT_CM, ALIGN_EDGE_MARGIN_CM, ALIGN_MIN_DRIVE_CM);
  Serial.printf("P1 turns   : tol %.1f deg, up to %d corrections | heading hold %d, square up %d\n",
                P1_TURN_TOL_DEG, P1_TURN_TRIES, HEADING_HOLD, SQUARE_UP);
 
  // ---- 0. starting readings (clearance is handled by the escape logic in the search) ----
  sense(3);
  Serial.printf("P1 start   : F=%6.1f L=%6.1f R=%6.1f\n", shown(usF), shown(usL), shown(usR));
 
  // ---- 1. turn until the side walls and the back wall both look right ----
  slowSpins(true);
  float stepDeg = (SEARCH_STEP_DEG >= 5.0f && SEARCH_STEP_DEG <= 180.0f) ? SEARCH_STEP_DEG : 45.0f;
  float searchLimit = 360.0f + stepDeg;
  float turned = 0, sum = -1.0f;
  bool found = false;
  int escapes = 0;
  while (turned <= searchLimit) {
    sense(3);
    sum = (ok(usL) && ok(usR)) ? (usL + usR + SIDE_SPAN_CM) : -1.0f;
    bool sumOK   = (sum > 0) && (fabsf(sum - W) <= SQUARE_TOL_CM);
    bool frontOK = ok(usF) && (usF <= FRONT_DETECT_CM);
    Serial.printf("P1 search  : %6.1f deg  F=%6.1f L=%6.1f R=%6.1f sum=%6.1f  %s %s\n",
                  turned, shown(usF), shown(usL), shown(usR), sum,
                  sumOK ? "WALLS" : "-----", frontOK ? "FRONT" : "-----");
 
    // A match wins over an escape. Being close to the back wall is exactly what the match
    // test is looking for, so a cart that is square and near the wall used to trigger the
    // front escape and reverse away from the answer it had just found. The approach step
    // handles a cart that is too close by reversing, and the 90 deg turns happen later, at
    // APPROACH_FRONT_CM, where there is room.
    if (sumOK && frontOK) { found = true; break; }
 
    // --- too close to keep spinning? move clear first (raw sensor-to-wall readings) ---
    bool tooFront = ok(usF) && usF < ESCAPE_FRONT_CM;
    bool tooL = ok(usL) && usL < ESCAPE_SIDE_CM;
    bool tooR = ok(usR) && usR < ESCAPE_SIDE_CM;
    if (tooFront || tooL || tooR) {
      if (++escapes > ESCAPE_MAX) return p1fail("boxed in - too many escape moves");
      slowDrive(true);
      if (tooFront) {
        Serial.printf("P1 escape  : front %.1f < %.1f, reversing %.1f cm\n", usF, ESCAPE_FRONT_CM, BACKOFF_CM);
        moveStraight(-BACKOFF_CM);               // heading unchanged: keep the sweep count
      } else {
        // If both sides are close, move away from the nearer one. Turning away swings the
        // short rear end toward the wall, not the long front end.
        bool left = tooL && (!tooR || usL <= usR);
        float reading = left ? usL : usR;
        float away = left ? 90.0f : -90.0f;          // turn so the near wall ends up behind
        Serial.printf("P1 escape  : %s %.1f cm < %.1f, turning %+.0f, moving %.1f cm%s\n",
                      left ? "left" : "right", reading, ESCAPE_SIDE_CM, away, BACKOFF_CM,
                      ESCAPE_TURN_BACK ? ", turning back" : "");
        turnBy(away, 0);
        moveStraight(BACKOFF_CM);
        if (ESCAPE_TURN_BACK) turnBy(-away, 0);      // heading restored: keep the sweep count
        else turned = 0;                             // heading changed: sweep again from here
      }
      slowDrive(false);
      if (aborted()) return p1fail("aborted");
      continue;                                      // re-test this heading from the new spot
    }
 
    if (aborted()) return p1fail("aborted");
    turnBy(stepDeg, 0);
    turned += stepDeg;
  }
  if (!found) return p1fail("no heading facing the back wall (check START_W_INNER, BOUNDARY_GAP_CM, SIDE_SPAN_CM, FRONT_DETECT_CM)");
  Serial.printf("P1 match   : after %.1f deg of search\n", turned);
 
  // ---- 2. square up to the side walls ----
  if (!DO_ALIGN) {
    Serial.println("P1 align   : skipped (DO_ALIGN = 0, relying on the 45 deg search steps)");
  } else {
    coarseSquare(W);                                 // gets within a few degrees
    for (int pass = 0; pass < ALIGN_MAX_PASSES; pass++) {
      float phi = driveAlign();                      // measures down to a fraction of a degree
      if (phi > 900.0f) return p1fail("lost a side wall during the alignment drive");
      if (fabsf(phi) < ALIGN_CONVERGE_DEG) { Serial.printf("P1 square  : within %.2f deg\n", fabsf(phi)); break; }
      turnBy(-phi, 0);
      if (pass == ALIGN_MAX_PASSES - 1) Serial.println("P1 WARN    : alignment still moving after the last pass");
    }
  }
  slowSpins(false);
 
  // No squareness re-check here: the search already required |sum - W| <= SQUARE_TOL_CM at
  // this heading, and with alignment off nothing has moved since. The real re-check is the
  // one after the approach, by which time the cart has driven.
  sense(5);
  if (!ok(usL) || !ok(usR)) return p1fail("lost a side wall after the search");
  Serial.printf("P1 square  : F=%6.1f L=%6.1f R=%6.1f sum=%6.2f (want %.2f, err %+.2f)\n",
                shown(usF), usL, usR, usL + usR + SIDE_SPAN_CM, W, usL + usR + SIDE_SPAN_CM - W);
 
  // ---- 2. close on the back wall ----
  if (!DO_APPROACH) {
    Serial.println("P1 approach: skipped (DO_APPROACH = 0) - turning from where the cart stands");
  } else {
  slowDrive(true);
  Serial.printf("P1 approach: checks %s, front watch %s\n",
                APPROACH_CHECK ? "ON" : "OFF (readings taken at face value)",
                APPROACH_CHECK ? "ON" : "OFF");
  sense(5);
  float fNow = usF;
  if (APPROACH_CHECK) {
    // The first reading has to be believable before anything is driven off it.
    for (int tryN = 0; tryN < 2 && !(ok(fNow) && fNow >= APPROACH_MIN_CM && fNow <= APPROACH_MAX_CM); tryN++) {
      Serial.printf("P1 approach: F=%.2f is not believable (want %.1f-%.1f), reversing %.1f cm\n",
                    shown(fNow), APPROACH_MIN_CM, APPROACH_MAX_CM, BACKOFF_CM);
      moveStraight(-BACKOFF_CM);
      sense(5);
      fNow = usF;
    }
    if (!(ok(fNow) && fNow >= APPROACH_MIN_CM && fNow <= APPROACH_MAX_CM)) {
      slowDrive(false);
      return p1fail("front sensor is not giving a believable distance to the back wall");
    }
  } else if (!ok(fNow)) {
    slowDrive(false);
    return p1fail("no front reading for the approach");
  }
 
  bool arrived = false;
  for (int pass = 0; pass < APPROACH_MAX_PASSES; pass++) {
    float d = fNow - APPROACH_FRONT_CM;
    if (fabsf(d) <= POS_TOL_CM) { arrived = true; break; }
 
    // Never commit to more than one step at a time: a wrong reading can only ever send the
    // cart APPROACH_STEP_CM in the wrong direction before it gets checked again.
    float step = constrain(d, -APPROACH_STEP_CM, APPROACH_STEP_CM);
    Serial.printf("P1 approach: F=%6.2f want %.2f  step %+.2f cm (of %+.2f)\n",
                  fNow, APPROACH_FRONT_CM, step, d);
 
    watchFrontStart = (APPROACH_CHECK && step > 0) ? fNow : -1.0f;
    moveStraight(step);
    watchFrontStart = -1.0f;
 
    // The move was abandoned because the cart looks like it reached the wall. The wheels
    // still know how far it went, so work out where it is and reverse the difference.
    if (moveHitWall) {
      float gap = fNow - lastMoveAlong;              // odometry's view of the distance left
      float back = APPROACH_FRONT_CM - gap;          // positive = too close, reverse this far
      Serial.printf("P1 approach: wheels say %.2f cm from the wall after %.2f cm\n", gap, lastMoveAlong);
      if (back > POS_TOL_CM) {
        Serial.printf("P1 approach: too close, reversing %.2f cm\n", back);
        moveStraight(-back);
        gap += fabsf(lastMoveAlong);
      }
      fNow = gap;
      continue;                                      // re-measure on the next pass
    }
 
    float expect = fNow - lastMoveAlong;             // what the front should read now
    sense(5);
 
    if (!APPROACH_CHECK) {                           // take it as it comes
      if (ok(usF)) fNow = usF;
      else { Serial.println("P1 approach: no echo, using odometry for this step"); fNow = expect; }
      continue;
    }
 
    bool believable = ok(usF) && usF >= APPROACH_MIN_CM && usF <= APPROACH_MAX_CM
                      && fabsf(usF - expect) <= FRONT_TRUST_CM;
 
    if (!believable) {
      // One artefact is not proof. Ask again before acting on it.
      float first = usF;
      waitMs(80);
      sense(5);
      believable = ok(usF) && usF >= APPROACH_MIN_CM && usF <= APPROACH_MAX_CM
                   && fabsf(usF - expect) <= FRONT_TRUST_CM;
      Serial.printf("P1 approach: F=%.2f then %.2f, but %.2f expected after moving %+.2f - %s\n",
                    shown(first), shown(usF), expect, lastMoveAlong,
                    believable ? "second look agrees" : "not believed");
    }
    if (believable) { fNow = usF; continue; }
 
    // Odometry is the fallback. If it says the cart has already arrived, it has: a sensor
    // that disagrees at the target distance is an artefact, not a reason to drive again.
    fNow = expect;
    if (fabsf(expect - APPROACH_FRONT_CM) <= POS_TOL_CM) {
      Serial.println("P1 approach: odometry says this is the target, taking it");
      arrived = true;
      break;
    }
 
    // Only back off when the cart really could be against the wall.
    if (lastMoveAlong > 0 && expect < WALL_TOUCH_CM) {
      Serial.printf("P1 approach: odometry says only %.2f cm left, reversing %.1f cm\n", expect, BACKOFF_CM);
      moveStraight(-BACKOFF_CM);
      sense(5);
      float back = expect - lastMoveAlong;           // lastMoveAlong is negative here
      fNow = (ok(usF) && usF >= APPROACH_MIN_CM && usF <= APPROACH_MAX_CM
              && fabsf(usF - back) <= FRONT_TRUST_CM) ? usF : back;
    }
    if (aborted()) { slowDrive(false); return p1fail("aborted"); }
  }
  slowDrive(false);
  Serial.printf("P1 approach: %s at F=%.2f (target %.2f)\n",
                arrived ? "settled" : "OUT OF PASSES", fNow, APPROACH_FRONT_CM);
  if (!arrived) return p1fail("could not settle on the back wall - see the readings above");
  }
 
  // ---- 3. where are we across the region? ----
  sense(5);
  if (!ok(usL) || !ok(usR)) return p1fail("lost a side wall before the shift");
  sum = usL + usR + SIDE_SPAN_CM;
  Serial.printf("P1 measure : F=%6.2f L=%6.2f R=%6.2f sum=%6.2f (err %+.2f)\n",
                shown(usF), usL, usR, sum, sum - W);
  if (fabsf(sum - W) > SQUARE_TOL_CM) {
    if (sum > W)
      return p1fail("side total too HIGH: the cart is off square, or a side sensor is past the end of its wall (lower APPROACH_FRONT_CM)");
    else
      return p1fail("side total too LOW: a side sensor is seeing something nearer than its wall - an obstacle, the back wall because the cart is turned, or cross-talk");
  }
  // The axle's distance from the right-hand wall, measured twice: from the right sensor, and
  // from the left sensor via the known width. Averaging cancels half the span (it appears
  // with opposite signs) and any bias the two sensors share; a bias in W shows up halved.
  float estR = usR + S2;
  float estL = W - usL - S2;
  float fromRight = 0.5f * (W + usR - usL);
  float delta = TARGET - fromRight;
  Serial.printf("P1 fix     : axle from right wall: right sensor %.2f, left sensor %.2f (disagree %+.2f) -> %.2f\n",
                estR, estL, estR - estL, fromRight);
  Serial.printf("P1 fix     : want %.2f, shift %+.2f cm (%s)\n",
                TARGET, delta, delta >= 0 ? "turn left" : "turn right");
  if (fabsf(delta) > W) return p1fail("shift impossible - check the geometry numbers");
 
  // ---- 4. turn, shift (closed loop on the front sensor), measure v, turn ----
  float t = (delta >= 0) ? -90.0f : 90.0f;
  // After this turn the front faces a side wall. The target front reading puts the axle on
  // the T1 line: turning left faces the far wall (W - TARGET away), turning right faces the
  // right-hand wall (TARGET away).
  const float expectF = ((delta >= 0) ? (W - TARGET) : TARGET) - FRONT_OFFSET_CM;
  turnExact(t, P1_TURN_TOL_DEG, P1_TURN_TRIES);
 
  float plan = fabsf(delta);                         // what odometry still thinks is left
  slowDrive(true);
  bool settled = false;
  for (int pass = 0; pass <= SHIFT_MAX_PASSES; pass++) {
    sense(5);
    float predicted = expectF + plan;                // where the front SHOULD read right now
    bool useF = ok(usF) && fabsf(usF - predicted) <= FRONT_TRUST_CM;
    float move = useF ? (usF - expectF) : plan;
    Serial.printf("P1 shift %d : F=%6.2f predicted %6.2f target %6.2f -> %s, move %+.2f cm\n",
                  pass, shown(usF), predicted, expectF,
                  useF ? "front" : (ok(usF) ? "ODOMETRY (front off)" : "ODOMETRY (no echo)"), move);
    if (fabsf(move) <= POS_TOL_CM) { settled = true; break; }
    if (pass == SHIFT_MAX_PASSES) break;
    moveStraight(move);                              // negative = back off after an overshoot
    plan -= lastMoveAlong;
    if (aborted()) { slowDrive(false); return p1fail("aborted"); }
  }
  slowDrive(false);
  if (!settled) Serial.println("P1 WARN    : shift did not settle within POS_TOL_CM");
 
  // Before the second turn the cart is sideways: one side sensor looks at the back wall.
  // Turned left, the back wall is on the cart's right; turned right, it is on the left.
  sense(5);
  float backRead = (t < 0) ? usR : usL;
  float nominalV = APPROACH_FRONT_CM + FRONT_OFFSET_CM;
  if (ok(backRead)) {
    p1BackWallCm = backRead + S2;
    if (DO_APPROACH)
      Serial.printf("P1 backwall: %s sensor %.2f -> axle %.2f cm from the back wall (nominal %.2f, diff %+.2f)\n",
                    (t < 0) ? "right" : "left", backRead, p1BackWallCm, nominalV, p1BackWallCm - nominalV);
    else
      Serial.printf("P1 backwall: %s sensor %.2f -> axle %.2f cm from the back wall (no approach, so this varies run to run)\n",
                    (t < 0) ? "right" : "left", backRead, p1BackWallCm);
  } else if (DO_APPROACH) {
    p1BackWallCm = nominalV;
    Serial.printf("P1 backwall: no echo, assuming nominal %.2f\n", nominalV);
  } else {
    p1BackWallCm = -1.0f;
    Serial.println("P1 backwall: NO ECHO and no approach was done - the distance from the back wall is UNKNOWN");
  }
 
  // The axle should not move during a spin in place, but the pivot wanders a little on a
  // real surface. Note where the cart is, and which way "away from the back wall" points,
  // so the measurement above can be carried through the turn.
  odomUpdate();
  float pvx = poseX, pvy = poseY;
  float awayDeg = (t < 0) ? (poseH - 90.0f) : (poseH + 90.0f);   // back wall is on the right
  float nx = cosf(awayDeg * DEG_TO_RAD), ny = sinf(awayDeg * DEG_TO_RAD);
 
  turnExact(t, P1_TURN_TOL_DEG, P1_TURN_TRIES);
 
  odomUpdate();
  float driftOut = (poseX - pvx) * nx + (poseY - pvy) * ny;      // + = drifted away from it
  p1BackWallCm += driftOut;
  Serial.printf("P1 backwall: after the final turn the axle is %.2f cm from the back wall "
                "(the turn moved it %+.2f cm; nominal %.2f, diff %+.2f)\n",
                p1BackWallCm, driftOut, nominalV, p1BackWallCm - nominalV);
 
  // ---- 5. verify against both walls, now facing the arena ----
  sense(5);
  float expL = TARGET - S2, expR = (W - TARGET) - S2;
  float off = (ok(usL) && ok(usR)) ? 0.5f * ((usL - expL) - (usR - expR)) : 0.0f;
  Serial.printf("P1 final   : L=%6.2f (want %.2f)  R=%6.2f (want %.2f)  sum=%6.2f\n",
                shown(usL), expL, shown(usR), expR,
                (ok(usL) && ok(usR)) ? usL + usR + SIDE_SPAN_CM : -1.0f);
  Serial.printf("P1 error   : sideways off the T1 line %+.2f cm\n", off);
  if (ok(usL) && ok(usR)) p1RightWallCm = 0.5f * (W + usL - usR);   // walls swapped sides
  else p1RightWallCm = TARGET;
 
  // ---- 6. this pose is the origin for everything that follows ----
  noInterrupts(); encL = 0; encR = 0; interrupts();
  odoLastL = 0; odoLastR = 0;
  poseX = poseY = poseH = 0;
  frontX = FRONT_POINT_CM; frontY = 0;
  Serial.printf("P1 origin  : axle (0,0) heading 0; measured %.2f cm from the back wall, %.2f from the right-hand wall\n",
                p1BackWallCm, p1RightWallCm);
  Serial.println("P1 DONE");
  quietMoves = false;
  return true;
}
 
// =====================================================================
// PART 3 - follow the black line
// Assumes the cart is already on the line. Runs until the line is lost for LINE_LOST_MS,
// until LINE_RUN_CM has been covered, or until any character arrives on the serial port.
// =====================================================================
bool irBlack(uint8_t pin) { return (digitalRead(pin) == HIGH) == (IR_BLACK_HIGH != 0); }
 
bool part3() {
  odomUpdate();
  float x0 = poseX, y0 = poseY;
  float saved = CRUISE_CMPS;                         // part 3 sets wheel speeds directly
  quietMoves = !P1_SHOW_MOVES;
 
  Serial.println("P3 ======== PART 3: FOLLOW THE LINE ========");
  Serial.printf("P3 config  : speed %.1f, inner %.1f gentle / %.1f sharp, sharp after %d ms\n",
                LINE_CMPS, LINE_SLOW_CMPS, LINE_SHARP_CMPS, SHARP_TURN_MS);
  Serial.printf("P3 config  : black is %s, lost after %d ms, run %.0f cm (0 = no limit)\n",
                IR_BLACK_HIGH ? "HIGH" : "LOW", LINE_LOST_MS, LINE_RUN_CM);
  Serial.println("P3 running : send any character to stop");
 
  int lastSide = 0;
  uint32_t sideSince = millis(), lostSince = 0, lastPrint = 0;
  const char* why = "stopped by hand";
 
  while (true) {
    tick();
    if (aborted()) break;
 
    bool l = irBlack(PIN_IR_L), r = irBlack(PIN_IR_R);
    // -1 = line under the left sensor only, +1 = right only, 0 = both or neither
    int side = (l && !r) ? -1 : (r && !l) ? 1 : 0;
    if (side != lastSide) { lastSide = side; sideSince = millis(); }
    bool sharp = (side != 0) && (millis() - sideSince > (uint32_t)SHARP_TURN_MS);
    float inner = sharp ? LINE_SHARP_CMPS : LINE_SLOW_CMPS;
 
    if (side < 0)      speedSet(inner, LINE_CMPS);   // drifted right: slow the left wheel
    else if (side > 0) speedSet(LINE_CMPS, inner);   // drifted left:  slow the right wheel
    else               speedSet(LINE_CMPS, LINE_CMPS);
 
    // Neither sensor on the line for too long means the line has run out or been lost.
    // Both sensors on it is a junction or a thick patch, which is not "lost".
    if (!l && !r) {
      if (lostSince == 0) lostSince = millis();
      if (LINE_LOST_MS > 0 && millis() - lostSince > (uint32_t)LINE_LOST_MS) { why = "line lost"; break; }
    } else lostSince = 0;
 
    odomUpdate();
    float gone = hypotf(poseX - x0, poseY - y0);
    if (LINE_RUN_CM > 0 && gone >= LINE_RUN_CM) { why = "reached LINE_RUN_CM"; break; }
 
    if (millis() - lastPrint >= (uint32_t)LINE_PRINT_MS) {
      lastPrint = millis();
      Serial.printf("P3 line    : L=%d R=%d  %-6s %-6s  %.1f cm  x=%.1f y=%.1f h=%.1f\n",
                    l, r, side < 0 ? "LEFT" : side > 0 ? "RIGHT" : (l && r) ? "BOTH" : "none",
                    sharp ? "sharp" : "", gone, poseX, poseY, poseH);
    }
    delayMicroseconds(1500);
  }
 
  speedStop();
  waitMs((uint32_t)SETTLE_MS);
  odomUpdate();
  CRUISE_CMPS = saved;
  quietMoves = false;
  Serial.printf("P3 DONE    : %s after %.1f cm, at x=%.1f y=%.1f h=%.1f\n",
                why, hypotf(poseX - x0, poseY - y0), poseX, poseY, poseH);
  return true;
}
 
// =====================================================================
// TESTS
// =====================================================================
// Closed loop at a fixed target speed, printing target vs actual.
void velRun(int which, float cmps, uint32_t ms) {     // which: 0 both, 1 left, 2 right
  odomUpdate();
  float h0 = poseH;
  float tl = (which == 2) ? 0.0f : cmps;
  float tr = (which == 1) ? 0.0f : cmps;
  speedSet(tl, tr);
  uint32_t t0 = millis(), lastPrint = 0;
  float accL = 0, accR = 0; int accN = 0;
  while (millis() - t0 < ms) {
    tick();
    if (aborted()) break;
    uint32_t el = millis() - t0;
    if (el > ms / 2) { accL += spMeasL; accR += spMeasR; accN++; }
    uint32_t every = (el < 600) ? 50 : 200;          // fine detail during the start transient
    if (millis() - lastPrint >= every) {
      lastPrint = millis();
      Serial.printf("  t=%4lu ms  L target %.1f actual %6.1f (pwm %4d) | R target %.1f actual %6.1f (pwm %4d)\n",
                    (unsigned long)(millis() - t0), tl, spMeasL, (int)spOutL, tr, spMeasR, (int)spOutR);
    }
    delayMicroseconds(1500);
  }
  speedStop();
  waitMs((uint32_t)SETTLE_MS);
  odomUpdate();
  if (accN > 0) {
    float mL = accL / accN, mR = accR / accN;
    Serial.printf("SETTLED (second half): L %.2f cm/s (%+.0f%% of target)  R %.2f cm/s (%+.0f%%)\n",
                  mL, (fabsf(tl) > 0.1f) ? 100.0f * (mL - tl) / tl : 0.0f,
                  mR, (fabsf(tr) > 0.1f) ? 100.0f * (mR - tr) / tr : 0.0f);
    Serial.println("More than about 10% off target means the feedforward for that wheel is wrong (see fffit).");
  }
  Serial.printf("heading change %+.2f deg\n", wrap180(poseH - h0));
  printPose();
}
 
// OPEN LOOP on purpose: both motors get the same PWM so the raw PWM-to-speed relationship
// of each one can be measured. The wheels running at different speeds here is the expected
// result, and is exactly what the two feedforward constants below are for.
void ffCal(int pwm) {
  if (pwm < 20) pwm = 150;
  setLeftRaw(pwm); setRightRaw(pwm);
  waitMs(600);                                        // spin up
  int32_t l0 = encL, r0 = encR; uint32_t t0 = millis();
  waitMs(1500);
  uint32_t t1 = millis(); int32_t l1 = encL, r1 = encR;
  stopMotors(); waitMs((uint32_t)SETTLE_MS);
  float dt = (t1 - t0) / 1000.0f; if (dt < 0.01f) dt = 0.01f;
  float vl = (l1 - l0) * cmPerCountL() / dt, vr = (r1 - r0) * cmPerCountR() / dt;
  Serial.printf("open loop at PWM %d: left %.1f cm/s, right %.1f cm/s (different speeds here are normal)\n", pwm, vl, vr);
  if (fabsf(vl) > 1 && fabsf(vr) > 1) {
    Serial.printf("suggested FF_PWM_PER_CMPS_L = %.2f\n", pwm / fabsf(vl));
    Serial.printf("suggested FF_PWM_PER_CMPS_R = %.2f\n", pwm / fabsf(vr));
    Serial.printf("the slower wheel caps the cart: keep CRUISE_CMPS below about %.0f cm/s\n", 0.8f * fminf(fabsf(vl), fabsf(vr)));
  } else Serial.println("one or both wheels did not move - check the motors first");
}
 
// Run both motors open loop at one PWM and return each wheel's steady speed.
void measureOpen(int pwm, float& vl, float& vr) {
  setLeftRaw(pwm); setRightRaw(pwm);
  waitMs(700);                                        // spin up, not measured
  int32_t l0 = encL, r0 = encR; uint32_t t0 = millis();
  waitMs(1200);
  uint32_t t1 = millis(); int32_t l1 = encL, r1 = encR;
  stopMotors(); waitMs((uint32_t)SETTLE_MS);
  float dt = (t1 - t0) / 1000.0f; if (dt < 0.01f) dt = 0.01f;
  vl = (l1 - l0) * cmPerCountL() / dt;
  vr = (r1 - r0) * cmPerCountR() / dt;
  Serial.printf("  PWM %3d -> L %6.2f cm/s   R %6.2f cm/s\n", pwm, vl, vr);
}
 
// Two-point fit of the whole feedforward line, per wheel:  pwm = slope * speed + offset.
// ffcal only ever measured one point and assumed the line passes through the origin, which
// is wrong for a motor with a dead-band and leaves the offset guessed. Needs about 1.5 m of
// clear floor, and must be run on the surface the cart will drive on.
void ffFit(int pwmA, int pwmB) {
  if (pwmA < 40 || pwmA > 255) pwmA = 110;
  if (pwmB < 40 || pwmB > 255) pwmB = 160;
  if (pwmB <= pwmA + 20) { Serial.println("use two PWMs at least 20 apart, e.g.  fffit 110 160"); return; }
  float vaL, vaR, vbL, vbR;
  Serial.println("two-point feedforward fit (keep 1.5 m of floor clear):");
  measureOpen(pwmA, vaL, vaR);
  if (aborted()) { Serial.println("aborted"); return; }
  measureOpen(pwmB, vbL, vbR);
 
  float dvL = vbL - vaL, dvR = vbR - vaR;
  if (fabsf(dvL) < 1.0f || fabsf(dvR) < 1.0f) { Serial.println("speeds too close together - try a wider PWM pair"); return; }
  float slopeL = (pwmB - pwmA) / dvL, offL = pwmA - slopeL * vaL;
  float slopeR = (pwmB - pwmA) / dvR, offR = pwmA - slopeR * vaR;
  Serial.printf("FF_PWM_PER_CMPS_L = %.2f   FF_MIN_PWM_L = %.1f\n", slopeL, offL);
  Serial.printf("FF_PWM_PER_CMPS_R = %.2f   FF_MIN_PWM_R = %.1f\n", slopeR, offR);
  Serial.printf("check: at 5 cm/s that predicts PWM %.0f (L) and %.0f (R); at 8 cm/s, %.0f and %.0f\n",
                slopeL * 5 + offL, slopeR * 5 + offR, slopeL * 8 + offL, slopeR * 8 + offR);
  Serial.printf("the slower wheel caps the cart: keep CRUISE_CMPS below about %.0f cm/s\n",
                0.8f * fminf(fabsf(vbL), fabsf(vbR)));
}
 
// Measures BOTH dead-band numbers per wheel: ramp up until it starts (static friction),
// then ramp back down until it stops (kinetic friction). They are usually far apart.
void deadBand() {
  Serial.println("ramping each motor up then down (keep the cart clear)...");
  for (int sdx = 0; sdx < 2; sdx++) {
    bool left = (sdx == 0);
    const char* name = left ? "LEFT" : "RIGHT";
    int up = 10; bool moved = false;
    for (; up <= 255 && !aborted(); up += 2) {       // find the breakaway PWM
      int32_t c0 = left ? encL : encR;
      if (left) setLeftRaw(up); else setRightRaw(up);
      waitMs(120);
      if (labs((long)((left ? encL : encR) - c0)) >= 3) { moved = true; break; }
    }
    if (!moved) { stopMotors(); Serial.printf("%s: no movement detected\n", name); continue; }
 
    int down = up; bool stopped = false;
    for (; down >= 10 && !aborted(); down -= 2) {    // now find where it stops again
      int32_t c0 = left ? encL : encR;
      if (left) setLeftRaw(down); else setRightRaw(down);
      waitMs(150);
      if (labs((long)((left ? encL : encR) - c0)) < 3) { stopped = true; break; }
    }
    stopMotors(); waitMs(300);
    Serial.printf("%s: starts at PWM %d, keeps turning down to PWM %d\n", name, up, stopped ? down + 2 : 10);
    Serial.printf("  -> FF_BREAK_PWM_%c = %d    FF_MIN_PWM_%c = %d\n",
                  left ? 'L' : 'R', up, left ? 'L' : 'R', stopped ? down + 2 : 10);
  }
  Serial.println("BREAK is a floor used only while the wheel is stopped; MIN is an offset added while it runs.");
}
 
void pwmTest(bool left) {
  const int duty[5] = {0, 64, 128, 192, 255};
  Serial.printf("%s motor, 1.5 s per step:\n", left ? "LEFT" : "RIGHT");
  for (int i = 0; i < 5 && !aborted(); i++) {
    int32_t c0 = left ? encL : encR;
    uint32_t t0 = millis();
    if (left) setLeftRaw(duty[i]); else setRightRaw(duty[i]);
    waitMs(1500);
    long dc = (long)((left ? encL : encR) - c0);
    float dt = (millis() - t0) / 1000.0f; if (dt < 0.01f) dt = 0.01f;
    Serial.printf("  duty %3d -> %6ld counts (%.0f counts/s)%s\n", duty[i], dc, dc / dt,
                  (duty[i] == 0 && labs(dc) > 20) ? "   <-- MOVING WHILE DISABLED" : "");
  }
  stopMotors();
  Serial.println("Counts should rise with duty, and duty 0 should give about zero.");
}
 
// =====================================================================
// COMMANDS
// =====================================================================
void showVars() {
  Serial.printf("INVERT_PWM=%d INVERT_DIR=%d LEFT_FWD_SWAP=%d RIGHT_FWD_SWAP=%d\n", INVERT_PWM, INVERT_DIR, LEFT_FWD_SWAP, RIGHT_FWD_SWAP);
  Serial.printf("ENC_L_INVERT=%d ENC_R_INVERT=%d USE_PWM=%d PWM_FREQ_HZ=%d BRAKE_ON_STOP=%d\n", ENC_L_INVERT, ENC_R_INVERT, USE_PWM, PWM_FREQ_HZ, BRAKE_ON_STOP);
  Serial.printf("COUNTS_PER_REV_L=%.1f  COUNTS_PER_REV_R=%.1f\n", COUNTS_PER_REV_L, COUNTS_PER_REV_R);
  Serial.printf("WHEEL_DIAMETER_L=%.3f  WHEEL_DIAMETER_R=%.3f  TRACK_WIDTH_CM=%.3f  FRONT_POINT_CM=%.2f\n",
                WHEEL_DIAMETER_L, WHEEL_DIAMETER_R, TRACK_WIDTH_CM, FRONT_POINT_CM);
  Serial.printf("SPEED_CONTROL=%d CRUISE_CMPS=%.1f TURN_CMPS=%.1f MIN_CMPS=%.1f\n", SPEED_CONTROL, CRUISE_CMPS, TURN_CMPS, MIN_CMPS);
  Serial.printf("FF_PWM_PER_CMPS_L=%.2f FF_PWM_PER_CMPS_R=%.2f\n", FF_PWM_PER_CMPS_L, FF_PWM_PER_CMPS_R);
  Serial.printf("FF_BREAK_PWM_L=%.0f FF_BREAK_PWM_R=%.0f FF_MIN_PWM_L=%.0f FF_MIN_PWM_R=%.0f BREAK_SPEED_CMPS=%.2f\n",
                FF_BREAK_PWM_L, FF_BREAK_PWM_R, FF_MIN_PWM_L, FF_MIN_PWM_R, BREAK_SPEED_CMPS);
  Serial.printf("KP_SPEED=%.2f KI_SPEED=%.2f SPEED_DT_MS=%d SPEED_FILTER=%.2f\n", KP_SPEED, KI_SPEED, SPEED_DT_MS, SPEED_FILTER);
  Serial.printf("KP_DIST_CMPS=%.2f STOP_EARLY_CM=%.2f KP_TURN_CMPS=%.2f KI_TURN_CMPS=%.2f TURN_STALL_DEG_S=%.1f MAX_TURN_BOOST_CMPS=%.1f\n",
                KP_DIST_CMPS, STOP_EARLY_CM, KP_TURN_CMPS, KI_TURN_CMPS, TURN_STALL_DEG_S, MAX_TURN_BOOST_CMPS);
  Serial.printf("KP_HEADING_CMPS=%.2f MAX_HEADING_CMPS=%.1f KP_CROSS_DEG_PER_CM=%.2f MAX_CROSS_DEG=%.1f FF_REV_SCALE=%.2f\n",
                KP_HEADING_CMPS, MAX_HEADING_CMPS, KP_CROSS_DEG_PER_CM, MAX_CROSS_DEG, FF_REV_SCALE);
  Serial.printf("open loop: BASE_SPEED=%d TURN_SPEED=%d MIN_SPEED=%d TRIM_L=%.3f TRIM_R=%.3f KP_TURN=%.2f KP_HEADING=%.2f\n",
                BASE_SPEED, TURN_SPEED, MIN_SPEED, TRIM_L, TRIM_R, KP_TURN, KP_HEADING);
  Serial.printf("HEADING_HOLD=%d SQUARE_UP=%d SQUARE_UP_TOL_DEG=%.2f MAX_STEER=%d TURN_TOL_DEG=%.2f SETTLE_MS=%d ACTIVE_HOLD_MS=%d\n",
                HEADING_HOLD, SQUARE_UP, SQUARE_UP_TOL_DEG, MAX_STEER, TURN_TOL_DEG, SETTLE_MS, ACTIVE_HOLD_MS);
  Serial.printf("cm per count: L=%.6f  R=%.6f\n", cmPerCountL(), cmPerCountR());
  Serial.printf("US_SCALE=%.6f SIDE_SPAN_CM=%.2f FRONT_OFFSET_CM=%.2f PING_GAP_MS=%d\n", US_SCALE, SIDE_SPAN_CM, FRONT_OFFSET_CM, PING_GAP_MS);
  Serial.printf("START_W_INNER=%.1f BOUNDARY_GAP_CM=%.2f T1_FROM_LEFT_CM=%.1f APPROACH_FRONT_CM=%.1f\n",
                START_W_INNER, BOUNDARY_GAP_CM, T1_FROM_LEFT_CM, APPROACH_FRONT_CM);
  Serial.printf("SEARCH_STEP_DEG=%.1f SQUARE_TOL_CM=%.1f FRONT_DETECT_CM=%.1f POS_TOL_CM=%.2f FRONT_TRUST_CM=%.1f BACKOFF_CM=%.1f\n",
                SEARCH_STEP_DEG, SQUARE_TOL_CM, FRONT_DETECT_CM, POS_TOL_CM, FRONT_TRUST_CM, BACKOFF_CM);
  Serial.printf("align: do=%d drive=%.1f converge=%.1f passes=%d coarsemin=%.1f\n",
                DO_ALIGN, ALIGN_DRIVE_CM, ALIGN_CONVERGE_DEG, ALIGN_MAX_PASSES, COARSE_MIN_DEG);
  Serial.printf("P1 turns: tol=%.1f tries=%d\n", P1_TURN_TOL_DEG, P1_TURN_TRIES);
  Serial.printf("speeds: SCAN_TURN_CMPS=%.1f APPROACH_CMPS=%.1f FRICTION_PWM=%.1f HOLD_DEAD_CMPS=%.2f\n",
                SCAN_TURN_CMPS, APPROACH_CMPS, FRICTION_PWM, HOLD_DEAD_CMPS);
  Serial.printf("escape: side<%.1f front<%.1f (sensor to wall) move %.1f turnback %d max %d\n",
                ESCAPE_SIDE_CM, ESCAPE_FRONT_CM, BACKOFF_CM, ESCAPE_TURN_BACK, ESCAPE_MAX);
  Serial.printf("line: IR_BLACK_HIGH=%d LINE_CMPS=%.1f slow=%.1f sharp=%.1f SHARP_TURN_MS=%d LOST_MS=%d RUN_CM=%.0f\n",
                IR_BLACK_HIGH, LINE_CMPS, LINE_SLOW_CMPS, LINE_SHARP_CMPS, SHARP_TURN_MS, LINE_LOST_MS, LINE_RUN_CM);
}
 
void help() {
  Serial.println(
    "show | enc | zero | where | cpr N | stop\n"
    "pose X Y H         set the pose; X Y are the FRONT point, H in degrees\n"
    "part1              localise in the start region and park on the T1 line\n"
    "part3              follow the black line\n"
    "ir [pings] [ms]    read both line sensors, default 10 pings 250 ms apart\n"
    "drive CMPS CM      straight move at a chosen speed (CM negative = backwards)\n"
    "dist [n] | calus F|L|R REAL_CM\n"
    "m l|r PWM MS       open loop, one motor at a fixed PWM\n"
    "pwmtest l|r        step the enable through 0/64/128/192/255\n"
    "vel [l|r] CMPS MS  closed loop at a target speed, prints target vs actual\n"
    "ffcal PWM          suggests FF_PWM_PER_CMPS (one point, assumes no dead-band)\n"
    "fffit PWMA PWMB    two-point fit: FF_PWM_PER_CMPS_* AND FF_MIN_PWM_* (use this)\n"
    "dband              measures both dead-bands -> FF_BREAK_PWM_*, FF_MIN_PWM_*\n"
    "fwd CM | back CM | spin DEG | arc l|r DEG\n"
    "any character while moving = abort");
}
 
char* tok[6]; int ntok = 0;
float argf(int i, float d) { return (i < ntok) ? (float)atof(tok[i]) : d; }
bool isL(int i) { return (i < ntok) && (tok[i][0] == 'l' || tok[i][0] == 'L'); }
bool isSide(int i) { return (i < ntok) && strlen(tok[i]) == 1 && (tolower(tok[i][0]) == 'l' || tolower(tok[i][0]) == 'r'); }
 
void handleCommand(char* line) {
  ntok = 0;
  for (char* p = strtok(line, " \t,"); p && ntok < 6; p = strtok(NULL, " \t,")) tok[ntok++] = p;
  if (!ntok) return;
  for (char* ch = tok[0]; *ch; ch++) *ch = tolower(*ch);
  abortFlag = false;
  const char* c = tok[0];
 
  if (!strcmp(c, "help") || !strcmp(c, "?")) help();
  else if (!strcmp(c, "show")) showVars();
  else if (!strcmp(c, "zero")) {
    noInterrupts(); encL = 0; encR = 0; interrupts();
    odoLastL = 0; odoLastR = 0;
    poseX = poseY = poseH = 0;                  // axle at the origin, facing +X
    frontX = FRONT_POINT_CM; frontY = 0;
    Serial.println("counts and pose reset (axle at the origin)");
    printPose();
  }
  else if (!strcmp(c, "pose")) {
    setPose(argf(1, 0), argf(2, 0), argf(3, 0));
    Serial.println("pose set (X Y are the FRONT point)");
    printPose();
  }
  else if (!strcmp(c, "where")) {
    odomUpdate();
    Serial.printf("counts L=%ld R=%ld = %.3f / %.3f revolutions\n", (long)encL, (long)encR, encL / COUNTS_PER_REV_L, encR / COUNTS_PER_REV_R);
    Serial.printf("wheels L=%.2f cm R=%.2f cm\n", wheelDistL(), wheelDistR());
    printPose();
  }
  else if (!strcmp(c, "enc")) {
    Serial.println("turn the wheels FORWARD by hand - both counts must go UP (10 s)");
    for (int i = 0; i < 40 && !aborted(); i++) { Serial.printf("L=%ld R=%ld\n", (long)encL, (long)encR); waitMs(250); }
  }
  else if (!strcmp(c, "cpr")) {
    float n = argf(1, 1);
    if (n < 0.5f) { Serial.println("usage: zero, turn each wheel N revolutions by hand, then  cpr N"); return; }
    Serial.printf("over %.0f revolutions:  COUNTS_PER_REV_L = %.1f   COUNTS_PER_REV_R = %.1f\n",
                  n, fabsf((float)encL) / n, fabsf((float)encR) / n);
    Serial.println("copy those two numbers into the TUNING block and re-flash");
  }
  else if (!strcmp(c, "m") && ntok > 2) {
    bool left = isL(1);
    int pwm = (int)argf(2, 0);
    uint32_t ms = (uint32_t)argf(3, 1000);
    const uint32_t SPINUP_MS = 300;                   // excluded, so this is steady-state speed
    odomUpdate();
    float hStart = poseH;
    if (left) setLeft(pwm); else setRight(pwm);
    waitMs(SPINUP_MS);
    uint32_t t0 = millis(); int32_t c0 = left ? encL : encR;
    waitMs(ms);
    uint32_t t1 = millis(); int32_t c1 = left ? encL : encR;
    stopMotors();
    waitMs((uint32_t)SETTLE_MS);
    odomUpdate();
    float dt = (t1 - t0) / 1000.0f; if (dt < 0.01f) dt = 0.01f;
    long dc = (long)(c1 - c0);
    float cm = dc * (left ? cmPerCountL() : cmPerCountR());
    Serial.printf("%s PWM %d, %.2f s at speed: %ld counts, %.2f cm, %.1f cm/s, %.2f rev/s\n",
                  left ? "LEFT" : "RIGHT", pwm, dt, dc, cm, cm / dt,
                  dc / dt / (left ? COUNTS_PER_REV_L : COUNTS_PER_REV_R));
    Serial.printf("heading change %+.2f deg\n", wrap180(poseH - hStart));
    printPose();
    if (dc == 0) Serial.println("no counts: check the encoder wiring, or the PWM is below the dead-band");
    if (dc < 0)  Serial.println("counts went DOWN on a positive PWM: flip that side's ENC_*_INVERT or *_FWD_SWAP");
  }
  else if (!strcmp(c, "part1") || !strcmp(c, "part")) part1();
  else if (!strcmp(c, "part3")) part3();
  else if (!strcmp(c, "ir")) {                      // ir [pings] [ms between]
    int n = (int)argf(1, 10); if (n < 1) n = 1; if (n > 200) n = 200;
    uint32_t gap = (uint32_t)argf(2, 250);
    int blackL = 0, blackR = 0;
    Serial.printf("IR on GP%d (left) and GP%d (right), %d pings, %lu ms apart\n",
                  PIN_IR_L, PIN_IR_R, n, (unsigned long)gap);
    Serial.printf("IR_BLACK_HIGH=%d, so a %s pin reads as BLACK\n", IR_BLACK_HIGH, IR_BLACK_HIGH ? "HIGH" : "LOW");
    for (int i = 1; i <= n && !aborted(); i++) {
      int rawL = (digitalRead(PIN_IR_L) == HIGH);
      int rawR = (digitalRead(PIN_IR_R) == HIGH);
      bool bL = irBlack(PIN_IR_L), bR = irBlack(PIN_IR_R);
      if (bL) blackL++;
      if (bR) blackR++;
      const char* where = (bL && bR) ? "both on the line" : bL ? "line on the LEFT"
                        : bR ? "line on the RIGHT" : "neither sees the line";
      Serial.printf("  ping %3d  raw L=%d R=%d  ->  L %-5s R %-5s   %s\n",
                    i, rawL, rawR, bL ? "BLACK" : "white", bR ? "BLACK" : "white", where);
      if (i < n) waitMs(gap);
    }
    Serial.printf("IR summary : LEFT black on %d of %d, RIGHT black on %d of %d\n", blackL, n, blackR, n);
    Serial.println("Move the cart over the line and off it while this runs. If black shows as");
    Serial.println("white and vice versa, set IR_BLACK_HIGH to the other value.");
  }
  else if (!strcmp(c, "dist")) {
    int n = (int)argf(1, 5);
    for (int i = 0; i < n && !aborted(); i++) {
      sense(3);
      Serial.printf("F=%6.1f L=%6.1f R=%6.1f  | L+R+span=%6.1f  | front from axle=%6.1f\n",
                    shown(usF), shown(usL), shown(usR),
                    (ok(usL) && ok(usR)) ? usL + usR + SIDE_SPAN_CM : -1.0f,
                    ok(usF) ? usF + FRONT_OFFSET_CM : -1.0f);
    }
  }
  else if (!strcmp(c, "calus") && ntok > 2) {
    char w = tolower(tok[1][0]); float real = argf(2, 0), acc = 0; int cnt = 0;
    for (int i = 0; i < 10; i++) {
      sense(1);
      float v = (w == 'f') ? usF : (w == 'l') ? usL : usR;
      if (ok(v)) { acc += v; cnt++; }
    }
    if (cnt >= 5 && real > 5) { float meas = acc / cnt; US_SCALE *= real / meas; Serial.printf("measured %.2f, real %.2f -> US_SCALE = %.6f\n", meas, real, US_SCALE); }
    else Serial.println("not enough valid readings, or the distance is too small");
  }
  else if (!strcmp(c, "pwmtest")) pwmTest(isL(1));
  else if (!strcmp(c, "dband")) deadBand();
  else if (!strcmp(c, "vel")) {
    bool one = isSide(1);
    int which = one ? (isL(1) ? 1 : 2) : 0;
    float cmps = argf(one ? 2 : 1, 20.0f);
    uint32_t ms = (uint32_t)argf(one ? 3 : 2, 3000);
    velRun(which, cmps, ms);
  }
  else if (!strcmp(c, "ffcal")) ffCal((int)argf(1, 150));
  else if (!strcmp(c, "fffit")) ffFit((int)argf(1, 110), (int)argf(2, 160));
  else if (!strcmp(c, "drive")) {                    // drive CMPS CM  (CM may be negative)
    if (ntok < 3) { Serial.println("usage: drive CMPS CM   e.g.  drive 8 5   (CM negative = backwards)"); }
    else {
      float saved = CRUISE_CMPS;
      CRUISE_CMPS = fabsf(argf(1, CRUISE_CMPS));
      Serial.printf("drive: %.2f cm at %.1f cm/s\n", argf(2, 5), CRUISE_CMPS);
      moveStraight(argf(2, 5));
      CRUISE_CMPS = saved;
    }
  }
  else if (!strcmp(c, "fwd"))  moveStraight(fabsf(argf(1, 50)));
  else if (!strcmp(c, "back")) moveStraight(-fabsf(argf(1, 50)));
  else if (!strcmp(c, "spin")) turnBy(argf(1, 90), 0);
  else if (!strcmp(c, "arc") && ntok > 2) turnBy(argf(2, 90), isL(1) ? 1 : 2);
  else if (!strcmp(c, "stop")) { speedStop(); Serial.println("braked"); }
  else Serial.println("unknown command - type help");
}
 
// =====================================================================
void setup() {
  const uint8_t outs[] = {PIN_ENA, PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, PIN_ENB};
  for (uint8_t i = 0; i < sizeof(outs); i++) pinMode(outs[i], OUTPUT);
  analogWriteRange(255);
  analogWriteFreq((uint32_t)PWM_FREQ_HZ);
  stopMotors();
 
  pinMode(PIN_IR_L, INPUT); pinMode(PIN_IR_R, INPUT);
  pinMode(PIN_TRIG_F, OUTPUT); digitalWrite(PIN_TRIG_F, LOW);
  pinMode(PIN_TRIG_L, OUTPUT); digitalWrite(PIN_TRIG_L, LOW);
  pinMode(PIN_TRIG_R, OUTPUT); digitalWrite(PIN_TRIG_R, LOW);
  pinMode(PIN_ECHO_F, INPUT); pinMode(PIN_ECHO_L, INPUT); pinMode(PIN_ECHO_R, INPUT);
  pinMode(PIN_ENC_L_A, INPUT_PULLUP); pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP); pinMode(PIN_ENC_R_B, INPUT_PULLUP);
  qPrevL = ((digitalRead(PIN_ENC_L_A) == HIGH) << 1) | (digitalRead(PIN_ENC_L_B) == HIGH);
  qPrevR = ((digitalRead(PIN_ENC_R_A) == HIGH) << 1) | (digitalRead(PIN_ENC_R_B) == HIGH);
  attachInterrupt(PIN_ENC_L_A, isrL, CHANGE); attachInterrupt(PIN_ENC_L_B, isrL, CHANGE);
  attachInterrupt(PIN_ENC_R_A, isrR, CHANGE); attachInterrupt(PIN_ENC_R_B, isrR, CHANGE);
 
  Serial.begin(115200);
  uint32_t t = millis();
  while (!Serial && millis() - t < 3000) {}
  Serial.println("\n=== CART v6.6 (closed-loop speed) === type help");
}
 
void loop() {
  static char buf[64]; static int n = 0;
  tick();
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') { if (n) { buf[n] = 0; n = 0; handleCommand(buf); Serial.println("> ok"); } }
    else if (n < (int)sizeof(buf) - 1) buf[n++] = ch;
  }
}
 