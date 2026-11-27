// Physics simulator for speed_race.ino.
// Compiles the REAL firmware file and runs it against a model of the car,
// the round wall, the L298N + opto-inverted PWM, the encoders and HC-SR04s.
//
//   g++ -O2 -std=c++17 -Isim sim/sim.cpp -o simrun && ./simrun [options]
//
// Options (all optional):
//   side=R|L        which side the wall is on when the car is placed
//   R=108           true wall radius (cm)
//   bump=0          wall waviness amplitude (cm)
//   panels=0        >0: wall made of this many panels
//   flat=1          how flat each panel is (1 = straight boards, 0.3 = mostly bent)
//   vl=62 vr=72     top speed of left/right wheel at full power (cm/s)
//   d0=0.08         motor dead-band (fraction of full power)
//   tau=0.08        motor time constant (s)
//   track=19.3      true wheel track (cm)
//   z0=7 psi0=0     start: side reading (cm) and angle away from wall (deg)
//   noise=0.15      ultrasonic noise (cm, + 1% of distance)
//   drop=0.02 spike=0.01   ultrasonic dropouts / multipath spikes
//   cone=20         ultrasonic half-cone (deg)
//   yawn=0.0015     random yaw disturbance per ms^0.5 (rad)
//   seed=1  trace=file.csv  verbose=1
#include "Arduino.h"
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <random>
#include <algorithm>
#include <map>

// ------------------------------------------------------------- config
static std::map<std::string, std::string> *gArgs;
static double P(const char *k, double def);
static char sideCfg = 'R';
static double R_WALL, BUMP, VL, VR, D0, TAU, TRACK_ACT, NOISE, DROP, SPIKE, CONE, YAWN;
static int PANELS;
static double FLAT = 1;
static FILE *traceF = nullptr;

// ------------------------------------------------------------- world
static uint64_t simT = 0;            // microseconds
static const uint64_t PHYS_DT = 50;  // us
static uint64_t nextPhys = 0;
static int pinLevel[32];
static int pwmVal[32];
static void (*isr[32])() = {nullptr};
static std::mt19937 rng;
static std::normal_distribution<double> gauss(0.0, 1.0);
static std::uniform_real_distribution<double> uni(0.0, 1.0);

struct Pose { double x, y, th; } pose;
static double vR = 0, vL = 0;          // wheel surface speeds cm/s
static double accR = 0, accL = 0;      // accumulated wheel distance -> encoder edges
static long edgesR = 0, edgesL = 0;
static int quadR = 0, quadL = 0;
static const double TPC_R_ACT = 60.7, TPC_L_ACT = 60.5;

// wall polyline
static std::vector<double> wx, wy, nx, ny;
static double wallRadius(double phi) {
  if (PANELS > 0) {
    double seg = 2 * M_PI / PANELS;
    double a = fmod(phi + 10 * M_PI, seg) - seg / 2;  // angle from panel middle
    double poly = R_WALL * cos(seg / 2) / cos(a);    // vertices on the circle
    return FLAT * poly + (1 - FLAT) * R_WALL;        // FLAT=1: fully flat panels
  }
  return R_WALL + BUMP * sin(7 * phi + 0.3) + 0.5 * BUMP * sin(3 * phi + 1.1);
}
static void buildWall() {
  const int N = 2880;
  for (int i = 0; i < N; i++) {
    double phi = 2 * M_PI * i / N, r = wallRadius(phi);
    wx.push_back(r * cos(phi)); wy.push_back(r * sin(phi));
  }
  for (int i = 0; i < N; i++) {  // inward normal
    int a = (i + N - 1) % N, b = (i + 1) % N;
    double tx = wx[b] - wx[a], ty = wy[b] - wy[a], l = hypot(tx, ty);
    nx.push_back(-ty / l); ny.push_back(tx / l);  // CCW order -> left normal points inward
  }
}

// car points in car frame (x forward, y left), from the axle middle
struct Pt { double x, y; };
static const Pt body[] = {
    {3.5, 10.65}, {-3.5, 10.65}, {3.5, -10.65}, {-3.5, -10.65},  // wheel corners
    {16.5, 7.5},  {16.5, -7.5},  {-3.5, 7.5},   {-3.5, -7.5},    // chassis
    {13.0, -10.0}, {13.5, 10.0}, {18.0, 0.0}};                   // sensors (with depth)
static double clearanceNow() {
  double best = 1e9;
  for (auto &p : body) {
    double gx = pose.x + p.x * cos(pose.th) - p.y * sin(pose.th);
    double gy = pose.y + p.x * sin(pose.th) + p.y * cos(pose.th);
    best = std::min(best, wallRadius(atan2(gy, gx)) - hypot(gx, gy));
  }
  return best;
}

// ------------------------------------------------------------- ultrasonics
struct Sonar { int trig, echo; double fx, fy, ax, ay; uint64_t ps = 0, pe = 0; } sonar[3] = {
    {8, 9, 18.0, 0.0, 1, 0},       // FRONT
    {6, 7, 13.0, -9.0, 0, -1},     // RIGHT
    {18, 19, 13.5, 9.0, 0, 1}};    // LEFT
static double handOn = 0.5e6, handOff = 1.2e6;

static double trueEcho(const Sonar &s) {
  double c = cos(pose.th), sn = sin(pose.th);
  double px = pose.x + s.fx * c - s.fy * sn, py = pose.y + s.fx * sn + s.fy * c;
  double ax = s.ax * c - s.ay * sn, ay = s.ax * sn + s.ay * c;
  double cosCone = cos(CONE * M_PI / 180), cosSpec = cos(6 * M_PI / 180);
  double best = -1;
  for (size_t i = 0; i < wx.size(); i++) {
    double dx = wx[i] - px, dy = wy[i] - py, d = hypot(dx, dy);
    if (d > 400 || d < 1e-6) continue;
    if ((dx * ax + dy * ay) / d < cosCone) continue;
    if ((-dx * nx[i] - dy * ny[i]) / d < cosSpec) continue;  // not a mirror point
    if (best < 0 || d < best) best = d;
  }
  return best;
}

static void triggerFired(Sonar &s) {
  if (simT < s.pe) return;  // still busy: trigger ignored
  double d;
  if (&s == &sonar[0] && simT >= handOn && simT <= handOff) d = 5.0;
  else {
    d = trueEcho(s);
    if (d > 0) {
      if (uni(rng) < DROP) d = -1;
      else {
        d += gauss(rng) * (NOISE + 0.01 * d);
        if (uni(rng) < SPIKE) d += 5 + 75 * uni(rng);
        if (d < 2.0) d = 2.0 + uni(rng) * 3;  // too close: junk
      }
    }
  }
  s.ps = simT + 450;
  s.pe = s.ps + (d > 0 ? (uint64_t)(d / 0.01715) : 38000);
}

// ------------------------------------------------------------- physics
static double raceStart = -1, lapPhi0 = 0, lapPhiPrev = 0, lapPhiAcc = 0;
static double dist30 = 0, laps30 = -1, distAcc = 0, minClear = 1e9, sumAbsGap = 0;
static long contacts = 0, gapN = 0;
static double rcStart = 0;

static double motorTarget(int en, int ia, int ib, double vmax, int &brake) {
  double duty = 1.0 - pwmVal[en] / 255.0;          // opto-inverted PWM
  int a = !pinLevel[ia], b = !pinLevel[ib];          // opto-inverted direction pins
  brake = 0;
  if (a == b) { brake = duty > 0.5; return 0; }
  int dir = (a == 1 && b == 0) ? 1 : -1;             // L298 IN_A=H,IN_B=L = forward wiring
  double eff = std::max(0.0, (duty - D0) / (1 - D0));
  return dir * vmax * eff;
}

static void physStep() {
  double dt = PHYS_DT * 1e-6;
  int bR, bL;
  double tR = motorTarget(0, 1, 2, VR, bR);
  double tL = motorTarget(5, 3, 4, VL, bL);
  vR += (tR - vR) * dt / (bR ? 0.03 : TAU);
  vL += (tL - vL) * dt / (bL ? 0.03 : TAU);
  double v = 0.5 * (vR + vL), w = (vR - vL) / TRACK_ACT;
  pose.th += w * dt + YAWN * gauss(rng) * sqrt(dt * 1000);
  pose.x += v * cos(pose.th) * dt;
  pose.y += v * sin(pose.th) * dt;

  // wall contact: push back inside, lose a little speed
  double c = clearanceNow();
  if (c < 0) {
    contacts++;
    double r = hypot(pose.x, pose.y);
    pose.x *= (r + c) / r; pose.y *= (r + c) / r;
    vR *= 0.995; vL *= 0.995;
  }

  // encoders: every edge = one interrupt, alternating C1 / C2
  accR += fabs(vR) * dt * TPC_R_ACT; accL += fabs(vL) * dt * TPC_L_ACT;
  while (accR >= 1) { accR -= 1; quadR ^= 1; int pin = quadR ? 12 : 13; if (isr[pin]) isr[pin](); }
  while (accL >= 1) { accL -= 1; quadL ^= 1; int pin = quadL ? 14 : 15; if (isr[pin]) isr[pin](); }

  // scoring
  if (raceStart < 0 && (pwmVal[0] < 255 || pwmVal[5] < 255)) {
    raceStart = simT;
    lapPhiPrev = lapPhi0 = atan2(pose.y, pose.x);
    rcStart = hypot(pose.x, pose.y);
  }
  if (raceStart >= 0) {
    double phi = atan2(pose.y, pose.x), d = phi - lapPhiPrev;
    if (d > M_PI) d -= 2 * M_PI;
    if (d < -M_PI) d += 2 * M_PI;
    lapPhiAcc += d; lapPhiPrev = phi;
    double t = (simT - raceStart) * 1e-6;
    if (t <= 30.0) {
      distAcc += fabs(v) * dt;
      minClear = std::min(minClear, c);
      if ((simT % 20000) == 0) { sumAbsGap += c; gapN++; }
    } else if (laps30 < 0) {
      laps30 = fabs(lapPhiAcc) / (2 * M_PI);
      dist30 = distAcc;
    }
    if (traceF && (simT % 20000) == 0)
      fprintf(traceF, "%.3f,%.2f,%.2f,%.4f,%.2f,%.2f,%.2f\n", t, pose.x, pose.y, pose.th, vL, vR, c);
  }
}

static void advance(uint64_t us) {
  uint64_t end = simT + us;
  while (nextPhys <= end) { simT = nextPhys; physStep(); nextPhys += PHYS_DT; }
  simT = end;
  if (simT > 80e6) { fprintf(stderr, "sim timeout (car never finished)\n"); exit(2); }
}

// ------------------------------------------------------------- Arduino API
SimSerial Serial;
int SimSerial::available() { return 0; }
int SimSerial::read() { return -1; }
void pinMode(int pin, int mode) { if (mode == INPUT_PULLUP) pinLevel[pin] = 1; }
void digitalWrite(int pin, int level) {
  int old = pinLevel[pin];
  pinLevel[pin] = level ? 1 : 0;
  for (auto &s : sonar)
    if (pin == s.trig && old == 1 && level == 0) triggerFired(s);
}
int digitalRead(int pin) { return pinLevel[pin]; }
void analogWrite(int pin, int value) { pwmVal[pin] = value; }
void analogWriteFreq(int) {}
void analogWriteRange(int r) { if (r != 255) { fprintf(stderr, "range!=255\n"); exit(3); } }
uint32_t micros() { advance(1); return (uint32_t)simT; }
uint32_t millis() { advance(1); return (uint32_t)(simT / 1000); }
void delay(uint32_t ms) { advance((uint64_t)ms * 1000); }
void delayMicroseconds(uint32_t us) { advance(us); }
void attachInterrupt(int pin, void (*fn)(), int) { isr[pin] = fn; }
unsigned long pulseIn(int pin, int, unsigned long timeout) {
  Sonar *s = nullptr;
  for (auto &x : sonar) if (x.echo == pin) s = &x;
  if (!s) return 0;
  uint64_t t0 = simT, abort = t0 + timeout;
  auto giveUp = [&]() { advance(abort - simT); return 0UL; };
  if (s->ps <= t0 && t0 < s->pe) {          // already high: wait for it to end, then no new pulse
    return giveUp();
  }
  if (s->ps > t0) {
    if (s->ps >= abort || s->pe >= abort) return giveUp();
    advance(s->pe - simT);
    return (unsigned long)(s->pe - s->ps);
  }
  return giveUp();
}

// ------------------------------------------------------------- firmware
#ifndef FIRMWARE_FILE
#define FIRMWARE_FILE "../speed_race/speed_race.ino"
#endif
#include FIRMWARE_FILE

// ------------------------------------------------------------- main
#include <map>
static double P(const char *k, double def) {
  auto it = gArgs->find(k);
  return it == gArgs->end() ? def : atof(it->second.c_str());
}
int main(int argc, char **argv) {
  std::map<std::string, std::string> args;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto eq = a.find('=');
    if (eq != std::string::npos) args[a.substr(0, eq)] = a.substr(eq + 1);
  }
  gArgs = &args;
  if (args.count("side")) sideCfg = args["side"][0];
  R_WALL = P("R", 108); BUMP = P("bump", 0); PANELS = (int)P("panels", 0); FLAT = P("flat", 1);
  VL = P("vl", 62); VR = P("vr", 72); D0 = P("d0", 0.08); TAU = P("tau", 0.08);
  TRACK_ACT = P("track", 19.3); NOISE = P("noise", 0.15); DROP = P("drop", 0.02);
  SPIKE = P("spike", 0.01); CONE = P("cone", 20); YAWN = P("yawn", 0.0015);
  rng.seed((unsigned)P("seed", 1));
  Serial.echo = P("verbose", 0) > 0;
  if (args.count("trace")) {
    traceF = fopen(args["trace"].c_str(), "w");
    fprintf(traceF, "t,x,y,th,vL,vR,clear\n");
  }
  for (int i = 0; i < 32; i++) pwmVal[i] = 255;
  buildWall();

  // place the car: bottom of the circle, side sensor z0 from the wall
  double z0 = P("z0", 7), psi0 = P("psi0", 0) * M_PI / 180;
  double Lf = (sideCfg == 'R') ? 13.0 : 13.5;
  double r0 = sqrt(wallRadius(-M_PI / 2) * wallRadius(-M_PI / 2) - Lf * Lf) - 9.0 - z0;
  pose.x = 0; pose.y = -r0;
  pose.th = (sideCfg == 'R') ? psi0 : M_PI - psi0;

  setup();
  while (!raceDone) loop();
  if (laps30 < 0) { laps30 = fabs(lapPhiAcc) / (2 * M_PI); dist30 = distAcc; }

  printf("side=%c laps30=%.2f dist30=%.0fcm avgV=%.1f minClear=%.1fcm avgClear=%.1fcm contacts=%ld | fw: laps=%.2f dist=%.0f bad=%lu panic=%lu eased=%lu sideErr=%.2f\n",
         raceWallRight ? 'R' : 'L', laps30, dist30, dist30 / 30.0, minClear,
         gapN ? sumAbsGap / gapN : 0, contacts, statTurn / 6.2832, statDist,
         (unsigned long)statBad, (unsigned long)statPanic, (unsigned long)statCapped,
         statSumErr / (statTicks ? statTicks : 1));
  if (Serial.echo) printSummary();
  if (P("fwlog", 0) > 0) { Serial.echo = true; dumpLog(); }
  if (traceF) fclose(traceF);
  return 0;
}
