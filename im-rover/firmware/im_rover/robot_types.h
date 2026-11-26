// robot_types.h: types used by im_rover.ino.
// They live in a header because the Arduino IDE writes a declaration for every function at the top of
// the sketch before compiling. A struct defined inside the .ino would come AFTER those declarations,
// giving "'Wheel' was not declared in this scope".
#pragma once
#include <Arduino.h>
#include <math.h>

struct Wheel { float target = 0, vel = 0, integ = 0, last = 0; };   // one drive wheel's speed loop

struct Sig {                                   // result of the spin that looks for the box pattern
  bool found = false;
  bool back = false;                           // true: facing the back wall, false: facing up-field
  float center = 0, L = NAN, R = NAN, F = NAN;
};

enum RouteAct : uint8_t { IGNORE_LEFT, IGNORE_RIGHT, SLOW, SCRIPT_TURN };
struct RouteRule { float from, to; RouteAct act; float preCm, turnDeg; };
