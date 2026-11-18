// field_map.h - IM Project Competition practice field (Fall 2026)
//
// Measured from a photo of the practice board, so expect errors of about
// 5 cm. Re-measure T1, the T2 rung and the passage with a tape before
// tuning numbers against it. The competition board's decoys will differ.
//
// Frame: origin at the start-end LEFT corner of the board (outer edge),
// standing behind the start wall and looking up the field.
//   x = cm to the right   (0 .. 122)
//   y = cm up the field   (0 .. 244)
//   heading in degrees: 0 = facing right (+x), 90 = facing up-field,
//   counter-clockwise positive (a left turn adds degrees).

#pragma once
#include <Arduino.h>

struct FieldPoint   { float x; float y; };
struct FieldSegment { FieldPoint a; FieldPoint b; };
struct FieldPose    { float x; float y; float headingDeg; };

const float BOARD_WIDTH_CM      = 122.0;
const float BOARD_LENGTH_CM     = 244.0;
const float BORDER_CM           = 5.0;   // black band on every edge
const float START_REGION_TOP_CM = 55.0;  // assumed; not drawn on the practice board

// Targets
const FieldPoint TARGET_T1     = { 58.4,  97.0 };  // green ring on the board
const FieldPoint TARGET_T2_EST = { 51.1, 179.5 };  // against the middle of rung C2 (estimate)

// Line landmarks
const FieldPoint LOOP_ENTRY    = { 97.0,  97.0 };  // drive +x from T1 to meet the line here
const FieldPoint LOOP_DEAD_END = { 94.0, 114.8 };  // wrong way round the loop ends here
const FieldPoint LINE_FORK     = { 73.4, 214.0 };  // keep LEFT here
const FieldPoint LINE_HAIRPIN  = { 18.5, 213.7 };  // about 140 deg left turn
const FieldPoint LINE_END      = { 50.5, 202.0 };  // mouth of the T2 pocket

const float POCKET_HEADING_DEG = -62.7;  // facing into the T2 pocket, along the rails
const float POCKET_WIDTH_CM    = 40.8;   // rail centre to rail centre
const float PASSAGE_WIDTH_CM   = 29.1;   // wall centre to wall centre

// Start poses marked in yellow on the practice board.
// Assumes the stem of each yellow "T" is the robot's forward direction.
const FieldPose START_POSES[6] = {
  {  13.9, 45.2,  -48 },  // 1
  {  47.1, 34.3,   39 },  // 2
  {  56.8, 19.4,   87 },  // 3
  {  76.7, 48.5, -123 },  // 4
  {  85.4, 33.4,    1 },  // 5
  { 105.6, 16.5,  134 },  // 6
};

// 30 cm walls at the start end (standing on the board edge)
const FieldSegment WALLS_30CM[3] = {
  { {   0.0, 0.0 }, { 122.0,  0.0 } },  // back wall
  { {   0.0, 0.0 }, {   0.0, 58.0 } },  // left wall
  { { 122.0, 0.0 }, { 122.0, 58.0 } },  // right wall
};

// 15 cm interior walls (wall centre lines)
const FieldSegment WALLS_15CM[7] = {
  { {  22.4, 190.6 }, {  48.7, 139.7 } },  // W1 left rail
  { {  58.5, 209.6 }, {  85.8, 156.8 } },  // W2 right rail (passage side)
  { {  19.1, 172.3 }, {  29.2, 177.5 } },  // C1 stub on W1
  { {  34.3, 167.5 }, {  70.6, 186.2 } },  // C2 upper rung (T2 side)
  { {  43.8, 149.3 }, {  80.0, 168.0 } },  // C3 lower rung
  { {  88.1, 215.8 }, { 106.1, 181.0 } },  // R1 passage wall
  { {  95.7, 201.2 }, { 109.3, 208.2 } },  // R2 stub on R1
};

// Centre of the black line, from the dead end of the loop round T1,
// up the right side, through the passage, round the hairpin to the pocket.
const FieldPoint LINE_PATH[] = {
  { 94.0, 114.8 }, { 65.2, 123.3 }, { 42.1, 125.9 }, { 19.0, 122.8 },
  { 16.2, 108.3 }, { 20.6,  91.2 }, { 28.4,  78.6 }, { 38.7,  68.9 },
  { 52.1,  63.7 }, { 65.0,  63.4 }, { 77.8,  66.4 }, { 85.9,  73.4 },
  { 91.9,  83.7 }, { 98.9, 101.2 }, {102.9, 118.7 }, {105.2, 134.1 },
  {104.7, 146.0 }, {102.3, 157.2 }, { 97.8, 164.9 }, { 90.1, 177.5 },
  { 80.0, 198.8 }, { 73.4, 214.0 }, { 70.5, 224.5 }, { 67.5, 228.9 },
  { 58.9, 229.8 }, { 44.2, 224.6 }, { 18.5, 213.7 }, { 32.9, 209.1 },
  { 50.5, 202.0 },
};
const int LINE_PATH_COUNT = sizeof(LINE_PATH) / sizeof(LINE_PATH[0]);

// Bearing in degrees (same convention as headings) from one point to another.
inline float fieldBearingDeg(FieldPoint from, FieldPoint to) {
  return atan2(to.y - from.y, to.x - from.x) * 180.0 / PI;
}

// Straight-line distance in cm.
inline float fieldDistanceCm(FieldPoint from, FieldPoint to) {
  return hypot(to.x - from.x, to.y - from.y);
}

// Turn needed (deg, left = +) to face `to` from a pose, wrapped to -180..180.
inline float fieldTurnToDeg(FieldPose pose, FieldPoint to) {
  float turn = fieldBearingDeg({ pose.x, pose.y }, to) - pose.headingDeg;
  while (turn > 180)  turn -= 360;
  while (turn < -180) turn += 360;
  return turn;
}
