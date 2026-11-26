// field_map.h: the IM practice field, traced from the photos of the board (Nov 2026).
//
// Frame: x = cm from the LEFT wall's inner face, y = cm from the BACK wall's inner face (start end).
//        Heading 0 = facing the right wall, 90 = facing up-field (away from the start box).
// Accuracy: the photo was rectified with a homography on the white area's corners, so positions are
//        good to roughly 2-4 cm. MEASURE T1, T2 and the line pickup spot with a tape before the event.
// The same numbers drive the simulation (im-rover/simulation/index.html).
#pragma once

// ---- board and the three 30 cm start-box walls ------------------------------------------
const float BOX_W        = 122.0;   // side wall to side wall (you measured 98.7 + 2 x sensor offset)
const float BOARD_L      = 244.0;
const float BORDER_W     = 5.0;     // black border inside the edge
const float SIDE_WALL_L  = 60.0;    // left and right walls run from the back wall up to y = 60
const float START_MAX_Y  = 52.0;    // practice start marks reach y = 49 (robot axle)

// ---- targets -------------------------------------------------------------------------------
const float T1_X = 57.1;            // green ring centre   (photo estimate: MEASURE)
const float T1_Y = 96.9;
const float T2_X = 49.0;            // up-field face of the ladder's upper rung, by the clip,
const float T2_Y = 188.5;           // ~14 cm past the line's end (estimate: confirm with the organisers)

// ---- 15 cm inner walls: {x1, y1, x2, y2} --------------------------------------------------
const float WALLS[][4] = {
  {19.5, 198.6, 46.6, 140.2},   // ladder, long left panel
  {56.8, 219.3, 83.7, 156.3},   // ladder, long right panel (left side of the narrow passage)
  {16.2, 173.6, 68.0, 192.6},   // ladder, upper rung (T2 is on its up-field face)
  {42.2, 155.9, 79.0, 175.4},   // ladder, lower rung
  {86.1, 227.8, 103.6, 181.3},  // T-wall, long panel (right side of the narrow passage)
  {97.5, 209.3, 108.3, 218.1},  // T-wall, short cross panel
};

// ---- the painted line (robot follows it from the pickup point) ----------------------------
// Oval around T1 with a loose end inside it, then up the right side, through the passage between
// the ladder and the T-wall, round the top, a sharp hairpin at the top-left, last leg toward T2.
const float LINE_PICKUP_X   = 96.5;   // where the robot meets the oval's right side when driving
const float LINE_PICKUP_Y   = 97.8;   //   straight right from T1
const float LINE_PICKUP_HDG = 74;     // line direction there (deg)
//  route distances (cm of line from the pickup point) of the features the follower must handle:
const float RT_OVAL_END     = 17;     // oval's loose end passes ~9 cm LEFT of the line
const float RT_PASSAGE_IN   = 62;     // entering the passage (ladder right panel / T-wall)
const float RT_BRANCH       = 120;    // a branch splits off to the RIGHT toward the T-wall (decoy)
const float RT_PASSAGE_OUT  = 125;
const float RT_TOP          = 138;    // top curve, line turns to head west
const float RT_HAIRPIN      = 188;    // hairpin vertex: line doubles back ~143 deg to the LEFT
const float RT_END          = 220;    // end of the line, heading about -18 deg
const float LINE_END_HDG    = -18;
const float RUNG_NORMAL_HDG = -70;    // direction that faces the upper rung square-on

// ---- decoy lines (not followed; listed for the simulation and for sanity checks) -----------
//  {13,159}-{32,131} left of the ladder, {62,156}-{69,127} below the ladder, branch near RT_BRANCH.

// ---- practice start marks 1-6: {x, y, heading}  (dot = axle, long leg = facing direction) --
const float START_MARKS[6][3] = {
  {13.7, 45.3, -50}, {46.1, 34.0, 40}, {56.3, 19.2, 88},
  {75.7, 48.9, -123}, {85.4, 34.1, -2}, {105.6, 16.2, 136},
};
