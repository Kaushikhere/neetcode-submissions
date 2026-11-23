# Approach 1: find the position from the start walls

Sketch: `approach1_walls.ino` (open this folder in the Arduino IDE).

## The idea

The robot measures exactly where it is in the start box using the three
30 cm start walls, then drives a calculated path straight to T1 with the
wheel encoders. The black loop round T1 is only used as an extra
correction on the way.

It does **not** need the loop round T1 to exist, so it also works if the
competition board's line near T1 is different.

## Steps

| # | Step | How | Sensors |
|---|---|---|---|
| 1a | Rough position | Spins once. The back and side walls read close, the open side (up the field) reads far. | Both ultrasonics, encoders |
| 1b | Exact position | Turns so one sensor faces the back wall and the other the nearer side wall, scans ±40°. Gives exact x, y and heading. | Both ultrasonics |
| 2 | Drive to T1 | Turns to T1 and drives there. When an IR crosses the black loop it corrects the position from the map. Slows for the last 15 cm. | Encoders, IRs |
| 3 | Drop | Servo opens over T1. | Servo |
| 4 | Find the line | Faces right, drives about 39 cm until an IR sees the loop. | IRs |
| 5 | Follow the line | Team's line-follower speeds. Ignores one IR at the fork (about 125 cm). Detects the line end by distance (about 228 cm) and resets its position there. | IRs, encoders |
| 6 | Into the T2 pocket | Lines up with the pocket, keeps centred with the right ultrasonic, stops with the front ultrasonic when the pointer tip is on T2. | Both ultrasonics |
| 7 | Stop | Motors off. Everything stops at 88 s anyway. | |

## Expected accuracy (computer simulation, not yet tested on the robot)

- Start position: about ±1 cm in x and y.
- Heading: median about 1°, within 3° in 90% of runs, up to 8° near corners.
- At T1: depends mainly on the heading and on `COUNTS_PER_CM` /
  `WHEEL_BASE_CM`. A few degrees of heading error becomes several cm
  sideways over the ~80 cm drive; the loop crossing removes part of it.

## Strengths and weaknesses

| Strengths | Weaknesses |
|---|---|
| Does not need the loop round T1 | Accuracy at T1 depends on calibration and wheel slip |
| Goes straight to T1, saves time | Heading error grows into sideways error on the way |
| Works from any start position | Spinning near a corner may brush a wall |

## Run modes (`RUN_MODE` at the top)

| Mode | Does |
|---|---|
| 1 | Prints encoder counts while you push the robot 100 cm by hand -> `COUNTS_PER_CM` |
| 2 | Drives 100 cm; adjust `COUNTS_PER_CM` from the real distance |
| 3 | Spins 360°; adjust `WHEEL_BASE_CM` |
| 4 | Finds its position only, prints x, y, heading (compare with a tape measure) |
| 5 | Position, drive to T1, drop, stop |
| 0 | Full run |

## Pins

| Part | Pins |
|---|---|
| Right wheel (L298N OUT1/OUT2) | ENA GP0, IN1 GP1, IN2 GP2 |
| Left wheel (L298N OUT3/OUT4) | IN3 GP3, IN4 GP4, ENB GP5 |
| Right ultrasonic | TRIG GP6, ECHO GP7 (1k/2k divider) |
| Front ultrasonic | TRIG GP8, ECHO GP9 (1k/2k divider) |
| Drop servo | GP10 |
| Right encoder | C1 GP12, C2 GP13 |
| Left encoder | C1 GP14, C2 GP15 |
| Line IR | left GP16, right GP17 |

## Before it will work

1. 5 V supply on battery for the ultrasonics and encoders.
2. Modes 1-3 calibration on the real board.
3. Measure the sensor offsets (`US_SIDE_CM`, `US_FRONT_CM`,
   `IR_FORWARD_CM`, `POINTER_BEYOND_SENSOR_CM`).
4. Tape-measure the board and update the FIELD MAP block. The current
   numbers come from a photo of the practice board (about ±5 cm); the
   official rules give no positions.
