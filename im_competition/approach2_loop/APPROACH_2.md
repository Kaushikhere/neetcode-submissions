# Approach 2: find T1 with the black loop

Sketch: `approach2_loop.ino` (open this folder in the Arduino IDE).

## The idea

The robot only takes a **rough** position from the start walls, then uses
the black loop that circles T1 to place itself exactly. Because the line
does the positioning, errors from the start and from wheel slip do not
carry over to T1.

It still **drops with the servo** at T1, exactly like approach 1. Only
the way it finds T1 is different. Everything after T1 is the same as
approach 1.

It **needs the loop round T1** on the competition board.

## How it finds T1

1. Aims at the middle of the loop and drives across it (**chord 1**),
   noting the two points where an IR met the line.
2. Drives back to the middle of chord 1, turns 90° and crosses the loop
   the other way (**chord 2**): two more points.
3. Fits the 4 points onto the loop's shape from the map. That shift gives
   its exact position. Then it drives the last few cm to T1.

If a chord fails (no line where expected), it fits whatever points it
has, or keeps the rough position.

## Steps

| # | Step | How | Sensors |
|---|---|---|---|
| 1 | Rough position | Spins once reading both ultrasonics -> rough x, y, heading | Both ultrasonics, encoders |
| 2 | Find T1 | Two chords across the loop, fit the 4 points to the map | IRs, encoders |
| 3 | Drop | Servo opens over T1 | Servo |
| 4 | Find the line | Faces right, drives about 39 cm to the loop | IRs |
| 5 | Follow the line | Same as approach 1 | IRs, encoders |
| 6 | Into the T2 pocket | Same as approach 1 | Both ultrasonics |
| 7 | Stop | Motors off, 88 s limit | |

## Expected accuracy (computer simulation, not yet tested on the robot)

- When all 4 points are found: T1 within about 1 cm typically, within
  about 2 cm in 9 out of 10 runs, worst about 7 cm.
- Starts tight in the right-hand corner often lose a chord and fall back
  to the rough position. Test those starts.

## Strengths and weaknesses

| Strengths | Weaknesses |
|---|---|
| The line places the robot, so wheel slip and start errors are removed | Needs the loop round T1 on the competition board |
| Only a rough start position is needed | Takes longer (two chords across the loop) |
| Typically more accurate at T1 | The green ring at T1 may read as black (code ignores black too close, but test it) |
| | Weaker from tight right-corner starts |

## Settings specific to this approach

| Setting | Meaning |
|---|---|
| `LOOP_X[]`, `LOOP_Y[]` | Points along the loop (from the map) - measure these |
| `LOOP_MID_X`, `LOOP_MID_Y` | Where it aims from the start |
| `CHORD1_MIN/MAX`, `SIDE_MIN/MAX`, `CHORD2_MIN/MAX` | Ignore black closer than MIN (the T1 ring); give up after MAX |

## Run modes (`RUN_MODE` at the top)

| Mode | Does |
|---|---|
| 1-3 | Same calibration as approach 1 |
| 4 | Rough position + both chords, drives to T1 and stops (no drop) |
| 5 | Same plus the drop |
| 0 | Full run |

## Pins

Same as approach 1: motors GP0-GP5, right ultrasonic GP6/GP7, front
ultrasonic GP8/GP9, servo GP10, encoders GP12-GP15, IRs GP16/GP17.

## Before it will work

Same as approach 1 (5 V on battery, calibration, sensor offsets), plus:
tape-measure the loop round T1 (at least its bottom, top, left and right
edges) and update `LOOP_X` / `LOOP_Y`.
