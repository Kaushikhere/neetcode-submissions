# Approach 3: safe driving with the team's wall rules

Sketch: `approach3_safe.ino` (about 440 lines, less than half of approach 1).

## The team's idea, and what changed

| Team's rule | Kept / changed | Why |
|---|---|---|
| FRONT < 9 cm -> move back to 10 cm | Now FRONT < **12 cm** -> back up until FRONT >= **27 cm**, then turn 35° away | At 10 cm the robot still has no room to turn (it needs ~20 cm lengthwise), so it would hit the wall again straight away. 1 cm between "too close" and "far enough" also makes it jitter back and forth. |
| RIGHT < 20 cm -> rotate away | Now **27 cm**, as a steer to the left proportional to how close the wall is | Smooth instead of on/off. |
| RIGHT > 20 cm -> back to the original rotation | Kept: the robot returns to its planned heading | Uses the encoder heading to know what "original" is. |
| (new) wheels stuck | Motors on but encoders not moving for 0.5 s -> back up and turn | Fixes "stuck at the wall with the wheels still turning". The front sensor misses a wall hit at an angle; the encoders don't. |
| (new) black border | Black is ignored as a line if a wall is closer than 15 cm ahead, or if the robot is at the board edge | Stops the border being taken for the line. |

Limits of the rules on their own: they only keep the robot safe. They
do not take it to T1 or T2, and with no LEFT sensor it cannot see a wall
on its left. So approach 3 uses them as a safety layer under a short
mission.

## The run

1. **Room**: if a wall is under 20 cm ahead, back up.
2. **Open side**: spin once; the widest direction with nothing within 80 cm ahead is up the field.
3. **Square up**: small sweep with the RIGHT sensor; shortest reading = square to the right wall -> exact heading and x.
4. **Up the field** (right rule on) until the IRs see the bottom of the black loop -> exact y from the map.
5. **T1**: turn, drive, servo drop.
6. **Line**: go back just below the loop's bottom edge, drive up, and when the IRs see black **rotate clockwise 90°** onto the line; follow it along the bottom, up the right side, to its end.
7. **T2**: into the pocket, centred with RIGHT, stop with FRONT.

The safety layer is on for every step except the last few cm at T2,
where driving up to the wall is the goal.

## Run modes

| Mode | Does |
|---|---|
| 3 (default) | Safety test: just drives forward with all the rules on. Put a wall in front of it, hold it, put a wall on its right. |
| 1 | Encoder counts while you push it 100 cm -> `COUNTS_PER_CM` |
| 2 | Spins 360° -> adjust `WHEEL_BASE_CM` |
| 0 | Full run |

## Compared with approaches 1 and 2

| | Approach 1 | Approach 2 | Approach 3 |
|---|---|---|---|
| Never gets stuck on a wall | No | No | **Yes** |
| Ignores the black border | No | No | **Yes** |
| Code size | ~960 lines | ~1040 lines | **~440 lines** |
| Start position | Fine wall scan | Rough sweep | Open side + right wall |
| T1 position from | Walls + encoders | Black loop (2 chords) | Right wall (x) + loop edge (y) |
| Needs the loop round T1 | No | Yes | Only for y (falls back to encoders) |

Pins are the same as approaches 1 and 2.
