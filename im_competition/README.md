# IM Project Competition Fall 2026 - everything in one place

Raspberry Pi Pico robot (Arduino IDE), L298N through 6 optocouplers,
2 x 12 V geared motors with Hall encoders, 2 IR line sensors,
2 HC-SR04 ultrasonics (right and front), drop servo.

## Folder layout

| Folder | What is in it |
|---|---|
| `code/` | All Arduino sketches (open a sketch's own folder in the Arduino IDE) |
| `docs/` | Competition rules PDF, the three approach write-ups, the project context prompt |
| `photos/` | Photos and screenshots you uploaded |
| `maps/` | Field map (interactive page + screenshot), map numbers for code, board photos with grid |
| `wiring/` | Circuit and breadboard pages (**old pins**, see the note below) |

## Code (`code/`)

| Sketch | What it does | Status |
|---|---|---|
| `curl_plan/` | **The 6-stage plan from the screen recording**: open side -> catch the curl (width check) -> ride to the dead end -> fixed hop to T1 + drop -> follow from A -> T2 pocket. Only your pin connections reused, logic written fresh | Compiled + simulated, not yet run on the robot |
| `final_plan/` | **Final Competition I plan**: approach 2 finds the position (curl ride + squaring on the main line), approach 3 safety, fixed line following. RUN_MODE 0-7 | Compiled + simulated, not yet run on the robot |
| `line_follower_V2/` | Line follower FINAL + line-lost search + IR test mode and motor test mode (`TEST_MODE`) | Compiled, not yet run on the robot |
| `line_follower_FINAL/` | **Working line follower**: encoder speed control per wheel, braking, speeds on the PWM scale (0 = full, 255 = stop) | Works on USB |
| `approach1_walls/` | Approach 1: exact position from the 3 start walls, odometry to T1, drop, line, T2 | Not tested on robot |
| `approach2_loop/` | Approach 2: rough position, then two chords across the black loop to find T1 | Not tested on robot |
| `approach3_safe/` | Approach 3: short version with the team's wall rules (front < 12 cm back up to 27 cm, right keep 27 cm, stuck-wheel escape, border is not the line, clockwise turn onto the line) | Not tested on robot |
| `tests/battery_check/` | Checks the supply voltage | **Old pins** |
| `tests/encoder_test/` | Prints encoder counts | **Old pins** |
| `tests/ir_test/` | Prints IR black/white | **Old pins**: use `line_follower_V2` TEST_MODE 1 instead |
| `old_versions/` | Earlier line followers and the first wall follower (`wall_detect_forward`) | Kept for reference |

Before running any approach: calibrate `COUNTS_PER_CM` and
`WHEEL_BASE_CM` (each sketch's RUN_MODE 1 and 2/3), and power the
encoders and ultrasonics from 5 V when running on battery.

## Docs (`docs/`)

| File | |
|---|---|
| `IM_Project_Competition_Fall_2026_v0.9.pdf` | Official rules |
| `rules_page1.png` | Page 1 of the rules as an image |
| `APPROACH_1.md`, `APPROACH_2.md`, `APPROACH_3.md` | How each approach works, steps, strengths and weaknesses, run modes |
| `CONTEXT.md` | Copy-paste prompt with the whole project context for a new chat |

## Photos (`photos/`)

| File | |
|---|---|
| `01_robot_build.jpg` | The robot |
| `02_practice_board_start_end.jpg` | Practice board, start end |
| `03_practice_board_full.jpg` | Practice board, whole board with walls and line |
| `04_motor_with_encoder.jpg` | Motor with Hall encoder |
| `05_breadboard_and_wiring.png` | Breadboard and wiring on the chassis |
| `06_encoder_connector.png` | Encoder connector close-up |
| `07_serial_log_right_wheel_not_slowing.png` | Serial log from the line-follower debugging |

## Maps (`maps/`)

| File | |
|---|---|
| `field_map.html` | Interactive field map (open in a browser). Online: https://claude.ai/artifact/YR7UutA1f7LYhL3ABchbL8 |
| `field_map_screenshot.png` | Picture of the map |
| `field_map.h` | Map numbers for the code |
| `rules_figure_competition1.png` | Competition I figure from the rules |
| `board_photo_top_with_grid.png`, `board_photo_bottom_with_grid.png` | Board photos with the measuring grid used to make the map |

Map numbers come from the practice-board photos (about ±5 cm); the
official rules give no positions. Tape-measure the real board.

Practice board (cm, origin = left corner at the start end, x right
0-122, y up the field 0-244):

| Point | Value |
|---|---|
| T1 | (58.4, 97.0) |
| T2 (estimate) | (51.1, 179.5) |
| Pocket mouth / heading | (39.1, 202.6), -62.7 deg |
| Line end / heading | (50.5, 202.0), -22 deg |
| Loop right side starts | (97.2, 97); fork 125 cm and line end 228 cm along the line from there |
| Start walls | x = 0, x = 122 (up to y ~58), y = 0 |

## Current pins

| Part | Pins |
|---|---|
| Right wheel (L298N OUT1/OUT2) | ENA GP0, IN1 GP1, IN2 GP2 |
| Left wheel (L298N OUT3/OUT4) | IN3 GP3, IN4 GP4, ENB GP5 |
| Right ultrasonic | TRIG GP6, ECHO GP7 (1k/2k divider) |
| Front ultrasonic | TRIG GP8, ECHO GP9 (1k/2k divider) |
| Drop servo | GP10 |
| Right encoder | C1 GP12, C2 GP13 (sign -1) |
| Left encoder | C1 GP14, C2 GP15 (sign +1) |
| Line IR | left GP16, right GP17 (black = HIGH) |

## Wiring (`wiring/`) - old pins

`robot_wiring_OLD_PINS.html` and its screenshots show how to connect the
optocouplers, L298N, encoders, sensors and power, but with the **old pin
numbers** (motors GP10-GP15, encoders GP4/5 and GP18/19, IRs GP0/1). The
circuits are still right; use the **Current pins** table above for the
GP numbers. Online version: https://claude.ai/artifact/EXYQazysHZgRYpmeCJL3yS
