# IM Competition robot - project context (paste into a new chat)

I'm building a robot for the IM Project Competition (Fall 2026). Please use this context.

## Competition
- Competition I: 4 ft x 8 ft board (122 x 244 cm), 5 cm black border. Start region at one end, with 30 cm walls behind and on both sides (sides up to y ~58 cm). The start position and direction are random, revealed only when we place the robot.
- Drive to Target 1 (T1, unmarked for the robot, known position) and drop a small item with a servo: closer = more points. Then follow a black line (2-4 cm wide; decoy lines exist) through a narrow passage (~27 cm clear) to Target 2 (T2, inside a pocket between 15 cm walls) and stop with a pointer on T2: closer = more points. 90 s limit, robot must shut itself down. Robot + folded pointer must fit a 25 cm box.
- Competition II: speed race around a ~2.4 m circular track with a smooth outer wall.

## Field map (board cm, from a photo of the practice board, +-5 cm)
- Origin = start-end LEFT corner. x right (0-122), y up the field (0-244). Heading 0 = facing right, 90 = up the field.
- T1 (58.4, 97.0), inside a black loop (loop spans x 16-105, y 63-126). T2 estimate (51.1, 179.5), against the middle of the pocket's back rung. Pocket heading -62.7 deg, ~40 cm wide.
- Line joins the loop's right side at ~(97, 97) (drive +x ~39 cm from T1). Fork ~125 cm along the line (keep to main line), hairpin ~194 cm, line end ~228 cm at (50.5, 202), last heading -22 deg.
- Map page: https://claude.ai/artifact/YR7UutA1f7LYhL3ABchbL8
- Wiring page: https://claude.ai/artifact/EXYQazysHZgRYpmeCJL3yS

## Hardware
- Raspberry Pi Pico, Arduino IDE (earlephilhower arduino-pico core).
- L298N driven through 6 PC817 optocouplers -> PWM and direction are INVERTED: analogWrite 0 = full speed, 255 = stop. analogWriteRange(255), analogWriteFreq(500). ENA/ENB jumpers removed, 5V-EN jumper on.
- Two 12 V JGB37-520-type gear motors with Hall encoders. MOTORS WERE SWAPPED SIDES:
  - RIGHT wheel = L298N OUT1/OUT2: ENA GP0, IN1 GP1, IN2 GP2. Forward = IN1 LOW, IN2 HIGH.
  - LEFT wheel  = L298N OUT3/OUT4: IN3 GP3, IN4 GP4, ENB GP5. Forward = IN3 HIGH, IN4 LOW.
  - Brake = both IN pins LOW + EN analogWrite 0.
- Encoders (VCC 3.3 V, interrupt on C1 CHANGE, compare C1 with C2):
  - Left wheel:  C1 GP14, C2 GP15, LEFT_SIGN = 1
  - Right wheel: C1 GP12, C2 GP13, RIGHT_SIGN = -1
  - MAX_RATE = 70 counts per 50 ms in the code.
- Line IR sensors: LEFT IR GP16, RIGHT IR GP17, ON_BLACK = HIGH.
- Two HC-SR04 (5 V, ECHO via 1k/2k divider): RIGHT side TRIG GP6 / ECHO GP7, FRONT TRIG GP8 / ECHO GP9.
- Drop servo: GP10.
- Power: 12 V battery -> L298N only. Works on laptop USB; on a 3.3 V battery it misbehaves (encoders/ultrasonic likely need 5 V) - plan: USB power bank or 5 V regulator into VBUS.
- Old note: Pico from 3 x AA into VSYS (pin 39). Encoders and IRs on 3V3 (pin 36), NOT VBUS (VBUS is dead without USB - that caused "works on USB, not on battery"). Pico GND and L298N GND never joined.

## Final line follower (working) - im_competition/final/line_follower_FINAL.ino
- Each wheel has a speed target in encoder counts per 50 ms; a PI controller every 25 ms sets ENA/ENB and BRAKES a wheel that is too fast (fixes one motor coasting).
- Settings on the PWM scale (0 = full, 255 = stop), converted by pwmToRate():
  - Straight: left 20, right 110 (MAX_RATE 70)
  - Left IR on black -> turn RIGHT: left 10, right 250
  - Right IR on black -> turn LEFT: left 250, right 10
  - Both black -> straight.

## Other code in the repo
- Repo: kaushikhere/neetcode-submissions, branch claude/wall-following-ultrasonic-robot-x8l7lw, folder im_competition/
- mission/mission.ino: APPROACH 1 full run (RUN_MODE 1-3 calibration, 4 position test, 5 to-T1 test, 0 full run). Finds start position by spinning with the right ultrasonic against the 3 start walls, refines on the back wall and nearer side wall, drives to T1 by encoder odometry with a loop-edge correction, drops, follows the line, resets pose at the line end, goes to T2. Not yet tested on the robot; COUNTS_PER_CM and WHEEL_BASE_CM must be calibrated.
- mission_loop/mission_loop.ino: APPROACH 2 - rough wall sweep only, then two chords across the black loop round T1; the 4 line points are fitted to the loop shape from the map (shift only) to place the robot; servo drop at T1; rest same as mission.ino. Needs the loop to exist on the competition board.
- battery_check/battery_check.ino, encoder_test/, ir_test/, field_map.h.

## Lessons / rules for code
- Arduino IDE auto-generates function prototypes before the first function: define every struct/enum BEFORE any function, or it fails with "'Wheel' was not declared".
- Always give me the complete sketch to paste, not fragments.
- Keep my style: big comment headers, PWM scale 0 = full / 255 = stop.
