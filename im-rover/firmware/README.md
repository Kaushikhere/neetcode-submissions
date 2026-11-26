# IM rover firmware: "Square, slide, shoot"

One Arduino sketch, `im_rover/im_rover.ino` plus `im_rover/field_map.h` (your practice field in cm, traced from the photos), for the Raspberry Pi Pico with the **arduino-pico** core (Earle Philhower). Board: *Raspberry Pi Pico*. No extra libraries are needed: `Servo` ships with the core.

- **Core 0** runs motion control at 100 Hz: a speed PI loop on each wheel, heading hold, odometry and stall detection.
- **Core 1** pings FRONT, RIGHT and LEFT in turn, 30 ms apart, so `pulseIn` never freezes the motors.
- A hardware alarm cuts both motor enables at 89 s, whatever the code is doing at that moment.

## What changes on the robot

| Change | Why | Cost |
|---|---|---|
| Flat rear bumper (wood or card), a little wider than the wheels, square to the axle | It is the reference face for wall squaring. The tyres must not touch the wall first. | free |
| SG90 servo trap door between the wheels, signal on **GP10** | The item lands under the axle, which is the point the code steers. Put a 470–1000 µF capacitor across the servo supply. | ~Rs 120–150 |
| Drop item: flat and heavy, e.g. 3–4 taped washers or a small beanbag, released 1–3 cm above the board | A ball rolls and a coin can land on its edge. | free |
| Fixed pointer: 3 cm stick at the front centre, above the IR sensors | 22 cm robot + 3 cm pointer still fits the 25 cm box | free |
| Optional start button from **GP11** to GND | BOOTSEL also works | free |
| Optional third IR (centre) on **GP22** | Lets the follower tell "lost the line" from "line between the sensors" in the zigzag | ~Rs 40–60 |

Pins are unchanged otherwise: motors GP0–5, FRONT 8/9, RIGHT 6/7, LEFT 18/19, encoders 12/13 and 14/15, IR 16/17.

## Calibrate in this order (Serial Monitor, 115200 baud)

1. **`e` encoder signs.** Push the robot forward by hand. Both counts must rise. If one falls, flip `ENC_SIGN_L` or `ENC_SIGN_R`.
2. **`m` monitor.** Check that the IR sensors read 1 over black. If not, set `IR_BLACK_LEVEL = LOW`. Check that all three sonars read sensibly.
3. **`d` drive 100 cm**, on the competition board surface. Measure the real distance and set `TICKS_PER_CM_x = old × 100 / measured`.
4. **`w` drive 200 cm.** Measure the sideways drift. For each 1 cm of drift to the left, lower `TICKS_PER_CM_R` by 0.1 %. Drift to the right: raise it.
5. **`s` spin test.** The robot turns 1800° counter-clockwise. Measure how far short it stops, in degrees (negative if it went past). Set `TRACK_CM = old × 1800 / (1800 − shortfall)`. This replaces the "hand-turn hdg = 360" check.
6. **`9`.** Four 90° turns should bring the robot back to its starting heading within about 1°.
7. **`AXLE_TO_FRONT`.** Face a wall with the axle exactly D cm from it (tape measure), read FRONT, and set `AXLE_TO_FRONT = D − FRONT`.
8. **`q` square test.** Put the robot facing up-field with the wall behind it. It reverses, squares and prints L, R and x.
   - Check that L + R is about `SUM_TARGET` (you measured 98.7).
   - Check that x matches a tape measure to within about 1 cm.
   - Tune `SQUARE_PWM`: it should pivot flat against the wall without climbing it.
9. **Map numbers (`field_map.h`).** The photo trace is good to about 2–4 cm. Measure these with a tape and overwrite them:
    - `T1_X`, `T1_Y`: the green ring centre, from the inner face of the left wall and of the back wall.
    - `LINE_PICKUP_X/Y`: where the oval's right side crosses y = T1_Y.
    - `T2_X/Y`: confirm T2's spot with the organisers.

    Then do three full runs. If the item lands consistently off, correct `T1_X` / `T1_Y` by that offset.
10. **`t` teach.**
    - Hold the robot with its pointer on T2 and send `t`. Copy FRONT into `T2_FRONT_TARGET`.
    - With the IR sensors at the line's end, send `t` again. Copy FRONT into `LINE_END_FRONT`.
11. **Route table** (`ROUTE[]` in the sketch). Each run prints the route distance where the line ended and where the scripted hairpin fired. Shift the `RT_*` numbers in `field_map.h` until the sections line up:
    - ignore LEFT at the oval's loose end
    - SLOW in the passage
    - ignore RIGHT at the branch
    - the hairpin pivot

    Check that the hairpin pivot clears the top of the ladder's left panel. If it doesn't, change `preCm`/`turnDeg`.
12. **On competition day:** check `SOUND_CM_PER_US` against a known distance. The speed of sound changes about 0.17 % per °C.

## Run

Place the robot (any of start marks 1–6 or anywhere in the box), then press BOOTSEL or the GP11 button and take your hand away. The robot starts 0.4 s after the button is released. The serial log shows each phase with timestamps.

The sketch has passed a strict C++ syntax and type check against stub headers, but it has not been compiled with the rp2040 core or run on the robot. Expect to tune the gains on the bench. The first ones to look at are `PWM_MIN`, `KP_W`/`KI_W` and `SQUARE_PWM`.
