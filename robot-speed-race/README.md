# Competition 2 – Speed (round track, 30 s)

`speed_race/speed_race.ino` is the race firmware for the Raspberry Pi Pico car.
Use the Arduino IDE with the **Raspberry Pi Pico/RP2040** board package by Earle Philhower.

## Strategy
- Drive laps around the inside of the round wall (85 in / 216 cm across).
- The **outer** wheel (next to the wall) always runs at full power. The **inner** wheel's speed is
  set from its encoder, so the car follows the curve.
- The wall-side ultrasonic holds the car `D_TARGET_CM` (7 cm) from the wall. An estimator blends
  the ultrasonic readings with the encoders, so noisy or missing echoes don't cause swerving.
- The FRONT ultrasonic is only for emergencies. The IR sensors are not used.

## Before the first run
1. Set `WHEEL_TEST 1`, lift the wheels off the floor, upload, and open the Serial Monitor at 115200.
   Each wheel should roll **forward**. If one rolls backwards, swap that wheel's two
   `*_FWD_IN*` values. The test also prints all three ultrasonic readings.
2. Set `WHEEL_TEST 0` and upload again.

## Race day
1. Turn the 12 V switch on. Place the car parallel to the wall, with the side sensor about 7 cm from it.
   The **wall on the car's right** (counter-clockwise) is faster, because the right motor is stronger.
2. LED: 1 blink per second = wall on the right, 2 blinks = wall on the left, fast flicker = no wall seen.
   **Don't start on a flicker**: straighten the car first.
3. Hold a hand 2–8 cm in front of the FRONT sensor until the LED stays on. Pull the hand away on "GO".
4. The car stops by itself after `RUN_TIME_MS` (32 s).

After a practice run, plug in USB and open the Serial Monitor to see the summary. Type `L` to print the full CSV log.

## Laps vs distance (`D_TARGET_CM`)
A smaller circle gives more laps for almost the same distance. Simulated, 30 s, wall on the right:

| D_TARGET_CM | laps | distance |
|---|---|---|
| 7 (default) | 3.37 | 19.3 m |
| 15 | 3.67 | 19.1 m |
| 25 | 4.12 | 18.9 m |
| 40 | 5.07 | 18.4 m |

At 5 cm the car touched the wall in noisy runs, so keep it at 7 cm or more.

## Simulator
`sim/` compiles the **real** `.ino` against a physics model. The model covers the motors (with the
L298N and inverted-opto logic), the encoder interrupts, and the HC-SR04 timing and echoes off a curved wall,
with noise, dropouts and wall bumps or flat panels.

```
bash sim/run_tests.sh                     # all scenarios
g++ -O2 -std=c++17 -Isim sim/sim.cpp -o simrun && ./simrun side=R verbose=1
```
Options are listed at the top of `sim/sim.cpp`.
