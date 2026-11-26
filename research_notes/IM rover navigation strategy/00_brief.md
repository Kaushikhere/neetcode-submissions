# Brief: IM Competition I (navigation) — context for all researchers and the report writer

This file is the coordinator's brief. It holds the competition rules and the team's robot data as the user gave them, plus the design hypotheses that research should confirm or refute. It is context, not research findings.

## Competition I rules (draft of 13 Nov 2026, competition on Sat 28 Nov 2026)
- Board 4 ft x 8 ft (122 x 244 cm), raised ~5 cm. 5 cm black border round the edge (IR can see it).
- Start region at one end, below a dashed line. Walls 30 cm high at the start end (green on the practice field): a back wall across the full 122 cm, and two side walls ~58 cm long (user's estimate). Up-field is open.
- Robot starts at a random position AND orientation in the start region (same pose on both field copies).
- T1: known location, unmarked to the robot. Robot drops an item; score = distance of the item's centre to T1. Actuator: a motor, a small cheap motor from the organisers, or a cheap actuator.
- Black line 2–4 cm wide runs from near T1 to T2. Decoy lines of the same width exist elsewhere "so a robot that gets lost can't just find a line and follow it". The line gets harder to follow as it approaches T2 (official figure shows a sharp zigzag section).
- Inner walls 15 cm high; two of them form a narrow passage the robot just fits through; the line runs along the passage centre line. T2 is in/near this passage, unmarked; robot "finds T2 using the nearby walls as a guide".
- Score for T2 = distance from the robot's pointer to T2 at the end. Pointer must be outside the normal outline, on the same side of the wall as T2. Robot incl. pointer must fit a 25 cm cube when measured (folding allowed; the robot must unfold itself).
- 90 s limit; the robot must shut itself down after 90 s. T1 and T2 scores add.
- Rs 500 extra-parts budget; tape, wire, glue, wood scraps, wheels free. Imaginative shortcuts are encouraged if asked about.
- Whiteboard: "Use green walls to find start position (Where am I?)", "Drop package at T1 — dead reckoning", "Don't get lost", "Robots have ultrasound, IR, odometry".

## Robot (user's data)
- Raspberry Pi Pico. Differential drive: two JGB37-520 12 V 200 RPM 1:30 gearmotors with Hall encoders, 1320 ticks/wheel turn (11 PPR x4 x30), wheel 7 cm, 60.5 (L) / 60.7 (R) ticks per cm. Track width 19.3 cm centre to centre. Calculated 10.2 ticks per degree of spin per wheel (unconfirmed). Right motor runs 4–21 % faster at equal power.
- Drive wheels at the REAR, ball caster at the front (from photos). Robot 22 cm long, 21.3 cm wide over the wheels.
- L298N driven through 6 PC817 optocouplers (inverted: PWM 255 = stop, 0 = full). Pico and motor grounds kept separate. 12 V battery for motors, 3xAA for the Pico.
- Three HC-SR04 ultrasonics: FRONT (centre of front face), RIGHT and LEFT (on the side panels near the front), at 90 degrees. Each sensor face is ~11 cm from the robot centre. Echo dividers to 3.3 V. Mounted low (a few cm above the board).
- Two IR line sensors at the front, either side of the front sonar (left GP16, right GP17). Spare pins GP10, 11, 20, 21, 22, 26–28.
- Existing code tunables: drive power 180, turn 180, kick 230 for 50 ms, PID KP 6 KI 0.8 KD 0.6; sonar timeout 15 ms.
- Measured in the start box at the back, facing up-field: FRONT 138.8, RIGHT 47.1, LEFT 51.6 (R+L = 98.7, so 98.7 + 2x11 ≈ 122 box width).

## The user's current plan ("sum method") — likely the same as the friend's approach
1. Rotate until RIGHT + LEFT ≈ 99 (robot parallel to the side walls). FRONT ≥ 50 = facing up-field, < 50 = facing the back wall.
2. Turn to face a side wall, drive until FRONT = 50 (robot centred across the box), turn 90° toward the larger of RIGHT/LEFT (up-field).
3. Drive until FRONT = 110 ±1 to the "far wall" up-field (FRONT reads 110–138 there; this object may be an angled inner wall) → fixed "home" pose.
4. From home, replay a fixed, hand-tuned move sequence to T1, drop, then find the line and follow it to T2.
- Many safety rules: FRONT < 6 back off; side < 7 reverse rotation; if both rotation ends blocked, turn to arc centre, drive 20 cm, retry; never reverse > 1.5 s.
- The user's friend's robot visibly "went to one point from any random start, then drove straight to T1" and completed the course. That is a "funnel to a home pose, then replay a calibrated path" strategy.

## Coordinator's design hypotheses (to verify, refine or refute with sources)
H1. HC-SR04 against a smooth flat wall returns the distance to the nearest point inside its beam (~15° cone), so within roughly ±(10–15)° of perpendicular it reads the PERPENDICULAR distance regardless of angle ("region of constant depth", Leonard & Durrant-Whyte). Beyond ~20–30° incidence the echo is reflected away (specular) and the reading drops out or jumps. Consequence: RIGHT+LEFT is almost flat near alignment, so "rotate until the sum ≈ 99" stops at the EDGE of the flat zone, giving a heading error of ~10° (≈17 cm lateral error per metre). Even geometrically, 122/cos θ − 22 changes only ~0.5 cm at 5°.
H2. Wall squaring (driving slowly backward into a long flat wall until both sides touch and the wheels stall) gives heading within ~0.5–1° and an exact distance to that wall. Widely used in FLL / VEX / WRO. Much better heading reference than any sonar trick.
H3. The best "home point" is the spot on the back wall directly below T1: square on the back wall, read LEFT/RIGHT (now exactly perpendicular) for x, slide sideways by ranging FRONT to a side wall, re-square on the back wall, then drive one straight line to T1 on encoders. No turn after the last squaring, so turn errors never reach T1. Expected T1 error ~1–2 cm vs ~10+ cm for the sum method.
H4. Encoder odometry: distance error ~0.5–1 % after calibration on the actual surface; per-wheel ticks/cm (60.5 vs 60.7) must be used separately or the robot curves ~1° per metre. Point turns are the worst error source (effective track width differs from centre-to-centre; calibrate with UMBmark-style multi-turn tests). Gentle trapezoidal speed profiles avoid slip.
H5. With rear-axle drive the front corners sweep a ~21 cm radius when spinning and the rear only ~11 cm, so spinning near walls is safe if the robot turns away from the nearer side wall, and if the front is close it should back off first.
H6. Line following: 2 straddling IR sensors fail on sharp zigzags (line escapes outside a sensor); a 3rd centre sensor or slower speed plus "last seen side" memory helps; decoys can be rejected by gating on odometry (only accept the line where it is expected).
H7. Implementation on RP2040: arduino-pico core (Earle Philhower) supports attachInterrupt on any pin, analogWriteRange/analogWriteFreq, pulseIn; quadrature decoding by interrupts or PIO; PC817 optocouplers limit PWM frequency (~1 kHz OK). Hardware timer / millis() guard for the 90 s shutdown.
H8. Drop item should be flat and non-rolling (washer stack, coin roll, small weighted disc) dropped from low height directly under the robot's reference point; a ball rolls and ruins T1.
H9. T2: teach-by-demonstration — place the robot by hand with its pointer on T2 and record FRONT/LEFT/RIGHT; at run time stop when the readings match. A fixed 3 cm pointer fits the 25 cm cube (robot is 22 cm long).
