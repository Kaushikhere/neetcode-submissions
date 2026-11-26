# Dead reckoning, wall squaring and "home then replay" for the IM rover (T1 drop)

Scope: error budget and move sequence for driving from a random start in the walled start box to the unmarked target T1 (about 1 m up-field) and dropping an item. Robot data are from 00_brief.md: rear drive wheels, front ball caster, 7 cm wheels, 19.3 cm track centre to centre, 21.3 cm wide over the wheels (so tyres are about 2.0 cm wide), 60.5 / 60.7 ticks/cm, 1320 ticks per wheel turn, no IMU.

Source-quality note: the strongest source is Borenstein & Feng's UMBmark paper (full text read). Pybricks, VEX-tutorial, Botball and Pololu pages were read in full. Some FLL items (FLLCasts, which returned HTTP 403) and the Arduino/MathWorks straight-driving items come only from search-result excerpts and are marked as such. **No source I found publishes measured heading repeatability for wall squaring.** That number below is a geometric estimate, not a measurement.

---

## Q1. Wall squaring (bump squaring, wall alignment): how it is done and how repeatable it is

### Takeaway
Wall squaring means driving slowly into a long flat wall, detecting contact or stall, pushing briefly so the chassis rotates flush, and then zeroing heading (and one position axis) in odometry. FLL, VEX and Botball coaching material uses it widely. Published parameters are about 15–20 % power, a push of about 0.2–1 s after contact, a 200–300 ms settle, and a 3–5 s timeout. No source gives a measured heading repeatability. Geometry suggests about 0.1–0.5° if the bumper is flat and square to the axle, so H2's "0.5–1°" is plausible but unverified. Treat it as a target to measure this week.

### Cited Findings
- **FLL method:** drive the robot into a wall or another solid field object so it "sticks"; any initial angle ends up perpendicular. One FLLCasts tutorial recommends a timed move at high power for about 1 s rather than a sensor-based move, to save time. — [FLLCasts 1395 (search excerpt; page returned 403)](https://fllcasts.com/tutorials/1395-Intorduction-to-alignments-with-a-competition-robot); [FLLCasts 1396](https://fllcasts.com/tutorials/1396-aligning-to-a-wall-with-a-competition-robot)
- **What alignment cancels:** FLLCasts lists different starting positions in base, motor backlash (lashback) and battery level as error sources that alignment resets for the next few moves. — [FLLCasts 1395 (search excerpt)](https://fllcasts.com/tutorials/1395-Intorduction-to-alignments-with-a-competition-robot)
- **VEX-style stall squaring (third-party tutorial, not official VEX):**
  - Approach at 20 % speed (15 % in the full-routine example) with a 3 s timeout.
  - Stall is declared when *both* drive motors exceed 1.5 A, polled every 20 ms.
  - Then HOLD, wait 200 ms, switch to COAST so the robot rests against the wall, and call `set_heading(wall_heading)` with a 100 ms wait.
  - Position on the wall axis is reset by the caller, e.g. `reset_position(x, 36)`.
  - The stall threshold changes with battery charge, so test at competition battery level, and wait 200–300 ms after contact before trusting sensors.
  - The page claims "motor stall" alignment is about ±0.5 in (±13 mm) at fast speed and "bumper" alignment about ±0.1 in (±2.5 mm). It gives no method, so treat these as anecdote.
  - Source: [VEX tutorials – alignment strategies](https://vextutorials.vercel.app/tutorials/intermediate/alignment-strategies/)
- **Botball/"raccoon" library method:**
  - Contact is detected by IMU acceleration (|a_xy| ≥ 0.5 m/s² by default; 0.15–0.3 m/s² for slow approaches).
  - A 0.3 s grace period ignores start-up acceleration, then the robot keeps driving for a 0.2 s "settle" after first contact so the chassis rotates flush. There is a 5 s safety timeout.
  - No heading correction is applied during the push.
  - Lock the heading reference immediately after the push, with no other drive step in between.
  - Listed mistakes:
    - letting the timeout, not a real bump, end the step (position undefined);
    - using the wall as the main distance mechanism instead of driving most of the way on odometry first;
    - false triggers from mat edges and cables.
  - Source: [Botball/raccoon docs – wall alignment](https://raccoon-docs.pages.dev/01-programming-guide/02-driving/09-wall-alignment/)
- **Squaring fixes only some axes:** an FTC/FRC team's wall reset sets only the coordinate normal to the wall (y) and deliberately leaves x unchanged. — [CtrlR BioBuzz PR #19](https://github.com/Shae904/CtrlR_BioBuzz/pull/19)
- **Launch jigs:** FLL coaches recommend lining up with the wall and/or a launch jig, and designing the robot to start from the same spot rather than aiming it by eye. — [Team 71229 advice](https://team71229.bearblog.dev/robot-advice)
- **Borenstein's lab practice:** UMBmark runs start and end at an L-shaped corner of two perpendicular walls used as a fixed reference. Their sonar calibrator measured pose against that corner to ±2 mm in x/y and about ±0.4° in heading. — [Borenstein & Feng 1996, UMBmark](https://cs.au.dk/~ocaprani/legolab/DigitalControl.dir/NXT/Lesson10.dir/paper58.pdf)

### Inferences
- **Stall detection without current sensing (this robot):** use encoder tick rate.
  - At 10 cm/s each wheel makes about 12 ticks per 20 ms (60.6 ticks/cm × 10 cm/s × 0.02 s).
  - Declare "both wheels stalled" when each wheel gives fewer than about 2 ticks per 20 ms for at least 100–200 ms.
  - Then keep pushing at low power for about 0.3 s, cut to brake or coast, wait about 200 ms, and zero heading and the wall-normal coordinate.
  - Use a 2–3 s timeout that flags failure rather than assuming success (per the raccoon docs).
- **Heading floor set by the bumper:** if one end of a 21.3 cm-wide rear bumper sits 1 mm proud, heading after squaring is off by atan(0.1/21.3) ≈ 0.27°. At 0.5 mm it is 0.13°; at 2 mm, 0.54°. The bumper must be flat, rigid, as wide as possible, and square to the wheel axle; check it with a set square. A wall that bows or has a seam gives the same kind of error.
- **Squaring backward on the 122 cm back wall suits this robot:**
  - The rear face is next to the drive axle.
  - The back wall is the longest straight reference on the field. The side walls are only about 58 cm long.
  - After squaring, the robot faces up-field toward T1, so no turn is needed afterwards.
  - The LEFT/RIGHT sonars are then genuinely perpendicular to the side walls. They sit about 15–20 cm from the back wall, well inside the 58 cm walls, so they give x.
- **Backlash after the push:** the encoders sit on the motor shaft, before the 1:30 gearbox. After pushing backward, the gear lash is loaded in reverse. When the robot drives forward, each motor turns a little before its wheel moves, and the encoder counts that motion.
  - 1° of lash at the wheel is about 0.06 cm of travel (π·7 cm/360).
  - A lash *difference* of 1° between the two sides gives roughly 0.06/19.3 rad ≈ 0.18° of heading.
  - Mitigation: after squaring, drive forward a few mm at very low power, or start the forward profile gently so both sides take up their lash before heading-hold starts integrating.
- **Approach angle:** pre-align with sonar to within about 10–20° before squaring. Large approach angles risk the leading corner sliding along the wall or catching on the board edge. This is an inference; no source gives a limit.
- **Verdict on H2:** CONFIRMED that the technique is standard and is the recommended heading reset. NOT VERIFIED that the error is 0.5–1°: no measured figure exists in the sources I found. The geometric floor is about 0.1–0.3°, so 0.5° (1σ) is a reasonable planning number to measure on the practice field. The "exact distance to that wall" part is CONFIRMED in principle: the bumper is touching it, so the distance from the reference point to the wall is a fixed design dimension.

### Gaps
- No published measurement of heading repeatability after wall squaring (FLL, VEX, WRO or FTC) was found. The FLLCasts pages were blocked (403), and the O'Reilly "Winning Design!" squaring chapter was blocked (403).
- No source gives push power or duration as a function of robot mass or tyre grip, or a maximum approach angle that still squares reliably.
- WRO-specific squaring guidance was not found.

---

## Q2. Typical encoder-odometry error; systematic vs random errors; UMBmark; quick calibration

### Takeaway
On smooth indoor floors, systematic errors dominate: the wheel-diameter ratio Ed, the effective wheelbase Eb, and the average-diameter scale Es. They stay almost constant over time, so a calibration (or a path tuned on the practice field) removes about 90–95 % of them. UMBmark calibration cut the systematic return error of a 16 m square path from 310–423 mm to 12–35 mm (10–22×). Heading errors matter most because they turn into lateral error that keeps growing with distance.

### Cited Findings
- **Error taxonomy (Borenstein & Feng 1996):**
  - Systematic: unequal wheel diameters; average diameter different from nominal; wheel misalignment; uncertainty about the effective wheelbase "due to non-point wheel contact with the floor"; limited encoder resolution; limited sampling rate.
  - Non-systematic: uneven floors, objects on the floor, and wheel slip from slippery floors, over-acceleration, fast turning (skidding), external forces, and internal forces such as castors.
  - Source: [UMBmark paper](https://cs.au.dk/~ocaprani/legolab/DigitalControl.dir/NXT/Lesson10.dir/paper58.pdf)
- "On most smooth indoor surfaces systematic errors contribute much more to odometry errors than non-systematic errors." The two dominant errors "stay almost constant over prolonged periods of time." — [UMBmark paper](https://cs.au.dk/~ocaprani/legolab/DigitalControl.dir/NXT/Lesson10.dir/paper58.pdf)
- **Definitions** (from the same paper):
  - Ed = D_R/D_L, the actual wheel-diameter ratio (nominal 1.00).
  - Eb = b_actual/b_nominal.
  - Eb affects only turning; Ed affects only straight-line motion.
  - The average-diameter "scale" error Es affects both straight lines and turns. It "is exceedingly easy to measure with just an ordinary tape measure" and should be corrected first.
  - Effective-wheelbase uncertainty "can be on the order of 1% in some commercially available robots" because rubber tyres touch the floor over a contact area.
- **Turning theorem:** unequal wheel diameters do *not* cause an orientation error in on-the-spot turns. The turn angle depends only on the average diameter (D_L+D_R)/2 and the wheelbase. — [UMBmark paper](https://cs.au.dk/~ocaprani/legolab/DigitalControl.dir/NXT/Lesson10.dir/paper58.pdf)
- **"Orientation errors are the main source of concern** because once they are incurred they grow without bound into lateral position errors." — [UMBmark paper](https://cs.au.dk/~ocaprani/legolab/DigitalControl.dir/NXT/Lesson10.dir/paper58.pdf)
  - Lecture notes built on Siegwart & Nourbakhsh: a 2° heading error gives sin = 0.0175, so errors perpendicular to the direction of travel "grow much larger." — [Princeton COS495 Lecture 5](https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf); [HMC E160 Lecture 3](https://www.hmc.edu/lair/E160/E160-Lecture03-Odometry.pdf)
  - Kleeman (1995): perpendicular-error variance grows with the cube of distance travelled. — [Kleeman, Monash MECSE-1-1995 (search excerpt)](https://ecse.monash.edu/techrep/reports/pre-2003/MECSE-1-1995.pdf)
- **UMBmark procedure:**
  1. Start at an L-shaped corner and measure the absolute pose.
  2. Drive a 4 × 4 m square clockwise: stop after each 4 m leg, make four on-the-spot 90° turns, run slowly to avoid slip, and measure the final pose.
  3. Repeat five times clockwise and five times counter-clockwise.
  4. Take the centre of gravity of each cluster.
  - Error Type A (affects both directions equally) comes from Eb; Type B (opposite effect in each direction) comes from Ed.
  - Corrections:
    - β = (x_cw − x_ccw)/(−4L)·180/π, then R = (L/2)/sin(β/2) and Ed = (R + b/2)/(R − b/2).
    - α = (x_cw + x_ccw)/(−4L)·180/π, then b_actual = 90°/(90° − α) · b_nominal.
    - Wheel correction factors c_L = 2/(Ed+1) and c_R = 2/(1/Ed+1), which keep the average diameter unchanged.
  - Source: [UMBmark paper](https://cs.au.dk/~ocaprani/legolab/DigitalControl.dir/NXT/Lesson10.dir/paper58.pdf)
- **Measured results (LabMate robot, 0.2 m/s straights, ±0.2 m/s wheel speed in turns):**
  - Calibration changed the wheelbase from 340.00 to 336.17 mm (Eb ≈ 0.989, a 1.1 % error) and D_R/D_L from 1.00000 to 1.00084.
  - Emax,syst fell from 317 to 21 mm (15×). Across 8 experiments it went from 232–423 mm to 12–35 mm, a 10–22× improvement.
  - Winding three loops of masking tape round one wheel noticeably raised the error (Emax 423 mm before correction).
  - Non-systematic scatter: standard error of the mean 11.2 mm over n = 5 runs, i.e. σ ≈ 25 mm per 16 m, 4-turn run.
  - Source: [UMBmark paper](https://cs.au.dk/~ocaprani/legolab/DigitalControl.dir/NXT/Lesson10.dir/paper58.pdf)
- **Odometry integration error** is negligible at sample times under 10 ms and speeds under 1 m/s. — [UMBmark paper](https://cs.au.dk/~ocaprani/legolab/DigitalControl.dir/NXT/Lesson10.dir/paper58.pdf)
- **Pybricks quick calibration (LEGO):**
  - Run `straight(1000)` and measure. If the robot falls short, decrease wheel_diameter; if it goes too far, increase it.
  - Run `turn(360)`. If it turns too little, increase axle_track; if too much, decrease it.
  - Calibrate wheel_diameter first, then retest both.
  - axle_track is "the distance between the points where both wheels touch the ground."
  - Source: [Pybricks robotics docs](https://docs.pybricks.com/en/latest/robotics.html)
- **Small-robot anecdote (Pololu forum, 42 × 19 mm wheels, micro-metal gearmotors):** with the wheels off the ground the encoders agree within 1–2 ticks, but after 1 m on the floor one wheel had travelled about 1 cm more than the other. The cause was put down to slip. Suggested fixes: avoid rapid acceleration, drive slowly, widen the wheel spacing. — [Pololu forum: wheel encoders and slip](https://forum.pololu.com/t/wheel-encoders-and-slip/4586)
- **FLL advice:** small wheels are more accurate and large wheels quicker; a slower robot is more accurate. Clean the wheels (a magic eraser works) and don't drive the robot on the floor. — [Team 71229 advice](https://team71229.bearblog.dev/robot-advice)

### Inferences
- **Resolution is not the limit:**
  - 1 tick = 0.165 mm of wheel travel.
  - A 1-tick left–right difference = 0.165 mm / 193 mm = 0.049° of heading.
  - So the error budget is set by Es/Ed/Eb residuals, slip and stopping, not by encoder resolution.
- **Distance error after calibration:** the limit is how well the calibration run is measured. A 100 cm run measured to ±2 mm calibrates Es to ±0.2 %; ±5 mm gives ±0.5 %. For a 1 m shot to T1 that is ±0.2–0.5 cm. Calibrate over the actual T1 distance on the competition-like surface, at the run speed and acceleration, with the drop item loaded (Borenstein notes rubber tyres compress, which changes Ed under load). The brief's H4 figure of 0.5–1 % is conservative. About 0.3 % is reachable with a careful 1–2 m tape measurement.
- **Random error at 1 m:** scaling Borenstein's σ ≈ 25 mm per 16 m with four turns down to a 1 m straight with no turns suggests mm-level random error for a big robot on a good floor. A small robot with 2 cm-wide rubber tyres, a ball caster and a board surface will be worse. Plan for σ ≈ 0.3–0.5 cm along-track and expect heading noise of a few tenths of a degree per metre. Measure it with five or more repeats.
- **Field copies:** the two field copies may differ in surface, so a 1 m straight-line check on the competition board before the run (if allowed) is worth more than any theory.
- **Verdict on H4 (distance part):** CONFIRMED in spirit. Calibrated distance error of a few tenths of a percent to about 1 % is consistent with the sources, but no source gives a clean "per metre" figure for small hobby robots.

### Gaps
- No source found with a clean, measured "distance error per metre" and "heading drift per metre" for a small (about 20 cm) hobby differential-drive robot with Hall-encoder gearmotors.
- The Siegwart textbook itself was not read; only lecture notes reproducing its figures.

---

## Q3. One ticks/cm for two wheels (60.5 vs 60.7); effective track width in point turns; ticks/degree calibration

### Takeaway
If the controller equalises tick counts but the wheels really differ by 60.5 vs 60.7 ticks/cm, the robot curves about 1.0° and drifts about 0.86 cm sideways per metre, rising to 3.4 cm at 2 m. That confirms H4's "~1°/m". However, the 0.33 % difference is smaller than a typical 1 m tape-measure calibration error, so it may be measurement noise. The 2 m straight-line curvature test resolves it to about ±0.03 %. Effective track width for point turns can lie anywhere within the tyre contact width: 17.3–21.3 cm worst case for these 2 cm tyres. It must be calibrated with multi-turn spins in both directions, which give ticks/degree to about 0.1 %.

### Cited Findings
- Ed (unequal diameters) makes the robot travel on a curved path of radius R, with Ed = (R + b/2)/(R − b/2). — [UMBmark paper](https://cs.au.dk/~ocaprani/legolab/DigitalControl.dir/NXT/Lesson10.dir/paper58.pdf)
- Effective wheelbase is uncertain because rubber tyres touch the floor over an area. The uncertainty is about 1 % on commercial robots; the measured LabMate correction was 1.1 %. — [UMBmark paper](https://cs.au.dk/~ocaprani/legolab/DigitalControl.dir/NXT/Lesson10.dir/paper58.pdf)
- Pybricks: axle_track is between the ground contact points and is hard to see. Estimate it from the wheel midpoints, then tune it with `turn(360)`. Gyro-based turns can also be a few degrees off over large or many turns; the fix is to command `turn(357)` or `turn(362)`. — [Pybricks robotics docs](https://docs.pybricks.com/en/latest/robotics.html)
- Unequal wheel diameters do not change the on-the-spot turn angle; only the average diameter and the wheelbase do. — [UMBmark paper](https://cs.au.dk/~ocaprani/legolab/DigitalControl.dir/NXT/Lesson10.dir/paper58.pdf)

### Inferences (calculated for this robot, b = 19.3 cm)
- **Single ticks/cm (60.6) with tick-equalising heading-hold over about 1 m:**
  - Left wheel travels 0.33 cm more than the right.
  - Heading change 0.98°, lateral offset 0.86 cm at 1 m (3.4 cm at 2 m), turn radius about 58 m.
  - The robot curves toward the right (the smaller-distance-per-tick side), if the 60.5/60.7 values are real.
- **Is the difference real?** It is 0.33 %. A 1 m tape test has about ±0.5 % uncertainty at ±5 mm.
  - Better test: command equal ticks for 200 cm, then measure the lateral offset y against a chalk line.
  - Ed − 1 = 2·b·y/L². With y read to ±0.3 cm at L = 200 cm, Ed is resolved to about ±0.03 %.
  - A 0.1 % Ed error gives 1.0 cm of drift at 2 m and 0.26 cm at 1 m.
  - This test also captures caster drag and weight-distribution effects at the run speed, which is what matters. Correct with Borenstein's c_L = 2/(Ed+1) and c_R = 2/(1/Ed+1) so the average distance calibration is not disturbed.
- **Open loop is hopeless** ("right motor 4–21 % faster at equal power"). Equal PWM with a 4 % speed ratio gives a turn radius of about 4.9 m: 11.6° and about 10 cm lateral per metre. At 10 % it is 28° and about 24 cm; at 21 % it is 56° and about 45 cm. Closed-loop wheel synchronisation is mandatory.
- **Ticks per degree:**
  - Nominal per-wheel arc is π·19.3/360 = 0.168 cm/°. That gives 10.19 (left) / 10.22 (right) / 10.21 (average) ticks/°, so the brief's 10.2 is correct *for b = 19.3 cm*.
  - The tyres are about 2.0 cm wide (21.3 − 19.3), so the effective b for spins is bounded by about 17.3–21.3 cm (±10 %) in the worst case. Realistically, expect a few percent; Borenstein's figure is about 1 % on narrower hard tyres.
  - Effect of an uncalibrated track error on a 90° turn: 1 % gives 0.9° (1.6 cm lateral per following metre); 3 % gives 2.7° (4.7 cm/m); 5 % gives 4.5° (7.9 cm/m).
- **Multi-turn calibration:**
  1. Mark the robot's heading on the board with tape.
  2. Command 10 full spins (3600°) clockwise and read the residual angle δ, e.g. with a long pointer or sightline.
  3. Repeat counter-clockwise.
  4. Scale ticks/° by 3600/(3600 + δ_avg).
  - Reading δ to ±3° gives ±0.08 %, i.e. ±0.08° per 90° turn.
  - A CW–CCW difference points to direction-dependent effects (caster, motor asymmetry, backlash), as in UMBmark's Type A/B logic.
  - Calibrate at the actual turn speed and acceleration, then check with a single 90° turn against a set square. Single turns can carry start/stop slip that a 10-turn test averages away.
- **Rotation centre:** with rear drive wheels, the robot spins about the rear-axle midpoint. Put the odometry reference point, and ideally the drop point, there. Otherwise a heading error δ shifts the drop point by offset·sin δ; for a 15 cm offset and δ = 1° that is 0.26 cm.
- **Verdict on H4 (curvature and turn parts):** CONFIRMED. A single ticks/cm gives about 1°/m of curvature, if the 0.33 % difference is real; verify it with the 2 m test. Point turns are the most error-prone primitive and need multi-turn calibration in both directions.

### Gaps
- No source measures effective track width against centre-to-centre distance for small robots with about 2 cm rubber tyres and a ball caster. The ±10 % bound is pure geometry.
- No source quantifies ball-caster drag during spins.

---

## Q4. Heading-hold control, speed profiles and stopping accuracy

### Takeaway
Standard practice is a speed or position controller on each wheel, plus a heading term on the left–right distance difference. The difference is in calibrated cm, not raw ticks, and has a zero setpoint. The commanded motion follows a trapezoidal (or S-curve) profile, because full power from standstill causes slip that corrupts odometry. Stopping is best done by ramping down to a creep speed and then braking or holding.

### Cited Findings
- **Pybricks DriveBase:**
  - Uses two PID loops: `distance_control` (per mm) and `heading_control` (per degree of the whole drive base).
  - `settings()` sets straight speed and acceleration, and turn rate and acceleration; acceleration and deceleration can be set separately. Defaults are about 40 % of maximum speed.
  - The default end-of-move behaviour is `Stop.HOLD`, which actively holds position.
  - `stalled()` is true when the motors "cannot reach the target speed or position, even with the maximum actuation signal."
  - The docs warn that distance and angle "are only estimates" because wheels can slip.
  - Source: [Pybricks robotics docs](https://docs.pybricks.com/en/latest/robotics.html)
- **Straight-driving pattern (search excerpts):** feed back the left–right encoder difference with a zero setpoint; P-only first, add I only if steady drift remains. MathWorks' Arduino example uses P on the encoder difference to synchronise two motors on straights and disables it while turning. A university lab sweeps the gain and calibrates ticks-to-distance first. — [MathWorks drive with PID control (search excerpt)](https://au.mathworks.com/help/supportpkg/arduino/ref/drive-with-pid-control.html); [Óbuda University drive-straight lab (search excerpt)](https://www.aut.uni-obuda.hu/es/Embedded%20Systems/Advanced/14-drive-straight); [Arduino forum thread (search excerpt)](https://forum.arduino.cc/t/moving-robot-in-straight-line/290596)
- **Motion profiles:** "applying maximum power from a standstill position will cause slip," which degrades traction and wheel odometry. A motion profile caps acceleration, deceleration and velocity, and trapezoidal profiles are "sufficient" for nearly any FTC mechanism. — [CTRL-ALT-FTC motion profiling](https://github.com/BenCaunt/CTRL-ALT-FTC/blob/main/motion-profiling.md)
  - Acceleration limits are often used to prevent slippage. — [MIT constant-acceleration controller note](https://people.csail.mit.edu/baxelrod/constant-accel)
  - Trapezoidal profiles have discontinuous acceleration; S-curves (jerk-limited) are smoother. — [Trajectory-tracking notes (search excerpt)](https://vishalkumarr2.github.io/Learning_from_building_systems/robotics/learn/control-systems/08-trajectory-tracking.html)
- **Over-acceleration and fast turning (skidding)** are listed causes of slip, and UMBmark tests were deliberately run slowly (0.2 m/s) to avoid slip. — [UMBmark paper](https://cs.au.dk/~ocaprani/legolab/DigitalControl.dir/NXT/Lesson10.dir/paper58.pdf)
- **Turn termination:** an FLL team found 100 % turn speed hurt accuracy and settled on about 30 % as "a good compromise of speed and accuracy." Stopping on "angle == target" sometimes missed and gave extra full rotations; use ≥. — [Lansing Homeschool Hackers FLL](https://lansing-homeschool.github.io/fll/2021/10/19/movement.html)

### Inferences
- **Suggested controller for this robot:**
  - Generate a trapezoidal reference for the average distance s(t) = (d_L·c_L + d_R·c_R)/2.
  - Each wheel tracks s(t) ± a heading correction, where heading error = (d_L − d_R)/b_eff in calibrated cm.
  - Run the loop at about 100–200 Hz (5–10 ms; Borenstein's integration error is negligible below 10 ms).
  - Add PWM feed-forward from measured speed vs PWM, since the right motor is 4–21 % stronger.
  - The existing "kick 230 for 50 ms" is the opposite of a gentle profile. Use it only to break static friction at creep speed, or drop it.
- **Example profile values (assumed, not sourced; tune):** cruise 25–30 cm/s (about 35–40 % of the 73 cm/s no-load speed), acceleration 40–60 cm/s², creep at the end of 5 cm/s over the last 2–3 cm. A 1 m move at 30 cm/s and 60 cm/s² takes about 3.8 s.
- **Stopping accuracy:** overshoot ≈ v_final × (loop and driver latency) plus coast. At 5 cm/s with 20–50 ms latency that is 0.1–0.25 cm; at 30 cm/s it would be 0.6–1.5 cm plus coast. Ramping down to a creep, then braking (L298N both inputs equal) or holding position for 200 ms, keeps stopping error at about 0.2–0.3 cm.
- **Opto-coupled inverted PWM:** dead bands or nonlinearity at low PWM affect creep-speed control. Characterise PWM vs speed at low duty before relying on 5 cm/s.

### Gaps
- No source gives a slip-free acceleration limit for rubber tyres on painted or laminated board. The friction-based ceiling (μ·g × the drive-wheel weight fraction) needs an assumed μ, and none was sourced.
- No source quantified stopping accuracy for small DC-gearmotor robots.

---

## Q5. "Home then replay" and comparing path shapes (angled shot vs L-path vs square-then-straight)

### Takeaway
"Home then replay" works because systematic odometry errors (Es, Ed, Eb, caster bias) are nearly constant. A path tuned on the practice field from a fixed home pose builds them in, leaving only the random part plus any error in reproducing the home pose. Heading errors grow into lateral error with distance, so the best sequence puts the last heading reset as close to T1 as possible, with no large turns after it. For a 1 m shot: about 0.5° → 0.9 cm, 1° → 1.75 cm, and 10° (sonar-sum alignment, H1) → 17 cm.

### Cited Findings
- Systematic errors "stay almost constant over prolonged periods" and dominate on smooth indoor floors; non-systematic errors are the random spread. — [UMBmark paper](https://cs.au.dk/~ocaprani/legolab/DigitalControl.dir/NXT/Lesson10.dir/paper58.pdf)
- Systematic errors can hide when tests are run in only one direction, because they can compensate for each other there and add up in the other direction. That is UMBmark's reason for running both directions. — [UMBmark paper](https://cs.au.dk/~ocaprani/legolab/DigitalControl.dir/NXT/Lesson10.dir/paper58.pdf)
- Orientation errors grow without bound into lateral error; perpendicular error variance grows with the cube of distance. — [UMBmark paper](https://cs.au.dk/~ocaprani/legolab/DigitalControl.dir/NXT/Lesson10.dir/paper58.pdf); [Kleeman 1995 (search excerpt)](https://ecse.monash.edu/techrep/reports/pre-2003/MECSE-1-1995.pdf)
- FLL: after an alignment, "the next few maneuvers of the robot will be perfect, as if the robot has just started from the base" (an overstatement, but it states the principle). Battery level and backlash are among the errors reset. — [FLLCasts 1395 (search excerpt)](https://fllcasts.com/tutorials/1395-Intorduction-to-alignments-with-a-competition-robot)
- Align before high-precision moves rather than after every action, and test at the battery level you'll compete at. — [VEX tutorials – alignment strategies](https://vextutorials.vercel.app/tutorials/intermediate/alignment-strategies/)
- Drive most of the distance on odometry; use the wall as a reset, not as the distance mechanism. — [Botball/raccoon docs](https://raccoon-docs.pages.dev/01-programming-guide/02-driving/09-wall-alignment/)

### Inferences
- **Why it works and where it fails:** a tuned replay absorbs constant biases, including ones you never modelled (caster drag, uneven motors, a T1 location measured slightly off).
  - **It fails when the home pose is not reproduced.** The brief's "drive until FRONT = 110 to an object that may be an angled inner wall" defines home on a possibly slanted object with a sonar whose heading sensitivity is poor (H1). The home heading error then passes straight into T1.
  - It also fails when conditions change: the other field copy's surface, battery voltage (speed affects slip and overshoot), a different load, or dust on the tyres.
  - Random errors are never cancelled.
  - Mitigations: tune the replay at the competition battery level; recalibrate Es on the competition board if allowed; keep the replay short and turn-free.
- **Error budget for the candidate sequences, T1 about 1 m from home.** Assumptions: squaring heading σ ≈ 0.5°; sonar x after squaring σ ≈ 0.5 cm (H1 researcher to confirm); calibrated distance ±0.3–0.5 %; residual Ed ±0.1 %; calibrated turn random error σ ≈ 0.5°; stop ±0.3 cm; drop scatter ±0.5 cm.
  - **A. Sum method → home on the far object → replay** (the brief's plan): heading after the sum method is about ±10° at worst (H1). That alone gives ±17 cm at 1 m unless the replay re-squares. Expected error 5–20 cm.
  - **B. Square on the back wall, then one angled shot** (turn by atan(dx/dy), drive hypot):
    - The single calibrated turn is small: dx ≤ ±40 cm at dy ≈ 100 cm means θ ≤ 22°.
    - The systematic turn residual is about 0.2 % × 20° = 0.04°, so random turn error (σ ≈ 0.5° → 0.9 cm) dominates.
    - RSS ≈ √(0.87² [square] + 0.9² [turn] + 0.5² [x] + 0.4² [dist] + 0.3² [stop] + 0.5² [drop]) ≈ 1.6 cm.
    - Total time is about 8–10 s.
  - **C. Square on the back wall, slide sideways (turn 90°, drive, turn −90°), re-square, then a straight shot with no turns** (H3):
    - RSS ≈ √(0.87² + 0.5² + 0.4² + 0.3² + 0.5² + 0.26² [Ed]) ≈ 1.3 cm.
    - Re-square pivoting shifts x by roughly (distance from the reference point to the contact face) × sin(pre-square heading error), about 0.4 cm at 3°. A re-check of x by sonar after re-squaring catches it.
    - Costs about 8–12 s more than B (two 90° turns, a slide, a second squaring).
  - **D. L-path from a squared home** (straight d1, 90° turn, straight d2): the turn error lands on d2. With a calibrated 90° turn of σ ≈ 0.5–1° and d2 = 40–100 cm, that is 0.35–1.75 cm. Uncalibrated (3–5 % track error) it is 1.9–7.9 cm.
- **Recommended order of preference:** C ≥ B ≫ D ≫ A. B and C differ by about 0.3 cm in this budget, so if time is short B is nearly as good. C is clearly better only if turn noise turns out larger than about 0.5° on the board; measure it.
- **Practical refinement:** after the last squaring, read LEFT/RIGHT (now perpendicular to the side walls) to get the actual x. Then either (B) compute the small correction angle, or (C) iterate the slide until x is within ±0.5 cm before the final re-square.
- **The friend's approach** (funnel to one point from any start, then straight to T1) matches the generic home-then-replay pattern. Its accuracy depends entirely on how the funnel point's heading is fixed. Wall squaring is the best reference among the methods compared here.
- **Verdict on H3:** CONFIRMED in principle. Squaring on the back wall and finishing with a turn-free straight is the lowest-error sequence, at an estimated 1–2 cm (about 1.3 cm RSS under the assumptions above) against 5–20 cm for the sum method. Refinement: a single small calibrated turn after squaring (option B) costs only about 0.3 cm more and saves about 10 s. Choose C only if the time budget allows (see Q7).

### Gaps
- No published side-by-side measurement of "angled shot vs L-path vs square-then-straight" for small robots was found. The comparison is a calculated error budget.
- No source on how much the two field copies (or wall placement relative to T1) differ.

---

## Q6. Drop mechanism and drop item

### Takeaway
No sourced comparison of student drop mechanisms was found. The physics and the few sources point to: a position-controlled micro servo (SG90, about 0.1–0.12 s per 60°) opening a trap door or releasing a latch close to the board, with a soft granular item (a beanbag) or a flat, heavy item (a washer stack) released with the robot stationary. Avoid balls and anything that can roll on an edge.

### Cited Findings
- **SG90 / SG92R ratings:** 0.1 s/60° and 1.8 kgf·cm at 4.8 V per the Rapid and SG92R sheets; other TowerPro listings give 0.12 s/60° at 4.8 V and 1.2 kgf·cm. M5Stack rates 1.6 kgf·cm at 4.8 V and 1.8 kgf·cm at 6 V. The rating depends on voltage and vendor. — [Rapid SG90 datasheet](https://static.rapidonline.com/pdf/37-1330t.pdf); [SG92R datasheet](https://cse.unl.edu/~jfalkinburg/cse_courses/2019/236/SG92R_servo_datasheet.pdf); [M5Stack SG90](https://docs.m5stack.com/en/accessory/SG90_Servo)
- **Restitution:** the coefficient of restitution runs from 0 (no bounce) to 1 (perfectly bouncy). — [CSU physics demo](https://www.physics.colostate.edu/physics-demos/coefficient-of-restitution)
  - An object that does not rebound on impact has a low coefficient of restitution. — [US patent 4106769, non-rebounding article](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/4106769)
  - Jugglers prefer beanbags partly because "they don't bounce or roll when dropped" (hobbyist source, not a measurement). — [Juggling wiki](https://juggle.fandom.com/wiki/File:13_Ball_Flash_(cleaner))
  - Granular fill in a sack dissipates energy through many inelastic collisions. — [Wooster footbag thesis 2019](https://physics.wooster.edu/wp-content/uploads/2021/08/Junior-IS-Thesis-Web_2019_Shideler.pdf)

### Inferences
- **Fall time and drift:** t = √(2h/g) is 64 ms from 2 cm, 78 ms from 3 cm, 101 ms from 5 cm and 143 ms from 10 cm. Even if the robot were still moving at 5 cm/s, that adds only 0.3–0.7 cm. The bigger risks are the item sliding off a tilted trap door with a sideways kick, bouncing, or rolling. So release from about 1–3 cm, with the robot fully stopped (wait 200 ms after the hold).
- **Item ranking (inference, no measured data):**
  1. A small beanbag (rice or sand in cloth, 20–50 g): COR near 0 and it doesn't roll.
  2. A flat washer stack taped together, or a weighted disc: lands flat if released flat from a low height; can slide a little on a smooth board.
  3. A coin roll: can roll if it lands on its edge.
  4. A ball: rolls, so avoid it.
- Score is to the item's *centre*, so a compact, symmetric item helps.
- **Mechanism (inference):**
  - An SG90 is position-controlled and repeatable, and needs no limit switch. It needs about 5 V and draws current spikes; check it against the 3×AA Pico supply, or power it from a regulated 5 V rail with a common ground to the signal.
  - A plain DC motor needs a mechanical stop or a timed pulse, which is less repeatable.
  - A vertical chute or tube ending about 1 cm above the board, under the rear-axle reference point, with a servo pin or flap at its bottom, removes the trap-door sideways kick.
- **Verdict on H8:** CONFIRMED by physics and the limited sources: a flat or non-rolling item dropped low, under the reference point, from a stationary robot. A beanbag is at least as good as a washer stack and probably better on a smooth board. Neither has measured scatter data.

### Gaps
- No source compares student-robot drop mechanisms (servo trap door vs DC motor) or measures landing scatter by item type and drop height. Test on the board: 10 drops each from 2 cm, then measure the spread.

---

## Q7. Rough timing for the 90 s budget

### Takeaway
At modest speeds, squaring takes about 2–3 s, a calibrated 90° spin about 1–1.5 s plus settle, and a 1 m profiled drive about 3–5 s. Sequence C (H3 with a slide and re-square) to T1 fits in about 20–30 s, leaving about 60 s for line-following to T2. Sequence B saves about 10 s more.

### Cited Findings
- **Wall-alignment timings:**
  - Approach at 15–20 % with a 3 s timeout, then 200 ms hold and a 200–300 ms settle. — [VEX tutorials – alignment strategies](https://vextutorials.vercel.app/tutorials/intermediate/alignment-strategies/)
  - 0.3 s grace, 0.2 s push after contact, 5 s timeout. — [Botball/raccoon docs](https://raccoon-docs.pages.dev/01-programming-guide/02-driving/09-wall-alignment/)
  - About 1 s timed push at high power (FLL). — [FLLCasts 1395 (search excerpt)](https://fllcasts.com/tutorials/1395-Intorduction-to-alignments-with-a-competition-robot)
- About 30 % turn speed as the accuracy compromise (FLL). — [Lansing Homeschool Hackers FLL](https://lansing-homeschool.github.io/fll/2021/10/19/movement.html)
- Pybricks defaults are about 40 % of maximum speed. — [Pybricks robotics docs](https://docs.pybricks.com/en/latest/robotics.html)
- SG90 at 0.1–0.12 s/60°, so a 90° flap takes about 0.15–0.18 s. — [Rapid SG90 datasheet](https://static.rapidonline.com/pdf/37-1330t.pdf)

### Inferences (calculated from the brief's motor data: 200 RPM no-load at 12 V, 7 cm wheel → 21.99 cm circumference → 73 cm/s no-load)
- **1 m trapezoidal drive:** 5.5 s at 20 cm/s and 40 cm/s²; 3.8 s at 30 cm/s and 60 cm/s²; 3.0 s at 40 cm/s and 80 cm/s².
- **90° spin:** each wheel's arc is π·19.3/4 = 15.2 cm. That takes 1.5 s at 10 cm/s wheel speed, 1.0 s at 15 cm/s and 0.76 s at 20 cm/s, plus ramps and a 0.2 s settle.
- **Squaring from about 10–20 cm away at 10 cm/s:** 1–2 s of approach, about 0.3 s push and about 0.2–0.3 s settle, so about 2–3 s.
- **Sequence C estimate:**

  | Step | Time |
  |---|---|
  | Initial sonar orientation / scan | 3–8 s |
  | Back up and square on the back wall | 3 s |
  | Read x | 0.2 s |
  | Turn 90° | 1.5 s |
  | Slide (≤ 40 cm) | 2–3 s |
  | Turn −90° | 1.5 s |
  | Re-square | 3 s |
  | Read x | 0.2 s |
  | Drive about 1 m | 4 s |
  | Settle and drop | 0.7 s |
  | **Total** | **about 20–27 s** |

- **Sequence B estimate:** about 12–18 s.
- **Shutdown:** the 90 s hard stop should be a timer independent of the navigation state machine (H7).

### Gaps
- Actual speed vs PWM through the inverted opto/L298N chain, and the real loaded speed of the JGB37-520 at 12 V, are not sourced. They must be measured on the robot.
