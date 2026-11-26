# HC-SR04 behaviour against flat walls, and sonar-based heading and position in the 3-walled start box

Scope note for the report writer. Sources marked "(summary only)" were seen only as a search-engine summary or snippet; their full text could not be fetched (403/404), so treat them as weaker. All other sources were read in full or in the relevant part. The geometry numbers in "Inferences" were computed by the researcher from the brief's dimensions: box 122 cm, side sensors 11 cm from the centre, and the measured aligned sum R+L = 98.7. They are not taken from a source.

## Q1. Real beam width, minimum range, resolution, noise at 20–150 cm, and temperature effects

### Takeaway
The datasheets give 15°, 2–400 cm and 0.3 cm, and say nothing about how these were measured. Independent tests show the spread is wider and depends on the target: the full cone is about 20° horizontally for small objects. A large flat panel is still detected with the sensor turned about ±20° off its axis, levelling to about ±23.5° beyond 1.25 m. Typical noise is 0.1–0.5 cm SD, and readings have a systematic short bias of about 0.5–1.5 cm. The speed of sound changes by about 0.17 % per °C, which is about 0.17 cm per °C on the team's R+L ≈ 99 cm. That is as large as the whole geometric signal the sum method relies on.

### Cited Findings
- Datasheet (AccuDIY manual): range 2–400 cm, "Accuracy: 0.3cm", "Detect angle: 15 degree", resolution 0.3 cm, single +5 V supply, 15 mA. The trigger repeat interval "should be greater than 60ms to avoid interference between connective measurements". The trigger high level must be >2.4 V (TTL), so 3.3 V Pico GPIO is enough to trigger. — [HC-SR04 Manual, AccuDIY](https://www-users.cs.york.ac.uk/~mjf/roach/Datasheets/HCSR04_Manual.pdf)
- Datasheet (Elecfreaks): 2 cm–4 m, "ranging accuracy can reach to 3mm", measuring angle 15°, 8-cycle 40 kHz burst, `uS / 58 = centimeters`, "we suggest to use over 60ms measurement cycle". The tested object should have an area "not less than 0.5 square meters and the plane requests as smooth as possible". — [Elecfreaks HC-SR04 datasheet (SparkFun mirror)](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
- Measured (RiGonz, Arduino forum, Test 2 with the speed of sound corrected). Range 5–200 cm. Horizontal FOV about 20° full cone, vertical about 13°. Relative error about 0.5 % (−1.3 ± 4.6 %). Absolute error −0.5 to −1.5 cm (−0.4 ± 1.2 cm), so the sensor reads slightly short. Precision SD 0.1–0.5 cm. The author's verdict: "Worse in all regards than the standard specifications." Test 1 was run at floor level and gave a vertical FOV of only about 4°. — [Arduino forum: HC-SR04 tests on accuracy, precision and resolution](https://forum.arduino.cc/t/hc-sr04-tests-on-accuracy-precision-and-resolution-of-ultrasonic-measurement/236505)
- Measured (vglfr, same thread). At about 25 cm, echo times ranged 1449–1483 µs with median 1459 µs. That spread of about 34 µs is roughly 0.6 cm peak to peak, which the author quotes as "about 2.5 %". Results were "nearly consistent" down to about 3–4 cm; the wording on minimum range is ambiguous. Pulse length, interval and surface texture had no effect. — [same thread](https://forum.arduino.cc/t/hc-sr04-tests-on-accuracy-precision-and-resolution-of-ultrasonic-measurement/236505)
- Measured (GaryDyr beam tests, a servo sweep with a flat panel perpendicular to the centreline, 1° steps, 5 echoes averaged per step). The average detection half-angle was about 20°. From 60 to 125 cm it grew by about 0.15°/cm, then levelled at about 23.5° beyond 1.25 m. Sweep endpoints repeated within 5° between cycles. In static tests the target was moved sideways while kept normal to the centreline. The angle fell from about 20° at 15 cm to about 9° at 45 cm and held about 10° from 45 to 145 cm. The author notes the beam "is not radially uniform" because of lobes. — [GitHub GaryDyr/HC-SR04-beam-tests](https://github.com/GaryDyr/HC-SR04-beam-tests)
- Conflicting claims of a much wider beam. One forum poster says the main lobe of the transmitter and receiver is about 60° wide ([Arduino forum: HC-SR04 at an angle](https://forum.arduino.cc/t/hc-sr04-ultrasonic-distance-sensor-at-an-angle/233968)). Another user says about 50° (summary only: [Home Assistant community](https://community.home-assistant.io/t/esphome-water-level-sensor/126504/16)). Neither gives a measurement. SparkFun staff said they "don't make the HC-SR04" and could not confirm the beam angle (summary only: [SparkFun community](https://community.sparkfun.com/t/using-the-hc-sr04-for-level-detection-in-a-bottle/36882)).
- Speed of sound: c ≈ 331.5 + 0.6·T [m/s], said to be better than 99.8 % accurate from −20 to +40 °C. Going from 0 to 20 °C changes readings by at least 3.5 %. 100 % humidity raises c by 0.375 % at 20 °C. — [Wolles Elektronikkiste](https://wolles-elektronikkiste.de/en/hc-sr04-and-jsn-sr04t-2-0-distance-sensors)
- Supply voltage: the HC-SR04 is sensitive to a supply below 5 V. In one thread, wrong constant readings ("791") over part of a servo scan came from a missing 5 V connection, not from acoustics. — [Arduino forum: Servo+ultrasonic wrong measures at some angles](https://forum.arduino.cc/t/servo-ultrasonic-sensor-wrong-measures-at-some-angles/1178791)

### Inferences
- The brief's measured aligned sum is 98.7. Geometry predicts 122 − 22 = 100.0. The 1.3 cm deficit matches RiGonz's measured short bias of −0.5 to −1.5 cm per reading, so a ~1 cm per-sensor offset is real and should be calibrated per sensor.
- Temperature, computed with the Wolles formula and 20 °C as reference:

  | Room temperature | Reading scale | R+L ≈ 98.7 reads | FRONT 110 reads |
  |---|---|---|---|
  | 15 °C | −0.87 % | 97.8 | 109.0 |
  | 25 °C | +0.87 % | 99.6 | 111.0 |
  | 30 °C | +1.75 % | 100.4 | 111.9 |

  A 5 °C difference between the practice room and the competition hall therefore moves the sum by about 0.9 cm and FRONT 110 by about 1 cm. That uses up the plan's "±1" bands before any geometry. The known 122 cm box can be used on the day to calibrate a scale factor (for example FRONT against the back wall at two tape-measured distances), or the code can take the room temperature as a parameter.
- At 15–150 cm a large flat wall perpendicular to the sensor is an easy target. Expect ±0.3 cm noise, so a median of 3–5 readings gives roughly 0.1–0.2 cm. The limiting errors are geometric (beam and specularity) and calibration errors, not random noise.
- The brief says the Pico runs from 3×AA, about 4.5 V nominal. If the HC-SR04s are fed from that rail rather than 5 V, they are under-volted, and dropouts or garbage values become more likely. Check the supply.
- The sensors are mounted only a few cm above the board. With a vertical spread of about ±6–7° (RiGonz Test 2), the lower edge of the beam meets the board within about 25–40 cm. A smooth board at grazing incidence mostly reflects forward, but seams, the raised board edge or tape could give short echoes. Raise the sensors or tilt them up 2–3° if short ghosts appear.

### Gaps
- No peer-reviewed beam-pattern plot (dB versus angle) for the HC-SR04 was found. The beam figures above are hobbyist measurements from essentially one sensor each.
- No measured HC-SR04 data on the receive-threshold "time walk" was found. Time walk means weaker off-axis echoes cross the detection threshold later and read slightly long; it is plausible from physics but unsourced.
- A DTIC report that tests rangefinders at incidence angles of 0–40° in 10° steps exists ([DTIC AD1023784](https://apps.dtic.mil/sti/pdfs/AD1023784.pdf)), but the fetch returned 403, so its results (and whether it includes the HC-SR04) are unknown.

## Q2. Against a smooth flat wall at an angle: perpendicular distance, along-axis distance, or no echo? At what incidence angle do echoes drop out?

### Takeaway
At 40 kHz a smooth wall acts like a mirror. If the wall's normal through the sensor lies inside the sensor's two-way beam, the echo comes back from the foot of the perpendicular, and the sensor reports the PERPENDICULAR distance, not the along-axis distance. Beyond that angle there is no direct echo: the reading drops out (timeout or zero) or jumps to a long multi-bounce value. Hobbyist data put the dropout angle at about 15–25° from perpendicular, and readings are clearly wrong by 30°. This confirms the mechanism in H1. It also suggests that H1's flat zone of "±10–15°" is, if anything, conservative: for a large wall it may extend to about ±20°.

### Cited Findings
- Specular physics: Polaroid-class transducers have a half-beamwidth of about ±12.5°. Because sonar wavelengths are "well above several millimeters", objects "reflect the ultrasonic waves specularly". All that a range reading tells you is that "the reflection point lies on a circular arc". Regions of constant depth are "circular arcs that are natural features of sonar readings from specularly reflecting surfaces, first reported by Kuc and Siegel (1987)". — [Başkent & Barshan, IJRR 18(8) 1999](https://dbspl.nl/pubs/1999_IJRR.pdf)
- RCD phenomenon in a real scan: range readings for "reflections perpendicular to the walls ... are in close correspondence to distance between the sensor and the reflectors". Further, "range values are equal over a range of bearings each side of these points". RCDs form "because the sonar beam is wide", and walls behave as "specular (i.e. mirror like)" objects because "the sonar wavelength is large compared to reflector surface fluctuations". — [Reece & Durrant-Whyte 1995, Qualitative Navigation by Sensor Centric Landmark Tracking (Oxford)](https://www.qrg.northwestern.edu/papers/Files/qr-workshops/QR95/Reece_1995_Qualitative_Navigation_Centric_Landmark.pdf)
- A direct return from a specular surface occurs only near 90° incidence. At oblique incidence there is "either no return" or "a multiple specular reflection" that produces wrong measurements. — (summary only) [Univ. of Minho ITM paper](https://repositorium.sdum.uminho.pt/bitstream/1822/1690/1/ITM_01.pdf)
- At a shallow approach angle "less and less signal returns ... until at a critical angle the echo is lost entirely". The critical angle depends on material and finish, and slightly rough materials such as cardboard scatter enough to still return something. — (summary only) [RS DesignSpark: Robot navigation with sonar](https://www.rs-online.com/designspark/robot-navigation-with-sonar)
- HC-SR04 user report: the sensor works head-on to about 3 m but fails at 30° or more. At 30° "it should be reading a little more than 100 cm, but instead is reading 270". — [Arduino forum: HC-SR04 at an angle](https://forum.arduino.cc/t/hc-sr04-ultrasonic-distance-sensor-at-an-angle/233968)
- HC-SR04 room-mapping project: "a wall at more than 15 degrees off perpendicular may not return a response". The author adds "Trying to read distance to a wall at a slant definitely doesn't work well for these sensors". The wall appears "as a flat line" in the scan, and readings outside it are "all over the place". — [Hackaday.io project 19241: Mapping a room with ultrasonic distance sensors](https://hackaday.io/project/19241-mapping-a-room-with-ultrasonic-distance-sensors)
- HC-SR04 angled at a wall gave "extremely large values (~2300 cm!)", which the user attributed to sound reflecting away at a low angle of incidence. — (summary only) [Arduino forum topic 582719](https://forum.arduino.cc/index.php?topic=582719.0)
- GaryDyr's "aimed" sweep is geometrically the same as a wall seen at incidence φ: a flat panel normal to the centreline, viewed with the sensor rotated by φ, so the panel normal passes through the sensor. That gave a detection half-angle of about 20° (up to about 23.5° beyond 1.25 m). — [GitHub GaryDyr/HC-SR04-beam-tests](https://github.com/GaryDyr/HC-SR04-beam-tests)
- Contradicting claim (unsourced blog): an off-angle target makes the sound bounce "at a slightly different path" and gives a longer time-of-flight. In other words, the blog claims the reading is longer, not the perpendicular distance. — (summary only) [DEV Community: Why your ultrasonic sensor lies to you](https://dev.to/_0c004e5fde78250aee362/why-your-ultrasonic-sensor-lies-to-you-2aj2)

### Inferences
- Verdict on H1, first half: confirmed and refined. For a smooth painted or wooden wall, an HC-SR04 at incidence θ reports approximately the perpendicular distance from the sensor face to the wall (the RCD) for |θ| up to roughly 15–20°, and possibly about 23° at over 1 m. Beyond about 20–30° the reading becomes a dropout or a long multipath value.
- The "longer reading at an angle" claim in hobby blogs describes behaviour outside the visibility zone (multipath) or with rough or diffuse targets. It is not the in-zone behaviour on a smooth wall.
- Near the edge of the zone the echo is weak. Expect occasional dropouts and possibly readings a few mm too long (time walk, unsourced). Use the median of several pings and reject timeouts and 0 values.
- If the green start walls are rougher than smooth paint (corrugated card, unpainted chipboard), the visibility zone widens. Readings then move toward "nearest point in the beam", which on a plane is still the perpendicular distance. Test the actual practice walls by mounting a sensor on a protractor and stepping 0–40° in 5° steps at 30, 50 and 100 cm. This takes about 30 minutes.
- The side walls are only about 58 cm × 30 cm (0.17 m²), and the back wall is 122 × 30 cm (0.37 m²). Both are smaller than the Elecfreaks "≥0.5 m²" target recommendation. Near the up-field end of a side wall, the perpendicular foot may fall past the end of the wall: there is then no wall echo, only an edge echo or none. The back wall is the more reliable reference.

### Gaps
- None of the sources gives a measured HC-SR04 table of "reported range versus incidence angle" on a large wall. The 15–20° figure combines a forum statement, the GaryDyr sweep, and the "fails by 30°" report. The team's own protractor test is the definitive number.

## Q3. Regions of constant depth (Leonard & Durrant-Whyte) and RCD-based wall and corner detection: wall normal from the centre of the flat region, and achievable heading accuracy

### Takeaway
An RCD is a run of bearings in a rotating scan over which the range stays constant. For a plane it is centred on the wall normal, and its range is the perpendicular distance. The orientation of the wall normal is estimated as the midpoint of the RCD's bounding angles, not from the minimum of the readings, which are flat. The midpoint's accuracy is set by angular sampling, edge jitter and beam asymmetry. With slow rotation and encoder-measured angles, about 1–3° is a reasonable expectation for this robot, but no source gives an HC-SR04-specific number.

### Cited Findings
- Leonard & Durrant-Whyte's monograph, *Directed Sonar Sensing for Mobile Robot Navigation* (Springer, 1992), has chapters on a sonar sensor model, model-based localisation and directed sensing (tracking sonars). — [TUM library record](https://search.ub.tum.de/vufind/Record/DE-604.BV045187326)
- Leonard's view (1991 IEE colloquium): the most reliable features in sonar scans are "not straight line segments but circular arcs ... regions of constant depth". — (summary only) [IEE colloquium record](https://search.isc.ac/inventory/49/277891.htm)
- The thesis version reports that real sonar data (Polaroid ranging system) "can be ... predicted" and that "a simple office scene is mapped to sub-centimeter accuracy". This is a range and mapping figure, not a heading figure. — (summary only) [Leonard thesis on Scribd](https://www.scribd.com/doc/43273983/sonar-phd)
- RCD parameters, as used by later work: "the angular width, and an absolute orientation, which is the midpoint of the bounding angles", with acknowledged but unquantified uncertainties. — (summary only; attributed to a McGill/Dudek-group paper, attribution not verified in full text) [McGill IROS97 paper](https://library.cim.mcgill.ca/data/websites/Mobile%20Robotics%20Lab/pubs/dudek/iros97.pdf)
- Walls versus corners and edges: "RCDs formed by reflections from walls move tangentially with the wall and abreast of the robot. Corner and edge RCDs rotate about the" reflector. For a plane, "the bearing ... to the plane remains constant" as the robot translates, while range changes. Leonard models planes as cylinders of infinite radius and edges and corners as cylinders of zero radius. — [Reece & Durrant-Whyte 1995](https://www.qrg.northwestern.edu/papers/Files/qr-workshops/QR95/Reece_1995_Qualitative_Navigation_Centric_Landmark.pdf)
- Planes and corners can be told apart using both amplitude and time of flight (Barshan & Kuc 1990). Kleeman & Kuc (1995) used matched filtering of the full echo for target classification. — [Başkent & Barshan 1999 (literature review)](https://dbspl.nl/pubs/1999_IJRR.pdf); [Kleeman & Kuc 1995 (summary only)](https://ecse.monash.edu/centres/irrc/LKPubs/Ijrr95/Ijrr95.html). Neither technique is possible with the HC-SR04, which outputs only the first-echo time.
- HC-SR04 room scans: walls appear as flat lines (RCDs), but corners do not appear as corners. The wide beam returns "the fastest echo from the wall closest to it", so the room looks "circular instead of a square". — [Hackaday.io project 19241](https://hackaday.io/project/19241-mapping-a-room-with-ultrasonic-distance-sensors)
- Sweep edge repeatability on the HC-SR04: endpoints differed by less than 5° between sweep cycles, and the beam has lobes. — [GaryDyr beam tests](https://github.com/GaryDyr/HC-SR04-beam-tests)

### Inferences
- **Recipe for this robot (RCD midpoint).**
  1. Spin slowly, at about 20–30°/s, through about ±35° around the expected alignment.
  2. Fire only the sensor being used, every 30–35 ms (about 1° per sample).
  3. Log the heading from the wheel encoders (≈10 ticks per degree, so 0.1° resolution) together with each range.
  4. Mark a sample as "in the RCD" when the echo is valid and lies within about 1 cm of the run's flat value.
  5. Take the entry and exit headings, set the wall normal to their average, and turn back to that average on encoders.
  6. Read the perpendicular distance as the median range inside the RCD.
- Random error from about 1° sampling is about ±0.5° per edge. Edge jitter of up to about 5° per edge (GaryDyr) could move the midpoint by up to about 2.5° if only one edge moves. Doing two passes (CW then CCW) and averaging, or averaging RIGHT and LEFT RCDs, reduces this. Beam asymmetry (lobes) gives a fixed bias. Calibrate it once with the robot squared to the wall by hand or by wall squaring (H2), and subtract it.
- **Rear-axle rotation centre matters for single-sensor RCDs.** The side sensors are near the front, a distance f (probably about 14–18 cm, to be measured) ahead of the rear axle that the robot spins about. While spinning, a side sensor's perpendicular distance to its wall changes by about f·sin θ, roughly 0.26 cm per degree for f = 15 cm. So a single side sensor's RCD is not flat during a spin: it slopes. Two fixes:
  - (a) Correct each reading by f·sin θ using the encoder heading.
  - (b) Use RIGHT+LEFT as the RCD membership test, because f cancels exactly in the sum (see Q4).
  
  FRONT facing the back wall has the same issue: about 11 cm × (1 − cos θ), which is small, under 0.7 cm at 20°.
- **"Scan minimum" does not work on a smooth wall.** Inside the RCD all readings equal the perpendicular distance plus noise, so the minimum falls at a random bearing inside a zone about 30–40° wide. Only the midpoint of the zone's edges carries heading information. Minimum-seeking works only for a narrow beam or a rough wall, where readings follow d/cos θ.
- **Corners.** A 90° corner (back-wall/side-wall junction) returns energy back toward the source over a wide range of angles, because it acts as a dihedral retroreflector (basic physics, unsourced here). When a side sensor points about 45° into a back corner, both walls are outside their specular visibility zones, so the sensor reports the corner distance. That reading belongs to neither wall, and it is a plausible source of false matches in any "sum ≈ 99" test.

### Gaps
- Leonard & Durrant-Whyte's own quantitative heading or orientation accuracy for plane RCDs (with Polaroid sensors) could not be retrieved. The book and IEEE papers were not accessible.
- No source reports RCD-midpoint heading accuracy for HC-SR04-class sensors. The 1–3° estimate is the researcher's, derived from the GaryDyr edge repeatability and sampling arithmetic.

## Q4. How much RIGHT+LEFT changes for 0–20° misalignment in the 122 cm box, and whether "first sample inside the band" is biased toward the edge of the flat zone

### Takeaway
RIGHT+LEFT is an even function of the misalignment θ, so it carries no information about which way to turn. It is also almost flat near alignment. With the real (perpendicular, RCD) behaviour it changes by only 0.33 cm at 10° and 1.33 cm at 20°. Even a narrow, ideal, along-axis sensor gives only +0.47 cm at 5° and +1.9 cm at 10°. A ±1 cm acceptance band therefore accepts everything out to the edge of the visibility zone, about 17–20° (or about 7° for an ideal narrow beam). A rotate-until-in-band rule stops at the entry edge of the zone. On top of that comes the rotation between pings, which is several degrees at normal turn speed. H1's conclusion is confirmed, and the error is probably larger than H1's "~10°".

### Cited Findings
- Visibility half-angle of about 15–20° (up to about 23.5°) for a flat target, and in-zone readings equal to the perpendicular distance: see Q2 — [GaryDyr](https://github.com/GaryDyr/HC-SR04-beam-tests); [Hackaday 19241](https://hackaday.io/project/19241-mapping-a-room-with-ultrasonic-distance-sensors); [Reece & Durrant-Whyte 1995](https://www.qrg.northwestern.edu/papers/Files/qr-workshops/QR95/Reece_1995_Qualitative_Navigation_Centric_Landmark.pdf)
- HC-SR04 noise SD of 0.1–0.5 cm per reading, and per-sensor bias of −0.5 to −1.5 cm — [RiGonz tests](https://forum.arduino.cc/t/hc-sr04-tests-on-accuracy-precision-and-resolution-of-ultrasonic-measurement/236505)
- A recommended spacing of at least 29–33 ms between pings (see Q5) limits the angular sampling while spinning — [mryslab Bots-in-Pieces (summary only)](https://mryslab.github.io/bots-in-pieces/arduino,telemetrix,hc-sr04,sonar/2020/10/10/phase4-sonar.html); [Elecfreaks datasheet](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)

### Inferences
- **Geometry, computed.** Box W = 122 cm. Side sensors are at lateral offset s = 11 cm, on a common axis. The robot sits anywhere laterally, with the rotation centre at any fore-aft offset f.
  - Ideal along-axis model: R+L = W/cos θ − 2s.
  - Perpendicular (RCD) model: R+L = W − 2s·cos θ.
  - The lateral offset e and the fore-aft offset f cancel exactly in both models. Individual readings shift by ±(f·sin θ − e), but the sum does not.
  - Both sensors have the same incidence θ on their walls, so they enter and leave their RCDs together.

  | θ | Along-axis sum (Δ) | RCD/perpendicular sum (Δ) | Lateral drift per metre driven |
  |---|---|---|---|
  | 0° | 100.00 (0) | 100.00 (0) | 0 cm |
  | 2° | 100.07 (+0.07) | 100.01 (+0.01) | 3.5 cm |
  | 5° | 100.47 (+0.47) | 100.08 (+0.08) | 8.7 cm |
  | 8° | 101.20 (+1.20) | 100.21 (+0.21) | 14.1 cm |
  | 10° | 101.88 (+1.88) | 100.33 (+0.33) | 17.6 cm |
  | 15° | 104.30 (+4.30) | 100.75 (+0.75) | 26.8 cm |
  | 20° | 107.83 (+7.83) | 101.33 (+1.33) | 36.4 cm |
  | 25° | 112.61 (+12.61) | dropout/multipath likely | 46.6 cm |

  The real readings show a −1.3 cm offset (98.7 instead of 100); the deltas above are unaffected by it.
- **Acceptance band versus model.** A band that accepts sums up to +0.5 / +1.0 / +1.3 / +2.0 cm above the aligned value accepts |θ| up to:
  - 5.2 / 7.3 / 8.3 / 10.3° in the ideal along-axis model;
  - 12.2 / 17.3 / 19.8 / 24.6° in the RCD model. In practice the limit is whichever is smaller, this or the echo-visibility edge of about 15–20°.
- **Noise floor.** The noise on a sum of two readings is about √2 × 0.3 ≈ 0.4 cm SD. Even the ideal geometric signal at 5° (0.47 cm) is about one standard deviation, so the sum cannot resolve misalignment below about 5–8° even in theory. In the RCD reality it cannot resolve anything inside about ±15°.
- **Edge bias.** Rotating from outside toward alignment, the first in-band sample occurs right after both sensors start receiving specular echoes, at the near edge of the zone. That is |θ| ≈ 15–20° in the RCD model, or about 7–8° in the ideal model. There is no gradient to continue on, and the sign is unknown because the sum is even in θ. So yes, the rule is strongly biased to the edge.
- **Rotation between pings.**
  - With F, R and L fired sequentially at 33 ms each, one full R+L pair takes about 66–100 ms.
  - The JGB37-520 at 200 RPM on 7 cm wheels gives about 73 cm/s per wheel. With a 19.3 cm track that is up to about 435°/s spin at full power.
  - At 60°/s and 100 ms per cycle the robot moves 6° per sample; at 90°/s, 9°.
  - Each reading in a pair is also taken at a different heading.
  - Add motor coast after the stop command. The total stop error is plausibly 10–25°. H1's "~10°, 17 cm per metre" is a lower bound.
- **Salvaging the sum method, a small code change.** Use "both echoes valid and |R+L − 98.7| < 1.5" only as an RCD-membership flag while spinning slowly, at 20–30°/s with alternating R and L pings at 33 ms. Record the encoder heading at entry and at exit, then turn back to the midpoint (Q3).
  - The sum is the right membership test here precisely because the fore-aft offset f cancels in it.
  - At the midpoint, the lateral position follows from the difference: e = (L − R)/2, after per-sensor bias calibration.
  - Expected heading error is about 1–3° instead of 10–20° (researcher's estimate).
- **Random starts.** Starting poses near the dashed line can put a side sensor's perpendicular foot beyond the end of the 58 cm side wall. There is then no echo on that side, and the sum method never finds the band without first driving. Starts with a side sensor aimed into a back corner give corner ranges that can accidentally fall in-band. The FRONT check (FRONT ≥ 50 means facing up-field) helps reject that.

### Gaps
- No source tests the sum method itself. All Q4 numbers are computed from geometry plus the beam findings in Q1–Q2.
- The robot's actual spin rate at "turn 180", and the distance f from the rear axle to the side sensors, are not in the brief. Both change the stop-error estimate.

## Q5. Crosstalk between three HC-SR04s fired in sequence: required delay, ghost echoes, and corner echoes in a box

### Takeaway
Fire the sensors one at a time with at least 30 ms between pings; the datasheet asks for 60 ms per sensor. Never trigger them together. The bigger trap is a module-level one. When an echo is lost (which happens exactly at oblique wall angles), many HC-SR04s keep their echo pin busy for about 38 ms, and some clones for about 200 ms. Retriggering early then returns 0 or near-zero readings. The team's 15 ms software timeout does not shorten this.

### Cited Findings
- Datasheets: trigger repeat interval >60 ms "to avoid interference between connective measurements" ([AccuDIY manual](https://www-users.cs.york.ac.uk/~mjf/roach/Datasheets/HCSR04_Manual.pdf)); "we suggest to use over 60ms measurement cycle" ([Elecfreaks](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)).
- NewPing practice: 33 ms between sensor pings, where "29ms is about the minimum to avoid cross-sensor echo". — (summary only) [mryslab Bots-in-Pieces](https://mryslab.github.io/bots-in-pieces/arduino,telemetrix,hc-sr04,sonar/2020/10/10/phase4-sonar.html); library: [teckel12 NewPing](https://bitbucket.org/teckel12/arduino-new-ping)
- Module timeout: documented as 38 ms, measured at "around 200ms" on some units. Retriggering before the timeout "returns a zero or near zero reading". In a 4-sensor array with trigger pins tied together (simultaneous firing), "dead spots" and "dead directions" appeared, and simultaneous firing gave no benefit. Taking the minimum of 4 readings was used to reject "spurious readings" from "slanted walls or previous emissions". — [Hackaday.io project 19241](https://hackaday.io/project/19241-mapping-a-room-with-ultrasonic-distance-sensors)
- Even a 200 ms delay after a 180 ms timeout still let small false values (about 5–6 cm) through on some sensors. Some modules emit a short false echo after a timeout, and one user worked around it by power-cycling the sensor. — (summary only) [NewPing issue / Arduino forum reports](https://forum.arduino.cc/t/hc-sr04-with-newping-library-returning-numbers-other-than-zero-when-out-of-range/429576.md)
- Error-Eliminating Rapid Ultrasonic Firing (EERUF) is a published method for rejecting crosstalk by alternating firing delays and comparing consecutive readings. The same patent describes a robot approaching a smooth wall at an angle too large for the beam to return to the sensor. — (summary only) [US Patent 5,239,515](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/5239515)
- Wide-beam sonars return the nearest wall first, so corners are hidden in scans. — [Hackaday.io project 19241](https://hackaday.io/project/19241-mapping-a-room-with-ultrasonic-distance-sensors)
- Başkent & Barshan list "spurious readings, crosstalk, and higher-order and multiple reflections" as the commonly noted disadvantages of sonar. — [IJRR 1999](https://dbspl.nl/pubs/1999_IJRR.pdf)

### Inferences
- **Timing in this box.** The longest first-echo path is about 2 × 140 cm, roughly 8 ms. Second-order bounces across the 122 cm box arrive at about 7–15 ms. A gap of 30–35 ms between different sensors is therefore enough acoustically. Keep at least 60 ms between pings of the same sensor (per the datasheet), and add about 40 ms (or up to 200 ms on bad clones) after any timeout from that sensor.
- **A practical schedule** is F → (33 ms) → R → (33 ms) → L → (33 ms), giving about 100 ms per triple. For heading work, ping only the sensors needed: R and L alternating gives each sensor about 66 ms.
- **The software timeout is not the module timeout.** The code's 15 ms timeout corresponds to 2.57 m maximum range. That is fine for the box, but it only stops the Pico from waiting. If the module has not received an echo, its echo line can stay high for 38–200 ms. Read the echo pin before triggering, and skip the trigger if it is still high. Treat 0 and values under 3 cm as invalid.
- **Safety-rule false triggers.** The "FRONT < 6 back off" and "side < 7 reverse rotation" rules can be falsely triggered by the near-zero post-timeout readings above. Gate those rules on two consecutive agreeing readings.
- **Corner and ghost risk in the box.** A side sensor aimed into a back corner reads the corner vertex distance (a real echo, but not a wall). A sensor whose specular echo has gone (incidence over about 20°) may pick up a later second-order bounce, for example a 270 cm reading when 100 cm was expected (Q2), or the far inner walls up-field. Plausibility checks against the known box dimensions (R+L+22 ≈ 122, FRONT ≤ about 160 in the box) reject most of these.

### Gaps
- No controlled measurement of HC-SR04-to-HC-SR04 acoustic coupling versus spacing or delay was found. The 29–33 ms figure is library-author practice, and the 60 ms figure is a datasheet recommendation without a stated derivation.
- No source measures corner echoes for the HC-SR04 specifically.

## Q6. Other practical ways to get heading from walls (two sensors on one side, scan minimum, wall squaring), and which is most accurate for a cheap robot

### Takeaway
Ranking by expected accuracy:
1. Mechanical wall squaring, about 0.5–1° (H2; covered by another researcher). This is the best option.
2. Two-point side-wall difference, either two sensors on one side or one sensor plus an encoder-measured translation. About 0.5–2° after averaging, and it gives the sign of the error, so it can be used for closed-loop correction.
3. RCD midpoint from a slow scan, about 1–3°.
4. Scan minimum: unreliable on smooth walls.
5. The R+L sum: 10–20°, and no sign.

### Cited Findings
- Two-sensor baseline method: the angle to the wall is θ = tan⁻¹((a − b)/L_u), where a and b are the two sensors' readings and L_u is the spacing between them, and the true distance is h = ((a + b)/2)·cos θ. It was implemented with two HC-SR04s at L_u = 12 cm. The paper reports only a simulation table and admits "the measurement results obtained are still less precise", with readings that "change quickly" due to "disconnection of sensors in detecting objects". It is weak evidence of accuracy. — [Turnip et al., CESIT 2020, Motor Speed Control toward Wall Surface Angle based on HC-SR04](https://www.scitepress.org/Papers/2020/103709/103709.pdf)
- Two same-side range sensors used for wall parallelism in a Trinity fire-fighting robot: "Two faced left, two faced right ... The two on the left were used in keeping the robot parallel to the left wall". These were IR distance sensors. — [Hazel & Shotwell, Wellesley, Trinity 2000 entry](https://cs.wellesley.edu/~rds/rds00/projects/lhazel/paper.html)
- For a plane, the RCD bearing stays constant as the robot translates, and only the range changes. This is the basis of the "drive and watch the side range" method. — [Reece & Durrant-Whyte 1995](https://www.qrg.northwestern.edu/papers/Files/qr-workshops/QR95/Reece_1995_Qualitative_Navigation_Centric_Landmark.pdf)
- HC-SR04 per-reading SD of 0.1–0.5 cm, and per-sensor bias of −0.5 to −1.5 cm — [RiGonz tests](https://forum.arduino.cc/t/hc-sr04-tests-on-accuracy-precision-and-resolution-of-ultrasonic-measurement/236505)
- The aimed visibility half-angle of about 20° bounds the range of θ over which a side sensor sees its wall at all — [GaryDyr](https://github.com/GaryDyr/HC-SR04-beam-tests)

### Inferences
- **Why the difference method survives the beam effect.** Inside the RCD each sensor reports its own perpendicular distance, so for two sensors spaced b apart along the robot, d1 − d2 = b·sin θ exactly. In the along-axis model it would be b·tan θ; the two differ by less than 4 % at 15°. The method is valid whichever beam model holds, as long as both sensors see the wall (|θ| ≲ 15–20°).
- **Noise and bias, computed.**
  - Heading SD ≈ √2·σ/b. With σ = 0.3 cm: 2.4° (b = 10 cm), 1.6° (b = 15 cm), 1.2° (b = 20 cm) for a single reading pair. Averaging 10 pairs gives 0.8°, 0.5° and 0.4° respectively.
  - A 1 cm uncalibrated offset between the two sensors causes a bias of 5.7° (b = 10 cm), 3.8° (b = 15 cm) or 2.9° (b = 20 cm). The offset must be calibrated with the robot squared to a wall, then stored.
- **Option A, no rewiring.** Use RIGHT (or LEFT) plus the encoders. Drive straight a measured D = 20–30 cm along the side wall and take Δd = d_end − d_start. Then θ = asin(Δd/D): with D = 20 cm, 1 cm of Δd is 2.9°. Because the same sensor is used at both ends, its bias cancels. Noise is about 1–1.5° with 5-ping medians. The error depends on the robot driving straight during D; the right motor is 4–21 % faster, so this needs the encoder PID. The 58 cm side walls limit D. The same idea works sideways against the back wall only with a sensor that faces the wall while the robot drives parallel to it, i.e. the side sensors after a 90° turn.
- **Option B, the cheapest hardware change.** Move LEFT to the same side as RIGHT, 15–20 cm behind it, for example at the rear corner. This gives an instantaneous, signed heading error that can drive a closed loop ("rotate until d1 − d2 = calibrated offset") at about 0.5–1° after averaging, and lateral position from (d1 + d2)/2. The cost is losing one side's ranging. The R+L box-width cross-check then has to be done with FRONT after turning.
- **Option C, no hardware change.** Use the RCD-midpoint scan of Q3 with the encoders. It depends on angle, not range, so per-sensor range bias does not matter. Expect about 1–3°.
- **Scan minimum** is not recommended on smooth walls, because the minimum is flat across ±15–20° (Q3).
- **Best combination for this week, consistent with H2/H3.**
  1. Coarse: use the sum or RCD to find "parallel to the side walls" and which way is up-field.
  2. Back into the back wall to square mechanically (about 0.5–1°, with exact distance to the wall).
  3. Read R and L, now truly perpendicular to the side walls, for the lateral position x.
  4. Optionally verify heading with Option A while driving away from the back wall. Use sonar only to confirm, not as the primary heading reference.

### Gaps
- No source gives measured heading accuracy for a two-HC-SR04 baseline. The Turnip et al. paper gives only simulated values, and the Wellesley robot used IR sensors. The 0.5–2° figures are computed from the measured noise.
- Wall-squaring accuracy numbers (H2) were out of scope here and are left to the odometry/FLL researcher.

## Q7. Similar student competitions where robots localise from a random or unknown start using walls, and which strategies won

### Takeaway
Only weak evidence was found in this pass. In Trinity fire-fighting contest entries, wall-following plus range-based wall parallelism (two range sensors per side) and avoiding long dead-reckoning legs are recurring designs. No source documented a winning robot's start-box localisation strategy.

### Cited Findings
- The Trinity College Fire-Fighting Home Robot Contest requires autonomous navigation of a model house to find and extinguish a candle. Later Level 1 rules describe a 244 cm × 244 cm maze. — [Wikipedia: Trinity Fire Fighting Robot Competition](https://en.wikipedia.org/wiki/Trinity_Fire_Fighting_Robot_Competition); (summary only) [ASEE paper on the Trinity contest](https://peer.asee.org/the-trinity-college-fire-fighting-home-robot-contest-a-medium-for-interdisciplinary-engineering-design.pdf)
- Wellesley 2000 entry: five IR distance sensors (two left, two right, one front). The two left ones kept the robot parallel to the wall. The team "chose not to use any form of dead reckoning" for navigation, though encoders were used for turns. They did not enter because integration failed. — [Hazel & Shotwell](https://cs.wellesley.edu/~rds/rds00/projects/lhazel/paper.html)
- University of Evansville 2019 fire-fighting robot: right-wall-following algorithm with a 3.8 kHz start-tone detector. — (summary only) [Evansville report](https://evansville.edu/majors/electricalengineering/downloads/projects-2019/fire-fighting-robot-report.pdf)
- Pacific "Firebot": an ultrasonic ranger for obstacles plus a wall-following algorithm. — (summary only) [Univ. of the Pacific](https://scholarlycommons.pacific.edu/purcc/2005/events/56)
- Embry-Riddle Eagle Robotics: tracked position and stored it as a digital map so it could retrace its route to the start. — (summary only) [ERAU Discovery Day 2016](https://commons.erau.edu/pr-discovery-day/2016/posters-and-demonstrations-presentations/31)

### Inferences
- The recurring pattern is to reference to walls locally (parallelism from two side ranges, or wall following) and keep open-loop dead-reckoning legs short. That fits the brief's observation of the friend's robot, which funnelled to a home pose and then replayed a calibrated path. It also supports H3: re-reference against the long back wall immediately before the single dead-reckoned leg to T1.

### Gaps
- No winners' strategy write-ups were found for Trinity, Micromouse, FLL, IEEE SoutheastCon or university "intelligent machines" courses specifically about localising from a random start in a walled box. These were not searched in depth because of the tool-call budget; FLL/WRO wall squaring is covered under H2 by another researcher.
- H5 (rear-axle sweep radii) is pure geometry, and no external source was sought. The researcher's check, assuming the rear axle is about 4 cm from the rear edge of a 22 × 21.3 cm robot: front corners sweep about √(18² + 10.65²) ≈ 20.9 cm, and rear corners about √(4² + 10.65²) ≈ 11.4 cm. If the axle is at the very rear, the front corners sweep about 24.4 cm. This is consistent with H5's "~21 cm / ~11 cm", provided the axle position is measured. The forward offset of the sensors from the rotation centre also makes individual side readings change by about f·sin θ during a spin (Q3/Q4), which matters for any single-sensor stop rule.
