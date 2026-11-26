# Pico (RP2040, arduino-pico) firmware facts, line following through zigzags and decoys, and the T2 approach

Scope: implementation facts for the coordinator's firmware (arduino-pico core by Earle Philhower), checked against the core's own docs and source on GitHub `master` (fetched 26 Nov 2026), the pico-examples PIO encoder, the Sharp PC817, Vishay TCRT5000 and HC-SR04 datasheets, the Raspberry Pi Pico datasheet, and line-tracking write-ups. Verdicts on H6, H7 and H9 are given in the matching sections. Robot data (pins, inverted PWM, 1320 ticks/turn and so on) comes from `00_brief.md`.

## Q1. arduino-pico core APIs: PWM, interrupts, pulseIn, Servo, timing, 90 s shutdown, dual core (H7)

### Takeaway
H7 is confirmed, with refinements. `analogWrite` works on any GPIO. Its default is 1 kHz with a range of 255, so the team's existing 0–255 inverted code runs unchanged. `attachInterrupt(pin, isr, CHANGE)` works on any GPIO. `pulseIn` is a busy-wait. `Servo` is PIO-based. The hardware watchdog cannot time 90 s (its maximum is 8.3 s). Use a pico-sdk `add_alarm_in_ms` alarm plus a `millis()` check for the 90 s stop. Use `setup1()`/`loop1()` to run blocking sonar code on core 1.

### Cited Findings
**PWM (analogWrite)**
- Signatures in the docs: `void analogWriteFreq(uint32_t freq)`, `void analogWriteRange(uint32_t range)`, `void analogWriteResolution(int res)`, `void analogWrite(pin_size_t pin, int val)`. The docs give frequency as "100Hz to 1MHz" and range as "16 to 65535". — [arduino-pico Analog I/O](https://arduino-pico.readthedocs.io/en/latest/analog.html)
- The current source has defaults `analogRange = 255` and `analogFreq = 1000`. It clamps the frequency to 100 Hz…10 MHz and accepts a range of 3…65535, which differs slightly from the docs. Values are clamped to 0…scale. Writing the full-range value sets level ≥ wrap, so the pin is constantly HIGH; 0 gives a constant LOW. — [wiring_analog.cpp](https://github.com/earlephilhower/arduino-pico/blob/master/cores/rp2040/wiring_analog.cpp)
- `analogWrite` accepts any pin below `__GPIOCNT`. It sets the pin function to PWM, and the frequency and range are global (one setting for all PWM pins). Changing the frequency or range re-initialises the slices on the next `analogWrite`. — [wiring_analog.cpp](https://github.com/earlephilhower/arduino-pico/blob/master/cores/rp2040/wiring_analog.cpp)
- "While up to 16 PWM channels can be generated, they are not independent and there are significant restrictions as to allowed pins in parallel." PWM output "will continue until a `digitalWrite` or other digital output is performed". When the requested frequency and range cannot both be met, "the frequency will be preserved but the accuracy (range) will be reduced automatically". — [arduino-pico Analog I/O](https://arduino-pico.readthedocs.io/en/latest/analog.html)
- Pad drive strength can be set with `pinMode(pin, OUTPUT_2MA / OUTPUT_4MA / OUTPUT_8MA / OUTPUT_12MA)`; the default `OUTPUT` is 4 mA. Inputs offer `INPUT`, `INPUT_PULLUP` and `INPUT_PULLDOWN`. — [arduino-pico Digital I/O](https://arduino-pico.readthedocs.io/en/latest/digital.html)

**Interrupts**
- `attachInterrupt(pin_size_t pin, voidFuncPtr callback, PinStatus mode)` accepts `LOW`, `HIGH`, `FALLING`, `RISING` and `CHANGE`; `CHANGE` enables both edge events. There is also `attachInterruptParam(pin, cb, mode, void* param)`. All pins share one SDK GPIO IRQ callback, which dispatches to the per-pin callback. Registration is done with `gpio_set_irq_enabled_with_callback`. The dispatcher is placed in RAM with `__not_in_flash_func`. — [wiring_private.cpp](https://github.com/earlephilhower/arduino-pico/blob/master/cores/rp2040/wiring_private.cpp)
- `noInterrupts()`/`interrupts()` are per-core and nestable, up to 15 levels by default. — [wiring_private.cpp](https://github.com/earlephilhower/arduino-pico/blob/master/cores/rp2040/wiring_private.cpp)

**pulseIn**
- `unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout)` busy-waits on `gpio_get()` against `time_us_64()`. The timeout is in µs and runs from function entry. It covers waiting for any pulse already in progress to end, waiting for the pulse to start, and waiting for it to end. The function returns 0 on any timeout. `pulseInLong()` simply calls `pulseIn()`. — [wiring_pulse.cpp](https://github.com/earlephilhower/arduino-pico/blob/master/cores/rp2040/wiring_pulse.cpp)

**Servo**
- The Servo library is PIO-based ("glitch-free and jitter-free"). "Up to 8 Servos can be controlled in parallel assuming no other tasks require the use of a PIO machine". The docs example is `myServo.attach(D3, 540, 2400)`. — [arduino-pico Servo](https://arduino-pico.readthedocs.io/en/latest/servo.html)
- The API is `int attach(pin)`, `attach(pin, min, max)`, `attach(pin, min, max, value)`, `detach()`, `write(int)` (a value below 200 is degrees 0–180, otherwise µs), `writeMicroseconds(int)` and `attached()`. In `Servo.h` the defaults are `DEFAULT_MIN_PULSE_WIDTH 1000`, `DEFAULT_MAX_PULSE_WIDTH 2000` and `REFRESH_INTERVAL 20000` µs. min/max are clamped to 200–3000 µs. `attach` returns -1 when no PIO slot is free. — [Servo.h](https://github.com/earlephilhower/arduino-pico/blob/master/libraries/Servo/src/Servo.h), [Servo.cpp](https://github.com/earlephilhower/arduino-pico/blob/master/libraries/Servo/src/Servo.cpp)
- `tone()` also uses PIO and "must share resources with other calls such as I2S or Servo objects". — [arduino-pico Digital I/O](https://arduino-pico.readthedocs.io/en/latest/digital.html)

**Timing, alarms, watchdog, reset reason**
- The full pico-sdk ships with the core; include its headers and call its functions directly. A warning applies: "the core and libraries are not aware of any changes to the Pico you perform". — [arduino-pico Using Pico-SDK](https://arduino-pico.readthedocs.io/en/latest/sdk.html)
- pico-sdk alarm API: `typedef int64_t (*alarm_callback_t)(alarm_id_t id, void *user_data);` and `alarm_id_t add_alarm_in_ms(uint32_t ms, alarm_callback_t callback, void *user_data, bool fire_if_past)`. Related calls are `add_alarm_in_us` and `cancel_alarm`. Alarms use a default alarm pool. — [pico-sdk pico/time.h](https://github.com/raspberrypi/pico-sdk/blob/master/src/common/pico_time/include/pico/time.h)
- `rp2040.wdt_begin(uint32_t delay_ms)`: on the RP2040 "once this function has called, the hardware watchdog can _not_ be disabled". The maximum is 8300 ms; longer values are truncated. `rp2040.wdt_reset()` reloads it. `rp2040.getResetReason()` returns `RP2040::resetReason_t`. — [arduino-pico RP2040 Helper](https://arduino-pico.readthedocs.io/en/latest/rp2040.html)
- The enum is `{UNKNOWN_RESET, PWRON_RESET, RUN_PIN_RESET, SOFT_RESET, WDT_RESET, DEBUG_RESET, GLITCH_RESET, BROWNOUT_RESET}`, and the code comment says the reason "May not always be able to determine accurately". — [RP2040Support.h](https://github.com/earlephilhower/arduino-pico/blob/master/cores/rp2040/RP2040Support.h)
- `BOOTSEL` can be read as a boolean (`if (BOOTSEL) {...}`), but only by a "relatively slow" method. It is usable as a free start or teach button. — [arduino-pico BOOTSEL](https://arduino-pico.readthedocs.io/en/latest/bootsel.html)

**Dual core**
- Adding `setup1()`/`loop1()` runs code on core 1. `setup()` and `setup1()` start together. Core 1 runs no background code: "No interrupts, exceptions, or other background processing". Flash erase or write from core 0 (LittleFS/EEPROM) pauses core 1. — [arduino-pico Multicore](https://arduino-pico.readthedocs.io/en/latest/multicore.html)
- In multicore mode the 8 KB stack splits into two 4 KB stacks, and core 0 can overwrite core 1's stack. `bool core1_separate_stack = true;` gives each core 8 KB. For cross-core data use `volatile` globals or `rp2040.fifo.push/push_nb/pop/pop_nb/available`. `rp2040.idleOtherCore()` "SHOULD NOT BE USED" in most cases. — [arduino-pico Multicore](https://arduino-pico.readthedocs.io/en/latest/multicore.html)

### Inferences
- **Existing PWM code needs no change.** The default (1 kHz, range 255) already matches "255 = stop, 0 = full". `analogWrite(pin,255)` gives a solid HIGH and `analogWrite(pin,0)` a solid LOW, with no stray pulses at either end. Do not mix `digitalWrite` and `analogWrite` on an enable pin unless you mean to leave PWM mode.
- **PWM channel sharing (from the RP2040 datasheet GPIO→PWM table; not re-fetched here).** GPIO n and n+16 map to the same PWM slice and channel, so they cannot carry different duties. The two enable pins must not be 16 apart (for example GP2 and GP18).
- **90 s shutdown pattern.** Start the alarm when the run actually starts, after the start trigger, not at power-up:
  ```cpp
  #include "pico/time.h"
  volatile bool g_timeUp = false;
  const uint8_t MOTOR_PINS[6] = {/* ENA, IN1, IN2, ENB, IN3, IN4 */};
  int64_t stopAll(alarm_id_t, void*) {           // runs in IRQ context: keep it short
    g_timeUp = true;
    for (uint8_t p : MOTOR_PINS) {                 // force plain GPIO HIGH = opto ON
      gpio_set_function(p, GPIO_FUNC_SIO);         // (inverted: HIGH on EN = stop; verify on bench)
      gpio_set_dir(p, GPIO_OUT); gpio_put(p, 1);
    }
    return 0;                                      // 0 = do not reschedule
  }
  // at start of run:  add_alarm_in_ms(89000, stopAll, nullptr, true);
  // in setMotors(): if (g_timeUp) return;  and in every while-loop: if (g_timeUp || millis()-t0 > 89000) break;
  ```
  Belt and braces: every blocking loop should also check `millis() - t0`. After time-up, `loop()` should only hold the stop state, and the main code must never call `analogWrite` again.
- **Watchdog as a hang guard, not a timer.** `rp2040.wdt_begin(1000)` plus `wdt_reset()` in every loop catches a hung loop. At boot, a `WDT_RESET` (or `BROWNOUT_RESET`) reason should make the firmware stay stopped rather than restart the run, because a reboot restarts `setup()` and with it the 90 s count.
- **Fail-safe state of the opto wiring.** This relies on the ST L298 truth table, recalled from memory and not re-fetched: Ven = H with C = D gives "fast motor stop", and Ven = L gives free-run stop. While the Pico is in reset or booting, its pins are high-impedance, so all six opto LEDs are off and the 1 kΩ pull-ups take every L298N input HIGH. That gives EN = H and IN1 = IN2, which brakes the motors. Confirm this with a bench test (Pico unplugged, motor battery connected) before trusting it.
- **Sonar on core 1.** Do all `pulseIn` calls on core 1 (see Q4), and do the motor control, line sensors and encoder reads on core 0. Attach any encoder interrupts in `setup()`, which runs on core 0. pico-sdk enables GPIO IRQs on the core that calls `gpio_set_irq_enabled_with_callback`; this is SDK behaviour, not re-verified here.
- Avoid `while(!Serial);` in a battery-powered run, since it can hang with no USB host. This is general native-USB Arduino behaviour, not verified in these docs. Print only from core 0.
- `millis()`/`micros()` are available. The core's timing uses the 64-bit µs hardware timer (`time_us_64()` in `pulseIn`). For µs intervals, use `time_us_64()` to avoid 32-bit wrap.

### Gaps
- No official arduino-pico figure was found for `attachInterrupt` latency in µs. The element14 latency article returned HTTP 503, and searches gave no measured number.
- The core docs and source disagree slightly on limits (docs: 100 Hz–1 MHz and range 16–65535; code: up to 10 MHz and range from 3). Either way it does not matter at 1 kHz with range 255.
- Whether `BROWNOUT_RESET` is detected on the RP2040 (as opposed to the RP2350) was not checked.

## Q2. Quadrature decoding at 1320 ticks/turn: interrupts vs PIO

### Takeaway
Both methods work at this rate. PIO is the robust choice because it is immune to CPU latency, flash stalls and interrupts being disabled, and the team's encoder pins (GP12/13 and GP14/15) are already the consecutive pairs PIO needs. Use the `rp2040-encoder-library` (`PioEncoder`), or keep an interrupt state-table decoder if it already works. About 4,400 edges/s per wheel is trivial for either.

### Cited Findings
- The pico-examples `quadrature_encoder.pio` uses a 16-entry computed-jump state table. It "must be loaded at address 0" (`.origin 0`). It handles "step rates up to sysclk / 10 (e.g., sysclk 125MHz, max step rate = 12.5 Msteps/sec)". The init uses `pin` and `pin + 1`, enables pull-ups on both, and has a 32-bit wrapping count. `quadrature_encoder_get_count()` drains the RX FIFO and returns a fresh value. — [quadrature_encoder.pio](https://github.com/raspberrypi/pico-examples/blob/master/pio/quadrature_encoder/quadrature_encoder.pio)
- The pico-examples demo states that the A phase goes on the base pin and "The B phase must be connected to the next pin". It also says the PIO approach "requires zero CPU time to keep the encoder count updated". Its example use is a two-wheel robot controlled at 100 Hz that subtracts successive counts. — [quadrature_encoder.c](https://github.com/raspberrypi/pico-examples/blob/master/pio/quadrature_encoder/quadrature_encoder.c)
- `rp2040-encoder-library` by G. Bruno, version 0.2.0, `architectures=mbed_rp2040, mbed_nano, rp2040`, include `pio_encoder.h`. Its README says "encoder pins must be consecutive (e.g. 2 and 3, 10 and 11 etc.)". — [gbr1/rp2040-encoder-library](https://github.com/gbr1/rp2040-encoder-library), [library.properties](https://github.com/gbr1/rp2040-encoder-library/blob/main/library.properties)
- Its API is `PioEncoder(const uint8_t pin, const bool flip=false, const int zero_offset=0, const uint8_t count_mode=COUNT_4X, PIO pio=pio0, const uint sm=-1, const int max_step_rate=0)`, with `begin()`, `getCount()`, `reset(int=0)`, `flip(bool)` and `setMode(COUNT_1X/2X/4X)`. — [pio_encoder.h](https://github.com/gbr1/rp2040-encoder-library/blob/main/src/pio_encoder.h)
- `begin()` loads the program once per class with `pio_add_program(pio, ...)` (no space check) and claims a state machine with `pio_claim_unused_sm(pio, true)`, which panics if none is free. `getCount()` returns `flip * (raw >> count_mode) + zero_offset`. — [pio_encoder.cpp](https://github.com/gbr1/rp2040-encoder-library/blob/main/src/pio_encoder.cpp)
- Servo claims a PIO state machine and program space through the core's `PIOProgram::prepare()` and fails with -1 when there is no room. — [Servo.cpp](https://github.com/earlephilhower/arduino-pico/blob/master/libraries/Servo/src/Servo.cpp)
- A missed transition makes a quadrature decoder lose track of position, so ISR design matters at high count rates (about 11.2 k counts/s in that article). This was seen in a search snippet; the page was not opened. — [DigiKey forum article](https://forum.digikey.com/t/design-a-robust-quadrature-encoder-isr-based-program-for-arduino-motor-control/39913)

### Inferences
- **Rate budget.** 200 RPM × 1320 ticks/rev ÷ 60 = 4,400 counts/s per wheel at rated no-load speed. Each phase is then a 1.1 kHz square wave, or 2,200 edges/s per pin. CHANGE interrupts on both pins of both wheels give about 8,800 IRQs/s, one every ~114 µs on average. Edges on one wheel are about 227 µs apart. A few µs of ISR time each comes to roughly 2–5 % of one core, so interrupts are fine unless something disables interrupts for longer than ~0.2 ms. The worst culprit is a flash write such as `EEPROM.commit()`; never do one while moving. PIO's margin is about 2,800× (12.5 M steps/s).
- **PIO pattern (recommended).** Construct and `begin()` the encoders before `Servo.attach()`. The 24-instruction quadrature program must sit at offset 0, so it should be loaded into a PIO block before anything else; alternatively pass `pio1` explicitly.
  ```cpp
  #include <pio_encoder.h>
  PioEncoder encL(12);          // A=GP12, B=GP13 (flip if forward counts negative)
  PioEncoder encR(14, true);    // A=GP14, B=GP15, mirrored motor
  void setup() { encL.begin(); encR.begin(); /* then servo.attach(...) */ }
  // in control loop: int32_t l = encL.getCount(), r = encR.getCount();
  ```
  The library is not in the arduino-pico distribution. Install it with the Library Manager or as a ZIP and test it on the bench first. If it is unavailable, copy the pico-examples `.pio` file, assemble it with the bundled `pioasm` or the online wokwi pioasm ([arduino-pico SDK page](https://arduino-pico.readthedocs.io/en/latest/sdk.html)), and include the generated header.
- **Interrupt pattern (fallback, no library).** Use the same transition table as the PIO program. Read both pins in one atomic `gpio_get_all()` call, then index with (old << 2) | new:
  ```cpp
  static const int8_t QDEC[16] = {0,-1,+1,0, +1,0,0,-1, -1,0,0,+1, 0,+1,-1,0};
  volatile int32_t cntL = 0; volatile uint8_t stL = 0;
  void __not_in_flash_func(isrL)() {
    uint8_t s = (gpio_get_all() >> 12) & 3;   // bit0 = GP12 (A), bit1 = GP13 (B)
    cntL += QDEC[(stL << 2) | s]; stL = s;
  }
  // setup(): stL = (gpio_get_all()>>12)&3; attachInterrupt(12, isrL, CHANGE); attachInterrupt(13, isrL, CHANGE);
  ```
  32-bit aligned reads of `cntL` are atomic on the Cortex-M0+, so the control loop can read it without disabling interrupts. Sign conventions must be checked on the robot.
- **Electrical check.** RP2040 GPIOs are 3.3 V. If the JGB37-520 Hall encoders are powered at 5 V with on-board pull-ups to 5 V, their outputs need dividers or a 3.3 V supply. This was not verified for this specific encoder board.
- Two days before the event: if the current ISR code counts correctly in a "push robot 1 m by hand, compare ticks" test, keep it. Switch to PIO only if counts drift at speed.

### Gaps
- No primary-source latency figure for arduino-pico GPIO ISRs (see Q1).
- No reliability reports specific to the `rp2040-encoder-library` beyond its README; its last push was 2024-11-12 according to GitHub search.

## Q3. PC817 switching speed with a 1 kΩ pull-up: usable PWM frequency for the L298N enable

### Takeaway
With RL = 1 kΩ the PC817's rise and fall times are each about 8–10 µs (read from the datasheet graph), and its −3 dB bandwidth is a few tens of kHz. At 1 kHz (arduino-pico's default) duty distortion is about 1–3 % of the period. At 5 kHz it is about 10 %, and at 20 kHz or more it is unusable near the ends of the range. H7's "~1 kHz OK" is confirmed. Stay at 1–2 kHz.

### Cited Findings
- Sharp PC817 at VCE = 2 V, IC = 2 mA, RL = 100 Ω: rise time tr typ 4 µs (max 18 µs), fall time tf typ 3 µs (max 18 µs). The cut-off frequency fc (VCE = 5 V, IC = 2 mA, RL = 100 Ω, −3 dB) is typ 80 kHz. — [Sharp PC817XxNSZ1B datasheet](https://global.sharp/products/device/lineup/data/pdf/datasheet/PC817XxNSZ1B_e.pdf)
- Fig. 13 (Response Time vs Load Resistance) shows tr and tf growing roughly in proportion to RL. Read off the graph, they are about 8–10 µs at 1 kΩ and about 30–40 µs at 10 kΩ, with td about 3 µs and ts about 1–2 µs at 1 kΩ. Fig. 15 (Frequency Response) puts the 1 kΩ curve's −3 dB point at roughly 20–30 kHz. The datasheet remarks that "all data in the graph are just for reference and not for guarantee". — [Sharp PC817XxNSZ1B datasheet](https://global.sharp/products/device/lineup/data/pdf/datasheet/PC817XxNSZ1B_e.pdf)
- CTR is a minimum of 50 % at IF = 5 mA, VCE = 5 V. LED VF is typically 1.2 V at 20 mA. VCE(sat) is 0.1–0.2 V at IF = 20 mA, IC = 1 mA. — [Sharp PC817XxNSZ1B datasheet](https://global.sharp/products/device/lineup/data/pdf/datasheet/PC817XxNSZ1B_e.pdf)
- arduino-pico's default PWM frequency is 1000 Hz. — [wiring_analog.cpp](https://github.com/earlephilhower/arduino-pico/blob/master/cores/rp2040/wiring_analog.cpp)

### Inferences
- Assume the combined turn-on plus turn-off delay mismatch at 1 kΩ is about 10–40 µs, allowing extra storage time when the transistor is driven hard into saturation (Fig. 13 is measured in the active region). The duty error is then roughly 1–4 % at 1 kHz (1000 µs period), 2–8 % at 2 kHz and 5–20 % at 5 kHz. Pulses shorter than about 20–40 µs may vanish, giving a small dead band near "stop" (255) and near "full" (0). The PID already absorbs the 4–21 % left/right motor mismatch, so 1 kHz is fine. Avoid tens of kHz.
- **Saturation margin.** Pulling a 1 kΩ load to 5 V needs IC ≈ 4.8 mA. At the minimum CTR of 50 % that means IF ≈ 10 mA. If the LED resistor gives only about 3–5 mA, a low-CTR part may not pull the L298N input fully low. Measure the L298N input voltage at duty 0 and 255 with a multimeter. If it is marginal, lower the LED resistor and use `pinMode(pin, OUTPUT_8MA/12MA)` before `analogWrite` ([Digital I/O](https://arduino-pico.readthedocs.io/en/latest/digital.html)). That the pad-strength setting survives `analogWrite` is an inference: `analogWrite` only changes the pin function.
- Lowering the pull-up to about 470 Ω would speed up the edges but needs more LED current, so do not change it two days before the event.

### Gaps
- Fig. 13 values are read by eye from a log-log graph (approximate). No datasheet figure covers deep saturation (storage time at IF ≫ IC/CTR).
- The L298 input thresholds (VIL/VIH) were not re-fetched.

## Q4. HC-SR04 on a 3.3 V MCU: firing three in sequence, gaps, non-blocking reads, median filtering

### Takeaway
Fire the three sensors round-robin from core 1 with `pulseIn(echo, HIGH, ~20000)` and leave at least about 30 ms between pings. NewPing gives 29 ms as the minimum; the datasheet suggests a measurement cycle over 60 ms. Take a median of 3 while moving and of 5–9 when stationary or teaching. Publish results through `volatile` variables so core 0's motor and line loop never blocks.

### Cited Findings
- HC-SR04 datasheet: working voltage DC 5 V, 15 mA. Range is 2 cm to 4 m, and the "ranging accuracy can reach to 3mm". Measuring angle is 15°, and the trigger is a ≥10 µs TTL pulse. The module sends "an 8 cycle burst of ultrasound at 40 kHz". Range = high time × 340 m/s / 2. It suggests "over 60ms measurement cycle, in order to prevent trigger signal to the echo signal". Targets should have an area of at least 0.5 m². — [HC-SR04 datasheet (SparkFun)](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
- NewPing's example says: "Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings." — [NewPingExample.ino](https://bitbucket.org/teckel12/arduino-new-ping/src/master/examples/NewPingExample/NewPingExample.ino)
- NewPing defines `PING_MEDIAN_DELAY 30000` (µs between pings in `ping_median`), `MAX_SENSOR_DELAY 5800` ("Maximum uS it takes for sensor to start the ping") and `ECHO_TIMER_FREQ 24`. `ping_median(iterations=5)` does multiple pings, discards out-of-range pings and returns the median. — [NewPing.h](https://bitbucket.org/teckel12/arduino-new-ping/src/master/src/NewPing.h)
- arduino-pico `pulseIn` busy-waits, returns 0 on timeout, and its timeout includes the wait for the echo to start (see Q1). — [wiring_pulse.cpp](https://github.com/earlephilhower/arduino-pico/blob/master/cores/rp2040/wiring_pulse.cpp)
- Core 1 has no background interrupts, so busy-waiting there does not disturb core 0. — [arduino-pico Multicore](https://arduino-pico.readthedocs.io/en/latest/multicore.html)

### Inferences
- **3.3 V operation.** Power the sensors at 5 V as the datasheet specifies, and keep the echo dividers (the Pico is not 5 V tolerant). A 3.3 V trigger exceeds the TTL VIH of 2.0 V, so driving the trigger directly from the Pico normally works. That is an inference from TTL levels; the datasheet does not state it.
- **Timeout.** With the existing 15 ms timeout, the maximum range is about 15 ms × 343 m/s ÷ 2 ≈ 2.5 m, less the sensor's start delay. That is enough on a 2.44 m board, but FRONT readings in the open up-field direction can time out and return 0. Treat 0 as "no reading", never as 0 cm. In the start box the long reading was 138.8 cm, about an 8 ms round trip.
- **Round-robin on core 1 (pattern):**
  ```cpp
  volatile uint16_t g_sonarMm[3] = {0,0,0}; volatile uint32_t g_sonarSeq = 0;
  const uint8_t TRIG[3] = {/*F,L,R*/}, ECHO[3] = {/*F,L,R*/};
  void setup1() { for (int i=0;i<3;i++){ pinMode(TRIG[i],OUTPUT); digitalWrite(TRIG[i],LOW); pinMode(ECHO[i],INPUT);} }
  void loop1() {
    static uint8_t i = 0; static uint16_t hist[3][3]; static uint8_t k[3];
    uint32_t t0 = millis();
    digitalWrite(TRIG[i], HIGH); delayMicroseconds(10); digitalWrite(TRIG[i], LOW);
    unsigned long us = pulseIn(ECHO[i], HIGH, 20000UL);          // 0 = no echo
    uint16_t mm = us ? (uint16_t)(us * 0.1715f) : 0;             // 343 m/s / 2 = 0.1715 mm/us
    hist[i][k[i]++ % 3] = mm;
    uint16_t a=hist[i][0], b=hist[i][1], c=hist[i][2];           // median of 3
    g_sonarMm[i] = max(min(a,b), min(max(a,b), c));
    g_sonarSeq++;
    i = (i + 1) % 3;
    while (millis() - t0 < 30) { }                               // >=30 ms per slot -> each sensor ~11 Hz
  }
  ```
  Exclude zeros from the median, or count them, when deciding whether the value is valid. Core 0 reads `g_sonarMm[]`; a 16-bit aligned read is atomic. Teach mode and final stops should use median-of-5 to 9 taken while stationary.
- **Why core 1 matters for line following.** A blocking `pulseIn` on core 0 freezes the line loop for up to 15–20 ms. At 30 cm/s the robot moves 4.5–6 mm in that time, and a 2 cm line crossed at an angle passes under a sensor in about 30–60 ms. A frozen loop can miss a zigzag corner entirely.
- Cross-talk between the three sensors is reduced by the 90° mounting and the 30 ms gaps. Inside the narrow passage, multipath is still likely (see Q6).
- **Non-blocking alternative on one core.** Put a CHANGE interrupt on each echo pin that records `time_us_64()` on the rising and falling edges. This is more complex; the core 1 approach is simpler and proven.

### Gaps
- The "no echo" behaviour (echo held HIGH for about 38 ms or longer on many modules) varies between HC-SR04 clones and was not found in the SparkFun datasheet. If timeouts are frequent, a sensor may still be busy when it is next triggered; the 30 ms slot plus round-robin gives it about 90 ms.
- NewPing's RP2040 compatibility was not verified. The pattern above does not need it.

## Q5. Line following with 2 vs 3 digital IR sensors: zigzags, gaps, crossings, decoys, border (H6)

### Takeaway
H6 is confirmed by the classic line-tracking literature. Two straddling sensors handle curves and turns below 90°, but not turns of 90° or more, line ends or crossings, unless the code remembers which side last saw the line. A third centre sensor resolves sharp turns versus line ends. The cheapest and most important fix is "last seen side" memory plus slower speed in the zigzag zone. Reject decoys and the border by gating on expected position and heading from odometry; no published source was found for decoy gating, so that part is inference.

### Cited Findings
- Two sensor pairs "can track straight lines, curvy lines, and lines with angled turns of less than 90 degrees… It also can't properly follow angles of 90 degrees or greater". At a sharp turn or line end, "the sensor pairs may both run off of the line at the same time", or exit in either order, so the robot may "turn around and follow the line backwards". — [Dubel, "Reliable Line Tracking", UF IMDL](https://mil.ufl.edu/5666/handouts/line_track.pdf)
- A three-pair module is arranged so that "at least one sensor will detect the line at any given time". It can handle "end-of-line and sharp turn situations… At a sharp turn, the right or left sensor will have to detect the line, at least by the time that the center sensor loses the line." — [Dubel, UF IMDL](https://mil.ufl.edu/5666/handouts/line_track.pdf)
- With three sensors, intersections are ambiguous "if the robot… is off track before entering an intersection". The robot cannot tell a turn from an intersection unless the centre sensors are aligned; a four-pair layout (two centre, two outer) fixes this. — [Dubel, UF IMDL](https://mil.ufl.edu/5666/handouts/line_track.pdf)
- "Keeping track of the previous direction is important because the robot may run completely off the line faster than its motors can react… This is what most otherwise decent line-tracking programs are missing." — [Dubel, UF IMDL](https://mil.ufl.edu/5666/handouts/line_track.pdf)
- Mount the module "as far forward as possible". "The closer the line detection is to the center of rotation for the robot, the less stable your robot will be when tracking a line". Keep sensors close to the surface, and an angle of 15–30° may reduce glare. Two-sensor modules should be designed for "the thickness of the line you intend to use". — [Dubel, UF IMDL](https://mil.ufl.edu/5666/handouts/line_track.pdf)
- Two sensors that "straddle the line" work only when there are "no crossed lines and no abrupt turns". The robot "needs to be able to remember if it saw a line on the left or right recently if it looses the line altogether". If the line was seen on neither side recently, the robot is probably at a gap; if it was seen more recently on one side, the turn is that way. With a centre sensor plus history, the robot handles "crossed lines and sharp angles up to but not including 90 degrees"; "For 90 degree turns and sharper angles more will be required." — [KPU APSC1299 Sumovore line following](https://kpu.ca/science/physics/coursemats/apsc1299/sumovore)
- A student report found two-sensor algorithms worked far better than one sensor, but the robot lost the line on sharp corners when driven fast. A Hackaday log says two sensors cause "a lot of wobbling" and recommends three: one on the line and one either side. Both come from search snippets; the pages were not opened. — [UNR lab report](https://www.cse.unr.edu/~dbigelow/Archive/Labs/report2.pdf), [Hackaday log](https://hackaday.io/project/26057/log/63910)
- TCRT5000: "Peak operating distance: 2.5 mm"; "Operating range within > 20 % relative collector current: 0.2 mm to 15 mm"; daylight blocking filter. — [Vishay TCRT5000 datasheet](https://www.vishay.com/docs/83760/tcrt5000.pdf)
- arduino-pico analog inputs are on a fixed set of pins (A0…A3; GP26–29, of which GP26–28 are free on this robot) and read 0–1023 by default. — [arduino-pico Analog I/O](https://arduino-pico.readthedocs.io/en/latest/analog.html)

### Inferences
- **Current layout.** The two sensors sit either side of the front sonar, so they are probably at least about 5 cm apart (measure this). That makes them a straddle layout: both see white when the robot is centred on a 2–4 cm line. Straddling needs an inner gap greater than the widest line (4 cm) plus the sensor spot. The cost is a dead band of (gap − line width) on each side, e.g. ±1.5–2 cm wobble on a 2 cm line, which is acceptable. The rear-axle drive with front sensors gives a long lever arm, which the UF note says is the stable arrangement.
- **Adding a third sensor (if time allows; GP22 digital or GP26 analog spare).** Mount it on the centre line, under or just in front of the sonar, a few mm above the board (TCRT5000 peaks at 2.5 mm). Centre-to-centre spacing should be about the line width plus 0.5 cm, so a 2 cm line is always under at least one sensor. Measure the real line width on the practice field first. Then:
  - C only: straight.
  - C+L or L only: steer left.
  - C+R or R only: steer right.
  - L+C+R: crossing, corner or border; go to the "all black" handler.
  - None: lost; go to the memory handler.
- **Two-sensor algorithm with memory (no hardware change), for the zigzag:**
  1. FOLLOW. If L is black, steer left and set `lastSide = L`; if R is black, steer right and set `lastSide = R`. If both are white and the distance since the last black (from the encoders) is under about 3 cm, keep going straight, because the line is between the sensors.
  2. LOST. If both have been white for more than about 3–4 cm of travel, stop. Pivot toward `lastSide` (spinning about the axle centre sweeps the front sensors through an arc) until a sensor sees black or an encoder limit is reached (about 120–150°). If nothing is found, pivot back the other way, up to twice that angle. If still nothing, reverse along the recorded path.
  3. BOTH BLACK. This is a corner, a crossing decoy or the border; decide with the gates below.
- **Speed.** Choose loop period × speed ≪ line width. A loop under 2 ms at 20 cm/s moves 0.4 mm per loop. Use the odometry distance along the line to slow down (to about 50–60 % power) when entering the zigzag section, which is known from the field drawing. Using pivots at sharp corners, rather than PID arcs, avoids overshoot.
- **Decoy and crossing rejection (H6 gating).**
  - Acquire the line only inside an expected window, for example ±10 cm of the expected crossing point and heading within ±30°, using encoder odometry from T1.
  - While following, keep a running "distance along line". Any black-on-both event (or all three with a centre sensor) that is not expected at that distance is treated as a crossing: drive straight 5–6 cm holding heading on the encoders, then resume FOLLOW.
  - Ignore any line reacquired after a LOST search that lies outside an encoder-limited sweep angle.
- **Border (5 cm) vs line (2–4 cm).** When crossed perpendicular, the black run is 5 cm against 2–4 cm (dwell distance ≈ width ÷ sin(crossing angle)). That is too close to separate reliably at unknown angles. Use position instead: the border lies within 5 cm of the board edge, which odometry or the side and front sonars to walls can confirm. Treat a both-black event near the field edge as "border: stop or back off". With straddled sensors at a gap of 4 cm or more, a 2–4 cm line crossed perpendicularly cannot cover both sensors at once, but the border (5 cm) can if the gap is under 5 cm. This weak cue only helps when the crossing angle is close to perpendicular.
- **Analog option.** The modules' analog outputs on GP26–28 would allow per-surface threshold calibration at start-up (sample white and black). This is optional; digital modules with the potentiometer set on the actual field are fine.

### Gaps
- No primary source quantifies the maximum speed for a given sensor spacing on zigzags. Tuning on the practice field is required.
- No published method was found specifically for "decoy lines" in competition rules. Odometry gating is a standard engineering inference, not a cited technique.
- The exact zigzag geometry (angles, segment lengths) is not known from the brief. If corners are 90° or sharper, the two-sensor scheme depends entirely on the LOST pivot search.

## Q6. Finding T2 at the end of a line in a narrow passage (front sonar, side sonars, teach mode) (H9)

### Takeaway
H9 is refined. Teach mode is sound, but only the FRONT reading (and odometry distance from the passage entry) should be trusted. In a passage the robot "just fits", the side sonars will be at or below the HC-SR04's 2 cm minimum range and prone to multipath. Let the line (which runs along the passage centre) do the centring, and stop on a median-filtered FRONT distance taught by hand. Contact with the end wall is a more accurate alternative if T2 is a fixed offset from it.

### Cited Findings
- The HC-SR04 minimum range is 2 cm, accuracy is up to 3 mm, and the measuring angle is 15°. Targets of at least 0.5 m² are recommended. — [HC-SR04 datasheet](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
- The simulated EEPROM is one 4 KB flash sector, with `EEPROM.begin(256…4096)`, `EEPROM.put/get(addr,val)` (for structs), `EEPROM.commit()` and a flash-wear warning ("do not frequently update"). — [arduino-pico EEPROM](https://arduino-pico.readthedocs.io/en/latest/eeprom.html)
- Flash writes from core 0 pause core 1. — [arduino-pico Multicore](https://arduino-pico.readthedocs.io/en/latest/multicore.html)
- `BOOTSEL` can be read as a boolean in sketches, making it usable as a "teach" button. — [arduino-pico BOOTSEL](https://arduino-pico.readthedocs.io/en/latest/bootsel.html)

### Inferences
- **Side sonars in the passage.** The robot is 21.3 cm wide and the side sensor faces are about 11 cm from the centre, so the passage leaves at most a few cm per side, which is at or below the 2 cm minimum. Echoes between parallel walls 15 cm high are likely to cause multipath. Ignore LEFT and RIGHT inside the passage, or use them only as a "wall present" flag. Centre on the line instead, since the brief says it runs along the passage centre line.
- **FRONT to the wall ahead.** The FRONT sonar is mounted low and sees the 15 cm walls, and its 15° cone has a radius of about 0.13·d at distance d. The passage side walls are grazing-incidence (specular), so they normally do not return echoes. A long passage could still produce a side-wall return, which the teach test will reveal.
- **Teach mode procedure.**
  1. Hold BOOTSEL at power-up to enter teach mode.
  2. Place the robot by hand with the pointer on T2.
  3. Take 9 FRONT pings while stationary, store the median, and optionally store LEFT and RIGHT too.
  4. `EEPROM.put` the values and `commit()` once, while stationary.
  5. At run time, after the line enters the passage, slow to a creep speed. Stop when the median-of-3 FRONT is at or below the taught value plus a small lead (calibrate the stopping overshoot at creep speed). Check plausibility against the encoder distance travelled since passage entry.
  6. Store the taught value also as a hard-coded default in case the EEPROM is blank.
- **Wall-contact alternative (ties to H2).** If T2 is at a fixed distance from the passage end wall, drive slowly into the end wall until the encoders stall, then back off a taught number of ticks. This avoids sonar noise entirely; the rear-wheel drive and front caster mean the front face makes contact. A cheap bumper or skid on the front corners helps the robot slide along the passage walls.
- **Deploying the pointer.** Unfold it only after stopping, and ensure the line and sonar logic cannot drive after deployment. The 25 cm cube and pointer geometry are covered by other researchers.

### Gaps
- No published source was found on HC-SR04 behaviour inside narrow corridors (multipath statistics), so the side-sonar concern is an inference from the minimum range and specular reflection.
- The exact T2 position relative to the passage walls is unknown from the brief; it decides whether FRONT stop or wall contact is better.

## Q7. SG90 servo powered from the same 3xAA supply as the Pico: brown-out risk and mitigation

### Takeaway
The Pico itself is fairly tolerant: VSYS accepts 1.8–5.5 V through an on-board buck-boost. The real risks are wiring 3xAA into the 3V3 pin instead of VSYS, sag disturbing anything else on that rail (the HC-SR04s need 5 V), and an unexpected reset restarting the run. Mitigations: a 470–1000 µF capacitor at the servo, moving the servo only while the drive motors are stopped, `detach()` after the drop, ideally a separate servo battery with a shared ground, and a reset-reason check at boot.

### Cited Findings
- "VSYS is the main system input voltage, which can vary in the allowed range 1.8 V to 5.5 V, and is used by the on-board SMPS". The RT6150 buck-boost generates the fixed 3.3 V. "If the USB port is not going to be used, it is safe to power Pico by connecting VSYS to your preferred power source (in the range ~1.8 V to 5.5 V)". A second source can be OR-ed into VSYS through a Schottky diode. — [Raspberry Pi Pico datasheet](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf)
- The HC-SR04 working voltage is DC 5 V at 15 mA. — [HC-SR04 datasheet](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
- arduino-pico `Servo::detach()` sets the pulse width to 0 and stops the state machine, so no more pulses are sent. `attach(pin, min, max, value)` sets an initial position. — [Servo.cpp](https://github.com/earlephilhower/arduino-pico/blob/master/libraries/Servo/src/Servo.cpp)
- Forum and vendor sources describe the mechanism: sudden servo current makes the supply dip and the MCU resets (brown-out). They suggest a bulk capacitor (470 µF/16 V on a 5 V rail, or ≥100 µF), and say a capacitor cannot compensate for an overloaded supply. They quote SG90 stall or start current of about 0.5–0.8 A, including "650 ± 80 mA" at 5 V. These come from search snippets only; the pages were not opened, so treat the numbers as indicative. — [Arduino forum: servos and ESP32 resetting](https://forum.arduino.cc/t/servos-and-esp32-constantly-resetting/1249421), [Sizing micro servo power for Pico](https://industrialmonitordirect.com/blogs/knowledgebase/sizing-micro-servo-power-for-raspberry-pi-pico-control)
- `RP2040::resetReason_t` includes `PWRON_RESET`, `WDT_RESET` and `BROWNOUT_RESET`, and detection "May not always be able to determine accurately". — [RP2040Support.h](https://github.com/earlephilhower/arduino-pico/blob/master/cores/rp2040/RP2040Support.h)

### Inferences
- **Risk assessment.** If 3xAA feeds VSYS, a stall of about 0.7 A through the cells' internal resistance might sag the rail by a few hundred mV to about 1 V. That is still well above VSYS's 1.8 V minimum, so a Pico reset from the servo alone is unlikely. It is not impossible with weak cells, long thin wires or a poor holder contact. If the HC-SR04s or the PC817 LED supply hang off the same 4.5 V rail, the sag can corrupt sonar readings during the servo move.
- **Never feed 3xAA into the Pico's 3V3 pin.** 4.5 V exceeds the 3.3 V domain; use VSYS (pin 39). This is an inference from the datasheet power architecture; confirm the robot's wiring.
- **Mitigations, in order of effort:**
  1. Move the servo only when the drive motors are stopped (drop at T1, then continue).
  2. Put 470–1000 µF electrolytic plus 100 nF across servo V+/GND at the servo connector.
  3. Use `servo.attach(pin, 500, 2400, startAngle)` just before the drop and `servo.detach()` about 0.5 s after. This removes holding and stall current. Set end-points so the servo never buzzes against a hard stop.
  4. Fresh alkaline cells on competition day.
  5. Best: a separate 4xAA or 5 V supply for the servo, with its ground tied to the Pico ground (needed for the signal). Keep it separate from the 12 V motor battery to preserve the opto isolation.
  6. In `setup()`, if `rp2040.getResetReason()` is not a normal power-on or RUN-pin reset, stay stopped. Reset-reason detection is imperfect (cited above), so also use the start trigger (a button or BOOTSEL) to begin a run.
- **SG90 end-points.** The arduino-pico Servo defaults of 1000–2000 µs typically give only part of the SG90's travel. The docs example uses 540/2400. Pick the end-points on the bench.

### Gaps
- No official TowerPro SG90 datasheet stall current was retrieved; the figures are from hobby sources only.
- The 3xAA pack's internal resistance and the robot's actual wiring (which rail feeds the HC-SR04s and opto LEDs) are unknown. Measure the VSYS dip with a multimeter's min-hold or a scope during a servo move.
