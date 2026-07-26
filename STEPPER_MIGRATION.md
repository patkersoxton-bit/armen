# Stepper Migration Guide (Uno + CNC Shield V3 + DRV8825)

## 0. Gearbox update (base axis overload fix)

The base (X) joint was carrying the whole arm's weight at direct drive (1:1)
and was overloading its stepper. Fix: a **37:1 gearbox** added between the
base motor and the output shaft. This is believed to be on X/base — **verify
it, don't assume it** (see calibrate.py's Gear Ratios panel below); the ratio
is stored per-axis, not hardcoded to a specific axis.

Consequence for calibration: with a 37:1 reduction, one motor revolution now
only moves the output shaft ~9.7° instead of 360°. Jogging the arm to its
mechanical limits therefore takes ~37x more raw motor steps than before —
symptom if you forget this: the arm appears to barely move and "immediately"
hits the old (pre-gearbox) EEPROM bounds, because those bounds were marked
in raw motor steps that meant something very different pre-gearbox.

What changed to handle this:
- `desktop_app/stepper_link.py` now holds a per-axis `GEAR_RATIOS` list
  (default `[37.0, 1.0, 1.0, 1.0]`), persisted to `gear_ratios.json`, plus
  `deg_per_step(axis)` / `steps_for_output_degrees(axis, deg)` helpers. All
  degree math in both GUIs goes through these instead of a flat constant.
- `calibrate.py` jog buttons are now labeled and computed in **output-shaft
  degrees** (not raw steps), so a "+360°" click always moves one real output
  revolution regardless of gearing — for a 37:1 axis that's 37 motor
  revolutions under the hood.
- `calibrate.py` has a new **Gear Ratios panel**: "Jog 1 Motor Rev" moves
  exactly one motor revolution ignoring any configured ratio, so you can
  physically measure how far the output shaft turned and either type a known
  ratio + Set, or enter the measured degrees and click Compute From
  Measurement (ratio = 360 / measured°).
- The firmware itself is unchanged in concept — it only ever counts raw
  motor steps and has no notion of gear ratios — but `MAX_JOG_STEPS` was
  raised from 12,800 to 500,000 so a single jog command can actually cover a
  geared axis's real travel (37:1 means one output rev = 118,400 steps).
  **Reflash `firmware/stepper_controller/stepper_controller.ino` for this.**
- Any bounds saved in EEPROM before the gearbox was installed are stale and
  unsafe to trust — **recalibrate from scratch**: `cal_start` → zero at the
  reference pose → verify the gear ratio → jog to real mechanical limits
  using the new degree-based buttons → mark min/max → `save_cal`.

## 1. Hardware reality check

The existing firmware/docs target an **ESP32 + PCA9685 + 6 hobby servos**. The
actual kit (parts.jpg — Longruner CNC kit) is a different stack:

| | Old design (repo docs/code) | Actual kit |
|---|---|---|
| MCU | ESP32 | Arduino Uno |
| Driver board | PCA9685 (I2C PWM) | CNC Shield V3 (step/dir) |
| Motor drivers | — | 4x DRV8825 |
| Motors | 6 hobby servos (absolute angle) | NEMA 17 steppers (open loop) |
| Position feedback | implicit (servo holds commanded angle) | **none** — position = counted steps |
| Max axes | 16 | **4** (X, Y, Z, A) |

Two consequences drive everything else:

1. **Steppers are open loop.** The Arduino cannot "read" a motor's position.
   It only knows how many steps it has commanded since a known reference.
   Moving a joint by hand, disabling the drivers, or power-cycling erases that
   knowledge.
2. **The shield supports 4 axes**, not 6. The 6-joint arm design shrinks to 4
   joints (or a second board later).

New firmware for this stack lives in
`firmware/stepper_controller/stepper_controller.ino`. The old servo firmware in
`desktop_app/arm_controller/` is kept for reference.

## 2. Wiring the kit

### 2.1 Drivers into the shield (do this first, power off)

- Orientation matters and a mistake destroys the driver: align the **EN pin
  on the DRV8825 with the EN marking on the shield socket** (the trimpot ends
  up toward the top of the shield on all four sockets — all drivers face the
  same way).
- Stick the heatsinks on the driver chips.
- Microstepping jumpers go **under** each driver socket (M0/M1/M2). Coarse
  microstepping is the main cause of low-speed grinding/growling.
  **CHOSEN CONFIG (current build): one cap on M2 per driver = 1/16 microstep
  = 3,200 steps/rev.** The calibration GUI and all its step/degree math are
  hardcoded to this. (All three caps = 1/32 is marginally smoother; changing
  it later means updating `STEPS_PER_REV` in calibrate.py and redoing
  calibration, since all step counts rescale.) The firmware boots with
  conservative speeds (500 steps/s, 1500 steps/s²) safe in any mode; the GUI
  applies its tuning values (default 2400/7200) on connect.
- To use the 4th (A) axis as an independent motor, install the **two jumpers
  next to the A socket** that route A-STEP/A-DIR to D12/D13. (Without them, A
  can only clone X/Y/Z.)

### 2.2 Set the DRV8825 current limit (Vref) before connecting motors

- Power the shield (motors **unplugged**), measure DC volts between the
  trimpot top and a GND pin.
- The current formula depends on the two sense resistors on each driver:
  **current = Vref / (5 x Rsense)**. Check their marking: `R100` (0.10 Ω)
  gives current = 2 x Vref; **`R250` (0.25 Ω — what this kit's drivers
  have) gives current = 0.8 x Vref**.
- Typical kit NEMA 17s are rated 1.2–1.7 A/phase; target ~0.8–1.0 A to run
  cool. On R250 boards that is **Vref ≈ 1.0–1.2 V** (on R100 boards it would
  be 0.4–0.5 V). Too-high Vref = hot drivers and thermal-shutdown stutter;
  too-low = weak, raspy, stall-prone motion.

### 2.3 Motor wires: black, green, red, blue

Your colors don't match the diagram because **stepper wire colors are not
standardized** — ignore the diagram's colors entirely. A bipolar stepper is
just two coils; the only thing that matters is that each coil's pair of wires
lands on adjacent header pins.

1. **Identify the coil pairs.** With a multimeter on continuity/resistance,
   wires of the same coil show ~1–4 Ω between them; wires of different coils
   show open circuit. (No meter? Touch two wires together and spin the shaft —
   if it drags, those two are a pair.) On most NEMA 17s with these colors the
   pairs are **black+green** (coil A) and **red+blue** (coil B), but verify.
2. **Plug into the 4-pin header** next to each driver. The shield pins are
   labeled `2B 2A 1A 1B`: one pair goes on `2B/2A`, the other on `1A/1B`.
   With the common color convention that's **black, green, red, blue in
   order**.
3. **Symptoms if wrong:**
   - Motor buzzes/vibrates but doesn't rotate → a coil is split across the
     two headers pairs; re-check pairing.
   - Motor turns the wrong direction → not a fault; flip **one** pair's two
     wires (or invert direction in software).
4. **Never plug or unplug a motor while the shield is powered** — the flyback
   spike kills DRV8825s.

Note the kit's extension cables (XH plug on the motor end, Dupont on the
shield end) may change colors mid-cable — pair-identify at the shield end.

### 2.4 Limit switches and power

- The 3 kit limit switches go to the shield's endstop pins: **X→D9, Y→D10,
  Z→D11**, wired switch-common to GND, normally-open to the signal pin. The
  firmware reports them in `get_state` (future homing support).
- Motor power: **12–24 V DC** into the shield's blue screw terminal —
  double-check polarity, there is no reverse protection. The Uno stays powered
  by USB; grounds are shared through the shield stack.
- The Uno + shield **does not power servos**; if the old 6-servo arm hardware
  is still in play it remains on the ESP32/PCA9685 stack.

## 3. Bounds strategy: evaluation

**Your idea:** position each motor at its min and max mobility, read the
positions, and record them as ranges.

**Verdict: right concept, but "reading the position" doesn't exist on this
hardware.** Steppers have no encoders — if you move a joint by hand there is
nothing to read. The workable variant keeps your min/max-marking idea but
inverts who does the moving:

1. **Establish a reference pose (the "offset" problem).** With drivers
   enabled, put the arm in one repeatable pose and declare it zero. All
   positions and bounds are step counts relative to this pose. It must be
   reproducible every power-up — pick a pose against hard stops or marked
   with tape/pencil lines, or (better, later) mount the kit's limit switches
   so a homing cycle finds it automatically.
2. **Software-jog to each extreme.** Drive each axis in small increments from
   the PC until you reach the mechanical limit, and **mark** min/max there.
   Because the firmware did the moving, it knows exactly where it is.
3. **Persist and enforce.** Bounds are saved to the Uno's EEPROM and every
   subsequent `move` is clamped to them on the firmware side (PC-side bugs
   can't over-travel a joint).

Caveats to keep in mind:

- **Bounds are only meaningful relative to the reference pose.** Zeroing at a
  different physical pose silently shifts every bound. This is the biggest
  operational risk; a homing cycle with limit switches removes it.
- **Skipped steps drift the reference.** If an axis stalls (too fast, too
  heavy, current too low), the count no longer matches reality. Keep speeds
  modest and re-zero if the arm looks off.
- **Disabling drivers loses the reference and holding torque** — a
  gravity-loaded arm will slump. The firmware intentionally marks the state
  "unreferenced" after `disable` and refuses `move` until re-zeroed.
- Per-axis independent min/max is exactly right *given your statement that
  joints can't intersect each other*. If a future arm geometry makes one
  joint's safe range depend on another's position, this upgrades to
  pose-dependent limits in the firmware's clamp function — the protocol
  doesn't change.

## 4. Calibration procedure (with the new tools)

Built as a **calibration mode inside the main firmware** (not a separate test
sketch) so the exact same motion code, EEPROM data, and limits are used in
calibration and normal operation, and you never reflash between them.

1. Flash `firmware/stepper_controller/stepper_controller.ino`
   (Arduino IDE: board *Arduino Uno*; install **AccelStepper** and
   **ArduinoJson 6.x** from Library Manager).
2. `pip install pyserial`, then run `python desktop_app/calibrate.py`.
3. Connect → **Enable Motors** → **Start Cal**.
4. Move the arm (by hand is fine *before* this point; from here the motors
   hold it) to the reference pose → **Zero All Here**.
5. **If this axis has a gearbox**, verify its ratio first in the Gear Ratios
   panel (Jog 1 Motor Rev, measure the output shaft's rotation, Compute From
   Measurement or Set) — the jog buttons below depend on it being correct.
6. Per axis: jog to the safe minimum → **Mark Min**; jog to the safe
   maximum → **Mark Max**. Jog buttons are labeled in *output-shaft* degrees
   (±1.125°, ±11.25°, ±90°, ±360°/one output rev) and are converted to raw
   motor steps using that axis's gear ratio — on a 37:1 axis, "one output
   rev" is 37 motor revolutions under the hood. Approach mechanical stops
   slowly with small jogs.
7. **Save Bounds to EEPROM**, optionally **Export bounds.json** for the
   desktop app. **End Cal**.
8. Every future session: power up → enable → put arm at the reference pose →
   `zero` → normal `move` commands, clamped to the saved bounds.

## 5. Serial protocol v2 (JSON lines)

Design goals, learned from the CBOR protocol's failure modes (see review in
project notes / COMMUNICATION_FIX.md):

- **Newline-delimited JSON, both directions.** Self-resynchronizing (a
  corrupted line costs one line, not the whole stream), debuggable in the
  Arduino Serial Monitor, matches what the project docs always claimed.
- **Strict request/response.** The firmware never sends unsolicited bytes —
  no telemetry stream to collide with responses. State is polled.
- **Request IDs.** Every request carries `"id": n`; the response echoes it.
  The PC matches on id, so stale or interleaved lines can never be
  mis-attributed.

### Commands (PC → Uno)

| cmd | fields | notes |
|---|---|---|
| `ping` | | returns `fw` version |
| `get_state` | | state, enabled/referenced/calibrated/moving flags, `pos[4]`, `limits[3]` |
| `get_cal` | | saved/marked bounds |
| `enable` / `disable` | | shared driver enable; `disable` drops holding torque and clears the reference |
| `zero` | | set current pose as 0 on all axes (requires motion stopped) |
| `cal_start` / `cal_end` | | enter/leave calibration mode |
| `jog` | `axis` 0–3, `steps` ±4000 | cal mode only; relative, unbounded |
| `mark` | `axis`, `which`: `min`/`max` | records current position as a bound |
| `save_cal` | | validates and writes bounds to EEPROM; unmarked axes locked at 0 |
| `move` | `targets[4]`, `speed` 0.05–1.0 | absolute steps; requires enabled + referenced + calibrated; bounded axes clamped. An axis saved with min == max (unmarked in cal) is **free** — no clamping, for continuous-rotation joints like the swashplate base |
| `set_motion` | `max_speed` 100–4000, `accel` 100–20000 | live motion tuning in steps/s and steps/s²; resets on power cycle |
| `stop` | | decelerate to stop |
| `estop` | | instant halt, drivers stay enabled (arm holds); requires `reset` |
| `reset` | | clear estop |

### Responses (Uno → PC)

One line per request: `{"cmd":..., "status":"ok"|"error", "id":n, ...}` with
`message`/`error` text and command-specific fields.

## 6. Troubleshooting: grinding / rough motion at low speed

Symptom: motor sounds like grinding gears during slow jogs and at the start
and end of fast jogs, but is smooth once up to speed. That is stepper
resonance — at low step rates each (micro)step is a discrete jolt — not a
wiring or mechanical fault. In order of impact:

1. **Finer microstepping.** Check the jumpers under each driver: none
   installed = full step, which grinds badly. Install all three (1/32 on
   DRV8825). Remember every position/bound scales with the microstep factor,
   so recalibrate after changing jumpers.
2. **Higher acceleration** gets through the rough low-speed band faster.
   Tune live with the GUI's motion-tuning row (`set_motion`); try accel
   6000–12000 steps/s².
3. **Slightly more current.** If still rough or the motor stalls, raise Vref
   in ~0.05 V increments (up to the motor's rating / 2); low current makes
   low-speed stepping weak and raspy.
4. Some rattle at very low speeds is normal for an unloaded motor on a bench;
   it usually quiets down once mounted with a load.

The firmware also streams responses without blocking (servicing the steppers
whenever the UART buffer is full), so GUI polling during a jog no longer
freezes motion for ~10 ms per response — that used to add a periodic stutter
on top of the resonance.

## 7. Known gaps / next steps

- **Homing:** mount the limit switches and add a `home` command so the
  reference pose is found automatically instead of by eye. The switch inputs
  are already read and reported.
- **Angles vs steps:** done for the base's 37:1 gearbox — see section 0.
  `stepper_link.GEAR_RATIOS` / `deg_per_step()` are the per-axis mechanism;
  if another joint gets a gearbox later, set its ratio the same way (verify
  with calibrate.py's Gear Ratios panel, don't just hardcode a number).
- **Main GUI:** `desktop_app/arm_control.py` is the post-calibration jog
  controller (sliders sized from EEPROM bounds, free-jog for the base).
  `desktop_app/main.py` is the dead CBOR/servo app, kept only as reference —
  idle animations and expressive motion still need porting into the new
  stack.
- **Gravity:** consider which joints back-drive when disabled; those are the
  first candidates for homing switches and for never disabling casually.
