# Stepper + Bus Servo Migration Guide (Uno + CNC Shield V3 + TMC2209, was
# DRV8825; Z/A axes now Hiwonder LX-16A bus servos — see section 8)

## 0. Gearbox update (base axis overload fix)

The base (X) joint was carrying the whole arm's weight at direct drive (1:1)
and was overloading its stepper. Fix: an orbital gearbox added between the
base motor and the output shaft. This started as a **37:1** unit and has
since been **swapped for a metal 10:1 gearbox** (a replacement ratio, not an
extra stage — don't multiply the two together). This is believed to be on
X/base — **verify it, don't assume it** (see calibrate.py's Gear Ratios
panel below); the ratio is stored per-axis, not hardcoded to a specific
axis.

**Every regear invalidates the saved EEPROM bounds again** — they're marked
in raw motor steps, and a different ratio makes the same step count mean a
different real-world angle. Treat any ratio change (this 10:1 swap included)
as a trigger to redo calibration from `cal_start` onward, same as the
original 37:1 install below.

Consequence for calibration: with the current 10:1 reduction, one motor
revolution now only moves the output shaft 36° instead of 360° (with the
original 37:1 unit it was ~9.7°). Jogging the arm to its mechanical limits
therefore takes ~10x more raw motor steps than direct drive — symptom if you
forget this: the arm appears to barely move and "immediately" hits stale
EEPROM bounds, because those bounds were marked in raw motor steps that
meant something very different under the old ratio (or no gearbox at all).

What changed to handle this:
- `desktop_app/stepper_link.py` holds a per-axis `GEAR_RATIOS` list (default
  `[10.0, 1.0, 1.0, 1.0]`), persisted to `gear_ratios.json`, plus
  `deg_per_step(axis)` / `steps_for_output_degrees(axis, deg)` helpers. All
  degree math in both GUIs goes through these instead of a flat constant. A
  negative ratio is a valid value — same magnitude, flipped sign — used as a
  software direction-flip (`flip_gear_direction()` / calibrate.py's "Flip
  Dir" button) for a motor that spins the "wrong" way, e.g. after a driver
  swap, without touching wiring.
- `calibrate.py` jog buttons are labeled and computed in **output-shaft
  degrees** (not raw steps), so a "+360°" click always moves one real output
  revolution regardless of gearing — for the 10:1 X axis that's 10 motor
  revolutions under the hood.
- `calibrate.py` has a **Gear Ratios panel**: "Jog 1 Motor Rev" moves
  exactly one motor revolution ignoring any configured ratio, so you can
  physically measure how far the output shaft turned and either type a known
  ratio + Set, or enter the measured degrees and click Compute From
  Measurement (ratio = 360 / measured°). On a direct-drive axis (Y, Z, A)
  this also doubles as a sanity check that `STEPS_PER_REV` still matches the
  driver's actual microstep setting — see the TMC2209 note in §2.1.
- The firmware itself is unchanged in concept — it only ever counts raw
  motor steps and has no notion of gear ratios — and `MAX_JOG_STEPS`
  (500,000, raised from the original 12,800) already has plenty of headroom
  for the 10:1 axis (one output rev = 32,000 steps); it was sized for the
  larger 37:1 case, so no further firmware/reflash change is needed for this
  regear.
- Any bounds saved in EEPROM before the current gearbox/driver were
  installed are stale and unsafe to trust — **recalibrate from scratch**:
  `cal_start` → zero at the reference pose → verify the gear ratio → jog to
  real mechanical limits using the new degree-based buttons → mark min/max →
  `save_cal`.
- **The arm has also been made longer** (more leverage/torque at the tip).
  This doesn't change any ratio math, but it does mean: the physical
  mechanical limits have likely moved (re-mark min/max regardless of whether
  you also touch gear ratios), the added leverage may want a bit more
  current on the base/shoulder drivers (see §2.2b), and a heavier/longer arm
  is more prone to skipped steps at aggressive acceleration — re-check
  `set_motion` values if jogging looks rough or the reference drifts.

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

### 2.1b Driver swap: DRV8825 → TMC2209 (BIGTREETECH)

The build has since moved from DRV8825 to BIGTREETECH TMC2209 drivers in the
same shield sockets. Nothing below in this doc's protocol/firmware sections
changes — the firmware only ever drives generic STEP/DIR/EN pins and has no
notion of which driver chip is installed — but re-verify these before
trusting the arm:

- **Pin orientation.** BTT TMC2209 modules are generally marketed as
  DRV8825-pin-compatible, but check this specific module's silkscreen
  against the shield socket (EN/STEP/DIR/MS pins) before powering on —
  don't assume identical orientation just because it physically fits.
- **Microstepping table is different.** DRV8825 used 3 jumper pins
  (M0/M1/M2); BTT TMC2209 standalone mode typically only uses 2 (MS1/MS2),
  with a different resulting microstep table. This can silently change the
  effective steps/rev away from the 3,200 (1/16) this whole stack assumes.
  **Verify it with a tool that already exists:** in calibrate.py's Gear
  Ratios panel, click **Jog 1 Motor Rev** on any direct-drive axis (Y, Z, or
  A — ratio 1.0). The output shaft must rotate exactly one full 360° turn.
  If it visibly over- or under-rotates, `STEPS_PER_REV` in
  `desktop_app/stepper_link.py` no longer matches reality and must be
  corrected first — every other measurement (gear ratio, bounds, degree
  readouts) derives from it.
- **Current limit (Vref) formula is different — see §2.2b below, don't
  reuse the DRV8825 math.**
- **Direction may flip.** A motor can come out spinning the opposite way
  after a driver swap; this is a normal, harmless quirk for a left/right jog
  axis. Fix it in software with calibrate.py's **Flip Dir** button (negates
  that axis's `GEAR_RATIOS` entry) instead of rewiring a coil pair.
- The 4th (A) axis isn't installed yet; when it is, no code changes are
  needed — firmware and both GUIs already treat it as a full axis, only the
  A-socket jumpers above and `AXIS_NAMES` labels need attention.

### 2.2 Set the DRV8825 current limit (Vref) before connecting motors

*(Superseded by TMC2209 on this build — kept for reference/contrast; see
§2.2b for the current driver.)*

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

### 2.2b Set the TMC2209 current limit (Vref) — current driver, not yet done

The potentiometers have **not been re-adjusted** since swapping to the BTT
TMC2209 modules — this is an open manual step, not a code issue.

- **The DRV8825 formula above does not apply.** TMC2209 uses a different
  sense-resistor value and a different Vref relationship. Look up BIGTREETECH's
  own TMC2209 documentation for this board's exact Rsense and formula rather
  than reusing `current = 0.8 x Vref`.
- **Confirm standalone (legacy) Vref mode is actually active** before
  trusting the trimpot at all: TMC2209 can be run from UART-set current
  registers instead, in which case the analog Vref pin is ignored. Check
  that this board's UART pins (PDN_UART/TX/RX) are left unconnected/floating
  per its standalone-mode wiring, since the firmware here never speaks UART
  to the driver.
- **Bring current up gradually and watch for heat/stalling**, the same
  conservative approach as the DRV8825 procedure above, just re-verified for
  this driver and re-checked now that the arm is longer (more leverage on
  the base/shoulder may call for slightly more current than was tuned for
  the shorter arm).

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
| `get_state` | | state, enabled/referenced/calibrated/moving flags, `pos[4]` (see section 8 for what "pos" means on a servo axis) |
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

- **Homing:** further deferred, not just unimplemented — D9/D10 (formerly
  the X/Y endstop-switch pins) are now the LX-16A servo bus's UART (see
  section 8), and no physical switch was ever actually wired to any of the
  three endstop headers in the first place. A future homing design would
  need a different pin plan for X/Y, and doesn't apply to the Z/A servo
  axes at all (they don't need homing — their position is always absolute).
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

## 8. Mixed architecture: Z/A converted to LX-16A bus servos

**X (base) and Y (shoulder) stay TMC2209 steppers; Z (elbow) and A
(spare/4th) are now Hiwonder LX-16A bus servos**, driven through a
BusLinker board. Hardware specifics (voltage, wiring to the BusLinker,
servo ID assignment, the packet protocol, bring-up gotchas) live in
`BusServo/LX16A_integration_notes.md` — this section only covers how that
folds into this project's firmware/protocol/GUIs.

**Pin reuse, not a new pin plan:** the servo UART sits on Arduino D9/D10,
because those are the CNC Shield's spare `X+`/`Y+` endstop-header
breakouts — unrelated to the fact that X/Y remain steppers, they were
simply the nearest already-broken-out free digital I/O (no switch was ever
physically wired to any of the 3 endstop headers, so this is a pure reuse,
not a conflict with existing hardware). `X+` carries the servo bus's TX
(servo → Arduino), so **D9 = Arduino RX**; `Y+` carries the Arduino's TX,
so **D10 = Arduino TX**: `SoftwareSerial busPort(9, 10); // RX, TX`. This
also means the endstop/limit-switch reporting that used to be part of
`get_state` is gone (see section 7).

`firmware/stepper_controller/LX16A.h` is the driver, kept sketch-local
(next to `stepper_controller.ino`, `#include "LX16A.h"`) rather than
installed as an Arduino library — keep the two files together if this
sketch ever moves. See `BusServo/LX16A_integration_notes.md` for the
driver's own history/design notes.

**No new protocol commands.** `jog`/`mark`/`move`/`get_state` etc. are
shared across both axis types; the firmware's `AXIS_TYPE[NUM_AXES]` table
(and `stepper_link.AXIS_TYPE` on the PC side — keep the two in sync, same
as `MAX_JOG_STEPS`) decides how the same fields are interpreted per axis:

| | Stepper axis (X, Y) | Servo axis (Z, A) |
|---|---|---|
| Position units | raw motor steps, relative to `zero` | servo position 0–1000 (linear 0–240°), always absolute |
| `zero` | resets the step counter | no-op — nothing to zero |
| `jog`'s `steps` field | raw step delta, ceiling `MAX_JOG_STEPS` | position-unit delta, ceiling ±1000 |
| `mark` | records `AccelStepper::currentPosition()` | queries the servo's real measured position (`LX16ABus::readPosition`) |
| `move`'s target | absolute step count | absolute position 0–1000, clamped |
| `enable`/`disable` | shared `PIN_ENABLE` line | `servos.setLoad()` (torque on/off) |
| Degree conversion (PC side) | `360 / (STEPS_PER_REV × GEAR_RATIOS[axis])` | fixed `240 / 1000` = 0.24°/unit, no ratio ever |

Because `deg_per_step()`/`steps_for_output_degrees()` in `stepper_link.py`
branch on axis type internally, calibrate.py's jog buttons/sliders/Mark
Min/Max and arm_control.py's sliders/nudges work **identically** across all
4 axes — no separate servo UI was built. The one place the GUI does
special-case a servo axis is calibrate.py's Gear Ratios panel, where a
fixed-mapping servo axis has no ratio to jog/measure/set, so that row shows
a static label instead of the stepper-only controls.

**Position feedback is real, not open-loop bookkeeping.** Unlike the
stepper axes (which only ever know "how many steps commanded since zero"),
`get_state`'s `pos[]` for a servo axis is a live query of the servo's own
potentiometer (`SERVO_POS_READ`, falls back to the last commanded position
if the servo doesn't answer). `mark` does the same, so bounds reflect where
the servo *actually* is, not just where it was last told to go.

**Known risk, not yet solved:** `SoftwareSerial` briefly disables interrupts
while sending/receiving each byte (~87µs at 115200 baud). `AccelStepper`'s
timing for the still-live X/Y axes depends on frequent, low-jitter calls
from the same `loop()`, so a servo command/poll happening at the same
moment as a fast X/Y jog could cause a small timing hiccup. Likely fine
since servo traffic is intermittent, not on every loop tick — but worth
watching for once X/Y and the servos are moving at the same time on real
hardware.

**`SERVO_POS_READ`'s command byte (28) has not been independently verified**
against an authoritative Hiwonder protocol doc from inside this repo — it
matches the common Hiwonder LX-series command table (alongside
`MOVE_TIME_WRITE=1`/`ID_WRITE=13`/`LOAD_OR_UNLOAD_WRITE=31`, already
confirmed working), but double-check it against real hardware behavior
before fully trusting `get_state`'s servo positions. A bad reply fails the
checksum check in `LX16ABus::readPosition()` rather than returning garbage.
