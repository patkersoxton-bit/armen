# LILARMEN — Puppet Input Device

LILARMEN is a hand-held scale replica of Armen's arm, built as a
*teleoperation input device*: physically posing it drives Armen's real
joints in real time. Three rotary encoders (no motors — this board only
ever reads) report relative joint rotation to the PC over USB serial,
exactly the way Armen's own steppers report step counts — same open-loop,
zero-then-calibrate model, just without any motors attached.

See [`WIRING_SCHEMATIC.md`](WIRING_SCHEMATIC.md) for the pin map and
[`firmware/lilarmen_controller/lilarmen_controller.ino`](firmware/lilarmen_controller/lilarmen_controller.ino)
for the firmware. The desktop side lives in `../desktop_app/lilarmen_link.py`
(serial link) and `../desktop_app/lilarmen_teleop.py` (the combined
calibration + teleop GUI, which also talks to Armen over its own separate
serial connection).

## Hardware

- Arduino Nano (ATmega328P)
- 3x incremental rotary encoders, KY-040-style (CLK/DT/SW/+/GND)
- USB cable — the only connection needed; no external power supply

## Firmware Setup

1. Arduino IDE → Library Manager → install:
   - **ArduinoJson** (6.x)
   - (no encoder library — decoding is hand-rolled in the .ino, see below)
2. Board: **Arduino Nano** — pick the bootloader (Old/New) that matches your
   specific board if upload fails with the default.
3. Open `firmware/lilarmen_controller/lilarmen_controller.ino`, select the
   port, Upload.

## If the count is noisy: debouncing cheap encoders

KY-040-style encoders are mechanical and bounce on every contact
transition. The firmware decodes them with **time-based debouncing done
before any direction decoding happens** (see the long comment at the top of
`lilarmen_controller.ino`) — a state is only trusted once it's held steady
for `DEBOUNCE_US` (1ms by default), and any transition ambiguous enough to
mean a step was missed is dropped rather than guessed, so a bad reading
costs a little resolution instead of corrupting the whole count. This
fixed an earlier interrupt-driven version (the `Encoder` library) that
decoded bounce directly, causing the count to run away and "zero" to drift
after a few turns — worse at speed, since bounce duration is roughly fixed
in time and a fast turn gives it no time to settle.

If it's still noisy after that fix, try these in order:

1. **Bump `DEBOUNCE_US`** in the .ino (e.g. to 2000–5000) and re-flash — a
   quick, free experiment.
2. **Add a 100nF ceramic capacitor** from CLK to GND and from DT to GND on
   each encoder, right at the encoder's pins. This is the standard hardware
   fix for bouncy encoders and works alongside the firmware debouncing, not
   instead of it — cheap and usually the single biggest improvement.
3. **Check wiring quality.** Loose breadboard jumpers produce intermittent
   contact that looks identical to switch bounce. Reseat connections,
   shorten wires, or move to solder/header connections if the breadboard is
   the weak link.

Only reach for potentiometers if all of the above still isn't enough —
they trade this problem for others that matter here: most hobby pots only
turn ~270°, which won't fit a joint that swings past a full rotation (check
Armen's own calibrated bounds per axis before assuming 270° is enough), and
they introduce wiper wear and analog noise of their own.

## Why this mirrors Armen's calibration model

LILARMEN's encoders are incremental, not absolute — like Armen's steppers,
the Nano only knows "how many ticks since the last zero," not a true angle.
That means the exact same discipline applies:

1. **Zero at a repeatable reference pose.** Every session, pose the puppet
   the same way before zeroing, or its calibrated bounds silently mean
   something different.
2. **Calibrate by marking real extremes.** Unlike Armen (which is
   *software*-jogged to its limits), you just physically move LILARMEN's
   joint to each mechanical extreme by hand and click Mark Min / Mark Max.
3. **Bounds persist to EEPROM** on the Nano itself, independent of Armen's
   own EEPROM bounds.

## Calibration Workflow (via `lilarmen_teleop.py`)

1. Connect to LILARMEN.
2. **Start Cal**.
3. Pose the puppet at a repeatable reference (e.g. all joints centered) →
   **Zero Here**.
4. For each joint: move it by hand to its safe minimum → **Mark Min**; move
   it to its safe maximum → **Mark Max**.
5. **Save Bounds** (writes to EEPROM). **End Cal**.
6. Every future session: power up → pose at the same reference → **Zero
   Here** → the puppet is ready to drive Armen.

## Teleop

Once *both* Armen and LILARMEN are calibrated (and Armen is enabled +
zeroed), `lilarmen_teleop.py`'s **Enable Teleop** starts forwarding
LILARMEN's live position to Armen, mapped ratiometrically per axis (see
that file's module docstring for the exact math — short version: it's
`(lil_pos - lil_min) / (lil_max - lil_min)` mapped onto Armen's own
calibrated range, which needs no knowledge of gear ratios or physical scale
at all).

**Enabling teleop moves Armen** toward wherever the puppet currently is —
pre-pose the puppet close to Armen's actual current pose before engaging to
avoid a large sudden move. The GUI asks for confirmation before the first
engage for this reason.

### Physical buttons (each encoder's SW)

| Axis | Default role |
|---|---|
| 0 (Base) | Toggle teleop enable/disable |
| 1 (Shoulder) | Reserved, unused |
| 2 (Elbow) | E-STOP Armen |

These are PC-side conventions (`lilarmen_teleop.py`), not firmware
behavior — the Nano only ever reports button state, it never acts on it
itself.

## Axis Mapping

LILARMEN's 3 encoders default to driving Armen's X (base), Y (shoulder),
and Z (elbow) axes, leaving Armen's A (spare) axis untouched by teleop.
This is a software constant (`AXIS_MAP` in `lilarmen_teleop.py`), not a
wiring constraint — remap there, not on the board.
