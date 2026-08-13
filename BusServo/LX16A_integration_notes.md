# Hiwonder LX-16A Bus Servo — Notes and Integration Guide

What we learned getting two LX-16A servos running from an Arduino Uno, and
how to fold that into the stepper-motor project (which lives on a
different machine, behind a stepper shield with its own pin breakouts).

## Hardware

**Servo — Hiwonder LX-16A**
- UART bus servo, **fixed 115200 baud**, half-duplex **single-wire** bus
  (not a normal 2-wire TX/RX link, and not a PWM servo signal).
- Multiple servos daisy-chain on one signal line via their 3-pin connectors.
- Working voltage **6.0–8.4V — hard limit, do not exceed.** We initially ran
  one at 10V because it "moved nicer"; that's expected (more voltage = more
  torque/speed) but is well past spec and risks cooking the driver
  electronics. Run at ~7.0–7.6V.
- Stall current ~1.1–1.9A per servo — size any shared power supply
  accordingly (≥3A headroom for two servos), and never power servos from
  an MCU's 5V/USB rail.
- ID range 0–253. **Every unit ships with the same factory-default ID**
  (usually 1) — before putting two servos on one bus, connect them
  individually and assign unique IDs first (Hiwonder's Bus Servo Terminal
  PC app via the BusLinker's USB port is the easy way).

**BusLinker V2.5** — bridges a normal 2-wire TTL UART to the servo's 1-wire
half-duplex bus, and injects the servo power rail. Pin layout: Battery
anode/cathode (power in), USB port (PC software), Serial port (TTL
TX/RX — this is what a microcontroller connects to), Signal test point,
Servo anode/cathode, Servo bus port (daisy-chain connector).

## Wiring (as validated on the Uno dev rig)

- Uno TX → BusLinker Serial-port RXD, Uno RX → BusLinker Serial-port TXD
  (crossed, not straight-across), common GND.
- Bench PSU (~7.0–7.6V, ≥3A) → BusLinker Battery+/−, PSU GND tied in with
  Uno/BusLinker GND.
- Servos daisy-chained off the BusLinker's servo bus port.

## Protocol

Hiwonder's bus-servo packet format (same family as LX-224, HTS-25H, etc.):

```
0x55 0x55 <id> <len> <cmd> <param0> <param1> ... <checksum>
```

- `len` = number of params + 3
- `checksum` = `~(id + len + cmd + sum(params)) & 0xFF`
- Commands used so far:
  - `MOVE_TIME_WRITE = 1` — params `[posLo, posHi, timeLo, timeHi]`.
    Position is 0–1000, linearly mapped to 0–240°. Time is ms.
  - `LOAD_OR_UNLOAD_WRITE = 31` — param `[0|1]`, cuts/restores holding
    torque. Useful for hand-verifying a servo is alive/wired correctly.
  - `ID_WRITE = 13` — param `[newId]`. Only ever send this with one servo
    on the bus.

## Gotcha found during bring-up

Sending frames to two different servo IDs back-to-back with **zero gap**
was unreliable — the second-addressed servo would silently drop the
command. A **20ms delay between frames aimed at different IDs** resolved
it consistently in interactive testing. The very first version of a fixed
auto-sweep loop kept failing on the second servo even with that same 20ms
gap in place, for reasons we didn't fully pin down (it stopped reproducing
once we moved to on-demand commands sent from an interactive console) — so
treat this as "always leave ≥20ms between frames to different IDs," and if
a *continuous, unattended* loop ever exhibits the same one-servo-silent
symptom again, look at power sag (add bulk capacitance near the servo
power rail) and inter-frame timing again as the first suspects.

## Software architecture

- **`LX16A.h`** — the reusable driver. It only knows about an Arduino
  `Stream` (the base class behind `HardwareSerial`, `SoftwareSerial`, and
  `AltSoftSerial`) — it never references a pin number. Whoever constructs
  the serial object decides the pins; the driver just gets handed a
  reference to it. **Canonical copy now lives at
  `firmware/stepper_controller/LX16A.h`**, sketch-local next to the real
  stepper+servo firmware (not installed as an Arduino library — keep the
  two files together if that sketch ever moves). It has since grown a
  `readPosition()` query on top of the original `move`/`setLoad`/`setId`
  write-only API, for real position feedback in `get_state`.
- **`lx16a_sweep/`** — this project's dev/test rig: an interactive serial
  console (see command reference below) built on top of the driver, using
  `AltSoftSerial` on pins 8/9 (confirmed working here). Its own local
  `LX16A.h` copy predates the `readPosition()` addition — treat
  `firmware/stepper_controller/LX16A.h` as canonical if the two drift.
- **`lx16a_diag/`** — the earlier fixed-sequence diagnostic harness, kept
  for reference.

## Integrating with the stepper project

**Done** — see STEPPER_MIGRATION.md section 8 for the live details (which
axes, protocol changes, GUI changes). Summary of what was decided once the
actual shield was in front of us: Z (elbow) and A (spare/4th) became the
two servo axes; X (base) and Y (shoulder) stayed TMC2209 steppers. The bus
landed on **D9 (RX) / D10 (TX)** — the CNC Shield's spare `X+`/`Y+`
endstop-header breakouts, unrelated to which axes are steppers vs. servos,
just the nearest already-broken-out free digital pins with no switch
wired to them. That ruled out `AltSoftSerial` (fixed to D8=RX/D9=TX on an
Uno — wrong direction for D9 here) in favor of plain
`SoftwareSerial busPort(9, 10);`, exactly the fallback anticipated below.

A shared `moveTo()`/`update()`-style interface spanning both the stepper
joints and the servo joints turned out not to be needed: the real firmware
instead uses a per-axis `AXIS_TYPE` table so the *existing* `jog`/`mark`/
`move`/`get_state` commands interpret the same fields differently per axis
(raw steps vs. servo position units), with no new protocol surface — see
STEPPER_MIGRATION.md section 8 for how that plays out end to end.

## Command console reference (`lx16a_sweep.ino`)

Serial Monitor, 9600 baud, newline line ending:

| Command | Meaning |
|---|---|
| `m <id> <pos> [time_ms]` | move one servo |
| `b <pos1> <pos2> [time_ms] [gap_ms]` | move both back-to-back (ID1 then ID2) |
| `g <gap_ms>` | set the default gap used by `b` and the auto-sweep |
| `l <id> <0\|1>` | unload(0)/load(1) torque |
| `a <0\|1>` | toggle background auto-sweep; manual commands still work while it runs |
| `r` | reset both servos to baseline (position 500) |
| `?` | print command list |
