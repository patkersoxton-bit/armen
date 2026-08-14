# LILARMEN Wiring Schematic

## Hardware Components

1. **Arduino Nano** (ATmega328P — old or new bootloader both work; just pick
   the matching one in Arduino IDE's board menu when uploading)
2. **3x incremental rotary encoders**, KY-040-style: each has 5 pins —
   `CLK`, `DT`, `SW`, `+` (VCC), `GND`
3. **USB cable** (Mini-USB or Micro-USB depending on Nano variant) — this is
   the *only* connection LILARMEN needs. It powers the Nano and carries the
   serial link to the PC; there is no separate power supply, because there
   are no motors on this board.

## Pin Map

| Axis | Joint (default mapping) | CLK | DT | SW |
|---|---|---|---|---|
| 0 | Base | D2 | D4 | D5 |
| 1 | Shoulder | D3 | D6 | D7 |
| 2 | Elbow | D8 | D9 | D10 |

(The firmware polls and time-debounces these pins rather than using
hardware interrupts, so D2/D3 have no special role — any digital pins would
work equally well. The assignment above is just the existing wiring.)

All three encoders' `+` pins go to the Nano's **5V** pin; all three `GND`
pins go to the Nano's **GND** pin (common ground, shared bus — encoders are
just mechanical switches, current draw is negligible).

The joint each axis *drives* is a pure software mapping (`AXIS_MAP` in
`desktop_app/lilarmen_teleop.py`), not a wiring constraint — if you'd rather
wire the encoders in a different physical order (e.g. because of cable
length or panel layout), just update `AXIS_MAP` to match instead of
re-wiring.

## Wiring Diagram (Text Format)

```
┌─────────────────────────────────────────────────────────────────┐
│                         LILARMEN WIRING                          │
└─────────────────────────────────────────────────────────────────┘

┌──────────────────┐
│   PC / Desktop    │
│  (lilarmen_teleop │
│      .py)         │
└────────┬──────────┘
         │ USB (power + serial, 115200 baud)
         ▼
┌──────────────────────────────────────────────────────────┐
│                     Arduino Nano                          │
│                                                             │
│   5V  ──────────────┬──────────────┬──────────────┐       │
│   GND ───────────┬──┼───────────┬──┼───────────┬──┼──┐    │
│                  │  │           │  │           │  │  │    │
│   D2 (CLK0) ─────┼──┼───┐       │  │           │  │  │    │
│   D4 (DT0)  ──────┼──┼───┼──┐    │  │           │  │  │    │
│   D5 (SW0)  ───────┼──┼───┼──┼──┐ │  │           │  │  │    │
│                  │  │   │  │  │ │  │           │  │  │    │
│   D3 (CLK1) ─────┼──┼───┼──┼──┼─┼──┼───┐       │  │  │    │
│   D6 (DT1)  ──────┼──┼───┼──┼──┼─┼──┼───┼──┐    │  │  │    │
│   D7 (SW1)  ───────┼──┼───┼──┼──┼─┼──┼───┼──┼──┐ │  │  │    │
│                  │  │   │  │  │ │  │   │  │  │ │  │  │    │
│   D8 (CLK2) ─────┼──┼───┼──┼──┼─┼──┼───┼──┼──┼─┼──┼──┼───┐│
│   D9 (DT2)  ──────┼──┼───┼──┼──┼─┼──┼───┼──┼──┼─┼──┼──┼───┼┤
│   D10(SW2)  ───────┼──┼───┼──┼──┼─┼──┼───┼──┼──┼─┼──┼──┼───┼┤
└──────────────────┼──┼───┼──┼──┼─┼──┼───┼──┼──┼─┼──┼──┼───┼┘
                    │  │   │  │  │ │  │   │  │  │ │  │  │   │
                    ▼  ▼   ▼  ▼  ▼ ▼  ▼   ▼  ▼  ▼ ▼  ▼  ▼   ▼
              ┌──────────┐ ┌──────────┐ ┌──────────┐
              │ Encoder 0│ │ Encoder 1│ │ Encoder 2│
              │  (Base)  │ │(Shoulder)│ │ (Elbow)  │
              │ +  CLK DT│ │ +  CLK DT│ │ +  CLK DT│
              │ GND    SW│ │ GND    SW│ │ GND    SW│
              └──────────┘ └──────────┘ └──────────┘
```

## Detailed Connection Table

| From Device | From Pin | To Device | To Pin | Notes |
|---|---|---|---|---|
| Nano | 5V | Encoder 0 | + | shared 5V rail |
| Nano | 5V | Encoder 1 | + | shared 5V rail |
| Nano | 5V | Encoder 2 | + | shared 5V rail |
| Nano | GND | Encoder 0 | GND | common ground |
| Nano | GND | Encoder 1 | GND | common ground |
| Nano | GND | Encoder 2 | GND | common ground |
| Nano | D2 | Encoder 0 | CLK | polled w/ internal pull-up |
| Nano | D4 | Encoder 0 | DT | polled w/ internal pull-up |
| Nano | D5 | Encoder 0 | SW | polled w/ internal pull-up |
| Nano | D3 | Encoder 1 | CLK | polled w/ internal pull-up |
| Nano | D6 | Encoder 1 | DT | polled w/ internal pull-up |
| Nano | D7 | Encoder 1 | SW | polled w/ internal pull-up |
| Nano | D8 | Encoder 2 | CLK | polled w/ internal pull-up |
| Nano | D9 | Encoder 2 | DT | polled w/ internal pull-up |
| Nano | D10 | Encoder 2 | SW | polled w/ internal pull-up |

## Notes

### No external resistors needed
The firmware enables the Nano's internal pull-up on every CLK/DT/SW pin,
and most KY-040-style modules already carry their own onboard pull-ups to
VCC on all three signal pins. Having both is harmless (just a slightly
stronger pull-up) — wire signal pins directly to the Nano, no extra
components.

### Power
USB alone is enough — three mechanical encoders draw negligible current.
Don't try to power anything else (like Armen) from this Nano's 5V pin.

### Noisy/jumpy counts
See the "If the count is noisy" section in `README.md` — this is a common
issue with cheap mechanical encoders (contact bounce), not a wiring
mistake, and the firmware already time-debounces for it. A 100nF capacitor
from each CLK/DT pin to GND is the standard next step if it's still rough.

### D2/D3 vs the rest
All three encoders are read the same way — polled every firmware loop
pass with time-based debouncing, not interrupt-driven — so no pin here has
special significance. The CLK-on-D2/D3 assignment above is just the
existing wiring, not a requirement; any digital pins work identically.

### Telling two same-looking Arduinos apart on Windows
If Armen's Uno and LILARMEN's Nano are both plugged in at once, they'll
often show up in Device Manager / the port list with the *same* description
(e.g. both "USB-SERIAL CH340") — only the COM port number differs. Plug them
in one at a time first to note which COM port is which, or just try each in
`lilarmen_teleop.py`'s port dropdown; it won't misbehave if you pick wrong,
`ping` will simply fail or return the wrong firmware version string
(`lilarmen-x.x` vs `stepper-x.x`).

### Never hot-plug
Same rule as the rest of this project: don't connect or disconnect an
encoder while the Nano is powered — while the current draw is negligible
enough that it's unlikely to damage anything, it can still cause spurious
interrupt edges that corrupt a count until the next `zero`.
