"""Serial link to the Armen stepper controller (firmware/stepper_controller).

Protocol: newline-delimited JSON at 115200 baud, strict request/response.
Every request carries an incrementing "id" which the firmware echoes in its
single-line reply, so responses can never be mismatched even if a stale line
is sitting in the OS buffer. The firmware sends nothing unsolicited; state is
obtained by polling get_state.

This module is deliberately GUI-free so both the calibration tool and the
future main control app can share it.
"""

import json
import os
import threading
import time
from typing import Any, Dict, List, Optional

import serial
from serial.tools import list_ports

NUM_AXES = 4

# Fixed hardware config: DRV8825 with one cap on M2 = 1/16 microstepping.
# This is a motor-shaft constant and never changes with gearing.
STEPS_PER_REV = 3200
DEG_PER_STEP = 360.0 / STEPS_PER_REV  # 0.1125° — only correct for direct-drive
                                       # (ratio 1.0) axes; see deg_per_step().

# Firmware sanity ceiling for a single jog command (see MAX_JOG_STEPS in
# stepper_controller.ino — keep these two in sync). Geared axes need far more
# raw motor steps to cover the same real jog distance, so this is sized for
# the largest ratio in GEAR_RATIOS, not for direct drive.
MAX_JOG_STEPS = 500_000

# Per-axis output gearbox ratio (motor revolutions per one output-shaft
# revolution; 1.0 = direct drive). The firmware only ever counts raw motor
# steps — it has no notion of gearing — so all degree math for geared axes
# happens here on the PC side, and every raw step count sent over the wire
# is derived from this ratio.
#
# The base (X) axis got a 37:1 gearbox added to stop it overloading the
# stepper (it was carrying the whole arm's weight at direct drive). Verify
# the ratio for real before trusting it: in calibrate.py, use "Jog 1 Motor
# Rev" on an axis, measure how far the OUTPUT shaft actually rotated, and
# either set the ratio directly or let "Compute From Measurement" derive it
# (ratio = 360 / measured_degrees). Persisted to gear_ratios.json so
# calibrate.py and arm_control.py always agree.
DEFAULT_GEAR_RATIOS = [37.0, 1.0, 1.0, 1.0]  # X (base), Y, Z, A
GEAR_RATIOS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gear_ratios.json")


def _load_gear_ratios() -> List[float]:
    try:
        with open(GEAR_RATIOS_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        ratios = list(data.get("ratios", DEFAULT_GEAR_RATIOS))
        if len(ratios) != NUM_AXES:
            raise ValueError(f"expected {NUM_AXES} ratios, got {len(ratios)}")
        return [float(r) if r else 1.0 for r in ratios]
    except (OSError, ValueError, json.JSONDecodeError):
        return list(DEFAULT_GEAR_RATIOS)


GEAR_RATIOS: List[float] = _load_gear_ratios()


def save_gear_ratios() -> None:
    with open(GEAR_RATIOS_FILE, "w", encoding="utf-8") as f:
        json.dump({"ratios": GEAR_RATIOS}, f, indent=2)


def set_gear_ratio(axis: int, ratio: float) -> None:
    if ratio <= 0:
        raise ValueError("gear ratio must be positive")
    GEAR_RATIOS[axis] = ratio
    save_gear_ratios()


def deg_per_step(axis: int) -> float:
    """Output-shaft degrees per motor microstep on this axis, accounting for
    its gearbox. Use this (not the flat DEG_PER_STEP) for any conversion
    that touches a possibly-geared axis."""
    return 360.0 / (STEPS_PER_REV * GEAR_RATIOS[axis])


def steps_for_output_degrees(axis: int, degrees: float) -> int:
    """Raw motor steps to command for a desired output-shaft rotation."""
    return round(degrees / deg_per_step(axis))


class LinkError(Exception):
    """Raised when a command times out or the link is not connected."""


class StepperLink:
    def __init__(self, port: Optional[str] = None, baudrate: int = 115200, log=None):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._next_id = 0
        self.log = log or print

    # ------------------------------------------------------------ lifecycle

    @staticmethod
    def available_ports() -> List[tuple]:
        return [(p.device, p.description) for p in list_ports.comports()]

    @staticmethod
    def guess_port() -> Optional[str]:
        # Uno clones enumerate as CH340; genuine boards as "Arduino".
        keywords = ("ARDUINO", "CH340", "USB-SERIAL", "USB SERIAL", "FTDI", "CP210")
        for p in list_ports.comports():
            desc = (p.description or "").upper()
            if any(k in desc for k in keywords):
                return p.device
        return None

    @property
    def connected(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def connect(self) -> None:
        if not self.port:
            self.port = self.guess_port()
        if not self.port:
            raise LinkError("no serial port found; pick one explicitly")
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        time.sleep(2.0)  # opening the port resets the Uno; let it boot
        self.ser.reset_input_buffer()
        self.log(f"connected to {self.port}")

    def close(self) -> None:
        if self.ser:
            try:
                self.ser.close()
            finally:
                self.ser = None

    # ------------------------------------------------------------ protocol

    def command(self, cmd: str, timeout: float = 2.0, **fields) -> Dict[str, Any]:
        """Send one command and return its matched response dict.

        Raises LinkError on timeout; a response with status "error" is
        returned as-is for the caller to inspect.
        """
        if not self.connected:
            raise LinkError("not connected")

        with self._lock:
            self._next_id += 1
            req_id = self._next_id
            msg = {"cmd": cmd, "id": req_id}
            msg.update(fields)
            self.ser.write((json.dumps(msg) + "\n").encode("ascii"))

            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                line = self.ser.readline()
                if not line:
                    continue
                try:
                    obj = json.loads(line.decode("utf-8", errors="replace"))
                except json.JSONDecodeError:
                    self.log(f"stray line: {line!r}")
                    continue
                if obj.get("id") == req_id:
                    return obj
                self.log(f"skipping unmatched response: {obj}")
            raise LinkError(f"timeout waiting for response to {cmd!r}")

    # ------------------------------------------------------------ commands

    def ping(self) -> Dict[str, Any]:
        return self.command("ping")

    def get_state(self, timeout: float = 1.0) -> Dict[str, Any]:
        return self.command("get_state", timeout=timeout)

    def get_cal(self) -> Dict[str, Any]:
        return self.command("get_cal")

    def enable(self) -> Dict[str, Any]:
        return self.command("enable")

    def disable(self) -> Dict[str, Any]:
        return self.command("disable")

    def zero(self) -> Dict[str, Any]:
        return self.command("zero")

    def cal_start(self) -> Dict[str, Any]:
        return self.command("cal_start")

    def cal_end(self) -> Dict[str, Any]:
        return self.command("cal_end")

    def jog(self, axis: int, steps: int) -> Dict[str, Any]:
        return self.command("jog", axis=axis, steps=steps)

    def mark(self, axis: int, which: str) -> Dict[str, Any]:
        return self.command("mark", axis=axis, which=which)

    def save_cal(self) -> Dict[str, Any]:
        return self.command("save_cal")

    def move(self, targets: List[int], speed: float = 0.5) -> Dict[str, Any]:
        return self.command("move", targets=targets, speed=speed)

    def set_motion(self, max_speed: int, accel: int) -> Dict[str, Any]:
        """Live-tune the step rate (steps/s) and acceleration (steps/s^2)."""
        return self.command("set_motion", max_speed=max_speed, accel=accel)

    def stop(self) -> Dict[str, Any]:
        return self.command("stop")

    def estop(self) -> Dict[str, Any]:
        return self.command("estop")

    def reset(self) -> Dict[str, Any]:
        return self.command("reset")
