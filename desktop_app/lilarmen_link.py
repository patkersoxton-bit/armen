"""Serial link to the LILARMEN puppet input controller
(../LILARMEN/firmware/lilarmen_controller).

Same request/response-over-JSON-lines protocol as stepper_link.py (see that
file's docstring / COMMUNICATION_FIX.md for why), reduced to what an
input-only board needs: no move/jog/enable/estop, because LILARMEN has no
motors and cannot itself do anything unsafe. This module is deliberately
GUI-free so lilarmen_teleop.py (and any future tool) can share it, same
reasoning as StepperLink.
"""

import json
import threading
import time
from typing import Any, Dict, List, Optional

import serial
from serial.tools import list_ports

NUM_AXES = 3


class LinkError(Exception):
    """Raised when a command times out or the link is not connected."""


class LilArmenLink:
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
    def guess_port(exclude: Optional[str] = None) -> Optional[str]:
        """Best-effort autodetect. A Nano usually enumerates with the same
        description as the Uno (CH340 clone or FTDI genuine), so with both
        boards plugged in this can't tell them apart by description alone —
        pass `exclude` (a port already claimed by the other link) to at
        least avoid double-picking the same port for both devices."""
        keywords = ("ARDUINO", "CH340", "USB-SERIAL", "USB SERIAL", "FTDI", "CP210")
        for p in list_ports.comports():
            if p.device == exclude:
                continue
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
        time.sleep(2.0)  # opening the port resets the Nano; let it boot
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

    def zero(self) -> Dict[str, Any]:
        return self.command("zero")

    def cal_start(self) -> Dict[str, Any]:
        return self.command("cal_start")

    def cal_end(self) -> Dict[str, Any]:
        return self.command("cal_end")

    def mark(self, axis: int, which: str) -> Dict[str, Any]:
        return self.command("mark", axis=axis, which=which)

    def save_cal(self) -> Dict[str, Any]:
        return self.command("save_cal")
