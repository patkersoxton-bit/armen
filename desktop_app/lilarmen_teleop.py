#!/usr/bin/env python3
"""Armen <-> LILARMEN teleoperation bridge.

LILARMEN is a hand-held scale puppet of Armen: three rotary encoders (no
motors) whose relative rotation is calibrated the same way Armen's stepper
axes are (mark real min/max, save to EEPROM — see
LILARMEN/firmware/lilarmen_controller/lilarmen_controller.ino and
lilarmen_link.py). This app holds two independent serial links open at
once — one to Armen (stepper_link.StepperLink), one to LILARMEN
(lilarmen_link.LilArmenLink) — and, once both are calibrated, continuously
maps LILARMEN's live encoder position onto Armen's target position per
axis.

The mapping is purely ratiometric between two independently calibrated
ranges:

    frac = (lil_pos - lil_min) / (lil_max - lil_min)        # 0..1
    armen_target = armen_min + frac * (armen_max - armen_min)

This is deliberately blind to Armen's gear ratios, step counts, or the fact
that LILARMEN is physically much smaller — both sides only ever need to
agree on "how far through its own travel is this joint right now", which is
exactly what each board's own bounds calibration already captures. So the
10:1 base gearbox (or any other ratio) never has to be entered here at
all — it's already baked into Armen's calibrated min/max in raw steps.

Axis mapping (LILARMEN encoder index -> Armen axis index) is a pure
software constant (AXIS_MAP below), independent of wiring order — see
LILARMEN/WIRING_SCHEMATIC.md.

Safety notes:
  - Moves are only ever sent while "Enable Teleop" is on, and only while
    Armen reports enabled+referenced+calibrated and LILARMEN reports
    calibrated. Any of those going false disables forwarding.
  - Enabling teleop snaps Armen toward LILARMEN's current pose (bounded,
    still speed/accel-limited by Armen's firmware, but the jump can be
    large if the puppet isn't near Armen's current pose) — the GUI asks
    for confirmation before enabling for that reason. Pre-pose the puppet
    to roughly match Armen's current position before engaging.
  - LILARMEN axis 2's button (SW) is wired as a physical E-STOP for Armen,
    independent of teleop state — see the button-role table in
    lilarmen_controller.ino / LILARMEN/README.md.
"""

import json
import time
import tkinter as tk
from tkinter import scrolledtext, messagebox

import serial
from serial.tools import list_ports

from stepper_link import StepperLink, LinkError as MainLinkError, NUM_AXES as MAIN_NUM_AXES
from lilarmen_link import LilArmenLink, LinkError as LilLinkError, NUM_AXES as LIL_NUM_AXES

# Which Armen axis each LILARMEN encoder drives. Purely a software mapping —
# relabel here to remap, no rewiring or reflash needed.
AXIS_MAP = [0, 1, 2]  # LILARMEN axis i -> Armen axis AXIS_MAP[i]
LIL_AXIS_NAMES = ["Base", "Shoulder", "Elbow"]
MAIN_AXIS_NAMES = ["X (base)", "Y (shoulder)", "Z (elbow)", "A (spare)"]

LIL_POLL_MS = 50     # ~20 Hz — how often we read the puppet
MAIN_POLL_MS = 200   # Armen's own status; forwarding cadence is driven by
                      # the LILARMEN poll above, this is just for the
                      # displayed actual-position readout + gating flags
DEFAULT_SPEED = 0.3
MOVE_DEADBAND = 4    # raw protocol units; below this, don't bother resending


class TeleopGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Armen <-> LILARMEN Teleop")

        self.main_link = StepperLink(log=lambda m: self.log(f"[armen] {m}"))
        self.lil_link = LilArmenLink(log=lambda m: self.log(f"[lilarmen] {m}"))

        self.main_polling = False
        self.lil_polling = False

        self.main_cal = None   # {"min":[...], "max":[...]} raw units, live from Armen
        self.lil_cal = None    # same shape, from LILARMEN
        self.main_flags = {"enabled": False, "referenced": False, "calibrated": False}
        self.lil_flags = {"calibrated": False}

        self.main_targets = [0] * MAIN_NUM_AXES
        self.lil_pos = [0] * LIL_NUM_AXES
        self.lil_buttons_prev = [False] * LIL_NUM_AXES

        self.teleop_enabled = False
        self._warned_axes = set()  # keys already logged, to avoid log spam

        self._build_widgets()
        self.refresh_ports()

    # ------------------------------------------------------------ layout

    def _build_widgets(self):
        ports_bar = tk.Frame(self.root)
        ports_bar.pack(fill=tk.X, padx=10, pady=(5, 0))
        tk.Button(ports_bar, text="↻ Refresh Ports", command=self.refresh_ports
                  ).pack(side=tk.LEFT, padx=2)
        tk.Button(ports_bar, text="Auto-Detect Both (probes firmware)",
                  command=self.auto_detect_both).pack(side=tk.LEFT, padx=2)

        cols = tk.Frame(self.root)
        cols.pack(fill=tk.X, padx=10, pady=5)

        self._build_main_panel(cols)
        self._build_lil_panel(cols)

        teleop = tk.LabelFrame(self.root, text="Teleop")
        teleop.pack(fill=tk.X, padx=10, pady=5)

        self.teleop_btn = tk.Button(teleop, text="Enable Teleop", bg="lightgray",
                                     command=self.toggle_teleop)
        self.teleop_btn.pack(side=tk.LEFT, padx=6, pady=6)

        tk.Label(teleop, text="Speed:").pack(side=tk.LEFT, padx=(10, 0))
        self.speed_var = tk.DoubleVar(value=DEFAULT_SPEED)
        tk.Scale(teleop, from_=0.05, to=1.0, resolution=0.05, orient=tk.HORIZONTAL,
                 length=200, variable=self.speed_var).pack(side=tk.LEFT, padx=5)

        tk.Button(teleop, text="E-STOP", bg="red", fg="white",
                  command=self.estop).pack(side=tk.LEFT, padx=12)
        tk.Button(teleop, text="Reset E-Stop",
                  command=lambda: self.main_cmd(self.main_link.reset)).pack(side=tk.LEFT, padx=4)

        self.teleop_state_label = tk.Label(teleop, text="disabled", fg="gray")
        self.teleop_state_label.pack(side=tk.LEFT, padx=10)

        self.override_var = tk.BooleanVar(value=False)
        tk.Checkbutton(teleop, text="Override readiness checks (unsafe)",
                       variable=self.override_var, fg="darkred"
                       ).pack(side=tk.LEFT, padx=(20, 4))

        mapping = tk.LabelFrame(self.root, text="Axis mapping (LILARMEN -> Armen)")
        mapping.pack(fill=tk.X, padx=10, pady=5)
        self.axis_rows = []
        for i in range(LIL_NUM_AXES):
            row = tk.Frame(mapping)
            row.pack(fill=tk.X, pady=1)
            tk.Label(row, text=f"{LIL_AXIS_NAMES[i]} -> {MAIN_AXIS_NAMES[AXIS_MAP[i]]}",
                     width=28, anchor="w").pack(side=tk.LEFT)
            lil_lbl = tk.Label(row, text="lil: —", width=16, anchor="w")
            lil_lbl.pack(side=tk.LEFT)
            frac_lbl = tk.Label(row, text="frac: —", width=12, anchor="w")
            frac_lbl.pack(side=tk.LEFT)
            tgt_lbl = tk.Label(row, text="target: —", width=16, anchor="w")
            tgt_lbl.pack(side=tk.LEFT)
            act_lbl = tk.Label(row, text="actual: —", width=16, anchor="w")
            act_lbl.pack(side=tk.LEFT)
            self.axis_rows.append((lil_lbl, frac_lbl, tgt_lbl, act_lbl))

        self.log_text = scrolledtext.ScrolledText(self.root, height=12, wrap=tk.WORD,
                                                  state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

    def _build_main_panel(self, parent):
        panel = tk.LabelFrame(parent, text="Armen (main arm)")
        panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        top = tk.Frame(panel)
        top.pack(fill=tk.X, padx=5, pady=5)
        tk.Label(top, text="Port:").pack(side=tk.LEFT)
        self.main_port_var = tk.StringVar()
        self.main_port_menu = tk.OptionMenu(top, self.main_port_var, "")
        self.main_port_menu.pack(side=tk.LEFT, padx=5)
        self.main_connect_btn = tk.Button(top, text="Connect", command=self.toggle_main_connect)
        self.main_connect_btn.pack(side=tk.LEFT, padx=5)
        self.main_state_label = tk.Label(top, text="disconnected", fg="red")
        self.main_state_label.pack(side=tk.LEFT, padx=5)

        ritual = tk.Frame(panel)
        ritual.pack(fill=tk.X, padx=5, pady=(0, 5))
        tk.Button(ritual, text="Enable Motors",
                  command=lambda: self.main_cmd(self.main_link.enable)).pack(side=tk.LEFT, padx=2)
        tk.Button(ritual, text="Zero Here",
                  command=lambda: self.main_cmd(self.main_link.zero)).pack(side=tk.LEFT, padx=2)
        tk.Button(ritual, text="Disable",
                  command=self.confirm_main_disable).pack(side=tk.LEFT, padx=2)

    def _build_lil_panel(self, parent):
        panel = tk.LabelFrame(parent, text="LILARMEN (puppet input)")
        panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 0))

        top = tk.Frame(panel)
        top.pack(fill=tk.X, padx=5, pady=5)
        tk.Label(top, text="Port:").pack(side=tk.LEFT)
        self.lil_port_var = tk.StringVar()
        self.lil_port_menu = tk.OptionMenu(top, self.lil_port_var, "")
        self.lil_port_menu.pack(side=tk.LEFT, padx=5)
        self.lil_connect_btn = tk.Button(top, text="Connect", command=self.toggle_lil_connect)
        self.lil_connect_btn.pack(side=tk.LEFT, padx=5)
        self.lil_state_label = tk.Label(top, text="disconnected", fg="red")
        self.lil_state_label.pack(side=tk.LEFT, padx=5)

        cal = tk.Frame(panel)
        cal.pack(fill=tk.X, padx=5, pady=(0, 5))
        tk.Button(cal, text="Start Cal", command=lambda: self.lil_cmd(self.lil_link.cal_start)
                  ).pack(side=tk.LEFT, padx=2)
        tk.Button(cal, text="Zero Here", command=lambda: self.lil_cmd(self.lil_link.zero)
                  ).pack(side=tk.LEFT, padx=2)
        tk.Button(cal, text="End Cal", command=lambda: self.lil_cmd(self.lil_link.cal_end)
                  ).pack(side=tk.LEFT, padx=2)
        tk.Button(cal, text="Save Bounds", command=self.lil_save_cal).pack(side=tk.LEFT, padx=2)

        self.lil_mark_frame = tk.Frame(panel)
        self.lil_mark_frame.pack(fill=tk.X, padx=5, pady=(0, 5))
        self.lil_mark_labels = []
        for i in range(LIL_NUM_AXES):
            row = tk.Frame(self.lil_mark_frame)
            row.pack(fill=tk.X, pady=1)
            tk.Label(row, text=LIL_AXIS_NAMES[i], width=10, anchor="w").pack(side=tk.LEFT)
            tk.Button(row, text="Mark Min", command=lambda a=i: self.lil_mark(a, "min")
                      ).pack(side=tk.LEFT, padx=2)
            tk.Button(row, text="Mark Max", command=lambda a=i: self.lil_mark(a, "max")
                      ).pack(side=tk.LEFT, padx=2)
            lbl = tk.Label(row, text="min: — max: —", anchor="w")
            lbl.pack(side=tk.LEFT, padx=6)
            self.lil_mark_labels.append(lbl)

    # ------------------------------------------------------------ ports

    def refresh_ports(self):
        """Re-scan serial ports and repopulate both dropdowns. Safe to call
        any time (e.g. after plugging in the second board) — connecting a
        board resets it, so this never disturbs an already-open link; it
        only rebuilds the *list of choices*, it doesn't touch either
        link's current connection."""
        ports = list(list_ports.comports())
        if not ports:
            self.log("no serial ports found — check both boards are plugged in "
                      "and drivers are installed")
        else:
            self.log(f"found {len(ports)} port(s): " +
                     ", ".join(f"{p.device} ({p.description})" for p in ports))
        descs = [p.description for p in ports]
        if len(descs) != len(set(descs)) and len(ports) > 1:
            self.log("multiple ports share the same description — if Auto-Detect "
                      "can't tell them apart either, unplug one board, click "
                      "↻ Refresh Ports, and see which port disappears")

        for menu_widget, var in ((self.main_port_menu, self.main_port_var),
                                  (self.lil_port_menu, self.lil_port_var)):
            menu = menu_widget["menu"]
            menu.delete(0, tk.END)
            for p in ports:
                label = f"{p.device} — {p.description}"
                if p.serial_number:
                    label += f" [SN {p.serial_number}]"
                menu.add_command(label=label, command=lambda d=p.device, v=var: v.set(d))

        main_guess = StepperLink.guess_port()
        if main_guess:
            self.main_port_var.set(main_guess)
        lil_guess = LilArmenLink.guess_port(exclude=main_guess)
        if lil_guess:
            self.lil_port_var.set(lil_guess)
        if main_guess and lil_guess and main_guess == lil_guess:
            self.log("both ports auto-detected the same device by description — "
                      "try Auto-Detect Both, or pick manually")

    def auto_detect_both(self):
        """Probe every serial port's firmware identity (ping's "fw" field:
        "stepper-..." for Armen, "lilarmen-..." for LILARMEN) and assign the
        two dropdowns by what's actually running on each board, rather than
        by description string — the two boards can enumerate with identical
        descriptions (e.g. both "USB-SERIAL CH340"), so this is the only
        fully reliable way to tell them apart. Refuses to run while either
        link is connected, since probing opens (and resets) every candidate
        port in turn."""
        if self.main_link.connected or self.lil_link.connected:
            self.log("disconnect both links before auto-detecting")
            return
        candidates = [p.device for p in list_ports.comports()]
        if not candidates:
            self.log("no serial ports found")
            return
        self.log(f"probing {len(candidates)} port(s) for firmware identity "
                 "(each takes ~2s while the board reboots)...")
        found_main = None
        found_lil = None
        for device in candidates:
            fw = self._probe_fw(device)
            if fw is None:
                self.log(f"{device}: no response")
            elif fw.startswith("stepper-"):
                self.log(f"{device}: Armen firmware ({fw})")
                found_main = found_main or device
            elif fw.startswith("lilarmen-"):
                self.log(f"{device}: LILARMEN firmware ({fw})")
                found_lil = found_lil or device
            else:
                self.log(f"{device}: unrecognized firmware ({fw})")
        if found_main:
            self.main_port_var.set(found_main)
        else:
            self.log("Armen not found on any port")
        if found_lil:
            self.lil_port_var.set(found_lil)
        else:
            self.log("LILARMEN not found on any port")

    @staticmethod
    def _probe_fw(device: str):
        """Open `device`, send one ping, return its "fw" string (or None on
        no/garbled response). Best-effort: any failure just means this
        device isn't a usable candidate, not a fatal error for the probe."""
        try:
            ser = serial.Serial(device, 115200, timeout=0.3)
        except (OSError, serial.SerialException):
            return None
        try:
            time.sleep(2.0)  # opening the port resets the board; let it boot
            ser.reset_input_buffer()
            ser.write(b'{"cmd":"ping","id":1}\n')
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                line = ser.readline()
                if not line:
                    continue
                try:
                    obj = json.loads(line.decode("utf-8", errors="replace"))
                except json.JSONDecodeError:
                    continue
                if obj.get("id") == 1:
                    return obj.get("fw")
            return None
        finally:
            ser.close()

    # ------------------------------------------------------------ Armen link

    def toggle_main_connect(self):
        if self.main_link.connected:
            self.main_polling = False
            self.main_link.close()
            self.main_connect_btn.config(text="Connect")
            self.main_state_label.config(text="disconnected", fg="red")
            return
        try:
            self.main_link.port = self.main_port_var.get() or None
            self.main_link.connect()
            pong = self.main_link.ping()
            self.log(f"[armen] firmware: {pong.get('fw', 'unknown')}")
            cal = self.main_link.get_cal()
            if not cal.get("calibrated"):
                self.log("[armen] no saved bounds in EEPROM — calibrate Armen first (calibrate.py)")
                self.main_link.close()
                return
            self.main_cal = {"min": cal["min"], "max": cal["max"]}
            st = self.main_link.get_state()
            self.main_targets = list(st.get("pos", [0] * MAIN_NUM_AXES))[:MAIN_NUM_AXES]
            self.main_connect_btn.config(text="Disconnect")
            self.main_polling = True
            self.root.after(MAIN_POLL_MS, self.poll_main)
        except (MainLinkError, OSError) as e:
            self.log(f"[armen] connect failed: {e}")
            self.main_link.close()

    def main_cmd(self, fn, *args):
        try:
            resp = fn(*args)
            if resp.get("status") == "ok":
                msg = resp.get("message", "")
                if msg:
                    self.log(f"[armen] ✓ {msg}")
            else:
                self.log(f"[armen] ✗ {resp.get('error', 'unknown error')}")
            return resp
        except MainLinkError as e:
            self.log(f"[armen] ✗ {e}")
            return None

    def confirm_main_disable(self):
        if messagebox.askyesno("Disable Armen",
                                "Disabling drops holding torque — the arm may slump. Continue?"):
            self.main_cmd(self.main_link.disable)

    def poll_main(self):
        if not (self.main_polling and self.main_link.connected):
            return
        try:
            st = self.main_link.get_state(timeout=0.15)
            pos = st.get("pos", [0] * MAIN_NUM_AXES)[:MAIN_NUM_AXES]
            for lil_axis, main_axis in enumerate(AXIS_MAP):
                if main_axis < len(pos):
                    self.axis_rows[lil_axis][3].config(text=f"actual: {pos[main_axis]}")
            # keep unmapped Armen axes' targets pinned to their actual
            # position so teleop moves never yank them
            for a in range(MAIN_NUM_AXES):
                if a not in AXIS_MAP and a < len(pos):
                    self.main_targets[a] = pos[a]
            self.main_flags = {
                "enabled": st.get("enabled", False),
                "referenced": st.get("referenced", False),
                "calibrated": st.get("calibrated", False),
            }
            state = st.get("state", "?")
            flags = [k for k, v in self.main_flags.items() if v]
            if st.get("moving"):
                flags.append("moving")
            color = {"ready": "green", "cal": "blue", "estop": "red"}.get(state, "orange")
            self.main_state_label.config(text=f"{state} [{', '.join(flags) or 'idle'}]", fg=color)
            if state == "estop" and self.teleop_enabled:
                self.set_teleop_enabled(False, reason="Armen is in E-STOP")
        except MainLinkError:
            pass
        self.root.after(MAIN_POLL_MS, self.poll_main)

    # ------------------------------------------------------------ LILARMEN link

    def toggle_lil_connect(self):
        if self.lil_link.connected:
            self.lil_polling = False
            self.lil_link.close()
            self.lil_connect_btn.config(text="Connect")
            self.lil_state_label.config(text="disconnected", fg="red")
            return
        try:
            self.lil_link.port = self.lil_port_var.get() or None
            self.lil_link.connect()
            pong = self.lil_link.ping()
            self.log(f"[lilarmen] firmware: {pong.get('fw', 'unknown')}")
            self.load_lil_cal_display()
            self.lil_connect_btn.config(text="Disconnect")
            self.lil_polling = True
            self.root.after(LIL_POLL_MS, self.poll_lilarmen)
        except (LilLinkError, OSError) as e:
            self.log(f"[lilarmen] connect failed: {e}")
            self.lil_link.close()

    def lil_cmd(self, fn, *args):
        try:
            resp = fn(*args)
            if resp.get("status") == "ok":
                msg = resp.get("message", "")
                if msg:
                    self.log(f"[lilarmen] ✓ {msg}")
            else:
                self.log(f"[lilarmen] ✗ {resp.get('error', 'unknown error')}")
            return resp
        except LilLinkError as e:
            self.log(f"[lilarmen] ✗ {e}")
            return None

    def lil_mark(self, axis: int, which: str):
        resp = self.lil_cmd(self.lil_link.mark, axis, which)
        if resp and resp.get("status") == "ok":
            self.load_lil_cal_display()

    def lil_save_cal(self):
        resp = self.lil_cmd(self.lil_link.save_cal)
        if resp and resp.get("status") == "ok":
            self.load_lil_cal_display()

    def load_lil_cal_display(self):
        try:
            cal = self.lil_link.get_cal()
        except LilLinkError as e:
            self.log(f"[lilarmen] ✗ {e}")
            return
        self.lil_flags = {"calibrated": cal.get("calibrated", False)}
        if cal.get("calibrated"):
            self.lil_cal = {"min": cal["min"], "max": cal["max"]}
        else:
            self.lil_cal = None
        marked_min = cal.get("marked_min", [False] * LIL_NUM_AXES)
        marked_max = cal.get("marked_max", [False] * LIL_NUM_AXES)
        for i in range(LIL_NUM_AXES):
            mn = cal["min"][i] if marked_min[i] else "—"
            mx = cal["max"][i] if marked_max[i] else "—"
            self.lil_mark_labels[i].config(text=f"min: {mn}  max: {mx}")

    def poll_lilarmen(self):
        if not (self.lil_polling and self.lil_link.connected):
            return
        try:
            st = self.lil_link.get_state(timeout=0.15)
            self.lil_pos = st.get("pos", [0] * LIL_NUM_AXES)[:LIL_NUM_AXES]
            buttons = st.get("buttons", [False] * LIL_NUM_AXES)
            self.handle_buttons(buttons)
            self.lil_state_label.config(text=st.get("state", "?"), fg="green")
            self.update_mapping_display()
            self.maybe_send_teleop_move()
        except LilLinkError:
            pass
        self.root.after(LIL_POLL_MS, self.poll_lilarmen)

    def handle_buttons(self, buttons):
        for i in range(min(LIL_NUM_AXES, len(buttons))):
            rising = buttons[i] and not self.lil_buttons_prev[i]
            if rising:
                if i == 0:
                    self.toggle_teleop()
                elif i == 2:
                    self.log("[lilarmen] button 2 pressed — E-STOP")
                    self.estop()
                # button 1 reserved, unused
            self.lil_buttons_prev[i] = buttons[i]

    # ------------------------------------------------------------ teleop core

    def toggle_teleop(self):
        if self.teleop_enabled:
            self.set_teleop_enabled(False)
            return
        if not self.main_link.connected or not self.lil_link.connected:
            self.log("connect both Armen and LILARMEN first")
            return
        override = self.override_var.get()
        if not override:
            unmet = self._unmet_readiness()
            if unmet:
                self.log("not ready: " + "; ".join(unmet) +
                         " (or check 'Override readiness checks' to skip this)")
                return
        warning = ("Armen will move to match LILARMEN's current position now. "
                    "Pre-pose the puppet close to Armen's current pose first if "
                    "you don't want a sudden move.")
        if override:
            warning += ("\n\nOverride is ON — readiness checks are being skipped. "
                         "Armen's own firmware will still refuse to actually move "
                         "if it isn't enabled/zeroed/calibrated, so this mainly "
                         "matters if the real blocker turns out to be something "
                         "else (e.g. stale cached bounds — try reconnecting first).")
        if messagebox.askyesno("Enable Teleop", warning + " Continue?"):
            self.set_teleop_enabled(True)

    def _unmet_readiness(self) -> list:
        """Human-readable list of exactly which readiness condition(s) are
        currently unmet, so a blocked teleop toggle is diagnosable instead
        of a single lumped message. 'calibrated' (EEPROM bounds) is
        independent of 'enabled'/'referenced' (this session's Enable Motors
        / Zero Here) — the latter two reset every session even with valid
        saved bounds, and are the most common reason this looks stuck right
        after a fresh connect."""
        unmet = []
        if not self.main_flags.get("enabled"):
            unmet.append("Armen not enabled (click Enable Motors)")
        if not self.main_flags.get("referenced"):
            unmet.append("Armen not zeroed this session (click Zero Here)")
        if not self.main_flags.get("calibrated"):
            unmet.append("Armen has no saved bounds (run calibrate.py)")
        if not self.lil_flags.get("calibrated"):
            unmet.append("LILARMEN has no saved bounds (Start Cal / mark / Save Bounds)")
        return unmet

    def set_teleop_enabled(self, enabled: bool, reason: str = ""):
        self.teleop_enabled = enabled
        if enabled:
            self.teleop_btn.config(text="Disable Teleop", bg="lightgreen")
            self.teleop_state_label.config(text="ENABLED — puppet is driving Armen", fg="green")
            self.log("teleop enabled")
        else:
            self.teleop_btn.config(text="Enable Teleop", bg="lightgray")
            suffix = f" ({reason})" if reason else ""
            self.teleop_state_label.config(text=f"disabled{suffix}", fg="gray")
            self.log(f"teleop disabled{suffix}")

    def update_mapping_display(self):
        for lil_axis, main_axis in enumerate(AXIS_MAP):
            lil_lbl, frac_lbl, tgt_lbl, _ = self.axis_rows[lil_axis]
            pos = self.lil_pos[lil_axis]
            lil_lbl.config(text=f"lil: {pos}")
            frac = self._normalized(lil_axis)
            frac_lbl.config(text=f"frac: {frac:.2f}" if frac is not None else "frac: —")
            tgt_lbl.config(text=f"target: {self.main_targets[main_axis]}")

    def _normalized(self, lil_axis: int):
        if self.lil_cal is None:
            return None
        mn, mx = self.lil_cal["min"][lil_axis], self.lil_cal["max"][lil_axis]
        if mn == mx:
            return None
        frac = (self.lil_pos[lil_axis] - mn) / (mx - mn)
        return max(0.0, min(1.0, frac))

    def maybe_send_teleop_move(self):
        if not self.teleop_enabled:
            return
        if not (self.main_link.connected and self.lil_link.connected):
            self.set_teleop_enabled(False, reason="link dropped")
            return
        if not self.override_var.get():
            unmet = self._unmet_readiness()
            if unmet:
                self.set_teleop_enabled(False, reason="; ".join(unmet))
                return
        if self.lil_cal is None or self.main_cal is None:
            # Not overridable — there's no bounds data to compute a
            # ratiometric target from, regardless of readiness flags. Most
            # likely cause: bounds were saved on the board *after* this app
            # connected (get_cal is only fetched once, at connect time) —
            # reconnect to pick up fresh bounds.
            key = "no_cal_data"
            if key not in self._warned_axes:
                self.log("no calibration data cached for one or both boards — "
                         "reconnect to refresh (bounds are only read at connect time)")
                self._warned_axes.add(key)
            return

        changed = False
        for lil_axis, main_axis in enumerate(AXIS_MAP):
            frac = self._normalized(lil_axis)
            if frac is None:
                key = ("lil_uncalibrated", lil_axis)
                if key not in self._warned_axes:
                    self.log(f"LILARMEN axis {lil_axis} ({LIL_AXIS_NAMES[lil_axis]}) not "
                             f"calibrated — not driving Armen axis {main_axis}")
                    self._warned_axes.add(key)
                continue
            mn, mx = self.main_cal["min"][main_axis], self.main_cal["max"][main_axis]
            if mn == mx:
                key = ("main_free", main_axis)
                if key not in self._warned_axes:
                    self.log(f"Armen axis {main_axis} ({MAIN_AXIS_NAMES[main_axis]}) has no "
                             f"saved bounds (free/uncalibrated) — not driving it from LILARMEN")
                    self._warned_axes.add(key)
                continue
            target = round(mn + frac * (mx - mn))
            if abs(target - self.main_targets[main_axis]) >= MOVE_DEADBAND:
                self.main_targets[main_axis] = target
                changed = True

        if changed:
            self.main_cmd(self.main_link.move, list(self.main_targets), self.speed_var.get())

    def estop(self):
        if self.main_link.connected:
            self.main_cmd(self.main_link.estop)
        self.set_teleop_enabled(False, reason="E-STOP")

    # ------------------------------------------------------------ logging

    def log(self, message: str):
        def append():
            self.log_text.config(state=tk.NORMAL)
            self.log_text.insert(tk.END, message + "\n")
            self.log_text.see(tk.END)
            self.log_text.config(state=tk.DISABLED)
        self.root.after(0, append)


def main():
    root = tk.Tk()
    TeleopGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
