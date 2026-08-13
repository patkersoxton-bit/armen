#!/usr/bin/env python3
"""Armen bounds-calibration tool for the stepper controller.

Workflow (see STEPPER_MIGRATION.md for the full procedure):
  1. Connect, then Enable Motors.
  2. Start Cal, physically pose the arm at its reference pose, click Zero All.
  3. Jog each axis to its safe minimum -> Mark Min; to its maximum -> Mark Max.
  4. Save to EEPROM (firmware enforces these bounds on every future move),
     and optionally export bounds.json for the desktop app.
"""

import json
import os
import tkinter as tk
from tkinter import scrolledtext

from stepper_link import (
    StepperLink, LinkError, NUM_AXES, STEPS_PER_REV, AXIS_TYPE,
    GEAR_RATIOS, deg_per_step, steps_for_output_degrees, set_gear_ratio,
    flip_gear_direction, max_jog_for_axis,
)

AXIS_NAMES = ["X (base)", "Y (shoulder)", "Z (elbow)", "A (spare)"]

# Jog sizes in OUTPUT-shaft degrees, not raw motor steps — so a click moves
# the same real-world angle on every axis regardless of gearing. Converted
# to raw steps per-axis at click time via steps_for_output_degrees(), which
# reads the *current* GEAR_RATIOS (editable live in the Gear Ratios panel).
# At ratio 1.0 these work out to the old raw-step sizes: 1.125°=10 steps,
# 11.25°=100, 90°=800, 360°=3200 (one motor rev).
JOG_DEG_SIZES = [-360, -90, -11.25, -1.125, 1.125, 11.25, 90, 360]
POLL_MS = 500
BOUNDS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bounds.json")


class CalibrateGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Armen — Stepper Bounds Calibration")
        self.link = StepperLink(log=self.log)
        self.polling = False

        self._build_widgets()
        self.refresh_ports()

    # ------------------------------------------------------------ layout

    def _build_widgets(self):
        top = tk.Frame(self.root)
        top.pack(fill=tk.X, padx=10, pady=5)

        tk.Label(top, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_menu = tk.OptionMenu(top, self.port_var, "")
        self.port_menu.pack(side=tk.LEFT, padx=5)
        tk.Button(top, text="↻", command=self.refresh_ports).pack(side=tk.LEFT)
        self.connect_btn = tk.Button(top, text="Connect", command=self.toggle_connect)
        self.connect_btn.pack(side=tk.LEFT, padx=10)
        self.state_label = tk.Label(top, text="disconnected", fg="red")
        self.state_label.pack(side=tk.LEFT, padx=10)

        mode = tk.Frame(self.root)
        mode.pack(fill=tk.X, padx=10, pady=5)
        self._btn(mode, "Enable Motors", lambda: self.cmd(self.link.enable))
        self._btn(mode, "Disable Motors", self.confirm_disable)
        self._btn(mode, "Start Cal", lambda: self.cmd(self.link.cal_start))
        self._btn(mode, "Zero All Here", lambda: self.cmd(self.link.zero))
        self._btn(mode, "End Cal", lambda: self.cmd(self.link.cal_end))
        tk.Button(mode, text="STOP", bg="orange",
                  command=lambda: self.cmd(self.link.stop)).pack(side=tk.LEFT, padx=4)
        tk.Button(mode, text="E-STOP", bg="red", fg="white",
                  command=lambda: self.cmd(self.link.estop)).pack(side=tk.LEFT, padx=4)
        self._btn(mode, "Reset E-Stop", lambda: self.cmd(self.link.reset))

        motion = tk.LabelFrame(
            self.root,
            text="Motion tuning — X/Y steppers only (motor-level: 1/16 microstep, 3200 steps/rev)")
        motion.pack(fill=tk.X, padx=10, pady=5)
        tk.Label(motion, text="Max speed (steps/s):").pack(side=tk.LEFT)
        self.speed_var = tk.StringVar(value="2400")
        tk.Entry(motion, textvariable=self.speed_var, width=7).pack(side=tk.LEFT, padx=4)
        tk.Label(motion, text="Accel (steps/s²):").pack(side=tk.LEFT, padx=(10, 0))
        self.accel_var = tk.StringVar(value="7200")
        tk.Entry(motion, textvariable=self.accel_var, width=7).pack(side=tk.LEFT, padx=4)
        tk.Button(motion, text="Apply", command=self.apply_motion).pack(side=tk.LEFT, padx=10)
        tk.Label(motion, text="Custom jog (raw units — steps for X/Y, servo units for Z/A):"
                 ).pack(side=tk.LEFT, padx=(10, 0))
        self.custom_jog_var = tk.StringVar(value=str(STEPS_PER_REV))
        tk.Entry(motion, textvariable=self.custom_jog_var, width=7).pack(side=tk.LEFT, padx=2)

        ratios = tk.LabelFrame(
            self.root,
            text="Gear ratios (motor revs per output rev — verify before calibrating a geared axis!)")
        ratios.pack(fill=tk.X, padx=10, pady=5)
        tk.Label(ratios, text="1. Jog 1 Motor Rev  2. measure how far the OUTPUT shaft actually "
                              "turned (degrees)  3. Compute From Measurement, or type a known ratio "
                              "and Set.  If the axis spins the wrong way, click Flip Dir (note: "
                              "Compute From Measurement resets the sign, so re-flip after using it).",
                 anchor="w", justify=tk.LEFT, wraplength=760
                 ).grid(row=0, column=0, columnspan=6, sticky="w", pady=(0, 4))
        self.ratio_vars = []
        self.measured_vars = []
        self.ratio_labels = []
        for i, name in enumerate(AXIS_NAMES):
            r = i + 1
            tk.Label(ratios, text=name, width=12, anchor="w").grid(row=r, column=0, sticky="w")
            if AXIS_TYPE[i] == "servo":
                # No ratio to calibrate on a servo axis — its 0-1000 -> 0-240°
                # mapping is fixed hardware, not something to jog/measure/set.
                tk.Label(ratios, text="servo — fixed 0.24°/unit, no ratio to calibrate",
                         anchor="w").grid(row=r, column=1, columnspan=7, sticky="w")
                self.ratio_labels.append(None)
                self.ratio_vars.append(None)
                self.measured_vars.append(None)
                continue
            rlabel = tk.Label(ratios, text=self._ratio_label_text(i), width=18, anchor="w")
            rlabel.grid(row=r, column=1)
            self.ratio_labels.append(rlabel)
            tk.Button(ratios, text="Jog 1 Motor Rev", width=14,
                      command=lambda a=i: self.jog_one_motor_rev(a)).grid(row=r, column=2, padx=4)
            rv = tk.StringVar(value=f"{GEAR_RATIOS[i]:g}")
            tk.Entry(ratios, textvariable=rv, width=7).grid(row=r, column=3, padx=2)
            self.ratio_vars.append(rv)
            tk.Button(ratios, text="Set", command=lambda a=i: self.set_ratio(a)
                      ).grid(row=r, column=4, padx=2)
            tk.Button(ratios, text="Flip Dir", command=lambda a=i: self.flip_ratio(a)
                      ).grid(row=r, column=5, padx=2)
            tk.Label(ratios, text="measured output °:").grid(row=r, column=6, padx=(10, 2))
            mv = tk.StringVar()
            tk.Entry(ratios, textvariable=mv, width=7).grid(row=r, column=7, padx=2)
            self.measured_vars.append(mv)
            tk.Button(ratios, text="Compute From Measurement",
                      command=lambda a=i: self.compute_ratio(a)).grid(row=r, column=8, padx=4)

        axes = tk.LabelFrame(
            self.root,
            text="Axes (raw protocol units — motor steps from reference pose for X/Y, "
                 "absolute servo position 0-1000 for Z/A)")
        axes.pack(fill=tk.X, padx=10, pady=5)

        self.pos_labels = []
        self.min_labels = []
        self.max_labels = []
        for i, name in enumerate(AXIS_NAMES):
            tk.Label(axes, text=name, width=12, anchor="w",
                     font=("Arial", 10, "bold")).grid(row=i, column=0, sticky="w", pady=2)
            pos = tk.Label(axes, text="—", width=15, anchor="e", relief=tk.SUNKEN)
            pos.grid(row=i, column=1, padx=4)
            self.pos_labels.append(pos)

            col = 2
            for deg in JOG_DEG_SIZES:
                label = f"{deg:+g}°"
                tk.Button(axes, text=label, width=6,
                          command=lambda a=i, d=deg: self.jog_deg(a, d)
                          ).grid(row=i, column=col, padx=1)
                col += 1

            tk.Button(axes, text="−C", width=4,
                      command=lambda a=i: self.custom_jog(a, -1)).grid(row=i, column=col, padx=1)
            col += 1
            tk.Button(axes, text="+C", width=4,
                      command=lambda a=i: self.custom_jog(a, +1)).grid(row=i, column=col, padx=1)
            col += 1

            tk.Button(axes, text="Mark Min",
                      command=lambda a=i: self.do_mark(a, "min")).grid(row=i, column=col, padx=4)
            mn = tk.Label(axes, text="min: —", width=16, anchor="w")
            mn.grid(row=i, column=col + 1)
            self.min_labels.append(mn)

            tk.Button(axes, text="Mark Max",
                      command=lambda a=i: self.do_mark(a, "max")).grid(row=i, column=col + 2, padx=4)
            mx = tk.Label(axes, text="max: —", width=16, anchor="w")
            mx.grid(row=i, column=col + 3)
            self.max_labels.append(mx)

        bottom = tk.Frame(self.root)
        bottom.pack(fill=tk.X, padx=10, pady=5)
        self._btn(bottom, "Save Bounds to EEPROM", self.do_save)
        self._btn(bottom, "Export bounds.json", self.do_export)
        self._btn(bottom, "Load Saved Bounds", self.load_cal_display)

        self.log_text = scrolledtext.ScrolledText(self.root, height=12, wrap=tk.WORD,
                                                  state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

    def _btn(self, parent, text, command):
        tk.Button(parent, text=text, command=command).pack(side=tk.LEFT, padx=4)

    # ------------------------------------------------------------ actions

    def refresh_ports(self):
        ports = StepperLink.available_ports()
        menu = self.port_menu["menu"]
        menu.delete(0, tk.END)
        for device, desc in ports:
            menu.add_command(label=f"{device} — {desc}",
                             command=lambda d=device: self.port_var.set(d))
        guess = StepperLink.guess_port()
        if guess:
            self.port_var.set(guess)
        elif ports:
            self.port_var.set(ports[0][0])

    def toggle_connect(self):
        if self.link.connected:
            self.polling = False
            self.link.close()
            self.connect_btn.config(text="Connect")
            self.state_label.config(text="disconnected", fg="red")
            self.log("disconnected")
            return
        try:
            self.link.port = self.port_var.get() or None
            self.link.connect()
            pong = self.link.ping()
            self.log(f"firmware: {pong.get('fw', 'unknown')}")
            self.connect_btn.config(text="Disconnect")
            self.load_cal_display()
            self.apply_motion()  # sync firmware to the visible tuning values
            self.polling = True
            self.root.after(POLL_MS, self.poll_state)
        except (LinkError, OSError) as e:
            self.log(f"connect failed: {e}")
            self.link.close()

    def cmd(self, fn, *args):
        """Run a link command, logging the outcome."""
        try:
            resp = fn(*args)
            if resp.get("status") == "ok":
                msg = resp.get("message", "")
                if msg:
                    self.log(f"✓ {msg}")
            else:
                self.log(f"✗ {resp.get('error', 'unknown error')}")
            return resp
        except LinkError as e:
            self.log(f"✗ {e}")
            return None

    def custom_jog(self, axis: int, sign: int):
        try:
            steps = int(self.custom_jog_var.get())
        except ValueError:
            self.log("✗ custom jog must be a whole number of steps")
            return
        ceiling = max_jog_for_axis(axis)
        if not 0 < abs(steps) <= ceiling:
            self.log(f"✗ custom jog must be 1-{ceiling} steps")
            return
        self.cmd(self.link.jog, axis, sign * abs(steps))

    def jog_deg(self, axis: int, degrees: float):
        """Jog by a real-world output-shaft angle, converted to this axis's
        native protocol units: raw motor steps via its current gear ratio
        (read live, so an edit in the Gear Ratios panel takes effect
        immediately) for a stepper axis, or servo position units for a
        servo axis."""
        steps = steps_for_output_degrees(axis, degrees)
        ceiling = max_jog_for_axis(axis)
        steps = max(-ceiling, min(ceiling, steps))
        self.cmd(self.link.jog, axis, steps)

    def jog_one_motor_rev(self, axis: int):
        """Jog exactly one MOTOR revolution, ignoring gear ratio entirely —
        for visually verifying/measuring how far the output shaft actually
        turns, independent of whatever ratio is currently configured. On a
        direct-drive axis (ratio 1.0) this also doubles as the STEPS_PER_REV
        sanity check after a driver/microstep change: the shaft must turn
        exactly 360°, or STEPS_PER_REV in stepper_link.py no longer matches
        the driver's actual microstep setting."""
        self.cmd(self.link.jog, axis, STEPS_PER_REV)
        self.log(f"{AXIS_NAMES[axis]}: jogged 1 motor rev ({STEPS_PER_REV} steps). "
                 f"Measure the OUTPUT shaft's rotation in degrees and enter it above "
                 f"(on a direct-drive axis this should be exactly 360°).")

    def _ratio_label_text(self, axis: int) -> str:
        ratio = GEAR_RATIOS[axis]
        suffix = " (reversed)" if ratio < 0 else ""
        return f"current: {abs(ratio):g}:1{suffix}"

    def set_ratio(self, axis: int):
        try:
            ratio = float(self.ratio_vars[axis].get())
            set_gear_ratio(axis, ratio)
        except ValueError as e:
            self.log(f"✗ {AXIS_NAMES[axis]}: {e}")
            return
        self.ratio_labels[axis].config(text=self._ratio_label_text(axis))
        self.log(f"✓ {AXIS_NAMES[axis]}: gear ratio set to {ratio:g}:1 (saved to gear_ratios.json)")

    def flip_ratio(self, axis: int):
        flip_gear_direction(axis)
        self.ratio_vars[axis].set(f"{GEAR_RATIOS[axis]:g}")
        self.ratio_labels[axis].config(text=self._ratio_label_text(axis))
        state = "reversed" if GEAR_RATIOS[axis] < 0 else "normal"
        self.log(f"✓ {AXIS_NAMES[axis]}: direction flipped ({state}, saved to gear_ratios.json)")

    def compute_ratio(self, axis: int):
        try:
            measured_deg = float(self.measured_vars[axis].get())
        except ValueError:
            self.log("✗ measured output degrees must be a number")
            return
        if measured_deg <= 0:
            self.log("✗ measured output degrees must be positive (use the magnitude)")
            return
        ratio = 360.0 / measured_deg
        set_gear_ratio(axis, ratio)
        self.ratio_vars[axis].set(f"{ratio:g}")
        self.ratio_labels[axis].config(text=self._ratio_label_text(axis))
        self.log(f"✓ {AXIS_NAMES[axis]}: 1 motor rev moved the output {measured_deg:g}°, "
                 f"so ratio = {ratio:g}:1 (saved). If this axis actually spins backward, "
                 f"click Flip Dir too.")

    def apply_motion(self):
        try:
            max_speed = int(self.speed_var.get())
            accel = int(self.accel_var.get())
        except ValueError:
            self.log("✗ speed/accel must be whole numbers")
            return
        resp = self.cmd(self.link.set_motion, max_speed, accel)
        if resp and resp.get("status") == "ok":
            self.log(f"✓ motion: {resp.get('max_speed')} steps/s, "
                     f"{resp.get('accel')} steps/s² (firmware-clamped)")

    def confirm_disable(self):
        self.log("NOTE: disabling drivers drops holding torque — support the arm!")
        self.cmd(self.link.disable)

    def do_mark(self, axis: int, which: str):
        resp = self.cmd(self.link.mark, axis, which)
        if resp and resp.get("status") == "ok":
            label = self.min_labels[axis] if which == "min" else self.max_labels[axis]
            value = resp.get("value", 0)
            label.config(text=f"{which}: {value} ({value * deg_per_step(axis):.0f}°)")

    def do_save(self):
        resp = self.cmd(self.link.save_cal)
        if resp and resp.get("status") == "ok":
            self.load_cal_display()

    def do_export(self):
        try:
            cal = self.link.get_cal()
        except LinkError as e:
            self.log(f"✗ {e}")
            return
        data = {
            "note": "Bounds relative to the reference (zero) pose for stepper axes "
                    "(X/Y); servo axes (Z/A) don't need a reference pose, their "
                    "position is always absolute. min/max are raw protocol units — "
                    "motor steps for a stepper axis, servo position units (0-1000) "
                    "for a servo axis; min_deg/max_deg are output-shaft degrees "
                    "(via gear_ratio for steppers, a fixed 0.24°/unit for servos).",
            "steps_per_rev": STEPS_PER_REV,
            "axes": [
                {"axis": i, "name": AXIS_NAMES[i], "axis_type": AXIS_TYPE[i],
                 "gear_ratio": GEAR_RATIOS[i] if AXIS_TYPE[i] == "stepper" else None,
                 "min": cal["min"][i], "max": cal["max"][i],
                 "min_deg": cal["min"][i] * deg_per_step(i),
                 "max_deg": cal["max"][i] * deg_per_step(i)}
                for i in range(NUM_AXES)
            ],
        }
        with open(BOUNDS_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        self.log(f"✓ exported {BOUNDS_FILE}")

    def load_cal_display(self):
        try:
            cal = self.link.get_cal()
        except LinkError as e:
            self.log(f"✗ {e}")
            return
        if not cal.get("calibrated"):
            self.log("no saved bounds in EEPROM yet")
            return
        for i in range(NUM_AXES):
            mn, mx = cal["min"][i], cal["max"][i]
            if mn == mx:
                self.min_labels[i].config(text="free (no bounds)")
                self.max_labels[i].config(text="")
            else:
                self.min_labels[i].config(text=f"min: {mn} ({mn * deg_per_step(i):.0f}°)")
                self.max_labels[i].config(text=f"max: {mx} ({mx * deg_per_step(i):.0f}°)")
        self.log("loaded saved bounds from EEPROM")

    # ------------------------------------------------------------ polling

    def poll_state(self):
        if not (self.polling and self.link.connected):
            return
        try:
            st = self.link.get_state(timeout=0.8)
            for i, pos in enumerate(st.get("pos", [])[:NUM_AXES]):
                self.pos_labels[i].config(text=f"{pos} ({pos * deg_per_step(i):.1f}°)")
            flags = []
            if st.get("enabled"):
                flags.append("enabled")
            if st.get("referenced"):
                flags.append("zeroed")
            if st.get("calibrated"):
                flags.append("calibrated")
            if st.get("moving"):
                flags.append("moving")
            state = st.get("state", "?")
            color = {"ready": "green", "cal": "blue", "estop": "red"}.get(state, "orange")
            self.state_label.config(text=f"{state} [{', '.join(flags) or 'idle'}]", fg=color)
        except LinkError:
            pass  # transient timeout; next poll will catch up
        self.root.after(POLL_MS, self.poll_state)

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
    CalibrateGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
