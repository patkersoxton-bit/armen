#!/usr/bin/env python3
"""Armen — arm jog controller (the post-calibration daily driver).

Reads the bounds saved in the Uno's EEPROM and builds matching controls:
  - Bounded joints get a degree slider spanning exactly [min, max] plus
    nudge buttons; targets are clamped in the UI *and* by the firmware.
  - Free axes (min == max in EEPROM, for continuous-rotation joints with no
    mechanical limit) get unlimited jog buttons instead of a slider. The
    base (X) used to be treated as free pre-gearbox; after the 37:1 gearbox
    it likely has real bounds instead — don't assume either way, check what
    calibrate.py actually saved.

Power-up ritual (open-loop steppers — positions mean nothing until zeroed):
  pose arm at reference -> Connect -> Enable -> Zero Here -> drive.
"""

import tkinter as tk
from tkinter import scrolledtext

from stepper_link import StepperLink, LinkError, NUM_AXES, deg_per_step

AXIS_NAMES = ["Base (X)", "Shoulder (Y)", "Elbow (Z)", "A (spare)"]
NUDGE_DEGREES = [-30.0, -5.0, -1.0, 1.0, 5.0, 30.0]
FREE_JOG_DEGREES = [-360.0, -90.0, -15.0, 15.0, 90.0, 360.0]
POLL_MS = 500
MOTION_SPEED = 2400   # steps/s, applied on connect (1/16 microstep scale)
MOTION_ACCEL = 7200


class ArmControlGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Armen — Arm Control")
        self.link = StepperLink(log=self.log)
        self.polling = False

        # Per-axis (min, max) in steps, or None for a free axis; filled from
        # the firmware's saved calibration on connect.
        self.bounds = [None] * NUM_AXES
        self.targets = [0] * NUM_AXES
        self.sliders = {}       # axis -> tk.Scale (bounded axes only)
        self.pos_labels = []

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

        ritual = tk.LabelFrame(self.root, text="Session (pose arm at reference before zeroing!)")
        ritual.pack(fill=tk.X, padx=10, pady=5)
        tk.Button(ritual, text="Enable Motors",
                  command=lambda: self.cmd(self.link.enable)).pack(side=tk.LEFT, padx=4)
        tk.Button(ritual, text="Zero Here (reference pose)",
                  command=self.zero_here).pack(side=tk.LEFT, padx=4)
        tk.Button(ritual, text="Go to Zero Pose",
                  command=self.go_zero).pack(side=tk.LEFT, padx=4)
        tk.Button(ritual, text="Disable (arm may slump!)",
                  command=lambda: self.cmd(self.link.disable)).pack(side=tk.LEFT, padx=4)
        tk.Button(ritual, text="E-STOP", bg="red", fg="white",
                  command=lambda: self.cmd(self.link.estop)).pack(side=tk.LEFT, padx=12)
        tk.Button(ritual, text="Reset E-Stop",
                  command=lambda: self.cmd(self.link.reset)).pack(side=tk.LEFT, padx=4)

        speed = tk.Frame(self.root)
        speed.pack(fill=tk.X, padx=10)
        tk.Label(speed, text="Speed:").pack(side=tk.LEFT)
        self.speed_scale = tk.Scale(speed, from_=0.1, to=1.0, resolution=0.05,
                                    orient=tk.HORIZONTAL, length=220)
        self.speed_scale.set(0.4)
        self.speed_scale.pack(side=tk.LEFT, padx=5)

        self.axes_frame = tk.LabelFrame(self.root, text="Joints (connect to load saved bounds)")
        self.axes_frame.pack(fill=tk.X, padx=10, pady=5)

        self.log_text = scrolledtext.ScrolledText(self.root, height=10, wrap=tk.WORD,
                                                  state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

    def _build_axis_rows(self):
        for child in self.axes_frame.winfo_children():
            child.destroy()
        self.sliders.clear()
        self.pos_labels.clear()

        for i, name in enumerate(AXIS_NAMES):
            row = tk.Frame(self.axes_frame)
            row.pack(fill=tk.X, pady=2)
            tk.Label(row, text=name, width=13, anchor="w",
                     font=("Arial", 10, "bold")).pack(side=tk.LEFT)
            pos = tk.Label(row, text="—", width=15, anchor="e", relief=tk.SUNKEN)
            pos.pack(side=tk.LEFT, padx=4)
            self.pos_labels.append(pos)

            if self.bounds[i] is None:
                tk.Label(row, text="free 360°:").pack(side=tk.LEFT, padx=(8, 2))
                for deg in FREE_JOG_DEGREES:
                    tk.Button(row, text=f"{deg:+.0f}°", width=6,
                              command=lambda a=i, d=deg: self.nudge(a, d)
                              ).pack(side=tk.LEFT, padx=1)
            else:
                mn, mx = self.bounds[i]
                dps = deg_per_step(i)
                scale = tk.Scale(row, from_=mn * dps, to=mx * dps,
                                 resolution=0.5, orient=tk.HORIZONTAL, length=260)
                scale.set(self.targets[i] * dps)
                scale.bind("<ButtonRelease-1>", lambda e, a=i: self.slider_released(a))
                scale.pack(side=tk.LEFT, padx=6)
                self.sliders[i] = scale
                for deg in NUDGE_DEGREES:
                    tk.Button(row, text=f"{deg:+.0f}°", width=5,
                              command=lambda a=i, d=deg: self.nudge(a, d)
                              ).pack(side=tk.LEFT, padx=1)

        self.axes_frame.config(text="Joints (targets clamped to saved bounds)")

    # ------------------------------------------------------------ motion

    def _clamp(self, axis: int, steps: int) -> int:
        if self.bounds[axis] is None:
            return steps
        mn, mx = self.bounds[axis]
        return max(mn, min(mx, steps))

    def send_targets(self):
        self.cmd(self.link.move, list(self.targets), self.speed_scale.get())

    def nudge(self, axis: int, degrees: float):
        delta = round(degrees / deg_per_step(axis))
        new_target = self._clamp(axis, self.targets[axis] + delta)
        if new_target == self.targets[axis] and self.bounds[axis] is not None:
            self.log(f"{AXIS_NAMES[axis]}: already at bound")
            return
        self.targets[axis] = new_target
        if axis in self.sliders:
            self.sliders[axis].set(new_target * deg_per_step(axis))
        self.send_targets()

    def slider_released(self, axis: int):
        steps = self._clamp(axis, round(self.sliders[axis].get() / deg_per_step(axis)))
        if steps != self.targets[axis]:
            self.targets[axis] = steps
            self.send_targets()

    def zero_here(self):
        resp = self.cmd(self.link.zero)
        if resp and resp.get("status") == "ok":
            self.targets = [0] * NUM_AXES
            for axis, scale in self.sliders.items():
                scale.set(0)

    def go_zero(self):
        self.targets = [self._clamp(i, 0) for i in range(NUM_AXES)]
        for axis, scale in self.sliders.items():
            scale.set(self.targets[axis] * deg_per_step(axis))
        self.send_targets()

    # ------------------------------------------------------------ connection

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
            self.link.set_motion(MOTION_SPEED, MOTION_ACCEL)

            cal = self.link.get_cal()
            if not cal.get("calibrated"):
                self.log("✗ no saved bounds in EEPROM — run calibrate.py first")
                self.link.close()
                return
            for i in range(NUM_AXES):
                mn, mx = cal["min"][i], cal["max"][i]
                self.bounds[i] = None if mn == mx else (mn, mx)
                kind = "free" if self.bounds[i] is None else \
                    f"{mn * deg_per_step(i):.0f}° … {mx * deg_per_step(i):.0f}°"
                self.log(f"{AXIS_NAMES[i]}: {kind}")

            st = self.link.get_state()
            self.targets = list(st.get("pos", [0] * NUM_AXES))[:NUM_AXES]
            self._build_axis_rows()

            self.connect_btn.config(text="Disconnect")
            self.log("ritual: pose at reference → Enable Motors → Zero Here → drive")
            self.polling = True
            self.root.after(POLL_MS, self.poll_state)
        except (LinkError, OSError) as e:
            self.log(f"connect failed: {e}")
            self.link.close()

    def cmd(self, fn, *args):
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

    def poll_state(self):
        if not (self.polling and self.link.connected):
            return
        try:
            st = self.link.get_state(timeout=0.8)
            for i, pos in enumerate(st.get("pos", [])[:NUM_AXES]):
                if i < len(self.pos_labels):
                    self.pos_labels[i].config(text=f"{pos} ({pos * deg_per_step(i):.1f}°)")
            flags = []
            if st.get("enabled"):
                flags.append("enabled")
            if st.get("referenced"):
                flags.append("zeroed")
            if st.get("moving"):
                flags.append("moving")
            state = st.get("state", "?")
            color = {"ready": "green", "estop": "red"}.get(state, "orange")
            self.state_label.config(text=f"{state} [{', '.join(flags) or 'idle'}]", fg=color)
        except LinkError:
            pass  # transient timeout; next poll catches up
        self.root.after(POLL_MS, self.poll_state)

    def log(self, message: str):
        def append():
            self.log_text.config(state=tk.NORMAL)
            self.log_text.insert(tk.END, message + "\n")
            self.log_text.see(tk.END)
            self.log_text.config(state=tk.DISABLED)
        self.root.after(0, append)


def main():
    root = tk.Tk()
    ArmControlGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
