#!/usr/bin/env python3
"""
AI Physical Desk Assistant - Desktop Control Application
Phase 1: Full servo control with GUI sliders and idle animations

COMMUNICATION ARCHITECTURE:
- Simple request-response pattern (no async telemetry confusion)
- Explicit ACK/NACK for every command
- Separate telemetry polling on timer
- Robust frame synchronization with length prefix validation
"""

import cbor2
import serial
import serial.tools.list_ports
import time
import sys
import threading
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from typing import Optional, Dict, Any, List
import queue
import struct

class ArmController:
    """Handles communication with the ESP32 robot arm controller."""

    def __init__(self, port: Optional[str] = None, baudrate: int = 115200, log_callback=None, telemetry_callback=None):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.log_callback = log_callback
        self.telemetry_callback = telemetry_callback
        self.running = False
        self.telemetry_thread: Optional[threading.Thread] = None
        self.serial_lock = threading.Lock()  # Protect serial access

    def log(self, message: str):
        """Log message to GUI if callback provided."""
        if self.log_callback:
            self.log_callback(message)
        else:
            print(message)

    def find_esp32_port(self) -> Optional[str]:
        """Try to automatically find the ESP32 serial port."""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # Look for ESP32 or common USB serial devices
            if any(keyword in port.description.upper() for keyword in ['ESP32', 'CP210', 'CH340', 'FTDI', 'USB-SERIAL']):
                return port.device
        return None

    def connect(self) -> bool:
        """Establish serial connection to ESP32."""
        if not self.port:
            self.port = self.find_esp32_port()
            if not self.port:
                available_ports = "\n".join([f"  {port.device}: {port.description}" for port in serial.tools.list_ports.comports()])
                self.log(f"Could not automatically find ESP32. Available ports:\n{available_ports}")
                return False

        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.5)
            time.sleep(2)  # Wait for ESP32 to initialize
            
            # Clear any startup messages
            while self.serial.in_waiting > 0:
                self.serial.read(self.serial.in_waiting)
                time.sleep(0.1)
            
            self.log(f"✓ Connected to {self.port}")

            # Start telemetry polling thread
            self.running = True
            self.telemetry_thread = threading.Thread(target=self._telemetry_poll_loop, daemon=True)
            self.telemetry_thread.start()

            return True
        except serial.SerialException as e:
            self.log(f"✗ Failed to connect to {self.port}: {e}")
            return False

    def disconnect(self):
        """Close serial connection."""
        self.running = False
        if self.telemetry_thread:
            self.telemetry_thread.join(timeout=2.0)

        if self.serial and self.serial.is_open:
            self.serial.close()
            self.log("Disconnected")

    def _send_cbor_message(self, cbor_data: bytes) -> bool:
        """Send length-prefixed CBOR message (internal helper)."""
        try:
            message_length = len(cbor_data)
            if message_length > 65535:  # Max uint16
                self.log(f"✗ Message too large: {message_length} bytes")
                return False
            
            # Create length-prefixed message (2-byte big-endian length + CBOR data)
            length_prefix = struct.pack('>H', message_length)
            message = length_prefix + cbor_data
            
            # Send with lock protection
            with self.serial_lock:
                self.serial.write(message)
                self.serial.flush()
            
            return True
        except Exception as e:
            self.log(f"✗ Send error: {e}")
            return False

    def _receive_cbor_message(self, timeout: float = 2.0) -> Optional[Dict[str, Any]]:
        """Receive length-prefixed CBOR message (internal helper)."""
        try:
            start_time = time.time()
            
            # Read length prefix (2 bytes)
            with self.serial_lock:
                while self.serial.in_waiting < 2 and time.time() - start_time < timeout:
                    time.sleep(0.01)
                
                if self.serial.in_waiting < 2:
                    return None
                
                length_bytes = self.serial.read(2)
                if len(length_bytes) != 2:
                    return None
                
                response_length = struct.unpack('>H', length_bytes)[0]
                
                # Validate length
                if response_length == 0 or response_length > 65535:
                    self.log(f"✗ Invalid message length: {response_length}")
                    return None
                
                # Read CBOR message
                start_read = time.time()
                while self.serial.in_waiting < response_length and time.time() - start_read < timeout:
                    time.sleep(0.01)
                
                if self.serial.in_waiting < response_length:
                    self.log(f"✗ Timeout: expected {response_length} bytes, got {self.serial.in_waiting}")
                    return None
                
                cbor_response = self.serial.read(response_length)
            
            # Decode CBOR
            if len(cbor_response) != response_length:
                self.log(f"✗ Read mismatch: expected {response_length}, got {len(cbor_response)}")
                return None
            
            response = cbor2.loads(cbor_response)
            return response
            
        except Exception as e:
            self.log(f"✗ Receive error: {e}")
            return None

    def send_command(self, command: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Send a CBOR command to ESP32 and wait for response."""
        if not self.serial or not self.serial.is_open:
            self.log("✗ Not connected to ESP32")
            return None

        try:
            # Encode command as CBOR
            cbor_data = cbor2.dumps(command)
            self.log(f"→ {command.get('cmd', 'unknown')}: {len(cbor_data)} bytes")
            
            # Send command
            if not self._send_cbor_message(cbor_data):
                return None
            
            # Wait for response
            response = self._receive_cbor_message(timeout=2.0)
            
            if response:
                # Log response (but not telemetry spam)
                if response.get('type') != 'telemetry':
                    self.log(f"← {response.get('cmd', 'response')}: {response.get('status', 'unknown')}")
                return response
            else:
                self.log(f"✗ No response for {command.get('cmd', 'unknown')}")
                return None

        except Exception as e:
            self.log(f"✗ Command error: {e}")
            return None

    def _telemetry_poll_loop(self):
        """Background thread to poll for telemetry updates."""
        while self.running and self.serial and self.serial.is_open:
            try:
                # Request state update periodically
                time.sleep(1.0)  # Poll every 1 second
                
                if not self.running:
                    break
                
                # Send get_state command
                cbor_data = cbor2.dumps({"cmd": "get_state"})
                if self._send_cbor_message(cbor_data):
                    response = self._receive_cbor_message(timeout=1.0)
                    
                    if response and self.telemetry_callback:
                        # Convert to telemetry format
                        if "state" in response and "joints" in response:
                            telemetry = {
                                "type": "telemetry",
                                "state": response["state"],
                                "joints": response["joints"]
                            }
                            self.telemetry_callback(telemetry)
                
            except Exception as e:
                if self.running:  # Only log if still running
                    self.log(f"Telemetry error: {e}")
                time.sleep(1.0)

    def ping(self) -> bool:
        """Test communication with ESP32."""
        self.log("--- Sending Ping ---")
        response = self.send_command({"cmd": "ping"})
        success = response is not None and "state" in response
        if success:
            self.log("✓ Ping successful")
        else:
            self.log("✗ Ping failed")
        return success

    def get_state(self) -> Optional[Dict[str, Any]]:
        """Get current arm state from ESP32."""
        return self.send_command({"cmd": "get_state"})

    def set_joints(self, angles: List[float], speed: float = 0.5) -> bool:
        """Set joint angles with speed scaling."""
        command = {
            "cmd": "set_joints",
            "targets": angles,
            "speed": speed
        }
        response = self.send_command(command)
        return response is not None and response.get("status") == "ok"

    def play_idle(self, animation: str) -> bool:
        """Play idle animation."""
        command = {
            "cmd": "play_idle",
            "name": animation
        }
        response = self.send_command(command)
        return response is not None and response.get("status") == "ok"

    def emergency_stop(self) -> bool:
        """Emergency stop."""
        command = {"cmd": "estop"}
        response = self.send_command(command)
        return response is not None and response.get("status") == "ok"

class ArmControllerGUI:
    """Tkinter GUI for the Arm Controller."""

    def __init__(self, root):
        self.root = root
        self.root.title("AI Physical Desk Assistant - Phase 1")
        self.root.geometry("900x700")

        self.controller = ArmController(log_callback=self.log_message, telemetry_callback=self.handle_telemetry)
        self.connected = False

        # Joint names
        self.joint_names = ["Base", "Shoulder", "Elbow", "Wrist Pitch", "Wrist Roll", "Gripper"]
        self.joint_limits = [
            (0, 180),    # Base
            (15, 165),   # Shoulder
            (0, 180),    # Elbow
            (30, 150),   # Wrist Pitch
            (0, 180),    # Wrist Roll
            (10, 90)     # Gripper
        ]
        self.current_angles = [90.0, 45.0, 120.0, 90.0, 0.0, 30.0]
        self.updating_from_telemetry = False  # Flag to prevent feedback loop

        # Create GUI elements
        self.create_widgets()

        # Try to connect on startup
        self.root.after(100, self.connect_to_esp32)

    def create_widgets(self):
        """Create the GUI widgets."""
        # Title
        title_label = tk.Label(self.root, text="AI Physical Desk Assistant - Full Control",
                              font=("Arial", 16, "bold"))
        title_label.pack(pady=10)

        # Connection status
        self.status_label = tk.Label(self.root, text="Status: Disconnected", fg="red")
        self.status_label.pack()

        # Port selection frame
        port_frame = tk.Frame(self.root)
        port_frame.pack(pady=5)

        tk.Label(port_frame, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = tk.ttk.Combobox(port_frame, textvariable=self.port_var, state="readonly", width=15)
        self.port_combo.pack(side=tk.LEFT, padx=5)
        self.refresh_ports()

        refresh_btn = tk.Button(port_frame, text="↻", command=self.refresh_ports, width=3)
        refresh_btn.pack(side=tk.LEFT)

        # Main control frame
        control_frame = tk.Frame(self.root)
        control_frame.pack(pady=10, fill=tk.X, padx=10)

        # Joint control sliders
        joints_frame = tk.LabelFrame(control_frame, text="Joint Control", padx=10, pady=10)
        joints_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        self.joint_scales = []
        self.joint_labels = []
        self.current_labels = []

        for i, (name, limits) in enumerate(zip(self.joint_names, self.joint_limits)):
            # Joint label
            label = tk.Label(joints_frame, text=f"{name}:", font=("Arial", 10, "bold"))
            label.grid(row=i, column=0, sticky="w", pady=2)

            # Current value label
            current_label = tk.Label(joints_frame, text=f"{self.current_angles[i]:.1f}°", width=8, anchor="e")
            current_label.grid(row=i, column=1, padx=5)
            self.current_labels.append(current_label)

            # Slider
            scale = tk.Scale(joints_frame, from_=limits[0], to=limits[1], resolution=0.5,
                           orient=tk.HORIZONTAL, length=200, command=lambda val, idx=i: self.on_joint_change(idx, val))
            scale.set(self.current_angles[i])
            scale.grid(row=i, column=2, padx=5)
            self.joint_scales.append(scale)

        # Speed control
        speed_frame = tk.LabelFrame(control_frame, text="Speed Control", padx=10, pady=10)
        speed_frame.pack(side=tk.TOP, fill=tk.X, pady=(0, 10))

        tk.Label(speed_frame, text="Speed:").grid(row=0, column=0, sticky="w")
        self.speed_scale = tk.Scale(speed_frame, from_=0.0, to=1.0, resolution=0.1,
                                  orient=tk.HORIZONTAL, length=200)
        self.speed_scale.set(0.5)
        self.speed_scale.grid(row=0, column=1, padx=5)

        # Buttons frame
        buttons_frame = tk.LabelFrame(control_frame, text="Controls", padx=10, pady=10)
        buttons_frame.pack(side=tk.TOP, fill=tk.X)

        # Connection buttons
        conn_frame = tk.Frame(buttons_frame)
        conn_frame.pack(fill=tk.X, pady=(0, 5))

        self.connect_btn = tk.Button(conn_frame, text="Connect", command=self.connect_to_esp32)
        self.connect_btn.pack(side=tk.LEFT, padx=5)

        self.ping_btn = tk.Button(conn_frame, text="Ping", command=self.send_ping, state=tk.DISABLED)
        self.ping_btn.pack(side=tk.LEFT, padx=5)

        # Control buttons
        ctrl_frame = tk.Frame(buttons_frame)
        ctrl_frame.pack(fill=tk.X, pady=(0, 5))

        self.idle_btn = tk.Button(ctrl_frame, text="Enable Idle", command=self.toggle_idle, state=tk.DISABLED)
        self.idle_btn.pack(side=tk.LEFT, padx=5)

        self.estop_btn = tk.Button(ctrl_frame, text="EMERGENCY STOP", command=self.emergency_stop,
                                 bg="red", fg="white", state=tk.DISABLED)
        self.estop_btn.pack(side=tk.LEFT, padx=5)

        # Idle animation buttons
        idle_frame = tk.Frame(buttons_frame)
        idle_frame.pack(fill=tk.X)

        tk.Label(idle_frame, text="Idle Animations:").pack(anchor=tk.W)

        anim_frame = tk.Frame(idle_frame)
        anim_frame.pack()

        self.breathing_btn = tk.Button(anim_frame, text="Breathing", command=lambda: self.play_idle("breathing"), state=tk.DISABLED)
        self.breathing_btn.pack(side=tk.LEFT, padx=5)

        self.curious_btn = tk.Button(anim_frame, text="Curious Tilt", command=lambda: self.play_idle("curious_tilt"), state=tk.DISABLED)
        self.curious_btn.pack(side=tk.LEFT, padx=5)

        self.micro_btn = tk.Button(anim_frame, text="Micro Adjust", command=lambda: self.play_idle("micro_adjust"), state=tk.DISABLED)
        self.micro_btn.pack(side=tk.LEFT, padx=5)

        self.reset_btn = tk.Button(anim_frame, text="Reset to Neutral", command=lambda: self.play_idle("idle_reset"), state=tk.DISABLED)
        self.reset_btn.pack(side=tk.LEFT, padx=5)

        # Traffic log
        log_label = tk.Label(self.root, text="Communication Traffic:")
        log_label.pack(anchor=tk.W, padx=10)

        self.log_text = scrolledtext.ScrolledText(self.root, height=15, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        # Make log read-only but allow selection
        self.log_text.config(state=tk.DISABLED)

    def log_message(self, message: str):
        """Add message to the log text area."""
        def update_log():
            self.log_text.config(state=tk.NORMAL)
            self.log_text.insert(tk.END, message + "\n")
            self.log_text.see(tk.END)  # Auto-scroll to bottom
            self.log_text.config(state=tk.DISABLED)
        self.root.after(0, update_log)

    def handle_telemetry(self, telemetry: Dict[str, Any]):
        """Handle incoming telemetry data."""
        if "joints" in telemetry:
            joints = telemetry["joints"]
            if len(joints) == 6:
                self.updating_from_telemetry = True  # Set flag
                self.current_angles = joints[:]
                # Update GUI labels
                for i, angle in enumerate(joints):
                    self.current_labels[i].config(text=f"{angle:.1f}°")
                    # Update slider positions (but don't trigger send)
                    self.joint_scales[i].set(angle)
                self.updating_from_telemetry = False  # Clear flag

        if "state" in telemetry:
            state = telemetry["state"]
            self.status_label.config(text=f"Status: {state.title()}", fg=self.get_state_color(state))

    def get_state_color(self, state: str) -> str:
        """Get color for state display."""
        if state == "idle":
            return "green"
        elif state == "manual":
            return "blue"
        elif state == "estop":
            return "red"
        else:
            return "orange"

    def on_joint_change(self, joint_index: int, value: str):
        """Handle joint slider change."""
        # Don't send if we're updating from telemetry
        if self.updating_from_telemetry:
            return
        
        try:
            angle = float(value)
            self.current_angles[joint_index] = angle
            self.current_labels[joint_index].config(text=f"{angle:.1f}°")

            # Send joint update to ESP32
            if self.connected:
                speed = self.speed_scale.get()
                self.controller.set_joints(self.current_angles, speed)
        except ValueError:
            pass

    def connect_to_esp32(self):
        """Connect to ESP32."""
        selected_port = self.port_var.get()
        if selected_port and ' - ' in selected_port:
            self.controller.port = selected_port.split(' - ')[0]
        elif selected_port:
            self.controller.port = selected_port
        
        if self.controller.connect():
            self.status_label.config(text="Status: Connected", fg="green")
            self.connect_btn.config(text="Disconnect", command=self.disconnect_from_esp32)
            self.connected = True

            # Enable controls
            self.ping_btn.config(state=tk.NORMAL)
            self.idle_btn.config(state=tk.NORMAL)
            self.estop_btn.config(state=tk.NORMAL)
            self.breathing_btn.config(state=tk.NORMAL)
            self.curious_btn.config(state=tk.NORMAL)
            self.micro_btn.config(state=tk.NORMAL)
            self.reset_btn.config(state=tk.NORMAL)
        else:
            self.status_label.config(text="Status: Connection Failed", fg="red")

    def disconnect_from_esp32(self):
        """Disconnect from ESP32."""
        self.controller.disconnect()
        self.status_label.config(text="Status: Disconnected", fg="red")
        self.connect_btn.config(text="Connect", command=self.connect_to_esp32)
        self.connected = False

        # Disable controls
        self.ping_btn.config(state=tk.DISABLED)
        self.idle_btn.config(state=tk.DISABLED)
        self.estop_btn.config(state=tk.DISABLED)
        self.breathing_btn.config(state=tk.DISABLED)
        self.curious_btn.config(state=tk.DISABLED)
        self.micro_btn.config(state=tk.DISABLED)
        self.reset_btn.config(state=tk.DISABLED)

    def send_ping(self):
        """Send ping command."""
        self.controller.ping()

    def toggle_idle(self):
        """Toggle idle animations."""
        if "Enable" in self.idle_btn.cget("text"):
            # Switch to idle mode with breathing animation
            if self.controller.play_idle("breathing"):
                self.idle_btn.config(text="Disable Idle")
        else:
            # Switch back to manual mode by setting current joint positions
            if self.controller.set_joints(self.current_angles, 0.5):
                self.idle_btn.config(text="Enable Idle")

    def play_idle(self, animation: str):
        """Play specific idle animation."""
        self.log_message(f"--- Playing {animation} animation ---")
        if self.controller.play_idle(animation):
            self.log_message(f"✓ Started {animation} animation")
        else:
            self.log_message(f"✗ Failed to start {animation} animation")

    def emergency_stop(self):
        """Emergency stop."""
        self.log_message("--- EMERGENCY STOP ---")
        if self.controller.emergency_stop():
            self.log_message("✓ Emergency stop activated")
        else:
            self.log_message("✗ Emergency stop failed")

    def refresh_ports(self):
        """Refresh the list of available serial ports."""
        ports = serial.tools.list_ports.comports()
        port_list = [f"{port.device} - {port.description}" for port in ports]
        self.port_combo['values'] = port_list
        if port_list:
            # Auto-select first port
            self.port_combo.current(0)

def main():
    """Main GUI application."""
    root = tk.Tk()
    app = ArmControllerGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
