#!/usr/bin/env python3
"""
AI Physical Desk Assistant - Test Mode Application
Simple single servo control for CH0 testing
"""

import cbor2
import serial
import serial.tools.list_ports
import time
import tkinter as tk
from tkinter import ttk, scrolledtext
from typing import Optional, Dict, Any
import struct

class TestModeController:
    """Handles test mode communication with ESP32."""

    def __init__(self, port: Optional[str] = None, baudrate: int = 115200, log_callback=None):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.log_callback = log_callback

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
            if any(keyword in port.description.upper() for keyword in ['ESP32', 'CP210', 'CH340', 'FTDI', 'USB-SERIAL']):
                return port.device
        return None

    def connect(self) -> bool:
        """Establish serial connection to ESP32."""
        if not self.port:
            self.port = self.find_esp32_port()
            if not self.port:
                available_ports = "\n".join([f"  {port.device}: {port.description}" for port in serial.tools.list_ports.comports()])
                self.log(f"Could not find ESP32. Available ports:\n{available_ports}")
                return False

        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.5)
            time.sleep(2)  # Wait for ESP32 to initialize
            
            # Clear any startup messages
            while self.serial.in_waiting > 0:
                self.serial.read(self.serial.in_waiting)
                time.sleep(0.1)
            
            self.log(f"✓ Connected to {self.port}")
            return True
        except serial.SerialException as e:
            self.log(f"✗ Failed to connect to {self.port}: {e}")
            return False

    def disconnect(self):
        """Close serial connection."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.log("Disconnected")

    def _send_cbor_message(self, cbor_data: bytes) -> bool:
        """Send length-prefixed CBOR message."""
        try:
            message_length = len(cbor_data)
            if message_length > 65535:
                self.log(f"✗ Message too large: {message_length} bytes")
                return False
            
            length_prefix = struct.pack('>H', message_length)
            message = length_prefix + cbor_data
            
            self.serial.write(message)
            self.serial.flush()
            
            return True
        except Exception as e:
            self.log(f"✗ Send error: {e}")
            return False

    def _receive_cbor_message(self, timeout: float = 2.0) -> Optional[Dict[str, Any]]:
        """Receive length-prefixed CBOR message."""
        try:
            start_time = time.time()
            
            # Read length prefix (2 bytes)
            while self.serial.in_waiting < 2 and time.time() - start_time < timeout:
                time.sleep(0.01)
            
            if self.serial.in_waiting < 2:
                return None
            
            length_bytes = self.serial.read(2)
            if len(length_bytes) != 2:
                return None
            
            response_length = struct.unpack('>H', length_bytes)[0]
            
            if response_length == 0 or response_length > 65535:
                self.log(f"✗ Invalid message length: {response_length}")
                return None
            
            # Read CBOR message
            start_read = time.time()
            while self.serial.in_waiting < response_length and time.time() - start_read < timeout:
                time.sleep(0.01)
            
            if self.serial.in_waiting < response_length:
                return None
            
            cbor_response = self.serial.read(response_length)
            
            if len(cbor_response) != response_length:
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
            cbor_data = cbor2.dumps(command)
            self.log(f"→ {command.get('cmd', 'unknown')}")
            
            if not self._send_cbor_message(cbor_data):
                return None
            
            response = self._receive_cbor_message(timeout=2.0)
            
            if response:
                self.log(f"← {response.get('cmd', 'response')}: {response.get('status', 'unknown')}")
                return response
            else:
                self.log(f"✗ No response")
                return None

        except Exception as e:
            self.log(f"✗ Command error: {e}")
            return None

    def enter_test_mode(self) -> bool:
        """Enter test mode."""
        response = self.send_command({"cmd": "test_mode"})
        return response is not None and response.get("status") == "ok"

    def exit_test_mode(self) -> bool:
        """Exit test mode."""
        response = self.send_command({"cmd": "exit_test_mode"})
        return response is not None and response.get("status") == "ok"

    def set_test_servo(self, angle: float) -> bool:
        """Set CH0 servo angle."""
        command = {
            "cmd": "set_test_servo",
            "targets": [angle, 0, 0, 0, 0, 0]  # Only first value matters
        }
        response = self.send_command(command)
        return response is not None and response.get("status") == "ok"

class TestModeGUI:
    """Simple GUI for test mode."""

    def __init__(self, root):
        self.root = root
        self.root.title("Servo Test Mode - CH0 Only")
        self.root.geometry("600x500")

        self.controller = TestModeController(log_callback=self.log_message)
        self.connected = False
        self.test_mode_active = False

        self.create_widgets()

    def create_widgets(self):
        """Create the GUI widgets."""
        # Title
        title_label = tk.Label(self.root, text="Servo Test Mode (Channel 0)",
                              font=("Arial", 16, "bold"))
        title_label.pack(pady=10)

        # Status
        self.status_label = tk.Label(self.root, text="Status: Disconnected", fg="red")
        self.status_label.pack()

        # Port selection
        port_frame = tk.Frame(self.root)
        port_frame.pack(pady=5)

        tk.Label(port_frame, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = tk.ttk.Combobox(port_frame, textvariable=self.port_var, state="readonly", width=15)
        self.port_combo.pack(side=tk.LEFT, padx=5)
        self.refresh_ports()

        refresh_btn = tk.Button(port_frame, text="↻", command=self.refresh_ports, width=3)
        refresh_btn.pack(side=tk.LEFT)

        # Control frame
        control_frame = tk.LabelFrame(self.root, text="CH0 Servo Control", padx=20, pady=20)
        control_frame.pack(pady=20, fill=tk.X, padx=20)

        # Servo angle display
        self.angle_label = tk.Label(control_frame, text="90.0°", font=("Arial", 24, "bold"))
        self.angle_label.pack(pady=10)

        # Slider
        self.servo_scale = tk.Scale(control_frame, from_=0, to=180, resolution=0.5,
                                   orient=tk.HORIZONTAL, length=400,
                                   command=self.on_servo_change,
                                   showvalue=False)
        self.servo_scale.set(90.0)
        self.servo_scale.pack(pady=10)

        # Limit labels
        limit_frame = tk.Frame(control_frame)
        limit_frame.pack()
        tk.Label(limit_frame, text="0°").pack(side=tk.LEFT, padx=10)
        tk.Label(limit_frame, text="90°").pack(side=tk.LEFT, padx=140)
        tk.Label(limit_frame, text="180°").pack(side=tk.LEFT, padx=10)

        # Buttons
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=10)

        self.connect_btn = tk.Button(button_frame, text="Connect", command=self.connect_to_esp32)
        self.connect_btn.pack(side=tk.LEFT, padx=5)

        self.test_mode_btn = tk.Button(button_frame, text="Enter Test Mode",
                                      command=self.toggle_test_mode, state=tk.DISABLED)
        self.test_mode_btn.pack(side=tk.LEFT, padx=5)

        # Log
        log_label = tk.Label(self.root, text="Communication Log:")
        log_label.pack(anchor=tk.W, padx=20)

        self.log_text = scrolledtext.ScrolledText(self.root, height=10, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=20, pady=5)
        self.log_text.config(state=tk.DISABLED)

    def log_message(self, message: str):
        """Add message to the log."""
        def update_log():
            self.log_text.config(state=tk.NORMAL)
            self.log_text.insert(tk.END, message + "\n")
            self.log_text.see(tk.END)
            self.log_text.config(state=tk.DISABLED)
        self.root.after(0, update_log)

    def on_servo_change(self, value: str):
        """Handle servo slider change."""
        if not self.test_mode_active:
            return
        
        try:
            angle = float(value)
            self.angle_label.config(text=f"{angle:.1f}°")
            self.controller.set_test_servo(angle)
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
            self.test_mode_btn.config(state=tk.NORMAL)
            self.connected = True
        else:
            self.status_label.config(text="Status: Connection Failed", fg="red")

    def disconnect_from_esp32(self):
        """Disconnect from ESP32."""
        if self.test_mode_active:
            self.controller.exit_test_mode()
            self.test_mode_active = False
        
        self.controller.disconnect()
        self.status_label.config(text="Status: Disconnected", fg="red")
        self.connect_btn.config(text="Connect", command=self.connect_to_esp32)
        self.test_mode_btn.config(state=tk.DISABLED, text="Enter Test Mode")
        self.connected = False

    def toggle_test_mode(self):
        """Toggle test mode on/off."""
        if not self.test_mode_active:
            # Enter test mode
            if self.controller.enter_test_mode():
                self.test_mode_active = True
                self.test_mode_btn.config(text="Exit Test Mode")
                self.status_label.config(text="Status: Test Mode Active", fg="blue")
                self.log_message("✓ Test mode enabled - telemetry disabled")
            else:
                self.log_message("✗ Failed to enter test mode")
        else:
            # Exit test mode
            if self.controller.exit_test_mode():
                self.test_mode_active = False
                self.test_mode_btn.config(text="Enter Test Mode")
                self.status_label.config(text="Status: Connected", fg="green")
                self.log_message("✓ Test mode disabled - telemetry re-enabled")
            else:
                self.log_message("✗ Failed to exit test mode")

    def refresh_ports(self):
        """Refresh the list of available serial ports."""
        ports = serial.tools.list_ports.comports()
        port_list = [f"{port.device} - {port.description}" for port in ports]
        self.port_combo['values'] = port_list
        if port_list:
            self.port_combo.current(0)

def main():
    """Main test mode application."""
    root = tk.Tk()
    app = TestModeGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
