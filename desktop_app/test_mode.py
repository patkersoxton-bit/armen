#!/usr/bin/env python3
"""
AI Physical Desk Assistant - Enhanced Test Mode Application
Individual servo control for channels 0-5 with joint-specific limits
"""

import cbor2
import serial
import serial.tools.list_ports
import time
import tkinter as tk
from tkinter import ttk, scrolledtext
from typing import Optional, Dict, Any
import struct

# Joint names and limits
JOINT_INFO = {
    0: {"name": "Base", "min": 0, "max": 180},
    1: {"name": "Shoulder", "min": 15, "max": 165},
    2: {"name": "Elbow", "min": 0, "max": 180},
    3: {"name": "Wrist Pitch", "min": 30, "max": 150},
    4: {"name": "Wrist Roll", "min": 0, "max": 180},
    5: {"name": "Gripper", "min": 10, "max": 90}
}

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
            
            # Wait for command response, skipping telemetry messages
            max_attempts = 5
            for attempt in range(max_attempts):
                response = self._receive_cbor_message(timeout=1.0)
                
                if not response:
                    self.log(f"✗ No response (attempt {attempt + 1}/{max_attempts})")
                    continue
                
                # Check if this is a telemetry message (skip it)
                if response.get('type') == 'telemetry':
                    self.log(f"← Telemetry (skipping): state={response.get('state')}")
                    continue
                
                # This is a command response
                self.log(f"← Response: {response}")
                cmd = response.get('cmd', 'unknown')
                status = response.get('status', 'unknown')
                self.log(f"  cmd={cmd}, status={status}")
                return response
            
            self.log(f"✗ No command response received after {max_attempts} attempts")
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

    def set_test_channel(self, channel: int) -> Optional[Dict[str, Any]]:
        """Set which channel to test."""
        command = {"cmd": "set_test_channel", "channel": channel}
        return self.send_command(command)

    def set_test_servo(self, angle: float, channel: Optional[int] = None) -> bool:
        """Set servo angle for current or specified channel."""
        command = {
            "cmd": "set_test_servo",
            "targets": [angle, 0, 0, 0, 0, 0]
        }
        if channel is not None:
            command["channel"] = channel
        
        response = self.send_command(command)
        return response is not None and response.get("status") == "ok"


class TestModeGUI:
    """Enhanced GUI for test mode with channel selection."""

    def __init__(self, root):
        self.root = root
        self.root.title("Servo Test Mode - All Channels")
        self.root.geometry("700x600")

        self.controller = TestModeController(log_callback=self.log_message)
        self.connected = False
        self.test_mode_active = False
        self.current_channel = 0

        self.create_widgets()

    def create_widgets(self):
        """Create the GUI widgets."""
        # Title
        title_label = tk.Label(self.root, text="Enhanced Servo Test Mode",
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

        # Channel selection frame
        channel_frame = tk.LabelFrame(self.root, text="Channel Selection", padx=20, pady=10)
        channel_frame.pack(pady=10, fill=tk.X, padx=20)

        # Create channel buttons in a grid
        button_frame = tk.Frame(channel_frame)
        button_frame.pack()
        
        self.channel_buttons = []
        for i in range(6):
            info = JOINT_INFO[i]
            btn = tk.Button(
                button_frame,
                text=f"CH{i}\n{info['name']}",
                width=10,
                height=3,
                command=lambda ch=i: self.select_channel(ch)
            )
            btn.grid(row=i//3, column=i%3, padx=5, pady=5)
            self.channel_buttons.append(btn)
        
        # Highlight initial channel
        self.channel_buttons[0].config(relief=tk.SUNKEN, bg="lightblue")

        # Control frame
        control_frame = tk.LabelFrame(self.root, text="Servo Control", padx=20, pady=20)
        control_frame.pack(pady=10, fill=tk.X, padx=20)

        # Current channel display
        self.channel_info_label = tk.Label(
            control_frame,
            text=f"CH{self.current_channel}: {JOINT_INFO[self.current_channel]['name']}",
            font=("Arial", 14, "bold")
        )
        self.channel_info_label.pack(pady=5)

        # Servo angle display
        self.angle_label = tk.Label(control_frame, text="90.0°", font=("Arial", 24, "bold"))
        self.angle_label.pack(pady=10)

        # Slider
        self.servo_scale = tk.Scale(
            control_frame,
            from_=0,
            to=180,
            resolution=0.5,
            orient=tk.HORIZONTAL,
            length=500,
            command=self.on_servo_change,
            showvalue=False
        )
        self.servo_scale.set(90.0)
        self.servo_scale.pack(pady=10)

        # Limit labels
        self.limit_frame = tk.Frame(control_frame)
        self.limit_frame.pack()
        self.min_label = tk.Label(self.limit_frame, text="0°", fg="red")
        self.min_label.pack(side=tk.LEFT, padx=10)
        self.mid_label = tk.Label(self.limit_frame, text="90°")
        self.mid_label.pack(side=tk.LEFT, padx=190)
        self.max_label = tk.Label(self.limit_frame, text="180°", fg="red")
        self.max_label.pack(side=tk.LEFT, padx=10)

        # Buttons
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=10)

        self.connect_btn = tk.Button(button_frame, text="Connect", command=self.connect_to_esp32)
        self.connect_btn.pack(side=tk.LEFT, padx=5)

        self.test_mode_btn = tk.Button(
            button_frame,
            text="Enter Test Mode",
            command=self.toggle_test_mode,
            state=tk.DISABLED
        )
        self.test_mode_btn.pack(side=tk.LEFT, padx=5)

        # Log
        log_label = tk.Label(self.root, text="Communication Log:")
        log_label.pack(anchor=tk.W, padx=20)

        self.log_text = scrolledtext.ScrolledText(self.root, height=8, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=20, pady=5)
        self.log_text.config(state=tk.DISABLED)

    def select_channel(self, channel: int):
        """Switch to a different channel."""
        if not self.test_mode_active:
            self.log_message("✗ Must be in test mode to select channels")
            return
        
        # Update visual feedback
        for i, btn in enumerate(self.channel_buttons):
            if i == channel:
                btn.config(relief=tk.SUNKEN, bg="lightblue")
            else:
                btn.config(relief=tk.RAISED, bg="SystemButtonFace")
        
        self.current_channel = channel
        info = JOINT_INFO[channel]
        
        # Update labels
        self.channel_info_label.config(text=f"CH{channel}: {info['name']}")
        
        # Update slider limits
        self.servo_scale.config(from_=info['min'], to=info['max'])
        mid_val = (info['min'] + info['max']) / 2
        self.servo_scale.set(mid_val)
        
        # Update limit labels
        self.min_label.config(text=f"{info['min']}°")
        self.max_label.config(text=f"{info['max']}°")
        self.mid_label.config(text=f"{mid_val:.0f}°")
        
        # Send command to ESP32
        response = self.controller.set_test_channel(channel)
        if response and response.get("status") == "ok":
            self.log_message(f"✓ Switched to CH{channel}: {info['name']}")
            # Update limits from ESP32 response if provided
            if "min" in response and "max" in response:
                self.log_message(f"  Limits: {response['min']}° - {response['max']}°")
        else:
            self.log_message(f"✗ Failed to switch to CH{channel}")

    def update_limit_display(self):
        """Update the limit labels for current channel."""
        info = JOINT_INFO[self.current_channel]
        mid_val = (info['min'] + info['max']) / 2
        
        # Adjust mid label position dynamically
        range_val = info['max'] - info['min']
        padding = int(190 * (range_val / 180.0))  # Scale padding based on range
        
        for widget in self.limit_frame.winfo_children():
            widget.destroy()
        
        self.min_label = tk.Label(self.limit_frame, text=f"{info['min']}°", fg="red")
        self.min_label.pack(side=tk.LEFT, padx=10)
        self.mid_label = tk.Label(self.limit_frame, text=f"{mid_val:.0f}°")
        self.mid_label.pack(side=tk.LEFT, padx=padding)
        self.max_label = tk.Label(self.limit_frame, text=f"{info['max']}°", fg="red")
        self.max_label.pack(side=tk.LEFT, padx=10)

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
                # Initialize to channel 0
                self.select_channel(0)
            else:
                self.log_message("✗ Failed to enter test mode")
        else:
            # Exit test mode
            if self.controller.exit_test_mode():
                self.test_mode_active = False
                self.test_mode_btn.config(text="Enter Test Mode")
                self.status_label.config(text="Status: Connected", fg="green")
                self.log_message("✓ Test mode disabled - telemetry re-enabled")
                
                # Reset visual feedback
                for btn in self.channel_buttons:
                    btn.config(relief=tk.RAISED, bg="SystemButtonFace")
                self.channel_buttons[0].config(relief=tk.SUNKEN, bg="lightblue")
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
