# Armen Project Summary

## Overview
**Armen** is an **AI Physical Desk Assistant** - a personality-driven robotic arm designed to live on an engineer's desk. The project is currently in **Phase 1**, focusing on robust communication and motion control framework without AI or vision components.

## Project Vision
Create an interactive, expressive robotic arm that:
- Can be controlled remotely via a desktop application
- Features smooth motion control with idle animations
- Communicates through JSON-based commands
- Provides real-time telemetry and feedback
- Serves as a foundation for future AI integration

## Project Architecture

### High-Level System Block Diagram
```
[ Desktop Control App (Python) ]
        │  USB (UART / JSON)
        ▼
[ ESP32 Motion Controller ]
        │  I2C
        ▼
[ PCA9685 Servo Driver ]
        │  PWM Signals (16 channels)
        ▼
[ 6-Axis Servo Robot Arm ]
```

### Directory Structure
```
Armen/
├── README.md                                    # Installation and setup guide
├── PROJECT_SUMMARY.md                           # This file
├── SERVO_CONTROLER.jpg                          # PCA9685 servo controller pinout
├── .clinerules/
│   └── Project_overview.md                     # Detailed architecture & scope document
├── desktop_app/
│   ├── main.py                                 # Python GUI control application
│   └── arm_controller/
│       └── arm_controller.ino                  # ESP32 firmware (reference)
└── esp32_firmware/
    └── arm_controller.ino                      # ESP32 firmware (for flashing)
```

## Core Components

### 1. Hardware Layer (PCA9685 Servo Controller)
**Component:** PCA9685 PWM Servo Driver Module
**Purpose:** Manages PWM signals for all 6 servos with minimal ESP32 load

**Key Specifications:**
- **Interface:** I2C (400 kHz)
- **Channels:** 16-channel PWM output (6 servos used, 10 spare)
- **Frequency:** 50 Hz (standard servo frequency)
- **Resolution:** 12-bit per channel
- **Input Voltage:** 2.3V - 5.5V (logic power)
- **Output Voltage:** Up to servo supply voltage (6-8V typical)
- **Addressing:** Configurable via device address pins (supports multiple modules via cascading)

**Power Architecture:**
- **Logic Power:** 3.3V from ESP32
- **Servo Power:** Dedicated 6-8V supply (shared ground with ESP32)
- Power indicator LED on module
- Separate power planes to avoid noise/brownout

**Connections:**
- **ESP32 to PCA9685:** I2C (SDA, SCL) + shared ground
- **PCA9685 to Servos:** 16 PWM output channels for signal, shared power rails
- **Note:** Left and right sides of the module are connected; cascade additional modules by connecting unused side to next controller

### 2. ESP32 Firmware (`arm_controller.ino`)
**Language:** Arduino (C++)
**Purpose:** Motion control authority - orchestrates servo movements and idle animations

**Key Features:**
- 6-axis servo control via I2C to PCA9685 (no direct GPIO PWM)
- Joint angle limits and safety constraints
- Motion smoothing with configurable speed scaling
- Idle animations (breathing, curious tilt, micro-adjustments)
- JSON command parsing over serial
- Real-time telemetry streaming
- Emergency stop (E-STOP) capability
- 50 Hz update rate for smooth motion

**Hardware Configuration:**
- **Microcontroller:** ESP32 (Adafruit Feather ESP32 or standard dev board)
- **Communication:** Serial UART (115200 baud) to desktop app
- **I2C Interface:** SDA (GPIO 21), SCL (GPIO 22) to PCA9685
- **6 Servos:** Controlled via PCA9685 PWM channels 0-5

**Motion Range (Joint Limits):**
- Base: 0-180°
- Shoulder: 15-165°
- Elbow: 0-180°
- Wrist Pitch: 30-150°
- Wrist Roll: 0-180°
- Gripper: 10-90°

### 3. Desktop Application (`main.py`)
**Language:** Python 3
**Purpose:** Operator interface for manual arm control and status monitoring

**Key Features:**
- Serial port auto-detection for ESP32
- GUI with servo control sliders (one per joint)
- Real-time telemetry display
- Command logging and debugging
- JSON-based communication protocol
- Threading for non-blocking communication
- Manual joint control override
- Idle animation triggering
- Speed control scaling

**Dependencies:**
- `pyserial` - Serial communication with ESP32
- `tkinter` - GUI framework

**Functionality:**
- Auto-connect to ESP32 via USB
- Slider controls for each servo (0-180°)
- Global speed scaling control (0.0-1.0)
- Enable/disable idle animations
- Play specific idle animations
- Emergency stop button
- Command history and logging
- Connection status indicator
- Live telemetry feedback

## Communication Protocol

### Layer 1: USB UART (ESP32 ↔ Desktop App)
**Channel:** Serial UART at 115200 baud
**Format:** JSON objects terminated with newline

### Layer 2: I2C (ESP32 ↔ PCA9685)
**Channel:** I2C bus
**Speed:** 400 kHz
**Message Type:** Servo register writes

### Command Format (Desktop → ESP32)

#### Set Joint Angles with Speed
```json
{
  "cmd": "set_joints",
  "targets": [90, 45, 120, 90, 0, 30],
  "speed": 0.5
}
```

#### Play Idle Animation
```json
{
  "cmd": "play_idle",
  "name": "breathing"
}
```

#### Emergency Stop
```json
{
  "cmd": "estop"
}
```

### Response Format (ESP32 → Desktop)

#### Command Acknowledgment
```json
{
  "status": "ok",
  "cmd": "set_joints",
  "state": "moving"
}
```

### Telemetry Stream (Continuous, every 100ms)
```json
{
  "telemetry": true,
  "state": "idle",
  "joints": [88, 46, 119, 91, 1, 29]
}
```

## Hardware Requirements

### Electronics
- **ESP32 Development Board** (Adafruit Feather ESP32 or standard dev board)
- **PCA9685 PWM Servo Driver Module** (16-channel, I2C interface)
- **6 Standard Hobby Servo Motors** (with servo cable connectors)
- **External Power Supply** (6-8V DC for servos, 500mA+ recommended)
- **USB Cable** (for ESP32 programming and communication)

### Power Considerations
- Servos: Dedicated 6-8V supply with shared ground to ESP32
- ESP32 Logic: 3.3V from on-board regulator
- PCA9685 Logic: 3.3V from ESP32
- Power planes must be separate to avoid brownout/noise issues

### Optional Components
- Status LEDs (mood indication - Phase 1+)
- Laser pointer module (on/off only - Phase 2)

### Computer Requirements
- Python 3.6+
- Arduino IDE (for firmware flashing)
- USB port for communication

## System States

### 1. **IDLE**
- Arm is inactive and running hand-authored idle animations
- Responds to commands but doesn't move unless directed
- Animations loop autonomously:
  - **Breathing:** Slow shoulder + elbow oscillation (mimics respiration)
  - **Curious Tilt:** Head-like tilting of base and wrist
  - **Micro-Adjust:** Small base rotation corrections (mechanical settling feel)
  - **Idle Reset:** Returns to neutral pose after long inactivity

### 2. **MANUAL_CONTROL**
- User is actively controlling the arm via desktop GUI sliders
- Commands are processed in real-time
- Overrides idle animations
- Resumes idle after configurable timeout

### 3. **ESTOP** (Emergency Stop)
- All servos immediately stop
- Arm maintains current position
- Overrides all other states
- Command timeout (5 seconds) triggers fallback to IDLE

## Motion Control Features

### Motion Smoothing
- Gradual acceleration/deceleration prevents jerky movements
- Configurable max speeds per joint (default: 90°/second)
- Global speed scaling (0.0 - 1.0)

### Idle Animations
- **Breathing:** Gentle shoulder and elbow oscillations
- **Curious Tilt:** Head-like tilting of base and wrist
- **Micro-Adjustments:** Small random position tweaks
- **Reset:** Returns to neutral pose

### Safety Features
- Joint angle limits enforce physical constraints
- Command timeout (5 seconds) disables servos
- E-STOP capability for emergency situations

## Current Phase (Phase 1)

### Completed
✓ Full servo hardware control
✓ Motion smoothing and speed control
✓ Idle animation system
✓ JSON communication protocol
✓ Desktop GUI with slider controls
✓ Telemetry streaming
✓ Serial port auto-detection
✓ Command logging and debugging

### Future Phases
- Phase 2: AI integration and personality
- Phase 3: Computer vision integration
- Phase 4: Advanced gesture recognition
- Phase 5: Full autonomous behavior

## Design Philosophy

### Core Principles
- **Expressive > Precise:** Motion should feel alive and intentional, not just reach target angles
- **Predictable > Clever:** Users should understand what the arm will do next
- **Physical Safety > Software Elegance:** Joint limits and timeouts are non-negotiable
- **Motion is Personality:** Smooth movements and idle animations define character

### Explicitly Out of Scope (Phase 1)
- AI or natural language control (Phase 2)
- Computer vision or camera integration (Phase 2)
- Inverse kinematics
- Object interaction or manipulation learning
- Autonomous decision-making

### Expansion Hooks (Phase 2+)
- Camera → PC vision processing
- Laser pointer targeting
- Mood-driven motion parameters
- AI action selection and pose planning

## Getting Started

1. **Install Dependencies:**
   - Arduino IDE with ESP32 board support
   - ArduinoJson library (v6.x)
   - Python 3 with pyserial

2. **Flash Firmware:**
   - Open `arm_controller.ino` in Arduino IDE
   - Select ESP32 board and COM port
   - Upload firmware to ESP32

3. **Run Desktop App:**
   - `python3 desktop_app/main.py`
   - App will auto-detect and connect to ESP32

4. **Control the Arm:**
   - Use GUI sliders to control each servo
   - Monitor telemetry and command responses
   - Enable/disable idle animations as needed

## File Reference

| File | Purpose | Lines |
|------|---------|-------|
| [arm_controller.ino](desktop_app/arm_controller/arm_controller.ino) | ESP32 firmware | ~384 |
| [main.py](desktop_app/main.py) | Desktop GUI application | ~480 |
| [README.md](README.md) | Setup and installation guide | ~120 |

## System Parameters

### Timing & Update Rates
- **Serial Baud Rate:** 115200 bps
- **I2C Clock:** 400 kHz
- **Motion Update Rate:** 50 Hz (20 ms per cycle)
- **Telemetry Interval:** 100 ms
- **Command Timeout:** 5 seconds (triggers IDLE fallback)
- **Idle Animation Timeout:** 30 seconds of inactivity

### Motion Constraints
- **Joint Angle Range:** 0-180 degrees
- **Max Speed per Joint:** 90°/second (default)
- **Speed Scaling Factor:** 0.0-1.0 (multiplicative)
- **Interpolation:** Linear per joint
- **Acceleration Model:** Rate-limited step response

### PCA9685 Configuration
- **I2C Address:** 0x40 (default, configurable via address pins)
- **PWM Frequency:** 50 Hz (servo standard)
- **Channels Used:** 0-5 (6 servos)
- **Channels Available:** 16 total (10 spare for expansion)
- **Resolution:** 12-bit per channel (0-4095)

---
**Project Status:** Active Development (Phase 1)  
**Last Updated:** January 9, 2026  
**Architecture:** Refer to [Project_overview.md](.clinerules/Project_overview.md) for complete Phase 1 scope and design rationale
