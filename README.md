# AI Physical Desk Assistant - Phase 1

This project implements a personality-driven physical robot arm that lives on an engineer's desk. **Phase 1 focuses on robust communication and motion control framework without AI or vision.**

## Project Structure

```
├── desktop_app/           # Python control application
│   ├── main.py           # CLI interface for arm control
│   └── arm_controller.ino # ESP32 firmware (for version coherence)
├── esp32_firmware/       # ESP32 Arduino project
│   └── arm_controller.ino # Copy of firmware for flashing
└── .clinerules/          # Project documentation
    └── Project_overview.md
```

## Hardware Requirements

- ESP32 development board (Adafruit Feather ESP32 or similar)
- USB cable for programming and communication
- Computer with Python 3.x and Arduino IDE

## Software Setup

### 1. Install Python Dependencies

```bash
pip install pyserial
```

### 2. Install Arduino IDE and ESP32 Support

1. Download Arduino IDE from https://www.arduino.cc/en/software
2. Add ESP32 board support:
   - Go to File > Preferences
   - Add this URL to "Additional Boards Manager URLs": `https://dl.espressif.com/dl/package_esp32_index.json`
   - Go to Tools > Board > Boards Manager
   - Search for "esp32" and install "esp32 by Espressif Systems"

### 3. Install ArduinoJson Library

In Arduino IDE:
- Go to Sketch > Include Library > Manage Libraries
- Search for "ArduinoJson" and install version 6.x

## Communication Testing (Phase 1)

### 1. Flash ESP32 Firmware

1. Open `esp32_firmware/arm_controller.ino` in Arduino IDE
2. Select your ESP32 board from Tools > Board
3. Select the correct COM port from Tools > Port
4. **Important**: The firmware was updated to remove debug prints - make sure you're uploading the latest version
5. Click Upload

### 2. Run Desktop Application

```bash
cd desktop_app
python main.py
```

The application will:
- Launch a GUI window with communication controls
- Show a dropdown list of available serial ports (refresh with ↻ button)
- Allow manual port selection for the ESP32
- Display all serial traffic in a scrollable log area (→ sent, ← received)
- Provide buttons for Ping and Get State commands
- Show connection status and detailed error messages

### 3. Test Commands

- **Ping**: Tests basic communication
- **Get State**: Retrieves current arm state (placeholder joint angles)

Expected JSON protocol:

```json
// Desktop → ESP32
{"cmd": "ping"}

// ESP32 → Desktop
{"state": "ready", "message": "pong", "joints": [90.0, 45.0, 120.0, 90.0, 0.0, 30.0]}
```

## Next Steps

Once communication is verified:

1. **Add servo control** to ESP32 firmware (use pins 12-17 for servos)
2. **Implement joint limits and safety** as defined in Project_overview.md
3. **Add motion smoothing and interpolation**
4. **Build GUI interface** to replace CLI
5. **Implement idle animations**

## Joint Configuration

Placeholder joint angles represent:
- Base rotation
- Shoulder pitch
- Elbow pitch
- Wrist pitch
- Wrist roll
- Gripper

Joint limits will be enforced in Phase 2.

## Troubleshooting

- **Port not found**: Check Device Manager (Windows) or `ls /dev/tty*` (Linux/Mac) for ESP32
- **JSON errors**: Ensure ArduinoJson library is installed
- **No response**: Check baud rate (115200) and USB connection

## Architecture Notes

- Communication uses JSON over UART for human-readability
- ESP32 is the motion authority and safety enforcer
- Desktop app acts as high-level command source
- No servo control in Phase 1 - pure communication testing
