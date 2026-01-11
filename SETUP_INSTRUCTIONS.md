# Setup Instructions - AI Physical Desk Assistant

## Overview

This document provides complete setup instructions for the AI Physical Desk Assistant with the updated architecture:
- CBOR binary communication protocol (replacing JSON)
- PCA9685 16-channel PWM servo controller (replacing direct ESP32 control)

## Hardware Requirements

1. **ESP32 Development Board** (any standard ESP32 dev board)
2. **PCA9685 16-Channel PWM Servo Controller**
3. **6x Hobby Servos** (standard analog servos, 5-6V)
4. **External Power Supply** (5-6V DC, minimum 3A, 4A recommended)
5. **USB Cable** (for ESP32 programming and PC communication)
6. **Jumper Wires** for connections

## Software Requirements

### Desktop Application (Python)

1. **Python 3.7 or higher**
2. **Required Python packages:**
   ```bash
   pip install -r desktop_app/requirements.txt
   ```
   Or manually:
   ```bash
   pip install pyserial>=3.5 cbor2>=5.4.6
   ```

### ESP32 Firmware (Arduino IDE)

1. **Arduino IDE** (1.8.x or 2.x)
2. **ESP32 Board Support:**
   - In Arduino IDE, go to File → Preferences
   - Add to "Additional Board Manager URLs": 
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Go to Tools → Board → Boards Manager
   - Search for "ESP32" and install "esp32 by Espressif Systems"

3. **Required Arduino Libraries** (install via Library Manager):
   - **Adafruit PWM Servo Driver Library** (by Adafruit)
   - **Adafruit Bus IO Library** (dependency, auto-installed)
   - **CBOR** (by Aron Strandberg) - Search for "CBOR" in Library Manager

## Hardware Assembly

### Step 1: Wire the ESP32 to PCA9685

Follow the wiring diagram in `WIRING_SCHEMATIC.md`:

| ESP32 Pin | PCA9685 Pin | Description |
|-----------|-------------|-------------|
| GPIO21    | SDA         | I2C Data    |
| GPIO22    | SCL         | I2C Clock   |
| GND       | GND         | Ground      |
| 3.3V or 5V| VCC         | Logic Power |

**CRITICAL:** Connect OE (Output Enable) pin on PCA9685 to GND to enable outputs.

### Step 2: Connect External Power Supply

1. Connect external 5-6V power supply:
   - **Positive (+)** → PCA9685 **V+** terminal
   - **Negative (-)** → PCA9685 **GND** terminal

2. **Establish Common Ground:**
   - Connect power supply GND to ESP32 GND
   - All grounds must be connected together!

### Step 3: Connect Servos to PCA9685

Connect servos to PCA9685 channels 0-5:

| Channel | Joint         | Servo Wire Colors |
|---------|---------------|-------------------|
| 0       | Base          | Yellow=Signal, Red=V+, Black=GND |
| 1       | Shoulder      | Yellow=Signal, Red=V+, Black=GND |
| 2       | Elbow         | Yellow=Signal, Red=V+, Black=GND |
| 3       | Wrist Pitch   | Yellow=Signal, Red=V+, Black=GND |
| 4       | Wrist Roll    | Yellow=Signal, Red=V+, Black=GND |
| 5       | Gripper       | Yellow=Signal, Red=V+, Black=GND |

**Note:** Wire colors may vary by manufacturer. Typically:
- Signal = Yellow, Orange, or White
- Power = Red
- Ground = Black or Brown

### Step 4: Connect ESP32 to PC

Connect ESP32 to your computer via USB cable. This provides:
- Power to ESP32
- Programming interface
- Serial communication for control

## Software Setup

### Step 1: Upload ESP32 Firmware

1. Open Arduino IDE
2. Open the firmware file: `desktop_app/arm_controller/arm_controller.ino`
3. Select your ESP32 board:
   - Tools → Board → ESP32 Arduino → (your ESP32 board model)
   - Common: "ESP32 Dev Module" or "DOIT ESP32 DEVKIT V1"
4. Select the correct port:
   - Tools → Port → (your ESP32's COM port)
5. Click **Upload** button
6. Wait for upload to complete (~30 seconds)

### Step 2: Verify ESP32 Operation

1. Open Arduino IDE Serial Monitor:
   - Tools → Serial Monitor
   - Set baud rate to **115200**
2. You should see:
   ```
   ESP32 Arm Controller initialized with CBOR and PCA9685
   ```
3. If you see errors about PCA9685 not found:
   - Check I2C wiring (SDA, SCL, GND, VCC)
   - Verify PCA9685 is powered
   - Run I2C scanner code (see Troubleshooting section)

### Step 3: Test Servo Movement

Before running the full application, test individual servos:

1. With Serial Monitor open, servos should move to initial positions
2. Observe servo behavior:
   - Servos should move smoothly to neutral positions
   - No jittering or erratic behavior
   - If servos don't move, check power supply

## Running the Desktop Application

### Step 1: Start the Application

```bash
cd desktop_app
python main.py
```

Or on Windows:
```bash
cd desktop_app
python.exe main.py
```

### Step 2: Connect to ESP32

1. The application will attempt to auto-connect on startup
2. If connection fails:
   - Use the port dropdown to select your ESP32's COM port
   - Click "Connect" button
3. Status should show: **"Status: Connected"** in green

### Step 3: Test Communication

1. Click **"Ping"** button
2. Check the communication log for:
   ```
   → CBOR(X bytes): {'cmd': 'ping'}
   ← CBOR(Y bytes): {'cmd': 'ping', 'status': 'ok', ...}
   ✓ Ping successful
   ```

### Step 4: Test Servo Control

1. Move joint sliders
2. Observe:
   - Servos should move smoothly
   - Current angle labels update
   - CBOR communication shows smaller byte counts than old JSON

### Step 5: Test Idle Animations

1. Click **"Breathing"** button
2. Observe arm performing slow breathing motion
3. Try other animations:
   - **Curious Tilt**: Head tilts left/right
   - **Micro Adjust**: Small random adjustments
   - **Reset to Neutral**: Returns to neutral pose

## Calibration

### Servo Pulse Width Calibration

If servos don't reach full range or move incorrectly:

1. Open `arm_controller.ino`
2. Locate these defines:
   ```cpp
   #define SERVOMIN 150   // Minimum pulse length count
   #define SERVOMAX 600   // Maximum pulse length count
   ```

3. Adjust values based on your servos:
   - Standard servos: SERVOMIN=150 (~1ms), SERVOMAX=600 (~2ms)
   - If servo doesn't reach 0°: Increase SERVOMIN
   - If servo doesn't reach 180°: Increase SERVOMAX
   - Typical range: SERVOMIN=100-200, SERVOMAX=500-650

4. Re-upload firmware after changes

### Per-Joint Calibration

For fine-tuning individual joints, you can modify pulse width mapping in `setServoAnglePCA()`:

```cpp
// Example: Adjust for specific joint
if (channel == 0) {  // Base joint
  pulseCount = map(angle * 10, 0, 1800, 140, 610);  // Custom range
} else {
  pulseCount = map(angle * 10, 0, 1800, SERVOMIN, SERVOMAX);
}
```

## Troubleshooting

### Issue: "Could not automatically find ESP32"

**Solution:**
1. Verify USB cable is data-capable (not charge-only)
2. Install ESP32 USB driver (CP210x or CH340 driver)
3. Manually select port from dropdown

### Issue: "PCA9685 not found" or I2C errors

**Solution:**
1. Verify wiring: SDA=GPIO21, SCL=GPIO22, GND, VCC
2. Check PCA9685 is powered (LED should be lit)
3. Verify I2C address is 0x40 (no jumpers soldered)
4. Run I2C scanner:

```cpp
// Add to setup() temporarily
Wire.begin(21, 22);
for(byte addr = 1; addr < 127; addr++) {
  Wire.beginTransmission(addr);
  byte error = Wire.endTransmission();
  if (error == 0) {
    Serial.print("Device at 0x");
    Serial.println(addr, HEX);
  }
}
```

### Issue: Servos not moving

**Solution:**
1. Check external power supply (5-6V, sufficient current)
2. Verify V+ and GND connected to PCA9685
3. Ensure OE pin connected to GND
4. Check common ground between ESP32, PCA9685, and power supply
5. Test with multimeter: V+ terminal should show 5-6V

### Issue: Servos jittering

**Solution:**
1. Increase power supply capacity (use 4A instead of 3A)
2. Add electrolytic capacitor (470-1000µF) across V+ and GND
3. Shorten wires between PCA9685 and servos
4. Check for loose connections

### Issue: "Communication error" or timeouts

**Solution:**
1. Verify CBOR libraries installed correctly
2. Check baud rate: 115200 on both sides
3. Restart ESP32 (press reset button)
4. Restart desktop application
5. Try different USB cable or port

### Issue: Python ImportError for cbor2

**Solution:**
```bash
pip install --upgrade cbor2
```

If still failing:
```bash
pip uninstall cbor2
pip install cbor2
```

## Performance Notes

### CBOR vs JSON Comparison

**Message Size Reduction:**
- Set joints command: ~60% smaller (JSON: ~85 bytes, CBOR: ~35 bytes)
- Telemetry: ~55% smaller (JSON: ~95 bytes, CBOR: ~43 bytes)
- Ping response: ~50% smaller

**Benefits:**
- Less serial bandwidth usage
- Faster parsing on ESP32
- More headroom for future features
- Reduced bus congestion

### PCA9685 Benefits

- Dedicated PWM hardware (smoother signals)
- Frees 6 ESP32 GPIO pins
- Scalable to 16 servos per board
- Can daisy-chain multiple boards

## Next Steps

1. **Mechanical Assembly:** Mount servos to robot arm frame
2. **Calibration:** Fine-tune servo ranges for your specific hardware
3. **Testing:** Verify all joints move through full range safely
4. **Phase 2 Planning:** Prepare for camera integration and AI control

## Support

For issues or questions:
1. Check troubleshooting section above
2. Review `WIRING_SCHEMATIC.md` for connection details
3. Review `servo_controler.md` for PCA9685 specifics
4. Check project README.md

## Safety Warnings

⚠️ **Important Safety Notes:**

1. **Power Supply:** Never exceed 6V on servo power (V+)
2. **Current Rating:** Ensure power supply can handle total servo current
3. **Common Ground:** ALWAYS connect all grounds together
4. **Hot Plugging:** Do not connect/disconnect servos or I2C while powered
5. **Movement Range:** Test servo limits before full assembly
6. **Emergency Stop:** Keep emergency stop button accessible during testing

## File Reference

- `WIRING_SCHEMATIC.md` - Complete wiring diagram
- `servo_controler.md` - PCA9685 detailed guide
- `desktop_app/main.py` - Python control application
- `desktop_app/arm_controller/arm_controller.ino` - ESP32 firmware
- `desktop_app/requirements.txt` - Python dependencies
- `.clinerules/Project_overview.md` - Project architecture
