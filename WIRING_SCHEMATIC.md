# Complete System Wiring Schematic

## Hardware Components
1. **ESP32 Development Board** (any standard ESP32 dev board)
2. **PCA9685 16-Channel PWM Servo Controller**
3. **6x Hobby Servos** (for 6-axis robot arm)
4. **External 5-6V Power Supply** (rated for servo current: recommend 3A minimum)
5. **USB Cable** (for ESP32 programming and communication with PC)

## Wiring Diagram (Text Format)

```
┌─────────────────────────────────────────────────────────────────┐
│                         COMPLETE SYSTEM WIRING                   │
└─────────────────────────────────────────────────────────────────┘

┌──────────────────┐
│                  │
│   PC / Desktop   │
│   Control App    │
│                  │
└────────┬─────────┘
         │
         │ USB Cable
         │ (Power + Data)
         │
         ▼
┌──────────────────────────────────────────┐
│         ESP32 Dev Board                  │
│                                          │
│  ┌────────────────────────────────┐    │
│  │  Pin Layout:                   │    │
│  │                                 │    │
│  │  GPIO21 (SDA) ──────────┐     │    │
│  │  GPIO22 (SCL) ──────────┼─┐   │    │
│  │  GND ───────────────────┼─┼─┐ │    │
│  │  3.3V or 5V ────────────┼─┼─┼─┐    │
│  │  (USB Power)            │ │ │ │    │
│  └─────────────────────────┼─┼─┼─┼───┘
│                            │ │ │ │
└────────────────────────────┼─┼─┼─┼─────
                             │ │ │ │
                             │ │ │ │
        ┌────────────────────┘ │ │ │
        │    ┌─────────────────┘ │ │
        │    │    ┌───────────────┘ │
        │    │    │    ┌─────────────┘
        │    │    │    │
        ▼    ▼    ▼    ▼
┌───────────────────────────────────────────────┐
│          PCA9685 Servo Controller             │
│  ┌─────────────────────────────────────────┐ │
│  │  LEFT SIDE PINS:                        │ │
│  │  • GND ────── (to ESP32 GND) ◄──────────┼─┤ ◄─── Common Ground!
│  │  • OE ─────── (to GND) [tied low]       │ │
│  │  • SCL ────── (to ESP32 GPIO22) ◄───────┼─┤
│  │  • SDA ────── (to ESP32 GPIO21) ◄───────┼─┤
│  │  • VCC ────── (to ESP32 3.3V/5V) ◄──────┼─┤
│  │  • V+ ─────── (to External 5V+) ◄───────┼─┤ ┐
│  │                                          │ │ │ External
│  │  RIGHT SIDE: (mirror of left for        │ │ │ Power Supply
│  │               daisy-chaining)            │ │ │ 5-6V, 3A+
│  │                                          │ │ │
│  │  SERVO CHANNELS (bottom):                │ │ │
│  │                                          │ │ │
│  │  Ch0 [●●●] ── Base Servo                │ │ │
│  │  Ch1 [●●●] ── Shoulder Servo            │ │ │
│  │  Ch2 [●●●] ── Elbow Servo               │ │ │
│  │  Ch3 [●●●] ── Wrist Pitch Servo         │ │ │
│  │  Ch4 [●●●] ── Wrist Roll Servo          │ │ │
│  │  Ch5 [●●●] ── Gripper Servo             │ │ │
│  │  Ch6-15: unused                          │ │ │
│  │                                          │ │ │
│  │  Each [●●●] = [Signal][V+][GND]        │ │ │
│  │               (Yellow)(Red)(Black)       │ │ │
│  └─────────────────────────────────────────┘ │ │
│                                               │ │
│  Address Jumpers: A0-A5 (all open = 0x40)   │ │
└───────────────────────────────────────────────┘ │
                                                  │
        ┌─────────────────────────────────────────┘
        │
        ▼
┌──────────────────────────────┐
│  External Power Supply        │
│  (5-6V DC, 3A minimum)       │
│                               │
│  (+) ──────┐                 │
│            │                 │
│  (-) ──────┼──────┐          │
│            │      │          │
└────────────┼──────┼──────────┘
             │      │
             │      └──────────────┐
             │                     │
             └─────────► V+ ───────┤
                                   │
                        GND ◄──────┘
                         ▲
                         │
                    Common Ground
                    (connects to ESP32 GND)
```

## Detailed Connection Table

| From Device    | From Pin      | To Device     | To Pin        | Wire Type     |
|----------------|---------------|---------------|---------------|---------------|
| ESP32          | GPIO21 (SDA)  | PCA9685       | SDA           | Signal wire   |
| ESP32          | GPIO22 (SCL)  | PCA9685       | SCL           | Signal wire   |
| ESP32          | GND           | PCA9685       | GND           | Ground wire   |
| ESP32          | 3.3V or 5V    | PCA9685       | VCC           | Power wire    |
| PCA9685        | OE            | PCA9685       | GND           | Jumper wire   |
| Power Supply   | (+) 5-6V      | PCA9685       | V+            | Heavy gauge   |
| Power Supply   | (-) GND       | PCA9685       | GND           | Heavy gauge   |
| Power Supply   | (-) GND       | ESP32         | GND           | Common ground |
| Servo (Base)   | Signal (Yel)  | PCA9685       | Ch0 Signal    | Servo cable   |
| Servo (Base)   | Power (Red)   | PCA9685       | Ch0 V+        | Servo cable   |
| Servo (Base)   | Ground (Blk)  | PCA9685       | Ch0 GND       | Servo cable   |
| Servo (Shoulder)| Signal (Yel) | PCA9685       | Ch1 Signal    | Servo cable   |
| Servo (Shoulder)| Power (Red)  | PCA9685       | Ch1 V+        | Servo cable   |
| Servo (Shoulder)| Ground (Blk) | PCA9685       | Ch1 GND       | Servo cable   |
| Servo (Elbow)  | Signal (Yel)  | PCA9685       | Ch2 Signal    | Servo cable   |
| Servo (Elbow)  | Power (Red)   | PCA9685       | Ch2 V+        | Servo cable   |
| Servo (Elbow)  | Ground (Blk)  | PCA9685       | Ch2 GND       | Servo cable   |
| Servo (Wrist P)| Signal (Yel)  | PCA9685       | Ch3 Signal    | Servo cable   |
| Servo (Wrist P)| Power (Red)   | PCA9685       | Ch3 V+        | Servo cable   |
| Servo (Wrist P)| Ground (Blk)  | PCA9685       | Ch3 GND       | Servo cable   |
| Servo (Wrist R)| Signal (Yel)  | PCA9685       | Ch4 Signal    | Servo cable   |
| Servo (Wrist R)| Power (Red)   | PCA9685       | Ch4 V+        | Servo cable   |
| Servo (Wrist R)| Ground (Blk)  | PCA9685       | Ch4 GND       | Servo cable   |
| Servo (Gripper)| Signal (Yel)  | PCA9685       | Ch5 Signal    | Servo cable   |
| Servo (Gripper)| Power (Red)   | PCA9685       | Ch5 V+        | Servo cable   |
| Servo (Gripper)| Ground (Blk)  | PCA9685       | Ch5 GND       | Servo cable   |

## Critical Notes

### IMPORTANT - Common Ground
- ESP32 GND, PCA9685 logic GND, and servo power supply GND **MUST** all be connected together
- This creates a common reference voltage for communication
- Without common ground, I2C communication will fail

### Power Supply Requirements
- Do NOT power servos from USB or ESP32's 3.3V/5V pins
- Each servo can draw 100-500mA under load
- 6 servos = potentially 3A total current
- Use regulated 5-6V supply rated for at least 3A (4A recommended for safety margin)

### OE Pin
- Connect OE (Output Enable) to GND to keep outputs always enabled
- Alternatively, connect to an ESP32 GPIO if you want software control of servo power
- OE is active LOW (LOW=enabled, HIGH=disabled)

### I2C Address
- Default address is 0x40 (with no jumpers closed)
- This is what your code should use
- You can verify with an I2C scanner if needed

## Power-Up Sequence

1. Connect all signal wires (I2C, servos)
2. Connect external power supply to PCA9685 (V+ and GND)
3. Ensure common ground is established
4. Connect USB to ESP32 (powers ESP32 and provides serial communication)
5. Upload firmware to ESP32
6. Run desktop control application

## Testing Procedure

1. **Without servos connected**, verify I2C communication:
   - Upload firmware with I2C scanner code
   - Should detect PCA9685 at address 0x40

2. **Connect one servo to channel 0**:
   - Test with simple sweep code
   - Verify smooth motion

3. **Connect remaining servos one by one**:
   - Test each channel individually
   - Verify no power supply brownouts

4. **Full system test**:
   - All servos connected
   - Run desktop control app
   - Test manual control and idle animations

## Safety Considerations

1. **Power Supply**:
   - Always use a separate power supply for servos (not USB)
   - Calculate total current: Number of servos × peak current per servo
   - Add 20% safety margin to power supply rating

2. **Common Ground**:
   - ALWAYS connect all grounds together
   - ESP32 GND, PCA9685 GND, and servo power GND must be common

3. **Servo Limits**:
   - Don't drive servos beyond mechanical limits
   - Use software limits in firmware
   - Test calibration with one servo before connecting all

4. **Hot Plugging**:
   - Don't connect/disconnect servos while powered
   - Don't connect/disconnect I2C while powered

5. **Overcurrent Protection**:
   - Use fuse or current-limited supply
   - Many servos drawing current simultaneously can trip power supplies
