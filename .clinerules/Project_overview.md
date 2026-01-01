# AI Physical Desk Assistant – Phase 1 Scope & Architecture

> **Purpose**: This document defines the full Phase 1 scope, architecture, and behavioral expectations for an AI-adjacent physical desk assistant built around a 6-axis servo robot arm controlled by an ESP32.
> This file is intended to be used as **context input for downstream AI coding tasks**.

---

## 1. Project Overview

This project implements a **personality-driven physical robot arm** that lives on an engineer’s desk.
In Phase 1, the system is **not AI-controlled** and **does not use vision**. Instead, it focuses on:

* Robust motion control
* Smooth, expressive movement
* Safety and joint limits
* Idle animations ("alive" behavior)
* Manual control via a desktop app
* A clean communication protocol between PC and MCU

AI control and camera-based perception are explicitly deferred to Phase 2.

---

## 2. System Architecture

### 2.1 High-Level Block Diagram

```
[ Desktop Control App (Python) ]
        │  USB (UART)
        ▼
[ ESP32 Motion Controller ]
        │  PWM / Servo Driver
        ▼
[ 6-Axis Servo Robot Arm ]
```

---

## 3. Hardware Platform

### 3.1 Microcontroller

* **Primary MCU**: ESP32 (standard dev board or Adafruit Feather ESP32)
* Programmed using **Arduino IDE** (ESP32 core)
* Dual-core FreeRTOS available but not required for Phase 1

### 3.2 Actuators

* 6 standard hobby servos (base, shoulder, elbow, wrist pitch, wrist roll, gripper)
* External servo power rail (shared ground with ESP32)

### 3.3 Optional Phase 1 Outputs

* Status LEDs (mood indication)
* Laser pointer (on/off only, no targeting yet)

---

## 4. Software Responsibilities

### 4.1 Desktop Application (Python)

The desktop app acts as the **operator interface and high-level command source**.

Responsibilities:

* Manual joint control via sliders
* Pose definition (named poses)
* Triggering idle animations
* Sending structured commands to the ESP32
* Receiving telemetry (current joint angles, state)

The desktop app folder will also **store the ESP32 `.ino` firmware file** for version coherence.

### 4.2 ESP32 Firmware

The ESP32 is the **motion authority**.

Responsibilities:

* Enforcing joint limits
* Motion smoothing and interpolation
* Idle animation playback
* Rejecting unsafe or malformed commands
* Maintaining current arm state

The ESP32 does **not**:

* Interpret natural language
* Perform vision
* Execute high-level planning

---

## 5. Communication Protocol (USB UART)

### 5.1 Design Goals

* Human-readable
* Easy to debug
* Deterministic
* Extensible

### 5.2 Message Format (JSON over UART)

All commands are JSON objects terminated with a newline.

#### Example: Set Joint Angles

```json
{
  "cmd": "set_joints",
  "targets": [90, 45, 120, 90, 0, 30],
  "speed": 0.5
}
```

#### Example: Play Idle Animation

```json
{
  "cmd": "play_idle",
  "name": "breathing"
}
```

#### Example: Emergency Stop

```json
{
  "cmd": "estop"
}
```

### 5.3 Telemetry from ESP32

```json
{
  "state": "idle",
  "joints": [88, 46, 119, 91, 1, 29]
}
```

---

## 6. Motion Model & Smoothing

### 6.1 Core Principle

The ESP32 never jumps directly to target angles.

All motion is:

* Interpolated
* Rate-limited
* Bounded by joint constraints

### 6.2 Interpolation

* Linear interpolation per joint
* Fixed update loop (e.g., 50–100 Hz)
* Per-joint max degrees/sec

### 6.3 Speed Scaling

A global `speed` parameter (0.0–1.0) scales:

* Max velocity
* Acceleration ramp

---

## 7. Joint Limits & Safety

### 7.1 Static Joint Limits

Each joint has hard-coded bounds:

| Joint       | Min (deg) | Max (deg) |
| ----------- | --------- | --------- |
| Base        | 0         | 180       |
| Shoulder    | 15        | 165       |
| Elbow       | 0         | 180       |
| Wrist Pitch | 30        | 150       |
| Wrist Roll  | 0         | 180       |
| Gripper     | 10        | 90        |

Commands exceeding limits are **clamped or rejected**.

### 7.2 Safety Behaviors

* Emergency stop command halts motion immediately
* Servo outputs disabled if communication times out

---

## 8. Idle Animations (Manual, Predefined)

Idle animations are **hand-authored sequences** that loop when the arm is not under manual control.

### 8.1 Animation Representation

Each animation consists of:

* Named poses
* Transition durations
* Loop behavior

Example internal representation:

```
Breathing:
  Pose A → Pose B → Pose A
  Duration: 3s per transition
```

### 8.2 Initial Idle Animations

#### 1. Breathing

* Slow shoulder + elbow oscillation
* Very small amplitude

#### 2. Curious Tilt

* Head/wrist tilt left/right
* Occasional pause

#### 3. Micro-Adjust

* Small base rotation corrections
* Mimics mechanical settling

#### 4. Idle Reset

* Returns to neutral pose after long inactivity

Idle animations are selected randomly or explicitly via command.

---

## 9. Manual Control Mode

### 9.1 Desktop UI

* Six sliders (one per joint)
* Live feedback of current angles
* Speed control slider
* Buttons:

  * Enable/Disable Idle
  * Play Specific Idle
  * Emergency Stop

### 9.2 Control Rules

* Manual input overrides idle animations
* Idle resumes after configurable timeout

---

## 10. State Machine (ESP32)

```
BOOT
 ↓
IDLE ↔ MANUAL_CONTROL
 ↓
ESTOP
```

* Idle animations only play in IDLE
* MANUAL_CONTROL blocks idle playback
* ESTOP overrides all states

---

## 11. Explicitly Out of Scope (Phase 2)

* Camera integration
* Vision processing
* AI / natural language control
* Inverse kinematics
* Object interaction

---

## 12. Design Philosophy

* Expressive > Precise
* Predictable > Clever
* Physical safety over software elegance
* Motion is personality

This system should feel **alive**, not optimized.

---

## 13. Future Expansion Hooks

* Camera → PC vision
* Laser pointer targeting
* Mood-driven motion parameters
* AI action selection

(These must not require re-architecting Phase 1.)
