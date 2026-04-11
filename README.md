# Quadruped Robot Controller

Low-level controller for a 4-legged walking robot, built on ESP32.  
Implements real-time inverse kinematics and gait pipeline using a FreeRTOS dual-core architecture.

> **Status:** In development (Phase 5/9 — Hardware Abstraction Layer & servo calibration)

---

## Architecture

```
[PC / Controller]
      │ UDP (target velocity: vx, vy, wz)
      ▼
┌─────────────────────────────────────────────┐
│              ESP32 (Dual Core)              │
│                                             │
│  Core 0 — CommTask                          │
│    UDP recv → timestamp → SharedData        │
│                                             │
│  Core 1 — ControlTask (100Hz target)        │
│    Snapshot → State Machine                 │
│    → Gait Sequencer → Foot Planner          │
│    → Trajectory Generator → Body Kinematics │
│    → Inverse Kinematics → HardwareOutput    │
│                     │                       │
└─────────────────────┼───────────────────────┘
                      │ I2C (PWM)
                   PCA9685
                      │
              12x Servo Motors
```

**State Machine:** `INIT → IDLE ⇄ TROT` / `ANY → ERROR`

---

## Quick Start

```bash
# 1. Set WiFi credentials
cp secrets.ini.example secrets.ini
# Fill in wifi_ssid and wifi_password in secrets.ini

# 2. Build and flash
pio run -e esp32dev
pio run -e esp32dev -t upload

# 3. Serial monitor
pio device monitor -b 115200
```

### Calibration Mode (servo zero-offset calibration)

```bash
pio run -e esp32dev_calibration -t upload
python test/phase5_calibration.py
```

---

## Tech Stack

| | |
|---|---|
| MCU | ESP32 (Xtensa LX6, Dual Core) |
| Framework | ESP-IDF + FreeRTOS |
| Language | C++17 |
| Build | PlatformIO + CMake |
| Servo Driver | PCA9685 (I2C, 12ch PWM) |
| Communication | UDP over WiFi |
| Math | Eigen 3 (fixed-size only: `Matrix3f`, `Vector3f`) |

---

## Development Phases

| Phase | Description | Status |
|---|---|---|
| 1 | Core types & state machine | ✅ PASS |
| 2 | Core 0 communication (UDP + Watchdog) | ✅ PASS |
| 3 | Core 1 control loop skeleton (50Hz) | ✅ PASS |
| 4 | Gait pipeline integration (IK + Trot) | ✅ PASS |
| 5 | Hardware Abstraction Layer + servo calibration | 🔄 In progress |
| 6 | INIT / TRANSITION / ERROR interpolation | ⏳ |
| 7 | First physical gait test | ⏳ |
| 8 | Parameter tuning & 100Hz upgrade | ⏳ |
| 9 | Modularization (header/cpp split) | ⏳ |

---

## Project Structure

```
├── include/                  # Module headers (interfaces)
│   ├── quadruped_types.h     # RobotParams, RobotState, SharedData
│   ├── StateMachine.h
│   ├── CommTask.h            # Core 0 UDP receiver
│   ├── GaitSequencer.h
│   ├── FootPlanner.h
│   ├── TrajectoryGenerator.h
│   ├── BodyKinematics.h
│   ├── IK.h
│   ├── LocomotionController.h
│   ├── HardwareOutput.h      # PCA9685 output layer
│   └── PCA9685.h
├── src/                      # Main gait pipeline firmware
├── src_calibration/          # Calibration-only firmware
├── test/                     # Python test scripts
│   ├── phase4_test.py
│   └── phase5_calibration.py
└── docs/
    └── API_REFERENCE.md      # Module interface reference
```
