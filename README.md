# Rotary Launcher
**An integrated embedded system for precise projectile launching with BLE control.**

## Video


---

## Project Overview
The **Rotary Launcher** is a "Central-Peripheral" system designed to calculate and execute precise projectile launches. The system uses a rotating arm driven by a DC motor, with an electromagnet release mechanism.

The project bridges the gap between high-level trajectory planning in **Python** and low-level, real-time execution in **C++** on a specialized microcontroller.

### How it works:
1. **Host App:** A user enters target coordinates (x, y) on a laptop.
2. **Trajectory Planning:** The Python-based host calculates the necessary angular velocity and the exact release angle using ballistic models.
3. **BLE Communication:** Parameters are transmitted wirelessly to the launcher.
4. **Execution:** The launcher accelerates to the target RPM, uses an optical sensor for speed measurements, and triggers the electromagnet at the precise microsecond to hit the target.

---

## Key Challenges Solved
- **System Identification:** Conducted a full characterization of the DC motor to map PWM duty cycles to actual angular velocity.
- **Timing Synchronization:** Achieved precise timing between the optical sensor feedback and the projectile release using hardware interrupts.

---

## Hardware Stack
- **Microcontroller:** `ESP32-S3-WROOM-1-N8R8`
- **Actuators:** 
  - DC Motor
  - Electromagnet
- **Sensors:** 
  - Optical slot sensor for RPM measurements
- **Host Device:** Laptop/PC with Bluetooth support

---

## Software & Libraries
### Embedded (ESP32-S3)
The firmware is written in **C++ (Arduino framework)** with a focus on real-time performance:
- **NimBLE-Arduino:** Used for memory-efficient BLE communication.
- **FreeRTOS:** Utilized for critical sections and task management to ensure data integrity during high-frequency interrupts.
- **Hardware Timers & Interrupts:** For microsecond-accurate projectile release.

### Host Application (Python)
The control and calibration tools require the following libraries:
- `bleak` – BLE communication (Central role).
- `numpy` & `scipy` – For complex trajectory calculations and optimization.
- `matplotlib` – Used for data visualization and motor characterization.

---

## Project Structure
```text
.
├── collected_data
│   └── .gitkeep
├── firmware
│   ├── launcher-calibration
│   │   └── remotely_controlled_Launcher.cpp
│   └── launcher-main
│       └── main_launcher.cpp
├── host_app
│   ├── calibration-tools
│   │   ├── BLE_remote_control.py
│   │   └── Launcher_data_collecting.ipynb
│   └── launcher-main
│       ├── BLEconnection.py
│       ├── launcher_app.py
│       └── TrajectoryPlanning.py
├── .gitignore
└── README.md

---
