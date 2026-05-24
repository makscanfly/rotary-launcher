# Rotary Launcher
**An integrated embedded system for precise projectile launching with BLE control.**

## Video
> [!IMPORTANT]

---

## Project Overview
The **Rotary Launcher** is a "Central-Peripheral" system designed to calculate and execute precise projectile launches. The system uses a rotating arm driven by a DC motor, with an electromagnet release mechanism.

The project bridges the gap between high-level trajectory planning in **Python** and low-level, real-time execution in **C++** on a specialized microcontroller.

### How it works:
1. **Host App:** A user enters target coordinates (x, y) on a laptop.
2. **Trajectory Planning:** The Python-based host calculates the necessary angular velocity and the exact release angle using ballistic models.
3. **BLE Communication:** Parameters are transmitted wirelessly to the launcher.
4. **Real-time Execution:** The launcher accelerates to the target RPM, uses an optical sensor for feedback, and triggers the electromagnet at the precise microsecond to hit the target.

---

## Hardware Stack
- **Microcontroller:** `ESP32-S3-WROOM-1-N8R8`
- **Actuators:** 
  - High-speed DC Motor (PWM controlled)
  - Electromagnet (MOSFET switched)
- **Sensors:** 
  - Optical slot sensor for RPM feedback (Hardware Interrupts)
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
- `/firmware/main_launcher` – **Final Production Firmware:** FSM-based logic for launch execution.
- `/firmware/calibration` – **Calibration Firmware:** Tools for motor system identification.
- `/host_app` – **Python Tools:** Includes the CLI controller and trajectory planning modules.
- `/docs` – Logs, system characteristics, and visual assets.

---

## Key Challenges Solved
- **System Identification:** Conducted a full characterization of the DC motor to map PWM duty cycles to actual angular velocity.
- **Timing Synchronization:** Achieved microsecond-level precision between the optical sensor feedback and the electromagnet de-energization.
- **BLE Data Handling:** Managed GATT MTU limits and ensured reliable command transmission in an electrically noisy environment.

---
*Developed as a project for Embedded Systems and Robotics studies.*
