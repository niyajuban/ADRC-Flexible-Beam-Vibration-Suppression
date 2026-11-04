# Real-Time Vibration Suppression of a Flexible Beam Using Active Disturbance Rejection Control (ADRC)

[![Platform](https://img.shields.io/badge/Platform-ESP32-blue.svg?logo=espressif&logoColor=white)](https://www.espressif.com/)
[![Language](https://img.shields.io/badge/Language-C%2B%2B%20%2F%20Arduino-00979D.svg?logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![Control Framework](https://img.shields.io/badge/Control-ADRC%20%2F%20LESO-brightgreen.svg)]()
[![Hardware Loop](https://img.shields.io/badge/Loop%20Rate-200%20Hz%20(5%20ms)-orange.svg)]()

An embedded control systems implementation of **Linear Active Disturbance Rejection Control (LADRC)** on an **ESP32** microcontroller to achieve real-time vibration suppression and rapid oscillation damping in an underdamped flexible cantilever beam driven by a 12V DC motor.

---

## Video Demonstration

Watch the real-time physical testbed comparison between **Uncontrolled Mode 0** and **Active ADRC Mode 1**:

[Click here to watch the Experimental Video Demonstration (Google Drive)](https://drive.google.com/file/d/1C9Qzz4ieF12GfXA0XHlePEaRpACWUpzA/view?usp=sharing)

---

## Hardware Testbed & System Architecture

### Physical Experimental Apparatus
The testbed features an elevated vertical support frame, a 12V brushed DC motor with a circular wheel hub, a 30 cm flexible polystyrene ruler acting as the cantilever link, a tip-mounted MPU-6050 accelerometer, an L298N motor driver, and an isolated 3S Li-ion battery pack.

<p align="center">
  <img src="hardware/hardware_setup.png" alt="Physical Hardware Setup" width="480">
</p>

### ADRC Closed-Loop Control Architecture
Structural elasticity, higher-order modal vibrations, aerodynamic resistance, and motor friction are lumped into an unmeasured **Total Disturbance** state. A 2nd-order **Linear Extended State Observer (LESO)** in the acceleration manifold reconstructs this disturbance in real time to cancel it dynamically at the actuator root.

<p align="center">
  <img src="hardware/adrc_block_diagram.png" alt="ADRC Closed-Loop Block Diagram" width="850">
</p>

---

## Key Contributions & Novelty

1. **Acceleration-Domain LESO Formulation**:
   Standard ADRC observers rely on position feedback, requiring numerical differentiation of noisy accelerometer readings. Here, the LESO is formulated directly within the acceleration manifold, completely avoiding derivative noise amplification, phase distortion, and actuator chatter.
2. **Model-Free Control on Compliant Consumer Components**:
   Rather than requiring expensive laser vibrometers, high-resolution optical encoders, or rigid machined alloys, this framework achieves fast oscillation damping on everyday compliant materials (polystyrene ruler on a plastic wheel hub).
3. **Deterministic 200 Hz Discrete Loop on ESP32**:
   Forward Euler state integration, high-speed 400 kHz I2C sensor acquisition, dynamic baseline tare filtering, and PWM actuation execute deterministically every 5 ms on low-cost silicon without external DSP hardware.

---

## Hardware Components & Bill of Materials (BOM)

| Component | Part / Model | Purpose / Specification |
| :--- | :--- | :--- |
| **Microcontroller** | ESP32-WROOM-32 DevKit | Dual-Core 240 MHz, 400 kHz I2C master, PWM generator |
| **Inertial Sensor** | InvenSense MPU-6050 | 6-DOF IMU, 16-bit ADC, +/- 2g transverse range (16,384 LSB/g) |
| **Motor Driver** | L298N Dual H-Bridge | Bidirectional PWM motor driver, ENA hardware jumper ON |
| **Actuator** | 12V Brushed DC Motor | Planetary geared motor with circular plastic wheel hub |
| **Flexible Link** | 30 cm Polystyrene Scale | Dimensions: 300 x 28 x 1.2 mm (Nataraj 621) |
| **Power Supply** | 3S 18650 Li-ion Battery | 11.1V nominal, 12.6V peak, high-current motor supply |
| **Base & Frame** | Elevated Cardboard Pillar | 320 mm height, ground-isolated foundation |

---

## Pinout & Wiring Table

| Component Pin | ESP32 GPIO | Function |
| :--- | :--- | :--- |
| **MPU-6050 SDA** | `GPIO 21` | I2C Serial Data (Fast-Mode 400 kHz) |
| **MPU-6050 SCL** | `GPIO 22` | I2C Serial Clock (Fast-Mode 400 kHz) |
| **MPU-6050 VCC / GND** | `3V3` / `GND` | Logic power and common ground |
| **L298N IN1** | `GPIO 25` | Forward PWM drive & counter-torque modulation |
| **L298N IN2** | `GPIO 26` | Direction ground (held `LOW` for forward) |
| **L298N ENA** | Jumper **ON** | Hardware enable tied HIGH (5V) |
| **L298N +12V / GND** | 3S Battery `+` / `GND` | Motor power & common ground |

---

## Getting Started

### Prerequisites
* [Arduino IDE](https://www.arduino.cc/en/software) or [VS Code + PlatformIO](https://platformio.org/)
* ESP32 Board Package installed via Arduino Board Manager:
  ```
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
  ```

### Flashing the Firmware
1. Open [`firmware/esp32_adrc_vibration/esp32_adrc_vibration.ino`](firmware/esp32_adrc_vibration/esp32_adrc_vibration.ino) in Arduino IDE.
2. Select your board: **Tools > Board > ESP32 Arduino > ESP32 Dev Module**.
3. Select your serial port under **Tools > Port**.
4. Click **Upload**.

### Operating the System
1. Open the **Serial Monitor** or **Serial Plotter** at `115200` baud.
2. The firmware automatically runs a 40-sample baseline gravity tare calibration.
3. Send **`0`** via serial: Executes an **Uncontrolled Baseline Step** (observe large whipping & sustained ringing).
4. Send **`1`** via serial: Executes an **ADRC Active Step** (observe rapid damping and smooth settling).

---

## Experimental Results

* **Structural Identification (Pluck Test)**:
  * Natural Frequency: fn approx 7.0 Hz (omega_n approx 44.0 rad/s)
  * Material Damping Ratio: zeta approx 0.05 (highly underdamped)
* **Mode 0 (Uncontrolled Baseline)**:
  * Abrupt motor start-stop triggers violent initial whipping followed by residual harmonic oscillations persisting for > 1.2 seconds.
* **Mode 1 (LADRC Active)**:
  * The LESO tracks the dynamic deflection forces within milliseconds and injects active counter-torque at the wheel hub, substantially reducing the initial whip amplitude and rapidly extinguishing residual oscillations.
