# Hardware Architecture & Wiring Schematics

This document provides complete connection tables, pinout assignments, power supply architecture, and mechanical specifications for the **ADRC Flexible Beam Vibration Suppression System**.

---

## 1. Electrical Pinout & Wiring Table

| Component | Pin / Signal | ESP32 GPIO | Description / Operating Function |
| :--- | :--- | :--- | :--- |
| **MPU-6050 IMU** | `VCC` | `3V3` or `5V` | System logic power |
| | `GND` | `GND` | Common ground reference |
| | `SDA` | `GPIO 21` | I2C Serial Data (400 kHz Fast-Mode) |
| | `SCL` | `GPIO 22` | I2C Serial Clock (400 kHz Fast-Mode) |
| | `AD0` | `GND` | I2C slave address selector (`0x68`) |
| **L298N H-Bridge** | `IN1` | `GPIO 25` | Direction control & PWM voltage modulation |
| | `IN2` | `GPIO 26` | Direction ground reference (held `LOW` for forward) |
| | `ENA` | Jumper **ON** | Jumper installed (5V pull-up); IN1 handles PWM |
| | `GND` | `GND` | Common ground connected to ESP32 ground |
| | `+12V` | Battery `+` | High-current DC power input from 3S Li-ion pack |
| **12V DC Motor** | `OUT1` | Motor Term 1 | Positive armature terminal |
| | `OUT2` | Motor Term 2 | Ground armature terminal |
| **3S Li-ion Battery** | Positive | L298N `+12V` | $11.1\text{ V}$ nominal, $12.6\text{ V}$ peak |
| | Negative | L298N `GND` | Common ground tie point |

> **IMPORTANT:** Ensure that the ground of the 3S battery pack, the L298N motor driver `GND`, and the ESP32 development board `GND` are all tied together to establish a shared reference potential.

---

## 2. Power Architecture

```
[ 3S 18650 Li-ion Battery Pack (11.1V - 12.6V) ]
         │
         ├─── (+) ───► L298N Power In (+12V) ───► 12V Brushed DC Motor
         │
         └─── (-) ───► L298N GND (Common Ground)
                           │
                           ├───► ESP32 GND
                           └───► MPU-6050 GND

[ USB 5V (PC/Power Bank) ] ───► ESP32 Dev Board (Micro-USB/Type-C)
                                   │
                                   └─── 3.3V Out ───► MPU-6050 VCC
```

---

## 3. Mechanical Assembly & Physical Specifications

* **Cantilever Beam**: Commercial 30 cm polystyrene rule (Nataraj 621 scale):
  * Length: $L = 300\text{ mm}$
  * Width: $b = 28\text{ mm}$
  * Thickness: $h = 1.2\text{ mm}$
  * Flexural Plane: Horizontal rotation plane
* **Root Mounting Hub**: Circular high-impact plastic toy wheel press-fitted onto the motor output shaft, providing a wide, rigid root clamping boundary condition.
* **Actuator**: High-torque 12V brushed DC motor with internal planetary gearbox ($320\text{ mm}$ elevation from base to eliminate ground proximity aerodynamic damping).
* **Sensor Placement**: InvenSense MPU-6050 MEMS IMU fixed securely at the distal tip ($x = L$) with transverse sensing axis aligned perpendicular to the beam's neutral axis.
