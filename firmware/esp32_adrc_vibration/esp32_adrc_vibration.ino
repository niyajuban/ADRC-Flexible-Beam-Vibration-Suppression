/*
 * ======================================================================================
 * Project: Real-Time Vibration Suppression of a Flexible Beam Using ADRC
 * Authors: Niya Rachel Juban, Swathi C P, Aparajita Yarramsetti, Disha Devarmani
 * Institution: Vellore Institute of Technology (VIT), Vellore, India
 * Platform: ESP32 Microcontroller
 * Sensors / Drivers: MPU-6050 (6-DOF IMU), L298N Dual H-Bridge Driver, 12V Brushed DC Motor
 * ======================================================================================
 * 
 * Description:
 * Implements a discrete Linear Active Disturbance Rejection Control (LADRC) pipeline
 * operating at a fixed 200 Hz (Ts = 5 ms) rate to actively damp elastodynamic residual
 * oscillations and whipping in a flexible cantilever beam during rapid point-to-point slewing.
 * 
 * Key Features:
 * - Second-order Linear Extended State Observer (LESO) formulated directly in the
 *   acceleration manifold to prevent derivative noise peaking.
 * - Dynamic baseline tare calibration to zero-out static gravitational acceleration & sag.
 * - Real-time total disturbance estimation (lumping beam elasticity, higher-order flexure,
 *   aerodynamic drag, and motor friction into state z2).
 * - Disturbance rejection law superimposing counter-torque onto nominal step motion.
 */

#include <Arduino.h>
#include <Wire.h>

// ======================================================================================
// 1. PIN DEFINITIONS & HARDWARE CONFIGURATION
// ======================================================================================
// Motor Driver Pins (L298N with ENA hardware jumper tied HIGH)
const int PIN_IN1 = 25; // Direction & PWM control (PWM output)
const int PIN_IN2 = 26; // Direction ground (LOW for forward rotation)

// I2C Pins for MPU-6050 (Default ESP32: SDA=21, SCL=22)
const int PIN_SDA = 21;
const int PIN_SCL = 22;
const int MPU_ADDR = 0x68; // Standard MPU-6050 I2C address (AD0 to GND)

// ======================================================================================
// 2. MOTOR ACTUATION & EXPERIMENTAL PARAMETERS
// ======================================================================================
const int TARGET_PWM           = 220; // Nominal slewing speed (8-bit PWM: 0 - 255)
const unsigned long STEP_DURATION_MS = 350; // Slew duration in milliseconds
const float Ts                 = 0.005; // Discrete sampling interval: 5 ms (200 Hz)

// ======================================================================================
// 3. ADRC (LESO) PARAMETERS IN ACCELERATION DOMAIN
// ======================================================================================
// Linear Extended State Observer (LESO) states:
//   z1: Estimated dynamic vibration acceleration (tracks sensor y[k])
//   z2: Estimated Total Disturbance f[k] (flexural forces, drag, friction)
float z1 = 0.0;
float z2 = 0.0;

// Observer Bandwidth Tuning (Gao's parameterization):
//   Observer poles placed at -omega_o:
//   beta1 = 2 * omega_o = 24.0
//   beta2 = omega_o^2    = 144.0 (for omega_o = 12.0 rad/s)
const float omega_o = 12.0;
const float beta1   = 2.0 * omega_o;       // 24.0
const float beta2   = omega_o * omega_o;   // 144.0

// Controller parameters:
//   k_p = omega_c (proportional gain on vibration error)
//   Disturbance compensation gain = 1 / b0
const float kp_vib  = 50.0; // Scaled proportional vibration damping gain
const float kd_dist = 3.0;  // Disturbance compensation scaling factor

// Runtime calibration offset
float baseline_accel = 0.0;

// ======================================================================================
// 4. LOW-LEVEL MOTOR CONTROL FUNCTIONS
// ======================================================================================
void setMotorForward(int pwm) {
  digitalWrite(PIN_IN2, LOW);
  analogWrite(PIN_IN1, constrain(pwm, 0, 255));
}

void stopMotor() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
}

// ======================================================================================
// 5. SENSOR ACQUISITION & ACCELERATION TRANSDUCTION
// ======================================================================================
float readVibration() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Accel X High register
  if (Wire.endTransmission(false) != 0) {
    return 0.0; // Prevent bus lockup if disconnected
  }

  Wire.requestFrom(MPU_ADDR, 6, true);
  if (Wire.available() < 6) {
    return 0.0;
  }

  // Skip X-axis
  Wire.read(); Wire.read();
  
  // Read transverse Y-axis acceleration (direction of beam flexure)
  int16_t ry = Wire.read() << 8 | Wire.read();
  
  // Skip Z-axis
  Wire.read(); Wire.read();

  // Convert raw 16-bit counts (±2g range: 16,384 LSB/g) to physical acceleration (g)
  float raw_g = (float)ry / 16384.0;

  // Subtract static gravitational baseline tare to isolate alternating dynamic vibration
  return (raw_g - baseline_accel);
}

// ======================================================================================
// 6. EXPERIMENTAL TEST SEQUENCE (MODE 0 vs. MODE 1)
// ======================================================================================
void runTest(int mode) {
  // Pre-test stabilization
  stopMotor();
  z1 = 0.0;
  z2 = 0.0;
  delay(400);

  Serial.println(mode == 0 ? ">>> EXECUTING MODE 0: UNCONTROLLED BASELINE STEP"
                           : ">>> EXECUTING MODE 1: ACTIVE ADRC CONTROLLED STEP");

  unsigned long t_start = micros();
  unsigned long duration_micros = STEP_DURATION_MS * 1000;

  // Real-time closed-loop actuation during the slewing step
  while ((micros() - t_start) < duration_micros) {
    float vib = readVibration();
    int cmd = TARGET_PWM;

    if (mode == 0) {
      // ----------------------------------------------------
      // Mode 0: Uncontrolled Open-Loop Step
      // Motor commanded with constant target voltage
      // ----------------------------------------------------
      cmd = TARGET_PWM;
    } 
    else if (mode == 1) {
      // ----------------------------------------------------
      // Mode 1: Active Disturbance Rejection Control (ADRC)
      // ----------------------------------------------------
      // Step A: Calculate observer innovation tracking error
      float e = z1 - vib;

      // Step B: Update Linear Extended State Observer (Forward Euler)
      z1 += Ts * (z2 - beta1 * e);
      z2 += Ts * (-beta2 * e);

      // Step C: Disturbance Rejection Law (Counter-Torque Calculation)
      int u_adrc = constrain((int)(-kp_vib * z1 - kd_dist * z2), -35, 35);

      // Step D: Superimpose corrective compensation onto base actuator command
      // Safe saturation clamping ensures motor never stalls (maintains forward motion)
      cmd = constrain(TARGET_PWM + u_adrc, 180, 255);
    }

    // Apply voltage to motor terminals via L298N H-bridge
    setMotorForward(cmd);

    // Stream telemetry to Arduino Serial Plotter / Monitor (format: "Vibration:<val>")
    Serial.print("Vibration:");
    Serial.println(vib, 4);

    delay(5); // Deterministic 5 ms loop interval (200 Hz)
  }

  // Actuator decels to stop
  stopMotor();

  // Monitor and stream residual ringing oscillations for 1.2 seconds after halt
  unsigned long t_stop = millis();
  while (millis() - t_stop < 1200) {
    Serial.print("Vibration:");
    Serial.println(readVibration(), 4);
    delay(5);
  }

  Serial.println(">>> RUN COMPLETE. READY FOR NEXT COMMAND.");
}

// ======================================================================================
// 7. ARDUINO SETUP ROUTINE
// ======================================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000); // Wait for serial console on USB
  delay(300);

  Serial.println("=============================================================");
  Serial.println(" Real-Time Flexible Beam Vibration Suppression System (ADRC)");
  Serial.println(" Vellore Institute of Technology (VIT), Vellore, India");
  Serial.println("=============================================================");

  // Initialize motor direction pins
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  stopMotor();

  // Initialize I2C bus at 400 kHz Fast-Mode
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  // Wake up MPU-6050 (clear SLEEP bit in PWR_MGMT_1 register 0x6B)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(100);

  // Configure Accelerometer Full Scale Range to ±2g (AFS_SEL=0 in ACCEL_CONFIG 0x1C)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(50);

  // Startup Tare Calibration: Calculate static 1g / tilt gravity baseline
  Serial.print("Calibrating static gravity baseline over 40 samples...");
  float sum = 0.0;
  for (int i = 0; i < 40; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    Wire.read(); Wire.read(); // Skip X
    int16_t ry = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // Skip Z
    sum += (float)ry / 16384.0;
    delay(10);
  }
  baseline_accel = sum / 40.0;
  Serial.println(" DONE!");
  Serial.print("Baseline Gravitational Offset: ");
  Serial.print(baseline_accel, 4);
  Serial.println(" g");

  Serial.println("\n--- SYSTEM ARMED AND READY ---");
  Serial.println("Send '0' via Serial to trigger UNCONTROLLED baseline step.");
  Serial.println("Send '1' via Serial to trigger ADRC ACTIVE vibration suppression.");
}

// ======================================================================================
// 8. MAIN ARDUINO LOOP
// ======================================================================================
void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == '0') {
      runTest(0); // Uncontrolled baseline
    } else if (cmd == '1') {
      runTest(1); // ADRC active control
    }
  }
}
