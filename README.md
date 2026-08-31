# DC Motor System Identification and Closed-Loop Digital Control

## Overview
This repository contains the firmware, data acquisition scripts, and controller design files for a complete closed-loop digital speed control system for a DC Motor. **This project was developed across two dedicated lab sessions as part of the broader Advanced Mechatronics course curriculum.** Implemented on an Arduino Uno R3, the project bridges theoretical continuous-time control systems with practical, real-time discrete implementation. 

The project was executed in two primary phases:
1.  **Phase 1: Open-Loop Analysis & System Identification:** Developing robust hardware interrupt-driven encoder interfaces, measuring velocity, analyzing PWM nonlinearities, and extracting a first-order transfer function from experimental step-response data.
2.  **Phase 2: Closed-Loop Digital Control:** Designing and simulating two-stage lead-lag compensators using Root Locus techniques, discretizing them via the Tustin method, and deploying them in a real-time 200 Hz control loop.

## Hardware Configuration
*   **Microcontroller:** Arduino Uno R3 (ATmega328P, 16 MHz)
*   **Actuator:** Brushed DC motor with integrated gearbox
*   **Driver:** L298N Dual H-Bridge module (Four-quadrant bidirectional control)
*   **Sensor:** Quadrature Optical Encoder (100 PPR, x1 decoding for 3.6° resolution)
*   **Power:** 12V DC Supply

## Key Features & Methodologies

### 1. Robust Encoder Interface & Signal Processing
*   **Hardware Interrupts:** Implemented microsecond-level deterministic event handling for quadrature phase analysis, preventing missed pulses up to theoretical limits of 6000 RPM.
*   **Software Debouncing:** Engineered a 50 µs debounce filter to reject spurious pulses from mechanical contact bounce and electrical noise.
*   **Real-Time Velocity Calculation:** Utilized finite differencing over precise 10ms/100ms intervals (driven by Timer1 hardware interrupts) to compute angular velocity.

### 2. System Identification
*   Logged open-loop step response data (PWM 0 to 200 at 12V) via custom MATLAB serial data acquisition scripts.
*   Modeled the DC motor's electromechanical dynamics to extract the first-order transfer function: `G(s) = 113.2 / (s + 18.05)`
*   Characterized the sublinear PWM-to-RPM relationship (58% speed increase for a 100% duty cycle increase) caused by back-EMF feedback and H-bridge voltage drops.

### 3. Compensator Design & Digital Implementation
*   **Root Locus Design:** Synthesized three distinct controllers to achieve specific natural frequencies (ωn) and damping ratios (ζ).
*   **Two-Stage Cascade Architecture:** Handled large deficiency angles (>60°) by splitting phase contributions across two identical cascaded lead-lag stages, ensuring desired poles remained strictly dominant.
*   **Discretization:** Converted continuous-time controllers to discrete-time difference equations using the Bilinear (Tustin) transformation for execution on the Arduino.

### 4. Nyquist-Shannon Theorem Validation
The project includes a practical demonstration of sampling rate limits. While Controllers 1 and 2 (designed for moderate bandwidths) exhibited excellent step-response tracking at the 200 Hz sampling rate, Controller 3 (designed for ωn = 99 rad/s) intentionally violated the rule-of-thumb sampling margin. This resulted in sustained limit-cycle oscillations, serving as a visceral hardware validation of the Nyquist-Shannon sampling criterion and the destabilizing effects of zero-order hold (ZOH) phase lag.

## Repository Structure
*   `/Arduino`: Firmware files for open-loop data acquisition and closed-loop digital controllers.
*   `/MATLAB`: Scripts for serial data capture, system identification (`tfest`), and automated root locus compensator design.
*   `/Data`: Experimental CSV logs for step responses and steady-state error analysis.

## Authors
*   Roozbeh Amini
*   Sara Karami
*   Iman Habibi
