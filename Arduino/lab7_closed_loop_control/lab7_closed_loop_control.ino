/*
 * Closed-Loop DC Motor Speed Control
 * Mechatronics Lab, Session 7 - University of Tehran
 *
 * Runs the discrete controller u[k] = b0*e[k] + b1*e[k-1] + b2*e[k-2]
 *                                     - a1*u[k-1] - a2*u[k-2]
 * in a 200 Hz Timer1 ISR, drives the motor in both directions through the
 * L298N, logs 2 s of speed data and then prints it over serial for
 * Matlab/lab7_closed_loop_logger.m.
 *
 * To switch controllers, uncomment one coefficient set below. The
 * coefficients come from Matlab/lab7_controller_design.m.
 *
 * Requires the TimerOne library (Library Manager: "TimerOne").
 *
 * Wiring:
 *   Encoder A -> D2 (INT0), Encoder B -> D4
 *   L298N ENA -> D5 (PWM), IN1 -> D7, IN2 -> D8, motor supply 12 V
 */

#include <Arduino.h>
#include <TimerOne.h>

// ================= USER SETTINGS =================
const float TARGET_RPM = 800.0;  // Speed setpoint

// Controller coefficients (paste from MATLAB). Uncomment ONE set.
// These are the values used in the lab; lab7_controller_design.m
// reproduces them to within 0.03%.

// Controller 1: moderate damping (zeta = 0.4, wn = 33)
float b0 = 0.278213, b1 = -0.480337, b2 = 0.207326;
float a1 = -1.986843, a2 = 0.986887;

// Controller 2: high damping (zeta = 0.75, wn = 42) - recommended
// float b0 = 0.584569, b1 = -1.032467, b2 = 0.455886;
// float a1 = -1.968746, a2 = 0.968990;

// Controller 3: high speed (zeta = 0.24, wn = 99) - unstable at 200 Hz
// float b0 = 0.942001, b1 = -1.249796, b2 = 0.414541;
// float a1 = -1.976380, a2 = 0.976520;

// ================= PIN DEFINITIONS =================
const uint8_t ENC_A = 2;  // Encoder channel A (INT0)
const uint8_t ENC_B = 4;  // Encoder channel B
const uint8_t ENA = 5;    // H-bridge enable (PWM, Timer0)
const uint8_t IN1 = 7;    // H-bridge direction 1
const uint8_t IN2 = 8;    // H-bridge direction 2

// ================= SYSTEM CONSTANTS =================
const float TICKS_PER_REV = 100.0f;         // Encoder resolution
const uint16_t FS_HZ = 200;                 // Sampling frequency (Hz)
const uint32_t Ts_us = 1000000UL / FS_HZ;   // Sampling period (us)
const uint16_t N_SAMPLES = 400;             // Logged samples (2 s)

// ================= STATE VARIABLES =================
// Controller state
volatile float currentRPM = 0.0;
volatile float e_k1 = 0, e_k2 = 0;  // Error history
volatile float u_k1 = 0, u_k2 = 0;  // Control output history

// Encoder state
volatile long ticks = 0;
volatile long ticks_prev = 0;

// Data logging
volatile float logActual[N_SAMPLES];
volatile uint16_t sampleIdx = 0;
volatile bool captureDone = false;

// ================= INTERRUPT SERVICE ROUTINES =================

// Encoder ISR: rising edge of channel A, channel B gives the direction
void isrEncA() {
  if (digitalRead(ENC_B)) {
    ticks++;  // Clockwise (CW)
  } else {
    ticks--;  // Counter-clockwise (CCW)
  }
}

// Control ISR: Timer1 at 200 Hz, runs the difference equation
void controlISR() {
  if (captureDone) return;

  // 1. Speed from the tick change over one sample period
  long t = ticks;
  long dt = t - ticks_prev;
  ticks_prev = t;
  currentRPM = ((float)dt / TICKS_PER_REV) * (60.0f * (float)FS_HZ);

  // 2. Error
  float e_k = TARGET_RPM - currentRPM;

  // 3. Control law
  // u[k] = b0*e[k] + b1*e[k-1] + b2*e[k-2] - a1*u[k-1] - a2*u[k-2]
  float u_k = (b0 * e_k) + (b1 * e_k1) + (b2 * e_k2)
              - (a1 * u_k1) - (a2 * u_k2);

  // 4. Saturation: clamp to the PWM range. The clamped value is what
  //    goes into the history, which keeps the controller from winding up.
  if (u_k > 255.0f) u_k = 255.0f;
  if (u_k < -255.0f) u_k = -255.0f;

  // 5. Shift the history for the next sample
  e_k2 = e_k1;
  e_k1 = e_k;
  u_k2 = u_k1;
  u_k1 = u_k;

  // 6. Signed effort -> direction + PWM magnitude
  int pwmOut = (int)u_k;
  if (pwmOut >= 0) {
    // Positive effort: forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    // Negative effort: reverse (braking)
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    pwmOut = -pwmOut;
  }
  analogWrite(ENA, pwmOut);

  // 7. Logging
  if (sampleIdx < N_SAMPLES) {
    logActual[sampleIdx] = currentRPM;
    sampleIdx++;
  } else {
    // Experiment complete: stop the motor
    captureDone = true;
    analogWrite(ENA, 0);
  }
}

// ================= MAIN PROGRAM =================

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;  // Wait for the serial port
  }

  // Encoder pins with pull-ups
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEncA, RISING);

  // Motor driver pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Motor stopped
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  // Startup information. F() keeps these strings in flash, which leaves
  // more of the Uno's 2 KB of RAM for the 400-sample log.
  Serial.println(F("\n========================================"));
  Serial.println(F("Mechatronics Lab 7: Closed-Loop Control"));
  Serial.println(F("========================================"));
  Serial.print(F("Target RPM: "));
  Serial.println(TARGET_RPM);
  Serial.print(F("Sampling Rate: "));
  Serial.print(FS_HZ);
  Serial.println(F(" Hz"));
  Serial.print(F("Duration: "));
  Serial.print((float)N_SAMPLES / FS_HZ);
  Serial.println(F(" seconds"));
  Serial.println(F("========================================\n"));
  Serial.println(F("Starting experiment in 2 seconds..."));
  delay(2000);

  // Timer1 drives the control loop
  Timer1.initialize(Ts_us);
  Timer1.attachInterrupt(controlISR);

  Serial.println(F("Controller active - collecting data...\n"));
}

void loop() {
  if (captureDone) {
    // Stop the control loop
    Timer1.detachInterrupt();

    // Send the logged data
    Serial.println(F("=== EXPERIMENTAL DATA START ==="));
    for (uint16_t i = 0; i < N_SAMPLES; i++) {
      Serial.println(logActual[i], 2);
    }
    Serial.println(F("=== EXPERIMENTAL DATA END ==="));

    // Steady-state statistics from the last 20 samples
    float sum = 0;
    for (uint16_t i = N_SAMPLES - 20; i < N_SAMPLES; i++) {
      sum += logActual[i];
    }
    float finalRPM = sum / 20.0;
    float error = ((TARGET_RPM - finalRPM) / TARGET_RPM) * 100.0;

    Serial.println(F("\n=== PERFORMANCE METRICS ==="));
    Serial.print(F("Target RPM: "));
    Serial.println(TARGET_RPM);
    Serial.print(F("Final RPM (avg last 20): "));
    Serial.println(finalRPM, 2);
    Serial.print(F("Steady-State Error: "));
    Serial.print(error, 2);
    Serial.println(F(" %"));
    Serial.println(F("===========================\n"));

    Serial.println(F("Experiment complete. Reset Arduino to run again."));

    // Halt
    while (1) {
      ;
    }
  }
}
