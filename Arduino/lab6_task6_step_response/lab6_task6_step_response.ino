/*
 * Task 6: Open-Loop Step Response Characterization
 * Mechatronics Lab, Session 6 - University of Tehran
 *
 * Timer1 samples the speed at 100 Hz. After a short wait the motor gets a
 * PWM 0 -> 255 step at 12 V, and every sample (RPM) is sent over serial,
 * one value per line, for Matlab/lab6_step_response_logger.m.
 *
 * Requires the TimerOne library (Library Manager: "TimerOne").
 *
 * Wiring:
 *   Encoder A -> D2 (INT0), Encoder B -> D4, Encoder VCC -> 3.3 V
 *   L298N ENA -> D5 (PWM), IN1 -> D7, IN2 -> D8, motor supply 12 V
 */

#include <TimerOne.h>

#define ENCA 2
#define ENCB 4
#define ENA 5
#define IN1 7
#define IN2 8

volatile long ticks = 0;
volatile long ticksPrev = 0;
volatile uint32_t lastISRmicros = 0;
const uint32_t minEdgeUs = 50;  // Debounce threshold (us)

volatile float rpmLatest = 0.0f;
volatile bool sampleReady = false;

const int ppr = 100;                  // Pulses per revolution
const float samplingPeriodMs = 10.0;  // 100 Hz

void isrEncA() {
  uint32_t now = micros();
  if (now - lastISRmicros < minEdgeUs) return;
  lastISRmicros = now;

  bool b = digitalRead(ENCB);
  ticks += (b ? -1 : 1);
}

// Timer1 ISR: runs every 10 ms (100 Hz)
void sampleISR() {
  // Velocity from the tick change over one sample period
  long dticks = ticks - ticksPrev;
  ticksPrev = ticks;

  float countsPerRev = (float)ppr;
  float timeInSec = samplingPeriodMs / 1000.0f;
  float revPerSec = dticks / countsPerRev / timeInSec;
  rpmLatest = revPerSec * 60.0f;

  // Tell the main loop a new sample is ready
  sampleReady = true;
}

void setup() {
  Serial.begin(115200);

  // Give MATLAB time to connect
  delay(2000);

  // Encoder
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), isrEncA, RISING);

  // Motor driver
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Start with the motor coasting
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  // 100 Hz sampling
  Timer1.initialize(10000);  // 10000 us = 10 ms
  Timer1.attachInterrupt(sampleISR);

  delay(1000);

  // Step input: PWM 0 -> 255 (full 12 V)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 255);
}

void loop() {
  if (sampleReady) {
    // Copy the sample with interrupts off
    noInterrupts();
    float rpm = rpmLatest;
    sampleReady = false;
    interrupts();

    // One value per line for MATLAB
    Serial.println(rpm, 4);
  }
}
