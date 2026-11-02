/*
 * Task 3: Velocity Measurement via Finite Differencing
 * Mechatronics Lab, Session 6 - University of Tehran
 *
 * Every 100 ms the sketch takes the change in tick count and converts it
 * to RPM and rad/s.
 *
 * Wiring:
 *   Encoder A -> D2 (INT0), Encoder B -> D4, Encoder VCC -> 3.3 V
 */

#define ENCA 2
#define ENCB 4

volatile long ticks = 0;
volatile uint32_t lastISRmicros = 0;
const uint32_t minEdgeUs = 50;  // Debounce threshold (us)

long ticksPrev = 0;
const int Ts_ms = 100;  // Sampling interval (ms)
const int ppr = 100;    // Pulses per revolution
// TWO_PI is already defined by Arduino.h, so it is not redefined here.

void isrEncA() {
  uint32_t now = micros();
  if (now - lastISRmicros < minEdgeUs) return;
  lastISRmicros = now;

  bool b = digitalRead(ENCB);
  ticks += (b ? -1 : 1);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Task 3: Velocity Measurement");

  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), isrEncA, RISING);
}

void loop() {
  // Atomic copy of the counter
  noInterrupts();
  long currentTicks = ticks;
  interrupts();

  long dticks = currentTicks - ticksPrev;
  ticksPrev = currentTicks;

  // dticks / ppr                  = revolutions in the interval
  // (dticks / ppr) / (Ts_ms/1000) = rev/s
  // rev/s * 60                    = RPM
  float countsPerRev = (float)ppr;
  float timeInSec = Ts_ms / 1000.0f;
  float revPerSec = dticks / countsPerRev / timeInSec;
  float rpm = revPerSec * 60.0f;
  float omega_rad = revPerSec * TWO_PI;  // rad/s

  Serial.print("Current tick: ");
  Serial.print(currentTicks);
  Serial.print(", delta ticks: ");
  Serial.print(dticks);
  Serial.print(", omega_rpm: ");
  Serial.print(rpm, 3);
  Serial.print(", omega_rad: ");
  Serial.println(omega_rad, 3);

  delay(Ts_ms);
}
