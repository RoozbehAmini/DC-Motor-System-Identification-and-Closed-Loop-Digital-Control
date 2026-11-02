/*
 * Task 2: Direction Detection with External Motor Power
 * Mechatronics Lab, Session 6 - University of Tehran
 *
 * Power the motor directly from 12 V and swap its leads to reverse it.
 * Every 200 ms the sketch prints the tick count, the change since the last
 * print, and the direction inferred from the sign of that change.
 *
 * Wiring:
 *   Encoder A -> D2 (INT0), Encoder B -> D4, Encoder VCC -> 3.3 V
 */

#define ENCA 2
#define ENCB 4

volatile long ticks = 0;
volatile uint32_t lastISRmicros = 0;
const uint32_t minEdgeUs = 50;  // Debounce threshold (us)

long lastTicks = 0;
String direction = "STOP";

void isrEncA() {
  uint32_t now = micros();
  if (now - lastISRmicros < minEdgeUs) return;
  lastISRmicros = now;

  bool b = digitalRead(ENCB);
  long step = b ? -1 : 1;
  ticks += step;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Task 2: Direction Detection");
  Serial.println("Apply 12V to motor. Swap motor leads to reverse direction.");

  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), isrEncA, RISING);
}

void loop() {
  // Atomic copy of the counter
  noInterrupts();
  long currentTicks = ticks;
  interrupts();

  // Tick change since the last sample (velocity proxy)
  long dticks = currentTicks - lastTicks;
  lastTicks = currentTicks;

  // Direction from the sign of the change (+/-10 tick dead band)
  if (dticks > 10) direction = "FWD";
  else if (dticks < -10) direction = "REV";
  else direction = "STOP";

  Serial.print("ticks: ");
  Serial.print(currentTicks);
  Serial.print("\t dticks: ");
  Serial.print(dticks);
  Serial.print("\t dir: ");
  Serial.println(direction);

  delay(200);  // 200 ms sampling interval
}
