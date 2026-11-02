/*
 * Task 1: Encoder Tick Counting with Hardware Interrupt
 * Mechatronics Lab, Session 6 - University of Tehran
 *
 * Counts encoder ticks with x1 decoding (rising edge of channel A only)
 * and prints the count at 5 Hz. Turn the shaft by hand to check that one
 * revolution gives 100 ticks.
 *
 * Wiring:
 *   Encoder A -> D2 (INT0), Encoder B -> D4, Encoder VCC -> 3.3 V
 *
 * Serial (115200 baud) commands: help, show, ppr=X, inv=0|1, reset
 */

// Pin definitions
#define ENCA 2  // Encoder channel A (INT0)
#define ENCB 4  // Encoder channel B (digital input)

// Shared with the ISR, so these must be volatile
volatile long ticks = 0;              // Encoder tick counter
volatile uint32_t lastISRmicros = 0;  // Time of the last accepted edge (debounce)
volatile bool invertDir = false;      // Direction inversion flag

// Configuration
const uint32_t minEdgeUs = 50;  // Debounce threshold (us)
int ppr = 100;                  // Pulses per revolution (x1 decoding)

// ISR: runs on every rising edge of channel A
void isrEncA() {
  // Debounce: ignore edges that arrive too close together
  uint32_t now = micros();
  if (now - lastISRmicros < minEdgeUs) return;
  lastISRmicros = now;

  // Channel B gives the direction:
  //   B = LOW at the rising edge of A  -> forward
  //   B = HIGH at the rising edge of A -> reverse
  bool b = digitalRead(ENCB);
  long step = b ? -1 : 1;

  if (invertDir) step = -step;

  ticks += step;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Task 1: Encoder Tick Counter");
  Serial.println("Commands: help, show, ppr=X, inv=0|1, reset");

  // Encoder pins with internal pull-ups
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);

  // Interrupt on the RISING edge of channel A for x1 decoding
  attachInterrupt(digitalPinToInterrupt(ENCA), isrEncA, RISING);
}

void loop() {
  // Copy the counter with interrupts off so the 4-byte read is atomic
  noInterrupts();
  long currentTicks = ticks;
  interrupts();

  Serial.print("ticks: ");
  Serial.println(currentTicks);

  // Serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "help") {
      Serial.println("Commands:");
      Serial.println("  show     - Display current configuration");
      Serial.println("  ppr=X    - Set pulses per revolution");
      Serial.println("  inv=0|1  - Disable/enable direction inversion");
      Serial.println("  reset    - Reset tick counter to zero");
    }
    else if (cmd == "show") {
      Serial.print("PPR: ");
      Serial.println(ppr);
      Serial.print("Invert: ");
      Serial.println(invertDir);
      Serial.print("Debounce: ");
      Serial.print(minEdgeUs);
      Serial.println(" us");
    }
    else if (cmd.startsWith("ppr=")) {
      ppr = cmd.substring(4).toInt();
      Serial.print("PPR set to ");
      Serial.println(ppr);
    }
    else if (cmd.startsWith("inv=")) {
      invertDir = cmd.substring(4).toInt();
      Serial.print("Direction inversion: ");
      Serial.println(invertDir);
    }
    else if (cmd == "reset") {
      noInterrupts();
      ticks = 0;
      interrupts();
      Serial.println("Tick counter reset to zero");
    }
  }

  delay(200);  // 5 Hz update rate
}
