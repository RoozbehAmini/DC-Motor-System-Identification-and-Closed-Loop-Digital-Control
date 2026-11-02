/*
 * Task 4: L298N H-Bridge Motor Driver Integration
 * Mechatronics Lab, Session 6 - University of Tehran
 *
 * Cycles the motor through FORWARD (2 s) -> COAST (800 ms) ->
 * REVERSE (2 s) -> BRAKE (800 ms), printing ticks every 200 ms.
 *
 * Wiring:
 *   Encoder A -> D2 (INT0), Encoder B -> D4, Encoder VCC -> 3.3 V
 *   L298N ENA -> D5 (PWM), IN1 -> D7, IN2 -> D8, motor supply 12 V
 */

#define ENCA 2
#define ENCB 4
#define ENA 5  // PWM pin for speed control
#define IN1 7  // Direction control 1
#define IN2 8  // Direction control 2

volatile long ticks = 0;
volatile uint32_t lastISRmicros = 0;
const uint32_t minEdgeUs = 50;  // Debounce threshold (us)

// Motor control modes
enum MotorMode { FORWARD, REVERSE, BRAKE, COAST };

void isrEncA() {
  uint32_t now = micros();
  if (now - lastISRmicros < minEdgeUs) return;
  lastISRmicros = now;

  bool b = digitalRead(ENCB);
  ticks += (b ? -1 : 1);
}

void setMotorMode(MotorMode mode, int pwmValue = 200) {
  switch (mode) {
    case FORWARD:
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, pwmValue);
      Serial.println("Mode: FORWARD");
      break;

    case REVERSE:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, pwmValue);
      Serial.println("Mode: REVERSE");
      break;

    case BRAKE:
      // IN1 = IN2 = HIGH ties both motor terminals together (fast stop)
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, 255);  // Full enable for maximum braking
      Serial.println("Mode: BRAKE");
      break;

    case COAST:
      // Bridge disabled: the motor freewheels
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0);
      Serial.println("Mode: COAST");
      break;
  }
}

// Print ticks and tick change every 200 ms, `samples` times
void logTicks(int samples, long &lastTicks) {
  for (int i = 0; i < samples; i++) {
    noInterrupts();
    long current = ticks;
    interrupts();
    Serial.print("ticks: ");
    Serial.print(current);
    Serial.print("\t dticks: ");
    Serial.println(current - lastTicks);
    lastTicks = current;
    delay(200);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Task 4: L298N H-Bridge Control");
  Serial.println("Testing 4 modes: Forward, Coast, Reverse, Brake");

  // Encoder pins
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), isrEncA, RISING);

  // Motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Start in coast mode
  setMotorMode(COAST);
}

void loop() {
  // static so dticks stays correct across repeats of the test sequence
  static long lastTicks = 0;

  // 1. FORWARD for 2 seconds
  Serial.println("--- FORWARD mode (2 seconds) ---");
  setMotorMode(FORWARD, 200);
  logTicks(10, lastTicks);

  // 2. COAST for 800 ms
  Serial.println("--- COAST mode (800 ms) ---");
  setMotorMode(COAST);
  logTicks(4, lastTicks);

  // 3. REVERSE for 2 seconds
  Serial.println("--- REVERSE mode (2 seconds) ---");
  setMotorMode(REVERSE, 200);
  logTicks(10, lastTicks);

  // 4. BRAKE for 800 ms
  Serial.println("--- BRAKE mode (800 ms) ---");
  setMotorMode(BRAKE);
  logTicks(4, lastTicks);

  // Wait before repeating
  delay(2000);
}
