/*
 * Task 5: PWM Speed Control and Nonlinearity Analysis
 * Mechatronics Lab, Session 6 - University of Tehran
 *
 * Runs the motor forward at a set PWM value and prints the measured RPM
 * every 100 ms. Send "pwm=X" (0-255) over serial to change the speed.
 *
 * Wiring:
 *   Encoder A -> D2 (INT0), Encoder B -> D4, Encoder VCC -> 3.3 V
 *   L298N ENA -> D5 (PWM), IN1 -> D7, IN2 -> D8, motor supply 12 V
 */

#define ENCA 2
#define ENCB 4
#define ENA 5
#define IN1 7
#define IN2 8

volatile long ticks = 0;
volatile uint32_t lastISRmicros = 0;
const uint32_t minEdgeUs = 50;  // Debounce threshold (us)

long ticksPrev = 0;
const int Ts_ms = 100;  // Sampling interval (ms)
const int ppr = 100;    // Pulses per revolution

int pwmValue = 100;  // Starting PWM

void isrEncA() {
  uint32_t now = micros();
  if (now - lastISRmicros < minEdgeUs) return;
  lastISRmicros = now;

  bool b = digitalRead(ENCB);
  ticks += (b ? -1 : 1);
}

void setMotorSpeed(int pwm) {
  pwmValue = constrain(pwm, 0, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, pwmValue);
}

float calculateRPM(long dticks) {
  float countsPerRev = (float)ppr;
  float timeInSec = Ts_ms / 1000.0f;
  float revPerSec = dticks / countsPerRev / timeInSec;
  return revPerSec * 60.0f;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Task 5: PWM Speed Control & Velocity Measurement");
  Serial.println("Commands: pwm=X to set speed (0-255)");

  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), isrEncA, RISING);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  setMotorSpeed(pwmValue);
}

void loop() {
  // Measure velocity
  noInterrupts();
  long currentTicks = ticks;
  interrupts();

  long dticks = currentTicks - ticksPrev;
  ticksPrev = currentTicks;

  float rpm = calculateRPM(dticks);

  Serial.print("Mode:FWD, PWM:");
  Serial.print(pwmValue);
  Serial.print(", RPM: ");
  Serial.println(rpm, 2);

  // Serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("pwm=")) {
      int newPWM = constrain(cmd.substring(4).toInt(), 0, 255);
      Serial.print("--- Changing to PWM ");
      Serial.print(newPWM);
      Serial.print(" (");
      Serial.print(newPWM * 100.0 / 255.0, 1);
      Serial.println("% duty) ---");
      setMotorSpeed(newPWM);
    }
  }

  delay(Ts_ms);
}
