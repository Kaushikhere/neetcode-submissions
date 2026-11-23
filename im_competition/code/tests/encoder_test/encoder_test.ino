/*
  ==========================================
       ENCODER TEST
       Raspberry Pi Pico + L298N (through 6 optocouplers)
       2 x 12 V gear motors with Hall encoders
  ==========================================

  HOW TO USE

  1. Wire the encoders as on the wiring page:
       Left  encoder: C1 -> GP18, C2 -> GP19, VCC -> 3V3, GND -> Pico GND
       Right encoder: C1 -> GP20, C2 -> GP21, VCC -> 3V3, GND -> Pico GND
     Leave the encoder cable's M1 / M2 wires unconnected (the motors are
     already powered through the soldered tabs).
  2. Lift the wheels off the table, upload, open Serial Monitor at 115200.
  3. After 3 s both motors run forward for 2 s with your calibrated PWM,
     then stop, and the sketch prints both counts.
       - Both counts should go UP. If one goes down, change that side's
         *_SIGN from 1 to -1 and upload again.
       - A count that stays at 0: check that encoder's wiring.
  4. After that the counts keep printing. Turn one wheel by hand exactly
     10 times and divide its count by 10: that is COUNTS_PER_WHEEL_REV.

  MOTOR PINS (UNCHANGED)

  ENA = GP10   IN1 = GP11   IN2 = GP12
  ENB = GP15   IN3 = GP13   IN4 = GP14

  Lower PWM = Faster, Higher PWM = Slower, 255 = Stop
*/

// ==========================================
// MOTOR PINS
// ==========================================

#define ENA 10
#define IN1 11
#define IN2 12

#define IN3 13
#define IN4 14
#define ENB 15

// ==========================================
// ENCODER PINS
// ==========================================

#define ENC_L_A 18   // left encoder C1
#define ENC_L_B 19   // left encoder C2
#define ENC_R_A 20   // right encoder C1
#define ENC_R_B 21   // right encoder C2

// ==========================================
// SETTINGS
// ==========================================

// Set to -1 if that wheel counts DOWN while the robot drives forward.
int LEFT_SIGN = 1;
int RIGHT_SIGN = 1;

// Your calibrated speeds (unchanged)
int LEFT_BASE_PWM = 0;
int RIGHT_BASE_PWM = 72;

// How long the motor test runs (ms)
unsigned long TEST_RUN_MS = 2000;

// Fill these in after the 10-turn test to also print centimetres.
float WHEEL_DIAMETER_CM = 6.5;     // measure your wheel
float COUNTS_PER_WHEEL_REV = 0;    // 0 = not measured yet

// ==========================================
// ENCODER COUNTS (updated by interrupts)
// ==========================================

volatile long rawLeft = 0;
volatile long rawRight = 0;

// Runs on every change of channel A. Comparing A with B gives direction.
void onLeftA() {
  if (digitalRead(ENC_L_A) != digitalRead(ENC_L_B)) rawLeft++;
  else rawLeft--;
}

void onRightA() {
  if (digitalRead(ENC_R_A) != digitalRead(ENC_R_B)) rawRight++;
  else rawRight--;
}

long leftCount() {
  noInterrupts();
  long c = rawLeft;
  interrupts();
  return c * LEFT_SIGN;
}

long rightCount() {
  noInterrupts();
  long c = rawRight;
  interrupts();
  return c * RIGHT_SIGN;
}

void resetCounts() {
  noInterrupts();
  rawLeft = 0;
  rawRight = 0;
  interrupts();
}

float countsToCm(long counts) {
  if (COUNTS_PER_WHEEL_REV <= 0) return 0;
  return counts * (PI * WHEEL_DIAMETER_CM) / COUNTS_PER_WHEEL_REV;
}

// ==========================================
// MOTORS (same as your wall-follow sketch)
// ==========================================

void stopMotors() {
  // Inverted PWM: 255 = stop
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void moveForward(int leftPWM, int rightPWM) {
  // LEFT MOTOR
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // RIGHT MOTOR (mounted on the opposite side)
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, constrain(leftPWM, 0, 255));
  analogWrite(ENB, constrain(rightPWM, 0, 255));
}

void printCounts() {
  long l = leftCount();
  long r = rightCount();

  Serial.print("Left: ");
  Serial.print(l);
  Serial.print("   Right: ");
  Serial.print(r);

  if (COUNTS_PER_WHEEL_REV > 0) {
    Serial.print("   (");
    Serial.print(countsToCm(l), 1);
    Serial.print(" cm, ");
    Serial.print(countsToCm(r), 1);
    Serial.print(" cm)");
  }

  Serial.println();
}

// ==========================================
// SETUP
// ==========================================

void setup() {

  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  analogWriteRange(255);
  analogWriteFreq(500);

  stopMotors();

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), onLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), onRightA, CHANGE);

  delay(3000);

  Serial.println("==============================");
  Serial.println(" ENCODER TEST");
  Serial.println("==============================");
  Serial.println("Motors forward for 2 s...");

  resetCounts();
  moveForward(LEFT_BASE_PWM, RIGHT_BASE_PWM);

  unsigned long start = millis();
  while (millis() - start < TEST_RUN_MS) {
    printCounts();
    delay(200);
  }

  stopMotors();
  delay(300);

  long l = leftCount();
  long r = rightCount();

  Serial.println("------------------------------");
  Serial.print("Left counted ");
  Serial.print(l);
  Serial.print(", right counted ");
  Serial.println(r);

  if (l <= 0) Serial.println("LEFT is not counting up: check wiring, or set LEFT_SIGN = -1");
  if (r <= 0) Serial.println("RIGHT is not counting up: check wiring, or set RIGHT_SIGN = -1");

  if (l > 0 && r > 0) {
    float diff = 100.0 * (r - l) / l;
    Serial.print("Right wheel turned ");
    Serial.print(abs(diff), 1);
    Serial.println(diff >= 0 ? "% more than left" : "% less than left");
  }

  Serial.println("------------------------------");
  Serial.println("Now turn the wheels by hand. Counts:");
  resetCounts();
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop() {
  printCounts();
  delay(200);
}
