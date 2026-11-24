/*
  Raspberry Pi Pico - Start-Box Ultrasonic Localization & Navigation to T1
  (your logic, on our connections)

  Coordinate System:
    Origin (0, 0) = Bottom-left inside corner of the 30cm start walls
    +X = Right (0 to 120 cm)
    +Y = Forward into the field (0 to 244 cm)
    Heading: +90 deg = +Y (Forward), -90 deg = -Y (Facing back wall)

  OUR CONNECTIONS
    Right motor : ENA GP0, IN1 GP1, IN2 GP2   (forward = IN1 LOW,  IN2 HIGH)
    Left motor  : IN3 GP3, IN4 GP4, ENB GP5   (forward = IN3 HIGH, IN4 LOW)
    PWM through the optocouplers is inverted: 255 - pwm
    Encoders    : right C1 GP12 / C2 GP13, left C1 GP14 / C2 GP15
    Ultrasonics : ALL TRIG on GP6 (shared)
                  ECHO front GP9, right GP7, left GP19
*/

#include <math.h>

// ---------------- MOTOR & ENCODER PINS ----------------
const int RIGHT_EN  = 0;   // ENA
const int RIGHT_IN1 = 1;   // IN1
const int RIGHT_IN2 = 2;   // IN2
const int LEFT_IN1  = 3;   // IN3
const int LEFT_IN2  = 4;   // IN4
const int LEFT_EN   = 5;   // ENB

const int RIGHT_A = 12;
const int RIGHT_B = 13;
const int LEFT_A  = 14;
const int LEFT_B  = 15;

// ---------------- ULTRASONIC PINS ----------------
const int US_TRIG       = 6;    // shared by all three sensors
const int US_FRONT_ECHO = 9;
const int US_RIGHT_ECHO = 7;
const int US_LEFT_ECHO  = 19;

// ---------------- ROBOT GEOMETRY ----------------
const float WHEEL_DIAMETER_CM = 7.0;
const float WHEELBASE_CM      = 17.0;
// Counts per ONE wheel turn, counting channel A only (CHANGE).
// From your cart_tuner (both channels: L 1560, R 975) halved.
// The two encoders really are different - each wheel has its own value.
const float COUNTS_PER_REV_L  = 780.0;
const float COUNTS_PER_REV_R  = 487.5;

const float FRONT_OFFSET_CM   = 16.0;  // Wheel-axle center to Front US face
const float SIDE_OFFSET_CM    = 13.5;  // Wheel-axle center line to Left/Right US face

const float CPCM_L = COUNTS_PER_REV_L / (PI * WHEEL_DIAMETER_CM);   // counts per cm
const float CPCM_R = COUNTS_PER_REV_R / (PI * WHEEL_DIAMETER_CM);
const float CM_PER_DEG = PI * WHEELBASE_CM / 360.0;                 // each wheel, in a pivot

// ---------------- FIELD & TARGET 1 CONSTANTS ----------------
const float FIELD_WIDTH_CM   = 120.0;
const float SIDE_WALL_LEN_CM = 29.0;
const float T1_X             = 62.0;
const float T1_Y             = 94.0;

// ---------------- TUNED PI + SYNC SETTINGS (now in cm/s) ----------------
const float DRIVE_CMPS    = 25.0;  // Straight cruise speed to T1
const float APPROACH_CMPS = 13.0;  // Slow speed for final 10 cm approach
const float TURN_CMPS     = 12.0;  // Pivot speed for accurate turns

const float BASE_PWM = 140.0;
const float MIN_PWM  = 80.0;
const float MAX_PWM  = 230.0;

const float Kp     = 0.04;      // per count/s of speed error (same feel as before)
const float Ki     = 0.12;
const float K_sync = 6.0;       // PWM per cm one wheel is ahead of the other

const unsigned long CONTROL_INTERVAL = 25;

// ---------------- ENCODERS ----------------
volatile long leftCount  = 0;
volatile long rightCount = 0;

// forward = counts go UP on both wheels (same signs as our line follower)
void leftEncoderISR() {
  if (digitalRead(LEFT_A) != digitalRead(LEFT_B)) leftCount++;
  else leftCount--;
}

void rightEncoderISR() {
  if (digitalRead(RIGHT_A) != digitalRead(RIGHT_B)) rightCount--;
  else rightCount++;
}

void resetEncoders() {
  noInterrupts();
  leftCount  = 0;
  rightCount = 0;
  interrupts();
}

// ---------------- MOTOR HELPERS (INVERTED PWM) ----------------
void setLeftForward()  { digitalWrite(LEFT_IN1, HIGH); digitalWrite(LEFT_IN2, LOW);  }
void setLeftReverse()  { digitalWrite(LEFT_IN1, LOW);  digitalWrite(LEFT_IN2, HIGH); }
void setRightForward() { digitalWrite(RIGHT_IN1, LOW);  digitalWrite(RIGHT_IN2, HIGH); }
void setRightReverse() { digitalWrite(RIGHT_IN1, HIGH); digitalWrite(RIGHT_IN2, LOW);  }

void stopMotors() {
  // Brief brake (both inputs the same + enable on), as in our line follower
  digitalWrite(LEFT_IN1, LOW);  digitalWrite(LEFT_IN2, LOW);
  digitalWrite(RIGHT_IN1, LOW); digitalWrite(RIGHT_IN2, LOW);
  analogWrite(LEFT_EN, 0);       // inverted PWM: 0 = enable fully on
  analogWrite(RIGHT_EN, 0);
  delay(40);

  // Release motors
  analogWrite(LEFT_EN, 255);
  analogWrite(RIGHT_EN, 255);
}

// ---------------- ULTRASONIC HELPERS (shared trigger) ----------------
// All three sensors fire together on GP6. Each ping times ONE echo pin,
// then waits for every sensor's echo to die out before the next ping.
float readUltrasonicSingle(int echoPin) {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);

  // 15ms timeout (~250 cm max) prevents blocking
  unsigned long duration = pulseIn(echoPin, HIGH, 15000);
  if (duration == 0) return 250.0;
  return (duration * 0.0343) / 2.0;
}

float readUltrasonicMedian(int echoPin, int samples = 3) {
  float vals[5];
  if (samples > 5) samples = 5;

  for (int i = 0; i < samples; i++) {
    vals[i] = readUltrasonicSingle(echoPin);
    delay(25); // Let all three sensors' echoes die out before the next ping
  }

  // Simple bubble sort to grab the median
  for (int i = 0; i < samples - 1; i++) {
    for (int j = i + 1; j < samples; j++) {
      if (vals[j] < vals[i]) {
        float tmp = vals[i];
        vals[i] = vals[j];
        vals[j] = tmp;
      }
    }
  }
  return vals[samples / 2];
}

void readAllUltrasonics(float &dFront, float &dLeft, float &dRight, int samples = 3) {
  dFront = readUltrasonicMedian(US_FRONT_ECHO, samples);
  delay(15);
  dLeft  = readUltrasonicMedian(US_LEFT_ECHO,  samples);
  delay(15);
  dRight = readUltrasonicMedian(US_RIGHT_ECHO, samples);
}

// ---------------- CLOSED-LOOP MOTION PRIMITIVES ----------------
float wrapAngle180(float deg) {
  while (deg >  180.0) deg -= 360.0;
  while (deg <= -180.0) deg += 360.0;
  return deg;
}

// One control step for both wheels. Distances and speeds are per wheel in cm
// (the encoders differ), the PI works in that wheel's counts per second.
void controlWheels(float targetCmps, long curLeft, long curRight, long &prevLeft, long &prevRight,
                   float &leftIntegral, float &rightIntegral, float dt) {
  float distL = fabs(curLeft) / CPCM_L, distR = fabs(curRight) / CPCM_R;

  float leftSpeed  = fabs(curLeft  - prevLeft)  / dt;      // counts/s
  float rightSpeed = fabs(curRight - prevRight) / dt;
  prevLeft  = curLeft;
  prevRight = curRight;

  float leftError  = targetCmps * CPCM_L - leftSpeed;
  float rightError = targetCmps * CPCM_R - rightSpeed;

  leftIntegral  = constrain(leftIntegral  + leftError  * dt, -2000.0f, 2000.0f);
  rightIntegral = constrain(rightIntegral + rightError * dt, -2000.0f, 2000.0f);

  float syncCorr = constrain(K_sync * (distL - distR), -25.0f, 25.0f);   // left ahead -> slow it

  float leftPWM  = constrain(BASE_PWM + Kp * leftError  + Ki * leftIntegral  - syncCorr, MIN_PWM, MAX_PWM);
  float rightPWM = constrain(BASE_PWM + Kp * rightError + Ki * rightIntegral + syncCorr, MIN_PWM, MAX_PWM);

  analogWrite(LEFT_EN,  255 - (int)leftPWM);
  analogWrite(RIGHT_EN, 255 - (int)rightPWM);
}

// Positive degrees = CCW (Turn Left), Negative degrees = CW (Turn Right)
void pivotDegrees(float degrees) {
  if (fabs(degrees) < 1.0) return;

  float targetCm = fabs(degrees) * CM_PER_DEG;   // each wheel travels this
  bool turnLeft = (degrees > 0);

  resetEncoders();
  long prevLeft = 0, prevRight = 0;
  float leftIntegral = 0.0, rightIntegral = 0.0;

  if (turnLeft) { setLeftReverse(); setRightForward(); }
  else          { setLeftForward(); setRightReverse(); }

  unsigned long prevTime = millis();

  while (true) {
    unsigned long now = millis();
    if (now - prevTime >= CONTROL_INTERVAL) {
      float dt = (now - prevTime) / 1000.0;
      prevTime = now;

      long curLeft, curRight;
      noInterrupts();
      curLeft  = leftCount;
      curRight = rightCount;
      interrupts();

      float avgCm = (fabs(curLeft) / CPCM_L + fabs(curRight) / CPCM_R) / 2.0;
      if (avgCm >= targetCm) break;

      controlWheels(TURN_CMPS, curLeft, curRight, prevLeft, prevRight, leftIntegral, rightIntegral, dt);
    }
  }

  stopMotors();
  delay(80);
}

// Drive straight forward with PI + Heading Sync (slows down for final 10 cm)
void driveStraightCm(float cm) {
  if (cm <= 0.5) return;

  resetEncoders();
  long prevLeft = 0, prevRight = 0;
  float leftIntegral = 0.0, rightIntegral = 0.0;

  setLeftForward();
  setRightForward();

  unsigned long prevTime = millis();

  while (true) {
    unsigned long now = millis();
    if (now - prevTime >= CONTROL_INTERVAL) {
      float dt = (now - prevTime) / 1000.0;
      prevTime = now;

      long curLeft, curRight;
      noInterrupts();
      curLeft  = leftCount;
      curRight = rightCount;
      interrupts();

      float avgCm = (fabs(curLeft) / CPCM_L + fabs(curRight) / CPCM_R) / 2.0;
      if (avgCm >= cm) break;

      float targetSpeed = (cm - avgCm < 10.0) ? APPROACH_CMPS : DRIVE_CMPS;
      controlWheels(targetSpeed, curLeft, curRight, prevLeft, prevRight, leftIntegral, rightIntegral, dt);
    }
  }

  stopMotors();
  delay(100);
}

void dropItem() {
  stopMotors();
  Serial.println(">>> ARRIVED AT T1! Dropping item... <<<");
  // TODO: Add servo / dropper actuator trigger here later
}

// ---------------- LOCALIZATION & T1 ROUTINE ----------------
void localizeAndGoToT1() {
  Serial.println("--- PHASE 1: 360-DEGREE START-BOX SCAN ---");

  const int STEP_DEG  = 30;
  const int NUM_STEPS = 360 / STEP_DEG;

  float bestBackAngle = 0.0;
  float bestScore     = 9999.0;
  bool  foundStrict   = false;

  for (int i = 0; i < NUM_STEPS; i++) {
    float curAngle = i * STEP_DEG;
    float dF, dL, dR;
    readAllUltrasonics(dF, dL, dR, 3);

    float measuredWidth = dL + dR + (2.0 * SIDE_OFFSET_CM);
    float widthErr      = fabs(measuredWidth - FIELD_WIDTH_CM);

    Serial.print("Ang:"); Serial.print(curAngle);
    Serial.print(" | F:"); Serial.print(dF, 1);
    Serial.print(" L:");   Serial.print(dL, 1);
    Serial.print(" R:");   Serial.print(dR, 1);
    Serial.print(" | W_est:"); Serial.println(measuredWidth, 1);

    // Back-wall signature: Front sees close wall AND Left+Right span ~120 cm
    if (dF < (SIDE_WALL_LEN_CM + 15.0) && widthErr < 15.0) {
      // Among valid back-wall candidates, prefer smallest dFront (most perpendicular)
      float score = dF + 0.5 * widthErr;
      if (!foundStrict || score < bestScore) {
        foundStrict   = true;
        bestScore     = score;
        bestBackAngle = curAngle;
      }
    } else if (!foundStrict && dF < 60.0) {
      // Fallback if strict width check hasn't matched yet
      float score = widthErr + dF;
      if (score < bestScore) {
        bestScore     = score;
        bestBackAngle = curAngle;
      }
    }

    pivotDegrees(STEP_DEG);
  }

  Serial.print("Back wall detected near relative angle: ");
  Serial.println(bestBackAngle);

  // Rotate back to the detected back-wall heading (shortest path from 360 deg)
  pivotDegrees(wrapAngle180(bestBackAngle));

  // --- PHASE 2: FINE ALIGNMENT TO BACK WALL (MINIMUM dFront) ---
  Serial.println("--- PHASE 2: FINE ALIGNMENT ---");
  pivotDegrees(-16.0); // Start 16 deg CW of best estimate

  float minFront = 999.0;
  float bestOffset = 0.0;

  for (int offset = -16; offset <= 16; offset += 4) {
    float dF = readUltrasonicMedian(US_FRONT_ECHO, 3);
    if (dF < minFront) {
      minFront   = dF;
      bestOffset = offset;
    }
    if (offset < 16) pivotDegrees(4.0);
  }

  // Turn back from +16 deg to the exact minimum-distance angle
  pivotDegrees(bestOffset - 16.0);
  delay(150);

  // --- PHASE 3: COMPUTE EXACT (X, Y) POSE ---
  // Robot is now facing the back wall (-Y, heading = -90 deg).
  // Right sensor points toward X = 0 (left wall), Left sensor points toward X = 120 (right wall).
  float dF, dL, dR;
  readAllUltrasonics(dF, dL, dR, 5);

  float robotY = dF + FRONT_OFFSET_CM;

  float xFromRight = dR + SIDE_OFFSET_CM;
  float xFromLeft  = FIELD_WIDTH_CM - (dL + SIDE_OFFSET_CM);
  float robotX;

  if (dR < 150.0 && dL < 150.0) {
    robotX = (xFromRight + xFromLeft) / 2.0;
  } else if (dR < 150.0) {
    robotX = xFromRight;
  } else {
    robotX = xFromLeft;
  }

  Serial.print("Localized Pose -> X: "); Serial.print(robotX, 1);
  Serial.print(" cm, Y: ");             Serial.print(robotY, 1);
  Serial.println(" cm, Heading: -90 deg");

  // --- PHASE 4: COMPUTE HEADING & DISTANCE TO T1 ---
  float dx = T1_X - robotX;
  float dy = T1_Y - robotY;

  float distToT1       = sqrt(dx * dx + dy * dy);
  float targetAngleDeg = atan2(dy, dx) * 180.0 / PI;
  float turnToT1Deg    = wrapAngle180(targetAngleDeg - (-90.0));

  Serial.print("Target T1 -> Turn: "); Serial.print(turnToT1Deg, 1);
  Serial.print(" deg, Drive: ");       Serial.print(distToT1, 1);
  Serial.println(" cm");

  // --- PHASE 5: TURN & DRIVE TO T1 ---
  pivotDegrees(turnToT1Deg);
  delay(150);
  driveStraightCm(distToT1);

  dropItem();
}

// ---------------- SETUP & LOOP ----------------
void setup() {
  Serial.begin(115200);

  pinMode(LEFT_IN1, OUTPUT);  pinMode(LEFT_IN2, OUTPUT);  pinMode(LEFT_EN, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT); pinMode(RIGHT_EN, OUTPUT);

  pinMode(LEFT_A, INPUT_PULLUP);  pinMode(LEFT_B, INPUT_PULLUP);
  pinMode(RIGHT_A, INPUT_PULLUP); pinMode(RIGHT_B, INPUT_PULLUP);

  pinMode(US_TRIG, OUTPUT); digitalWrite(US_TRIG, LOW);
  pinMode(US_FRONT_ECHO, INPUT);
  pinMode(US_LEFT_ECHO,  INPUT);
  pinMode(US_RIGHT_ECHO, INPUT);

  analogWriteRange(255);      // same PWM setup as our working line follower
  analogWriteFreq(500);

  attachInterrupt(digitalPinToInterrupt(LEFT_A),  leftEncoderISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A), rightEncoderISR, CHANGE);

  stopMotors();
  delay(2000); // Time to step back after placing robot in start box

  localizeAndGoToT1();
}

void loop() {
  // Mission complete; stays stopped until reset
}
