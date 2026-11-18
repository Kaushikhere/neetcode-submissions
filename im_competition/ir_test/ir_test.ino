/*
  IR LINE SENSOR TEST - motors stay OFF

  Left IR OUT  = GP1
  Right IR OUT = GP0

  Open Serial Monitor at 115200 and move each sensor
  over white paper, then over the black line.
  Each sensor should print a different value (0 or 1)
  on white and on black.
*/

#define LEFT_IR  1
#define RIGHT_IR 0

// Motor pins, only used to keep the motors stopped
#define ENA 10
#define IN1 11
#define IN2 12
#define IN3 13
#define IN4 14
#define ENB 15

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  analogWriteRange(255);
  analogWrite(ENA, 255);   // inverted PWM: 255 = stop
  analogWrite(ENB, 255);

  pinMode(LEFT_IR, INPUT);
  pinMode(RIGHT_IR, INPUT);

  delay(1000);
  Serial.println("IR TEST - move the sensors over white and black");
}

void loop() {
  Serial.print("LEFT (GP1): ");
  Serial.print(digitalRead(LEFT_IR));
  Serial.print("    RIGHT (GP0): ");
  Serial.println(digitalRead(RIGHT_IR));
  delay(200);
}
