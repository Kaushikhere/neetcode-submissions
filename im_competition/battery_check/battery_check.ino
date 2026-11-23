/*
  BATTERY CHECK - run this on BATTERY power only (no USB cable)

  Motors stay OFF. Turn each wheel slowly by hand:
    - The Pico's small green LED flickers while that wheel turns
      -> that encoder works on battery.
    - LED stays dark -> that encoder gets no power or no ground on
      battery (see the notes in the chat).

  Also hold each IR sensor over white and black: the small LED on
  the IR module itself must switch. If it stays dark, the IR module
  has no power on battery.

  Encoders: left GP18/GP19, right GP4/GP5
*/

#define ENA 10
#define IN1 11
#define IN2 12
#define IN3 13
#define IN4 14
#define ENB 15

#define ENC_L_A 18
#define ENC_R_A 4

volatile long ticks = 0;

void onTick() {
  ticks++;
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  analogWriteRange(255);
  analogWrite(ENA, 255);   // inverted: 255 = motors off
  analogWrite(ENB, 255);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), onTick, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), onTick, CHANGE);
}

void loop() {
  // each encoder pulse flips the LED, so a turning wheel makes it flicker
  digitalWrite(LED_BUILTIN, (ticks / 4) % 2);
}
