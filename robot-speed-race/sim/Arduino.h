// Minimal Arduino / arduino-pico stand-in so speed_race.ino compiles on a PC.
// Every call is backed by the physics model in sim.cpp.
#pragma once
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstdarg>

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 3
#define LED_BUILTIN 25

void pinMode(int pin, int mode);
void digitalWrite(int pin, int level);
int digitalRead(int pin);
void analogWrite(int pin, int value);
void analogWriteFreq(int hz);
void analogWriteRange(int range);
unsigned long pulseIn(int pin, int state, unsigned long timeoutUs);
uint32_t micros();
uint32_t millis();
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);
void attachInterrupt(int irq, void (*fn)(), int mode);
inline int digitalPinToInterrupt(int pin) { return pin; }
inline void noInterrupts() {}
inline void interrupts() {}

template <class T, class A, class B>
inline T constrain(T x, A lo, B hi) { return x < (T)lo ? (T)lo : (x > (T)hi ? (T)hi : x); }

struct SimSerial {
  bool echo = false;
  void begin(long) {}
  int available();
  int read();
  void printf(const char *fmt, ...) {
    if (!echo) return;
    va_list ap; va_start(ap, fmt); vprintf(fmt, ap); va_end(ap);
  }
};
extern SimSerial Serial;
