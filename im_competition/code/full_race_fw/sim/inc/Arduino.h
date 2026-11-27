#pragma once
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstddef>
#include <cstdarg>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
#define CHANGE 3
#define LED_BUILTIN 25
#ifndef PI
#define PI 3.14159265358979
#endif
typedef unsigned int uint;
class __FlashStringHelper;
#define F(s) ((const __FlashStringHelper*)(s))
void simAdvance(unsigned long us);
extern unsigned long simUs;
extern bool simVerbose;
inline unsigned long millis(){ simAdvance(20); return simUs/1000; }
inline unsigned long micros(){ simAdvance(2); return simUs; }
inline void delay(unsigned long ms){ simAdvance(ms*1000); }
inline void delayMicroseconds(unsigned int us){ simAdvance(us); }
void pinMode(int,int);
void digitalWrite(int,int);
int  digitalRead(int);
void analogWrite(int,int);
inline void analogWriteFreq(uint32_t){} inline void analogWriteRange(int){}
unsigned long pulseIn(int pin,int lvl,unsigned long to);
inline void noInterrupts(){} inline void interrupts(){}
inline int digitalPinToInterrupt(int p){return p;}
inline void attachInterrupt(int, void(*)(), int){}
inline long map(long x,long a,long b,long c,long d){ return (x-a)*(d-c)/(b-a)+c; }
template<class T,class A,class B> T constrain_(T v,A a,B b){ return v<a?(T)a:(v>b?(T)b:v); }
#define constrain(v,a,b) constrain_((v),(a),(b))
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
struct SerialT{
  void out(const char*s){ if(simVerbose) fputs(s,stdout); }
  void begin(long){}
  int available(){return 0;} int read(){return -1;}
  explicit operator bool(){return true;}
  void print(const __FlashStringHelper*s){out((const char*)s);} void print(const char*s){out(s);}
  void print(char c){char b[2]={c,0};out(b);}
  void print(int v){char b[32];snprintf(b,32,"%d",v);out(b);} void print(unsigned v){char b[32];snprintf(b,32,"%u",v);out(b);}
  void print(long v){char b[32];snprintf(b,32,"%ld",v);out(b);} void print(unsigned long v){char b[32];snprintf(b,32,"%lu",v);out(b);}
  void print(double v,int d=2){char b[64];snprintf(b,64,"%.*f",d,v);out(b);}
  void print(bool v){out(v?"1":"0");}
  template<class T> void println(T v){print(v);out("\n");}
  void println(double v,int d){print(v,d);out("\n");}
  void println(){out("\n");}
};
extern SerialT Serial;
