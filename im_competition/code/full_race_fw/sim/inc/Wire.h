#pragma once
#include <cstdint>
struct WireT{ void setSDA(int){} void setSCL(int){} void begin(){} void end(){} void setClock(long){}
 void beginTransmission(uint8_t){} int write(uint8_t){return 1;} int endTransmission(bool=true){return 0;}
 int requestFrom(uint8_t,uint8_t n){return n;} int read(){return 0;} };
extern WireT Wire;
