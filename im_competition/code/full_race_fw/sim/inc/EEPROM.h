#pragma once
#include <cstring>
struct EEPROMT{ unsigned char buf[512]; void begin(int){memset(buf,0,512);} template<class T> void get(int a,T&t){memcpy(&t,buf+a,sizeof(T));}
 template<class T> void put(int a,const T&t){memcpy(buf+a,&t,sizeof(T));} bool commit(){return true;} };
extern EEPROMT EEPROM;
