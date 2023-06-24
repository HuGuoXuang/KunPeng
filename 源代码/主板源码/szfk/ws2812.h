#ifndef ws2812_h
#define ws2812_h

#include <Arduino.h>
//#include "sbus.h"
#include "ibus.h"

void ledint(void);
void ShowColor(void);
void ledColor(int x, int r, int g, int b);
void allLed(int r, int g, int b); //灯灭
void ledMs(int x,int r, int g, int b,int ms,int c);
void ledShow(void);

#endif
