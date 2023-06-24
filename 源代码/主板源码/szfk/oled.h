#ifndef oled_h
#define oled_h

#include <U8g2lib.h>
#include <Wire.h>
#include "battery.h"

//oled初始化
void OledInit(void);
//oled显示
void oled_show(void);

#endif
