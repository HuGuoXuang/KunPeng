#ifndef battery_h
#define battery_h

#include <Arduino.h>
#include "filter.h"

extern pt1Filter_t battery_pt1F;
extern float battery_voltage;//电池电压
extern float battery_voltageS;
extern uint8_t battery_FlashSign;
extern float battery_ADC;

void battery_init(void);//电池初始化
void voltage_measure(void);//电压测量

#endif
