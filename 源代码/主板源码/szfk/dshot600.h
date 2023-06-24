#ifndef dshot600_h
#define dshot600_h

#include <Arduino.h>

#define Accelerator_Min  99
#define Accelerator_Max  1500

//电调初始化
void dshot_init(void);

//设置左边电调值：0~2000
void dshotOutput1(uint16_t value, bool telemetry);

//设置右边电调值：0~2000
void dshotOutput2(uint16_t value, bool telemetry);
#endif
