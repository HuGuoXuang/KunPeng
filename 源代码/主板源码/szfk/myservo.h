#ifndef myservo_h
#define myservo_h

#include <Arduino.h>

//设置舵机角度
extern int  Servo1_Angle;//0.5ms=-900  1.5ms=0   2.5ms=900
extern int  Servo2_Angle;//0.5ms=-900  1.5ms=0   2.5ms=900

//舵机微调
extern int  Servo1_Angle_offset;//补偿
extern int  Servo2_Angle_offset;


//舵机初始化
void Servo_Initialization(void); // 初始化程序

//设置舵机角度-900~900 = -90~90°
void Servo_Angle(int a_servo, int b_servo);

#endif
