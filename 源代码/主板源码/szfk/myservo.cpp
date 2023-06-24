#include "myservo.h"

#define Servo1pin 13  //左边舵机
#define Servo2pin 15  //右边舵机

//舵机io配置
int freq = 50;// 频率
int resolution0 = 16;   // 分辨率，取值0~20，duty最大取值为2^resolution-1
int channel0 = 0;    // 通道0，共16个通道，0~15
int channel1 = 1;    // 

//设置舵机角度
int  Servo1_Angle = 0;//0.5ms=-900  1.5ms=0   2.5ms=900
int  Servo2_Angle = 0;//0.5ms=-900  1.5ms=0   2.5ms=900

//舵机微调
int  Servo1_Angle_offset = 0;//补偿
int  Servo2_Angle_offset = 0;


//舵机初始化
void Servo_Initialization(void) // 初始化程序
{
  ledcSetup(channel0, freq, resolution0); // 设置通道0
  ledcSetup(channel1, freq, resolution0); // 

  ledcAttachPin(Servo1pin, channel0);  // 将通道0与引脚1Servo1pin连接 左边舵机
  ledcAttachPin(Servo2pin, channel1);  // 右边舵机

  Servo_Angle(Servo1_Angle, Servo2_Angle);
}

//设置舵机角度-900~900 = -90~90°
void Servo_Angle(int a_servo, int b_servo)
{
    int a = a_servo+Servo1_Angle_offset;
    int b = b_servo+Servo2_Angle_offset;

    
    a = constrain(a, -900, 900);
    b = constrain(b, -900, 900);

    a = map(a, -900, 900, 1638, 8191);//0.5ms=1638  1.5ms=4915   2.5ms=8191
    b = map(b, -900, 900, 1638, 8191);//0.5ms=1638  1.5ms=4915   2.5ms=8191

    

    ledcWrite(channel0,a);  //0.5ms=1638  1.5ms=4915   2.5ms=8191
    ledcWrite(channel1,b);  //0.5ms=1638  1.5ms=4915   2.5ms=8191  
}
