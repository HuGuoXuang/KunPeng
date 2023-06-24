#include "myBUZZER.h"
#include "ws2812.h"

#define buzzer 14
int freq1 = 2000;//设置频率
int channel2 = 2;//通道号，取值0 ~ 15
int resolution = 8;//计数位数，2的8次幂=256



void BUZZER_init(void)//
{
  ledcSetup(channel2, freq1, resolution);
  ledcAttachPin(buzzer, channel2);
  ledcWriteTone(channel2, 10000);
}

void BuzzerMs(int x,int ms)//
{
    for(int i=0;i<x;i++)
    {
        ledcWriteTone(channel2, 0);
        allLed(222, 0, 0); 
        vTaskDelay(ms);    
        ledcWriteTone(channel2, 10000);
        allLed(0, 0, 222); 
        vTaskDelay(ms);          
    }
    allLed(0, 0, 0); 
}
