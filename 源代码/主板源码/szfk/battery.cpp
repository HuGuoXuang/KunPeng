#include "battery.h"

#define ANALOG_PIN_ch6   34  //电压检测的IO 

uint8_t battery_FlashSign = 0;
float battery_voltageS = 196.69;
float battery_voltage = 0;//电池电压
float battery_ADC = 0;//

pt1Filter_t battery_pt1F;

//pt1获取滤波增益(截止频率 采样时间)
float pt1FilterGain(float f_cut, float dT);
//pt1初始化低通滤波器
void pt1FilterInit(pt1Filter_t *filter, float k);
//更新滤波增益
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k);
//pt1低通滤波器应用
float pt1FilterApply(pt1Filter_t *filter, float input);


void battery_init(void)//电池初始化
{
    pinMode(ANALOG_PIN_ch6 ,INPUT);//电压测量IO  
    pt1FilterInit(&battery_pt1F, pt1FilterGain(10, 0.02));
    battery_pt1F.state = 3333;
    battery_pt1F.k = 0.1;
    
}

void voltage_measure(void)//电压测量
{
    battery_ADC = analogRead(ANALOG_PIN_ch6);//LPF_filter(analogRead(ANALOG_PIN_ch6));//(范围从0到4096)
    battery_ADC =  pt1FilterApply(&battery_pt1F, battery_ADC);  
    
    battery_voltage = battery_ADC/battery_voltageS;
    if(battery_voltage>16.8)
      battery_voltage = 16.8;
    
    
}
