#ifndef filter_h
#define filter_h
#include <Arduino.h>
#include "maths.h"

//1. 结构体类型定义
typedef struct 
{
    float LastP;//上次估算协方差 初始化值为0.02
    float NOWP ;//当前估算协方差 初始化值为0
    float out;//卡尔曼滤波器输出 初始化值为0
    float Kg;//卡尔曼增益 初始化值为0
    float Q;//过程噪声协方差 初始化值为0.05
    float R;//观测噪声协方差 初始化值为0.99
}KFP;//卡尔曼滤波参数

typedef struct pt1Filter_s {
    float state;
    float k;
} pt1Filter_t;

typedef enum {
    FILTER_LPF,    // 2nd order Butterworth section
    FILTER_NOTCH,
    FILTER_BPF,
} biquadFilterType_e;

/* this holds the data required to update samples thru a filter */
typedef struct biquadFilter_s {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquadFilter_t;

float kalmanFilter(KFP *kfp,float input);
float filterGetNotchQ(float centerFreq, float cutoffFreq);
//二阶滤波器初始化
void biquadFilterUpdate(biquadFilter_t *filter, uint16_t filterFreq, uint16_t refreshRate, float Q, biquadFilterType_e filterType);
//二阶陷波器初始化
void biquadFilterInitNotch(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq, uint16_t cutoffHz);
//二阶低通滤波器初始化
void biquadFilterInitLPF(biquadFilter_t *filter, uint16_t filterFreq ,uint16_t samplingFreq);
// Computes a biquad_t filter on a sample
float biquadFilterApply(biquadFilter_t *filter, float input);
void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType);
//pt1获取滤波增益(截止频率 采样时间)
float pt1FilterGain(float f_cut, float dT);
//pt1初始化低通滤波器
void pt1FilterInit(pt1Filter_t *filter, float k);
//更新滤波增益
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k);
//pt1低通滤波器应用
float pt1FilterApply(pt1Filter_t *filter, float input);


#endif
