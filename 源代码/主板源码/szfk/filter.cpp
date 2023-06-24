#include "filter.h"

#define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - butterworth*/

//2. 以高度为例 定义卡尔曼结构体并初始化参数
//KFP KFP_X={0.02,0,0,0,0.001,0.005};

/**
 *卡尔曼滤波器
 *@param KFP *kfp 卡尔曼结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
 float kalmanFilter(KFP *kfp,float input)
 {
     //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
     kfp->NOWP  = kfp->LastP + kfp->Q;
     //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
     kfp->Kg = kfp->NOWP  / (kfp->NOWP  + kfp->R);
     //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
     kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//因为这一次的预测值就是上一次的输出值
     //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
     kfp->LastP = (1-kfp->Kg) * kfp->NOWP ;
     return kfp->out;
 }

// PT1低通滤波器
//pt1获取滤波增益(截止频率 采样时间)
float pt1FilterGain(float f_cut, float dT)
{
    float RC = 1 / (2 * M_PIf * f_cut);
    return dT / (RC + dT);
}

//pt1初始化低通滤波器
void pt1FilterInit(pt1Filter_t *filter, float k)
{
    filter->state = 0.0f;
    filter->k = k;
}

//更新滤波增益
void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k)
{
    filter->k = k;
}

//pt1低通滤波器应用
float pt1FilterApply(pt1Filter_t *filter, float input)
{
    filter->state = filter->state + filter->k * (input - filter->state);
    return filter->state;
}


//获取陷波滤波器Q给定的中心频率(f0)和较低的截止频率(f1)
//Q = f0 / (f2 - f1);F2 = f0²/ f1
//（中心频率 截止频率）
float filterGetNotchQ(float centerFreq, float cutoffFreq)
{
    return centerFreq * cutoffFreq / (centerFreq * centerFreq - cutoffFreq * cutoffFreq);
}

//二阶低通滤波器初始化
void biquadFilterInitLPF(biquadFilter_t *filter, uint16_t filterFreq ,uint16_t samplingFreq)
{
    biquadFilterInit(filter, filterFreq, samplingFreq,  BIQUAD_Q, FILTER_LPF);
}


void biquadFilterInit(biquadFilter_t *filter, float filterFreq, uint32_t refreshRate, float Q, biquadFilterType_e filterType)
{
    biquadFilterUpdate(filter, filterFreq, refreshRate, Q, filterType);

    // zero initial samples
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
}




//二阶滤波器初始化
void biquadFilterUpdate(biquadFilter_t *filter, uint16_t filterFreq, uint16_t refreshRate, float Q, biquadFilterType_e filterType)
{
    // setup variables
    const float omega = 2.0f * M_PIf * filterFreq * refreshRate * 0.000001f;
    const float sn = sin_approx(omega);
    const float cs = cos_approx(omega);
    const float alpha = sn / (2.0f * Q);

    switch (filterType) {
    case FILTER_LPF:
        // 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
        // described in http://www.ti.com/lit/an/slaa447/slaa447.pdf
        filter->b1 = 1 - cs;
        filter->b0 = filter->b1 * 0.5f;
        filter->b2 = filter->b0;
        filter->a1 = -2 * cs;
        filter->a2 = 1 - alpha;
        break;
    case FILTER_NOTCH:
        filter->b0 = 1;
        filter->b1 = -2 * cs;
        filter->b2 = 1;
        filter->a1 = filter->b1;
        filter->a2 = 1 - alpha;
        break;
    case FILTER_BPF:
        filter->b0 = alpha;
        filter->b1 = 0;
        filter->b2 = -alpha;
        filter->a1 = -2 * cs;
        filter->a2 = 1 - alpha;
        break;
    }

    const float a0 = 1 + alpha;

    // precompute the coefficients
    filter->b0 /= a0;
    filter->b1 /= a0;
    filter->b2 /= a0;
    filter->a1 /= a0;
    filter->a2 /= a0;

    // update weight
    ///filter->weight = weight;
}




//二阶陷波器初始化(结构体变量 采样频率 中心频率 截止频率)
void biquadFilterInitNotch(biquadFilter_t *filter, uint16_t samplingFreq, uint16_t filterFreq, uint16_t cutoffHz)
{
    float Q = filterGetNotchQ(filterFreq, cutoffHz);
    biquadFilterInit(filter, samplingFreq, filterFreq, Q, FILTER_NOTCH);
}





/* Computes a biquadFilter_t filter in direct form 2 on a sample (higher precision but can't handle changes in coefficients */
float biquadFilterApply(biquadFilter_t *filter, float input)
{
    const float result = filter->b0 * input + filter->x1;

    filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
    filter->x2 = filter->b2 * input - filter->a2 * result;

    return result;
}
