#ifndef PID_v1_h
#define PID_v1_h
#include <Arduino.h>
#include "filter.h"
#include "maths.h"

typedef struct
{

  float set_speed;//设定速度 
  float actual_speed;//实际速度
  float error_next;//上一个偏差  
  float error_last;//上上一个偏差 
  
  float desired;    //< set point
  float error;        //< error
  float prevError;    //< previous error
  float integ;        //< integral
  bool integ_start;
  float deriv;        //< derivative
  float kp;           //< proportional gain
  float ki;           //< integral gain
  float kd;           //< derivative gain
  float outP;         //< proportional output (debugging)
  float outI;         //< integral output (debugging)
  float outD;         //< derivative output (debugging)
  float output;         //< derivative output (debugging)
  float iLimit;       //< integral limit
  float outputLimit;  //< total PID output limit, absolute value. '0' means no limit.
  float dt;           //< delta-time dt
  bool enableDFilter; //< filter for D term enable flag
  biquadFilter_t dFilter;  //< filter for D term
} PidObject;


typedef struct
{
    float set_speed;//设定速度 
    float actual_speed;//实际速度
    float error;//偏差  
    float error_next;//上一个偏差  
    float error_last;//上上一个偏差 
    float kp;           //< proportional gain
    float ki;           //< integral gain
    float kd;           //< derivative gain
    float outP;         //< proportional output (debugging)
    float outI;         //< integral output (debugging)
    float outD;         //< derivative output (debugging)
    float output;         //< derivative output (debugging)
    float dt;           //< delta-time dt
    biquadFilter_t dFilter;  //< filter for D term
     
}PidIncrement;

    
//pid 初始化
void pidInit(PidObject* pid, float kp, float ki, float kd, float iLimit, float outputLimit, float dt, bool enableDFilter, float cutoffFreq);
//pid应用
float pidUpdate(PidObject* pid, float error);//误差=期望-测量
//PID清除微积分
void pidReset(PidObject* pid);
//PID清除积分
void pidResetIntegral(PidObject* pid);
//PID积分赋值
void pidSetIntegral(PidObject* pid, float integ);
//增量式PID
float pid_increment(PidObject* pid,float error);


#endif
