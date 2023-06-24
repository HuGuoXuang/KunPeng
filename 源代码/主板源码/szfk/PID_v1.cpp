#include "PID_v1.h"

//pid 初始化
void pidInit(PidObject* pid, float kp, float ki, float kd, float iLimit, float outputLimit, float dt, bool enableDFilter, float cutoffFreq)
{
  pid->desired   = 0;
  pid->error     = 0;
  pid->prevError = 0; 
  pid->integ     = 0;
  pid->integ_start = 1;
  pid->deriv     = 0;
  pid->kp      = kp;
  pid->ki        = ki;
  pid->kd        = kd;
  pid->outP      = 0;
  pid->outI      = 0;
  pid->outD      = 0;
  pid->output    = 0;
  pid->iLimit    = iLimit;
  pid->outputLimit = outputLimit;
  pid->dt        = dt;
  pid->enableDFilter = enableDFilter;
  if (pid->enableDFilter)
  {
    biquadFilterInitLPF(&pid->dFilter, (1.0f/dt), cutoffFreq);
  }
}

//pid应用
float pidUpdate(PidObject* pid, float error)//误差=期望-测量
{
  float output = 0.0f;
  
  pid->error = error;

  if(pid->integ_start == 1)
  {
    pid->integ += pid->error * pid->dt;    
  }

  
  //积分限幅
  if (pid->iLimit != 0)
  {
    pid->integ = constrainf(pid->integ, -pid->iLimit, pid->iLimit);
  }
  
  pid->deriv = (pid->error - pid->prevError) / pid->dt;
  if (pid->enableDFilter)
  {
    pid->deriv = biquadFilterApply(&pid->dFilter, pid->deriv);
  }
  
  pid->outP = pid->kp * pid->error;
  pid->outI = pid->ki * pid->integ;
  pid->outD = pid->kd * pid->deriv;

  output = pid->outP + pid->outI + pid->outD;
  
  //输出限幅
  if (pid->outputLimit != 0)
  {
    output = constrainf(output, -pid->outputLimit, pid->outputLimit);
  }
  
  pid->prevError = pid->error;

  pid->output = output;
   
  return pid->output;
}


//增量式PID
float pid_increment(PidObject* pid,float error)
{
  pid->error = error;
  float increment_speed;//增量
  
  //积分限幅
  if (pid->iLimit != 0)
  {
    pid->integ = constrainf(pid->integ, -pid->iLimit, pid->iLimit);
  }
  
  pid->deriv = (pid->error-2*pid->error_next+pid->error_last)/ pid->dt;
  if (pid->enableDFilter)
  {
    pid->deriv = biquadFilterApply(&pid->dFilter, pid->deriv);
  }
  
  pid->outP = pid->kp * (pid->error - pid->error_next);
  pid->outI = pid->ki * pid->integ;
  pid->outD = pid->kd * pid->deriv;

  increment_speed = pid->outP + pid->outI + pid->outD;

  pid->actual_speed+= increment_speed;
  pid->error_last = pid->error_next;//下一次迭代  
  pid->error_next = pid->error;
  

  
  //输出限幅
  if (pid->outputLimit != 0)
  {
    pid->actual_speed = constrainf(pid->actual_speed, -pid->outputLimit, pid->outputLimit);
  }
  

  pid->output = pid->actual_speed;
   
  return pid->output;
    
}


//PID清除微积分
void pidReset(PidObject* pid)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
  pid->actual_speed = 0;
  pid->error_last = 0;
  pid->error_next = 0;

  pid->outP      = 0;
  pid->outI      = 0;
  pid->outD      = 0;
  pid->output    = 0;
  
}

//PID清除积分
void pidResetIntegral(PidObject* pid)
{
  pid->integ     = 0;
  pid->actual_speed = 0;
}

//PID积分赋值
void pidSetIntegral(PidObject* pid, float integ)
{
  pid->integ     = integ;
}
