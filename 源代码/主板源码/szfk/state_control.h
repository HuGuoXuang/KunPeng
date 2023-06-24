#ifndef state_control_h
#define state_control_h
#include <Arduino.h>
#include "filter.h"
#include "config_param.h"
#include "PID_v1.h"
#include "sensors.h"
#include "ibus.h"
#include "myservo.h"
#include "maths.h"

#define PID_DT 0.002

/*角度PID积分限幅（单位：deg）*/
#define PID_ANGLE_ROLL_INTEGRATION_LIMIT    2.0
#define PID_ANGLE_PITCH_INTEGRATION_LIMIT  2.0
/*角速度PID积分限幅（单位：deg/s）*/
#define PID_RATE_YAW_INTEGRATION_LIMIT    30.0
#define PID_RATE_ROLL_INTEGRATION_LIMIT    30.0
#define PID_RATE_PITCH_INTEGRATION_LIMIT    30.0

/*角速度PID输出限幅（单位：油门值）*/
#define PID_RATE_ROLL_OUTPUT_LIMIT      400.0
#define PID_RATE_PITCH_OUTPUT_LIMIT     300.0
#define PID_RATE_YAW_OUTPUT_LIMIT     300.0

/*角度PID输出限幅（单位：deg/s）*/
#define PID_ANGLE_ROLL_OUTPUT_LIMIT       300.0
#define PID_ANGLE_PITCH_OUTPUT_LIMIT      300.0
#define PID_ANGLE_YAW_OUTPUT_LIMIT        150.0

//角速度PID D项低通截止频率（单位Hz）
#define PID_RATE_LPF_CUTOFF_FREQ      80.0


//XYZ轴速度PID积分限幅（单位cm/s）
#define PID_VZ_INTEGRATION_LIMIT 			300.0
#define PID_VX_INTEGRATION_LIMIT      40.0
#define PID_VY_INTEGRATION_LIMIT      40.0

//XYZ轴速度PID输出限幅（单位油门值）
#define PID_VZ_OUTPUT_LIMIT					1900.0
#define PID_VX_OUTPUT_LIMIT        40.0
#define PID_VY_OUTPUT_LIMIT        40.0

//XYZ位置PID输出限幅（单位cm/s）
#define PID_POS_Z_OUTPUT_LIMIT        80.0
#define PID_POS_X_OUTPUT_LIMIT        60.0
#define PID_POS_Y_OUTPUT_LIMIT        60.0

//XYZ轴位置PID积分限幅（单位cm）
#define PID_POS_Z_INTEGRATION_LIMIT       300.0
#define PID_POS_X_INTEGRATION_LIMIT      100.0
#define PID_POS_Y_INTEGRATION_LIMIT      100.0


extern biquadFilter_t dFilter;
extern int unlock_motor;  //解锁电机

enum pidIndex
{
  RATE_ROLL = 0,
  RATE_PITCH,
  RATE_YAW,
  ANGLE_ROLL,
  ANGLE_PITCH,
  ANGLE_YAW,
  VELOCITY_Z,
  VELOCITY_Z1,
  POSHOLD_Z,
  VELOCITY_X,
  VELOCITY_Y,
  POSHOLD_X, 
  POSHOLD_Y, 
  PID_NUM
};

typedef struct 
{
  int m1; //值范围：0-2000
  int m2;
  int Servo1; //值范围：-900-900
  int Servo2;
  
}motorPWM_t;

//控制数据结构
typedef struct
{
  float roll;
  float pitch;
  float yaw;
  float thrust;
} control_t;

extern biquadFilter_t BiquadPosAcc[3];//二阶低通滤波器
extern PidObject pid[PID_NUM];
extern control_t control;
extern motorPWM_t motorPWM;

extern float altThrustAdj;
extern float altHoldThrust;

//所有PID初始化
void allPidInit(void);
//姿态控制
void stateControl(void);

void PIDErrorAbnormal(void);
void updateEstimatedTopic1(float dt);


#endif
