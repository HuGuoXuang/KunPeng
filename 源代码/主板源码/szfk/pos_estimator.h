#ifndef _POS_ESTIMATOR_H
#define _POS_ESTIMATOR_H
#include <Arduino.h>
#include "mpu6000.h"
#include "IMU.h"
#include "maths.h"
#include "SerialDat.h"
#include "flash_memory.h"

#define GRAVITY_CMSS    980.665f
#define GRAVITY_MSS     9.80665f

typedef struct 
{
    Axis3f     pos;
    Axis3f    vel;
    float       eph;
    float       epv;
} posEstimatorEst_t;

typedef struct 
{
    float       alt; //(cm)
    float       epv;
} navPositionEstimatorBARO_t;

typedef struct 
{
    Axis3f     accelNEU;
    Axis3f     accelBias;
    int       gravityCalibrationComplete;
} navPosisitonEstimatorIMU_t;

typedef struct 
{
    Axis3f CMSS;
    int16_t  CMSSFlashSign;
} GravityCMSS_t;

typedef struct 
{
    // Data sources
    navPositionEstimatorBARO_t  baro;
    navPosisitonEstimatorIMU_t  imu;

    // Estimate
    posEstimatorEst_t  est;
  
} posEstimator_t;

extern posEstimator_t posLight;
extern posEstimator_t posUltrasonic;
extern posEstimator_t posEstimator;
extern int csbok;
extern float W_ACC_BIAS;
extern float  W_Z_BARO_P;    //气压修正权重
extern float  W_XY_Light_P;    //光流修正权重
extern float  W_XY_Light_V;
extern GravityCMSS_t calibratedGravity;
extern biquadFilter_t AccNotchingFilter[3];//陷波濾波器
extern float accpos[3];

//更新预估器，由主循环调用
void updatePositionEstimator(const sensorData_t *sensorData, state_t *state, float dt);
void Gravityinit(void);
void initPosUltrasonic(void);
int32_t applyDeadband(int32_t value, int32_t deadband);

#endif /*_POS_ESTIMATOR_H */
