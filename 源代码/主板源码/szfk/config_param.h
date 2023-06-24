#ifndef config_param_h
#define config_param_h
#include <Arduino.h>
#include "state_control.h"


typedef struct 
{
  float kp;
  float ki;
  float kd;
} pidInit_t;

typedef struct
{
  int16_t accZero[3];
  int16_t accGain[3];
} accBias_t;

typedef struct
{
  int16_t magZero[3];
} magBias_t;

typedef struct 
{
    int16_t rollDeciDegrees;
    int16_t pitchDeciDegrees;
    int16_t yawDeciDegrees;
} boardAlignment_t;


#endif
