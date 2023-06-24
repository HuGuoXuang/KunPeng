#ifndef sensors_h
#define sensors_h
#include <Arduino.h>
#include "filter.h"
#include "maths.h"
#include "mpu6000.h"
#include "IMU.h"
#include "pos_estimator.h"
#include "ICM42688.h"
extern sensorData_t 	sensorData;	/*传感器数据*/
extern state_t  state;		/*四轴姿态*/

void sensorsInit(void);
/*传感器任务*/
void sensorsTask(void);


#endif
