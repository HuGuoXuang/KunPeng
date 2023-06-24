#ifndef sensor_h
#define sensor_h
#include <Arduino.h> 
#include "mpu6000.h"
#include "maths.h"
#include "sensors.h"
#include "SENSORS_TYPES.h"
#include "ICM42688.h"
extern uint32_t stateFlags;


#define ENABLE_STATE(mask) (stateFlags |= (mask))
#define DISABLE_STATE(mask) (stateFlags &= ~(mask))
#define STATE(mask) (stateFlags & (mask))



void imuInit(void);
//四元数和欧拉角计算
void imuUpdateAttitude(const sensorData_t *sensorData, state_t *state, float dt);


//旋转机体坐标系加速度到NEU坐标系
void imuTransformVectorBodyToEarth(Axis3f *v);


//NEU坐标系加速度到NEU坐标系
void imuTransformVectorEarthToBody(Axis3f *v);



#endif /* _SENSOR_H_ */

/*******************************END OF FILE************************************/
