#ifndef __SENSORS_TYPES_H
#define __SENSORS_TYPES_H

#include "mpu6000.h"


#define X 0
#define Y 1
#define Z 2

#define u32 unsigned long 

typedef enum 
{
    ALIGN_DEFAULT = 0, 
    CW0_DEG = 1,
    CW90_DEG = 2,
    CW180_DEG = 3,
    CW270_DEG = 4,
    CW0_DEG_FLIP = 5,
    CW90_DEG_FLIP = 6,
    CW180_DEG_FLIP = 7,
    CW270_DEG_FLIP = 8
} sensor_align_e;

//运行状态标志位
typedef enum 
{
  ACCELEROMETER_CALIBRATED  = (1 << 1),
  COMPASS_CALIBRATED        = (1 << 2),
  CALIBRATE_MAG             = (1 << 3),
  SMALL_ANGLE               = (1 << 4),
  FLASH_WRITING             = (1 << 5),
} stateFlags_t;

typedef union 
{
	struct 
	{
		int32_t x;
		int32_t y;
		int32_t z;
	};
	int32_t axis[3];
} Axis3i32;

typedef union 
{
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
	};
	int16_t axis[3];
} Axis3i16;

typedef union 
{
	struct 
	{
		float x;
		float y;
		float z;
	};
	float axis[3];
} Axis3f;

struct  vec3_s 
{
  u32 timestamp;
  float x;
  float y;
  float z;
};

typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;

/* 姿态角数据结构体 */
typedef struct
{
    u32 timestamp;  /*时间戳*/
    float roll;
    float pitch;
    float yaw;
    int ok;
} attitude_t;

/* 陀螺仪加速度数据结构体 */
typedef struct
{
    int16_t gyro[3];  /*!< 陀螺仪原始数据 */
    int16_t acc[3];   /*!< 加速度原始数据 */
    Axis3f  fgyroD;  /*!< 陀螺仪转速°/S */
    Axis3f  faccG; /*!< 加速度重力 G */
    int ok;
} gyroAcc_t;

//四轴姿态数据结构
typedef struct
{
  attitude_t  attitude; //姿态角度（deg）
  point_t   Position; //估算的位置（cm）
  velocity_t  Velocity; //估算的速度（cm/s）
  acc_t acc;        //估算的加速度（cm/ss）
} state_t;





//所有传感器数据集合
typedef struct
{
	Axis3f acc;				//加速度（G）
	Axis3f gyro;			//陀螺仪（deg/s）
  //baro_t baro;      
} sensorData_t;

#endif
