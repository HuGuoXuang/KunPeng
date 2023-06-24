#include "IMU.h"

#define SPIN_RATE_LIMIT     20    //旋转速率
#define DCM_KP_ACC      0.600f    //加速度补偿陀螺仪PI参数
#define DCM_KI_ACC      0.005f
#define IMU_SMALL_ANGLE    15.0f   //满足水平状态的最小角度（单位deg）


uint32_t stateFlags = 0;

float imuAttitudeYaw;//范围：-180~180，用于上传到匿名上位机（支持范围-180~180）
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;//四元数
static float rMat[3][3];//四元数的旋转矩阵
static float smallAngleCosZ;//水平最小角余弦值

//旋转机体坐标系加速度到NEU坐标系
void imuTransformVectorBodyToEarth(Axis3f *v)
{
    const float x = rMat[0][0] * v->x + rMat[0][1] * v->y + rMat[0][2] * v->z;
    const float y = rMat[1][0] * v->x + rMat[1][1] * v->y + rMat[1][2] * v->z;
    const float z = rMat[2][0] * v->x + rMat[2][1] * v->y + rMat[2][2] * v->z;

    v->x = x;
    v->y = -y;//
    v->z = z;
}

//NEU坐标系加速度到NEU坐标系
void imuTransformVectorEarthToBody(Axis3f *v)
{
    v->y = -v->y;

    /* From earth frame to body frame */
    const float x = rMat[0][0] * v->x + rMat[1][0] * v->y + rMat[2][0] * v->z;
    const float y = rMat[0][1] * v->x + rMat[1][1] * v->y + rMat[2][1] * v->z;
    const float z = rMat[0][2] * v->x + rMat[1][2] * v->y + rMat[2][2] * v->z;

    v->x = x;
    v->y = y;
    v->z = z;
}


float degreesToRadians(int16_t degrees1)
{
    return degrees1 * RAD;
}


//计算四元数的旋转矩阵
static void imuComputeRotationMatrix(void)
{
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}


void imuInit(void)
{

    smallAngleCosZ = cos_approx(degreesToRadians(IMU_SMALL_ANGLE));//最小倾角余弦值
    imuComputeRotationMatrix();//计算四元数的旋转矩阵
}

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

//计算四元数和旋转矩阵
static void imuMahonyAHRSupdate(float gx, float gy, float gz,
                                float ax, float ay, float az,
                                float dt)
{
  static float integralAccX = 0.0f,  integralAccY = 0.0f, integralAccZ = 0.0f;    //加速度积分误差
  float ex, ey, ez;

    //计算旋转速率(rad/s)
    const float spin_rate_sq = sq(gx) + sq(gy) + sq(gz);

    //Step 1: Yaw correction
    //Step 2: Roll and pitch correction
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))//加速度计输出有效时,利用加速度计补偿陀螺仪
  {
    //单位化加速计测量值
    const float accRecipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= accRecipNorm;
    ay *= accRecipNorm;
    az *= accRecipNorm;

    //加速计读取的方向与重力加速计方向的差值，用向量叉乘计算
    ex = (ay * rMat[2][2] - az * rMat[2][1]);
    ey = (az * rMat[2][0] - ax * rMat[2][2]);
    ez = (ax * rMat[2][1] - ay * rMat[2][0]);

    //累计误差补偿
    if (DCM_KI_ACC > 0.0f) 
    {
      //如果旋转速率大于限制值则停止积分
      if (spin_rate_sq < sq(DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)))
      {
        /*误差累计，与积分常数相乘*/
        integralAccX += DCM_KI_ACC * ex * dt;
        integralAccY += DCM_KI_ACC * ey * dt;
        integralAccZ += DCM_KI_ACC * ez * dt;

        gx += integralAccX;
        gy += integralAccY;
        gz += integralAccZ;
      }
    }

    //误差补偿 用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量
    gx += DCM_KP_ACC * ex;
    gy += DCM_KP_ACC * ey;
    gz += DCM_KP_ACC * ez;
  }
  
  //一阶近似算法，四元数运动学方程的离散化形式和积分
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    const float qa = q0;
    const float qb = q1;
    const float qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

  //单位化四元数
    const float quatRecipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= quatRecipNorm;
    q1 *= quatRecipNorm;
    q2 *= quatRecipNorm;
    q3 *= quatRecipNorm;

    //计算四元数的旋转矩阵
    imuComputeRotationMatrix();
}

// https://github.com/Crashpilot1000/HarakiriWebstore1/blob/396715f73c6fcf859e0db0f34e12fe44bace6483/src/mw.c#L1292
// http://http.developer.nvidia.com/Cg/atan2.html (not working correctly!)
// Poly coefficients by @ledvinap (https://github.com/cleanflight/cleanflight/pull/1107)
// Max absolute error 0,000027 degree
float atan2_approx(float y, float x)
{
    #define atanPolyCoef1  3.14551665884836e-07f
    #define atanPolyCoef2  0.99997356613987f
    #define atanPolyCoef3  0.14744007058297684f
    #define atanPolyCoef4  0.3099814292351353f
    #define atanPolyCoef5  0.05030176425872175f
    #define atanPolyCoef6  0.1471039133652469f
    #define atanPolyCoef7  0.6444640676891548f

    float res, absX, absY;
    absX = fabsf(x);
    absY = fabsf(y);
    res  = MAX(absX, absY);
    if (res) res = MIN(absX, absY) / res;
    else res = 0.0f;
    res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) / ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);
    if (absY > absX) res = (M_PIf / 2.0f) - res;
    if (x < 0) res = M_PIf - res;
    if (y < 0) res = -res;
    return res;
}

// http://http.developer.nvidia.com/Cg/acos.html
// Handbook of Mathematical Functions
// M. Abramowitz and I.A. Stegun, Ed.
// Absolute error <= 6.7e-5
float acos_approx(float x)
{
    float xa = fabsf(x);
    float result = sqrtf(1.0f - xa) * (1.5707288f + xa * (-0.2121144f + xa * (0.0742610f + (-0.0187293f * xa))));
    if (x < 0.0f)
        return M_PIf - result;
    else
        return result;
}


//更新欧拉角
static void imuUpdateEulerAngles(attitude_t *attitude)
{
  attitude->roll = RADIANS_TO_DEGREES(atan2_approx(rMat[2][1], rMat[2][2]));
  attitude->pitch = RADIANS_TO_DEGREES((0.5f * M_PIf) - acos_approx(-rMat[2][0]));//arcsin = 0.5PI - arccos
  attitude->yaw = RADIANS_TO_DEGREES(atan2_approx(rMat[1][0], rMat[0][0]));

  imuAttitudeYaw = attitude->yaw;

  if (attitude->yaw < 0.0f)//转换位0~360
    //attitude->yaw += 360.0f;

  //更新最小倾角状态
  if (rMat[2][2] > smallAngleCosZ) 
    ENABLE_STATE(SMALL_ANGLE);
  else 
    DISABLE_STATE(SMALL_ANGLE);
    
}


//四元数和欧拉角计算
void imuUpdateAttitude(const sensorData_t  *sensorData, state_t *state, float dt)
{
	
	Axis3f gyro = sensorData->gyro;
	Axis3f acc  = sensorData->acc;
   


	//角速度单位由度转为弧度
	gyro.x = gyro.x * DEG2RAD;
	gyro.y = gyro.y * DEG2RAD;
	gyro.z = gyro.z * DEG2RAD;

	
  //计算四元数和旋转矩阵
  imuMahonyAHRSupdate(gyro.x, gyro.y, gyro.z,
                        acc.x, acc.y, acc.z,
                        dt);
	
    //计算欧拉角               
    imuUpdateEulerAngles(&state->attitude);
    
}
