#include "pos_estimator.h"

//static float wBaro = 0.35f;      /*气压校正权重*/
//static float wOpflowP = 1.0f;   /*光流位置校正权重*/
//static float wOpflowV = 2.0f;   /*光流速度校正权重*/
//static float wAccBias = 0.01f;    /*加速度校正权重*/


float  W_XY_Light_P = 1.0f;    //光流位置修正权重
float  W_XY_Light_V = 2.0f;   //光流速度校正权重
float  W_Z_CSB_P = 1.0f;    //超声位置波修正权重
float  W_Z_BARO_P = 0.9f;    //气压修正权重
float  W_ACC_BIAS = 0.01f;   //加速度计修正权重


float accpos[3]={0,0,0};

biquadFilter_t AccNotchingFilter[3];//陷波濾波器
biquadFilter_t BiquadPosAcc[3];//二阶低通滤波器
GravityCMSS_t calibratedGravity;

posEstimator_t posLight;
posEstimator_t posUltrasonic;
posEstimator_t posEstimator;
int csbok = 1;

void Gravityinit(void)
{
  calibratedGravity.CMSS.z = GRAVITY_CMSS;
  calibratedGravity.CMSS.x = 0;
  calibratedGravity.CMSS.y = 0;
}

void initPosUltrasonic(void)
{
    for (int axis = 0; axis < 3; axis++) 
    {

        posUltrasonic.imu.accelBias.axis[axis] = 0;//偏差
        posUltrasonic.est.pos.axis[axis] = 0;
        posUltrasonic.est.vel.axis[axis] = 0;
    }  
}

//预估器初始化
static void initializePositionEstimator(void)
{
    if(calibratedGravity.CMSSFlashSign==333)
    {
       posEstimator.imu.gravityCalibrationComplete = 2;      
    }
    else
    {
       Gravityinit();    
    }
            
    initPosUltrasonic();
    for (int axis = 0; axis < 3; axis++) 
    {

        posLight.imu.accelBias.axis[axis] = 0;//偏差
        posLight.est.pos.axis[axis] = 0;
        posLight.est.vel.axis[axis] = 0;

      
        posEstimator.imu.accelBias.axis[axis] = 0;//偏差
        posEstimator.est.pos.axis[axis] = 0;
        posEstimator.est.vel.axis[axis] = 0;
        state.Velocity.x = 0;
        state.Velocity.y = 0;
        state.Velocity.z = 0;
        state.Position.x = 0;
        state.Position.y = 0;
        state.Position.z = 0;
    }
}

int32_t applyDeadband(int32_t value, int32_t deadband)
{
    if (abs(value) < deadband) 
    {
        value = 0;
    } else if (value > 0) 
    {
        value -= deadband;
    } else if (value < 0) 
    {
        value += deadband;
    }
    return value;
}

//更新NEU坐标系的加速度
//accBF：为机体坐标系加速度
static void updateIMUTopic(const Axis3f accBF)
{
  static u32 gravityCalibrationTimeout = 0;
  Axis3f accelCMSS;

  //机体坐标系的加速度转为（cm/ss）
  accelCMSS.x = accBF.x * GRAVITY_CMSS;
  accelCMSS.y = accBF.y * GRAVITY_CMSS;
  accelCMSS.z = accBF.z * GRAVITY_CMSS;

  //去除加速度偏置
  accelCMSS.x -= posEstimator.imu.accelBias.x;
  accelCMSS.y -= posEstimator.imu.accelBias.y;
  accelCMSS.z -= posEstimator.imu.accelBias.z;

  //旋转机体坐标系加速度到NEU坐标系
  imuTransformVectorBodyToEarth(&accelCMSS);
  
  //在水平状态校准重力方向的加速度零偏
  if (posEstimator.imu.gravityCalibrationComplete == 1 ) //(!posEstimator.imu.gravityCalibrationComplete && STATE(SMALL_ANGLE)) 
  {
    //慢慢收敛校准重力零偏
    const float gravityOffsetError = accelCMSS.z - calibratedGravity.CMSS.z;
    const float gravityOffsetErrory = accelCMSS.y - calibratedGravity.CMSS.y;
    const float gravityOffsetErrorx = accelCMSS.x - calibratedGravity.CMSS.x;
    
    calibratedGravity.CMSS.z += gravityOffsetError * 0.0025f;
    calibratedGravity.CMSS.x += gravityOffsetErrorx * 0.0025f;
    calibratedGravity.CMSS.y += gravityOffsetErrory * 0.0025f;



    Serial.print(" XOffsetError:");
    Serial.print(abs(gravityOffsetErrorx));
    Serial.print(" YOffsetError:");
    Serial.print(abs(gravityOffsetErrory));
    Serial.print(" ZOffsetError:");
    Serial.println(abs(gravityOffsetError));
    if ((abs(gravityOffsetError)<5)&&(abs(gravityOffsetErrorx)<5)&&(abs(gravityOffsetErrory)<5))//误差要小于5cm/ss
    {
      if ((millis() - gravityCalibrationTimeout) > 250) 
      {
        posEstimator.imu.gravityCalibrationComplete = 2;
        Serial.println(calibratedGravity.CMSS.z);
        Serial.println(calibratedGravity.CMSS.y);
        Serial.println(calibratedGravity.CMSS.x);
        Serial.print("posEstimator.imu.gravityCalibrationComplete ok:");
        EEPROM_write(5);
      }
    }
    else 
    {
      gravityCalibrationTimeout = millis();
    }
  }

  //NEU坐标系加速度处理
  if (posEstimator.imu.gravityCalibrationComplete == 2) //重力是否校准完成
  {
    accelCMSS.z -= calibratedGravity.CMSS.z;//去除重力   
    accelCMSS.y -= calibratedGravity.CMSS.y;
    accelCMSS.x -= calibratedGravity.CMSS.x;    
     
    for (int axis = 0; axis < 3; axis++)
    {
      accelCMSS.axis[axis] = applyDeadband(accelCMSS.axis[axis], 4);//去除4(cm/ss)死区
      posEstimator.imu.accelNEU.axis[axis] += (accelCMSS.axis[axis] - posEstimator.imu.accelNEU.axis[axis]) * 0.3f;//一阶低通

      posUltrasonic.imu.accelNEU.axis[axis] += (accelCMSS.axis[axis] - posUltrasonic.imu.accelNEU.axis[axis]) * 0.3f;//一阶低通
      
    } 
    //posEstimator.imu.accelNEU.z = biquadFilterApply(&BiquadPosAcc[Z], posEstimator.imu.accelNEU.z);  
  }
  else 
  {
    posEstimator.imu.accelNEU.x = 0;
    posEstimator.imu.accelNEU.y = 0;
    posEstimator.imu.accelNEU.z = 0;

    posUltrasonic.imu.accelNEU.x = 0;
    posUltrasonic.imu.accelNEU.y = 0;
    posUltrasonic.imu.accelNEU.z = 0;
     
    
  }
}


//加速度预估位移和速度
static void posAndVelocityPredict(int axis, float dt, float acc)
{
    posEstimator.est.pos.axis[axis] += posEstimator.est.vel.axis[axis] * dt + acc * dt * dt / 2.0f;
    posEstimator.est.vel.axis[axis] += acc * dt;
}

//加速度预估位移和速度
static void posAndVelocityPredict1(int axis, float dt, float acc)
{
    posUltrasonic.est.pos.axis[axis] += posUltrasonic.est.vel.axis[axis] * dt + acc * dt * dt / 2.0f;
    posUltrasonic.est.vel.axis[axis] += acc * dt;
}


//加速度预估位移和速度
static void posAndVelocityPredict2(int axis, float dt, float acc)
{
    posLight.est.pos.axis[axis] += posLight.est.vel.axis[axis] * dt + acc * dt * dt / 2.0f;
    posLight.est.vel.axis[axis] += acc * dt;
}


//误差修正预估的位移和速度
static void posAndVelocityCorrect(int axis, float dt, float e, float w)
{
    float ewdt = e * w * dt;
    posEstimator.est.pos.axis[axis] += ewdt;
    posEstimator.est.vel.axis[axis] += w * ewdt;
}

//误差修正预估的位移和速度
static void posAndVelocityCorrect1(int axis, float dt, float e, float w)
{
    float ewdt = e * w * dt;
    posUltrasonic.est.pos.axis[axis] += ewdt;
    posUltrasonic.est.vel.axis[axis] += w * ewdt;
}

//误差修正预估的位移和速度
static void posAndVelocityCorrect2(int axis, float dt, float e, float w)
{
    float ewdt = e * w * dt;
    posLight.est.pos.axis[axis] += ewdt;
    posLight.est.vel.axis[axis] += w * ewdt;
}

/*速度校正*/
static void inavFilterCorrectVel2(int axis, float dt, float e, float w)
{
   posLight.est.vel.axis[axis] += e * w * dt;
}




//速度预估和位置预估
static void updateEstimatedTopic(float dt)
{
  static int sw = 0;
  //使用加速度预估位移和速度
  posAndVelocityPredict(0, dt, posEstimator.imu.accelNEU.x);
  posAndVelocityPredict(1, dt, posEstimator.imu.accelNEU.y);
  posAndVelocityPredict(2, dt, posEstimator.imu.accelNEU.z);


  posAndVelocityPredict1(0, dt, posUltrasonic.imu.accelNEU.x);
  posAndVelocityPredict1(1, dt, posUltrasonic.imu.accelNEU.y);
  posAndVelocityPredict1(2, dt, posUltrasonic.imu.accelNEU.z);  

  //加速度偏置值
  const bool updateAccBias = (W_ACC_BIAS > 0);//加速度计修正权重
  Axis3f accelBiasCorr = { { 0, 0, 0} };//加速度偏差校正


  //使用气压计高度误差修正预估的位移和速度（Z轴）
  const float baroResidual =   baro.F_alt - posEstimator.est.pos.z;//两个传感器的高度差 
  posAndVelocityCorrect(Z, dt, baroResidual, W_Z_BARO_P);//W_Z_BARO_P 气压修正权重 误差修正预估的位移和速度


  //使用超声波计高度误差修正预估的位移和速度（Z轴）
  static int ultrasonic_recover = 0;
  if(ultrasonic.hz >=8)
  {
      ultrasonic_recover++;
      if(ultrasonic_recover>=2)
      {
          ultrasonic_recover = 2;
          const float baroResidual1 =   ultrasonic.begin_cm - posUltrasonic.est.pos.z;//两个传感器的高度差 
          posAndVelocityCorrect1(Z, dt, baroResidual1, W_Z_BARO_P);// 气压修正权重 误差修正预估的位移和速度           
      }
      if(ultrasonic_recover<2)
      {
          posUltrasonic.est.pos.axis[Z] = ultrasonic.begin_cm;
          posUltrasonic.est.vel.axis[Z] = posEstimator.est.vel.axis[Z];         
      }
  }
  else
  {      
      ultrasonic_recover = 0;  
      posUltrasonic.est.pos.axis[Z] = posEstimator.est.pos.axis[Z]; 
      posUltrasonic.est.vel.axis[Z] = posEstimator.est.vel.axis[Z];          
  }


  //是否要修正加速度计
  if (updateAccBias) 
  {
    accelBiasCorr.z -= baroResidual * sq(W_Z_BARO_P); //加速度偏差校正值
  }
  
  //修正加速度偏置值
  if (updateAccBias)//是否要修正 
  {
        const float accelBiasCorrMagnitudeSq = sq(accelBiasCorr.x) + sq(accelBiasCorr.y) + sq(accelBiasCorr.z);//加速度偏差校准值的平方和
        if (accelBiasCorrMagnitudeSq < sq(GRAVITY_CMSS * 0.25f))//偏置小于0.25G时可以修正 
        {
            //将加速度偏置值由世界坐标系转换为机体坐标系
            imuTransformVectorEarthToBody(&accelBiasCorr);
      
            posEstimator.imu.accelBias.x += accelBiasCorr.x * W_ACC_BIAS * dt;
            posEstimator.imu.accelBias.y += accelBiasCorr.y * W_ACC_BIAS * dt;
            posEstimator.imu.accelBias.z += accelBiasCorr.z * W_ACC_BIAS * dt;
        }
  }

  if(accCalibration.accZeroFlashSign==123)//加速度計校準后執行
      updateEstimatedTopic1(dt);
  
}

void updateEstimatedTopic1(float dt)
{
    //重力加速度分量
    float gax = -sin_approx(DEGREES_TO_RADIANS(state.attitude.pitch));//ax
    float gay = sin_approx(DEGREES_TO_RADIANS(state.attitude.roll));//ay 

    //機體軸向加速度
    float jax = (sensorData.acc.x - gax)*GRAVITY_CMSS;
    float jay = (sensorData.acc.y - gay)*GRAVITY_CMSS;  

    //機體水平面加速度
    float pax = cos_approx(DEGREES_TO_RADIANS(state.attitude.pitch))*jax;
    float pay = cos_approx(DEGREES_TO_RADIANS(state.attitude.roll))*jay;

    
    //pax = biquadFilterApply(&AccNotchingFilter[X], pax);
    //pay = biquadFilterApply(&AccNotchingFilter[Y], pay);
   
    pax = applyDeadband(pax, 4);//去除4(cm/ss)死区
    pay = applyDeadband(pay, 4);//去除4(cm/ss)死区

    posLight.imu.accelNEU.x += (pax - posLight.imu.accelNEU.x) * 0.3f;//一阶低通
    posLight.imu.accelNEU.y += (pay - posLight.imu.accelNEU.y) * 0.3f;//一阶低通

    //posLight.imu.accelNEU.x = constrain(posLight.imu.accelNEU.x, -80, 80);
    //posLight.imu.accelNEU.y = constrain(posLight.imu.accelNEU.y, -80, 80);
    //PcSerial.print(" acc_x:");  
    //PcSerial.print(pax);  
    //PcSerial.print(" acc_y:");  
    //PcSerial.println(pay); 
    //PcSerial.print(" facc_x:");  
    //PcSerial.print(posLight.imu.accelNEU.x); 
    
    //posLight.imu.accelNEU.x = biquadFilterApply(&BiquadPosAcc[X], pax);
    //posLight.imu.accelNEU.y = biquadFilterApply(&BiquadPosAcc[Y], pay);
     
    //PcSerial.print(" bacc_x:");  
    //PcSerial.println(posLight.imu.accelNEU.x); 

 

    //使用加速度预估位移和速度
    posAndVelocityPredict2(0, dt, posLight.imu.accelNEU.x);
    posAndVelocityPredict2(1, dt, posLight.imu.accelNEU.y);

    float opResidualX = Light.sum_flow_y - posLight.est.pos.x;//
    float opResidualY = Light.sum_flow_x - posLight.est.pos.y;//
    float opResidualXVel = Light.speed_y - posLight.est.vel.x;//
    float opResidualYVel = Light.speed_x - posLight.est.vel.y;//

    
    posAndVelocityCorrect2(X, dt, opResidualX, W_XY_Light_P);//修正权重 
    posAndVelocityCorrect2(Y, dt, opResidualY, W_XY_Light_P);//修正权重 
    
    inavFilterCorrectVel2(X, dt, opResidualXVel, W_XY_Light_V);
    inavFilterCorrectVel2(Y, dt, opResidualYVel, W_XY_Light_V);
    

    accpos[X] = posLight.est.vel.x;//applyDeadband(posLight.est.vel.x, 4);//去除4(cm/s)死区
    accpos[Y] = posLight.est.vel.y;//applyDeadband(posLight.est.vel.y, 4);//去除4(cm/s)死区
 
}




//发布预估位置和速度
static void publishEstimatedTopic(state_t *state)
{
  static u32 publishTime;
  
  //更新世界坐标系的加速度
  state->acc.x = posEstimator.imu.accelNEU.x;
  state->acc.y = posEstimator.imu.accelNEU.y;
  state->acc.z = posEstimator.imu.accelNEU.z;
  
  //更新估计的位置和速度（10ms->100Hz）
  if ((millis() - publishTime) >= 2)
  {
    state->Position.x = posEstimator.est.pos.x;
    state->Position.y = posEstimator.est.pos.y;
    state->Position.z = posEstimator.est.pos.z;
    
    state->Velocity.x = posEstimator.est.vel.x;
    state->Velocity.y = posEstimator.est.vel.y;
    state->Velocity.z = constrainf(posEstimator.est.vel.z, -150.0f, 150.0f);//限制Z轴的速度为150cm/s
    
    publishTime = millis();
  }
}


//更新预估器，由主循环调用
void updatePositionEstimator(const sensorData_t *sensorData, state_t *state, float dt)
{
  static bool isInitialized = false;
  if(isInitialized == false) 
  {

      //二阶陷波器初始化(结构体变量 采样频率 中心频率 截止频率)
      biquadFilterInitNotch(&AccNotchingFilter[X], 500, 80, 5);
      biquadFilterInitNotch(&AccNotchingFilter[Y], 500, 80, 5);
      biquadFilterInitNotch(&AccNotchingFilter[Z], 500, 80, 5);
      
      biquadFilterInitLPF(&BiquadPosAcc[X],20, 500);
      biquadFilterInitLPF(&BiquadPosAcc[Y],20, 500);
      biquadFilterInitLPF(&BiquadPosAcc[Z],20, 500);   
      PcSerial.println(" acc ok:");   
  }
  
  //初始化预估器
  if ((isInitialized == false)||(unlock_motor == 0))  //
  {
      initializePositionEstimator();//清零赋值
      isInitialized = true;

  }


    //更新预估器的加速度（世界坐标系）
    updateIMUTopic(sensorData->acc);/*!< 加速度重力 G */
  
    //预估速度和位置
    updateEstimatedTopic(dt);

    //发布预估的速度和位置
    publishEstimatedTopic(state);


}
