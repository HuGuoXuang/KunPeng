#include "mpu6000.h"

//IMU IO
#define SCL   22//33//13
#define SDA   19//25//15
//#define ADO   27//0
//#define CS   23//2
//#define INTERRUPT_PIN   18


TwoWire I2Cone1 = TwoWire(1);

// an ICM42688 object with the ICM42688 sensor on I2C bus 0 with address 0x68
ICM42688 IMU(I2Cone1, 0x68);


Axis3i16 gyroADCRaw;	//陀螺仪原始AD数据
Axis3i16 gyroADC;		//校准的ADC数据
Axis3f gyrof;			//转换单位为°/s的数据

Axis3i16 accADCRaw;    //加速度原始AD数据
Axis3i16 accADC;    //校准后的AD数据      
Axis3f accf;      //转换单位为G的数据


gyroCalibration_t gyroCalibration; //陀螺仪校准结构体参数
accCalibration_t accCalibration;  //加速度校准结构体参数

biquadFilter_t gyroFilterLPF[3];//二阶低通滤波器
biquadFilter_t accFilterLPF[3];//二阶低通滤波器




TwoWire I2Cone = TwoWire(0);

uint8_t buffer[14];

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high



void ICM42688_Init(void)
{
  // serial to display data
  Serial.begin(500000);
  while(!Serial) {}

  // start communication with IMU
  int status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }


  IMU.setGyroNotchFilterFHz(1,ALL);//設置陷波頻率 1KH 
  IMU.setGyroNFbandwidth(1);//帶寬
  IMU.setGyroNotchFilter(true);//使能了NF 
  IMU.setAAFBandwidth(GYRO,6);//設置二階濾波器帶寬258hz
  IMU.setAAF(GYRO,true);//使能了AAF  
  IMU.setUIFilter(GYRO,2 ,1);// 3阶 Accel LPF的带宽=ODR/4  
  IMU.setODRAndFSR(/* who= */GYRO,/* ODR= */ODR_500HZ, /* FSR = */FSR_0);



  
  IMU.setAAFBandwidth(ACCEL,6);//設置二階濾波器帶寬258hz
  IMU.setAAF(ACCEL,true);//使能了AAF  
  IMU.setUIFilter(ACCEL,2 ,7);// 3阶 Accel LPF的带宽=OODR/40
  IMU.setODRAndFSR(/* who= */ACCEL,/* ODR= */ODR_500HZ, /* FSR = */FSR_1);
}


bool mpu6000IICWriteRegister(uint8_t reg, uint8_t data)
{
  uint8_t status = 0;
  I2Cone.beginTransmission(MPU6000_WHO_AM_I_CONST);   
  I2Cone.write(reg);  
  I2Cone.write(data);  
  status = I2Cone.endTransmission();   
  return status;
}

uint8_t mpu6000IICreadByte(uint8_t registerAddress)
{
    uint8_t readByte;                                       // byte to store data that is read
    I2Cone.beginTransmission(MPU6000_WHO_AM_I_CONST);           // begins comms with sensor specified
    I2Cone.write(registerAddress);                            // identifies register for data to be read from
    I2Cone.endTransmission();                                 // end transmission
    I2Cone.requestFrom(MPU6000_WHO_AM_I_CONST, uint8_t (1) );   // request 1 byte from the sensor address
    readByte = I2Cone.read();                                 // read data and store in the readByte variable
    return readByte;                                        // return the read data byte
}


void mpu6000IICReadRegister(uint8_t registerAddress, uint8_t nBytes, uint8_t * data)
{
    I2Cone.beginTransmission(MPU6000_WHO_AM_I_CONST);           // begins forming transmission to sensor
    I2Cone.write(registerAddress);                            // Add register address to transmission
    I2Cone.endTransmission();                                 
    I2Cone.requestFrom(MPU6000_WHO_AM_I_CONST, nBytes);         // Request and listen for response
    // Record response, wire will be available until nBytes are read
    int i = 0;
    while(I2Cone.available()){
        data[i] = I2Cone.read();
        i++;
    }
}


uint8_t MPU_Set_LPF(uint16_t lpf)//设置数字低通滤波器 
{
  uint8_t data=0;
  if(lpf>=188)data=1;
  else if(lpf>=98)data=2;
  else if(lpf>=42)data=3;
  else if(lpf>=20)data=4;
  else if(lpf>=10)data=5;
  else data=6; 
  return mpu6000IICWriteRegister(MPU_RA_CONFIG, data);//设置数字低通滤波器  
}
 
uint8_t MPU_Set_Rate(uint16_t rate)//采样频率分频器
{
  uint8_t data;
  if(rate>1000)rate=1000;
  if(rate<4)rate=4;
  data=1000/rate-1;
  mpu6000IICWriteRegister(MPU_RA_SMPLRT_DIV, data);  //采样频率分频器
  delay(15);
  return MPU_Set_LPF(rate/2);  //自动设置LPF为采样率的一半
}
 

bool mpu6000Init(void)
{
  uint8_t status = 0;
 // pinMode(ADO, OUTPUT);
  //pinMode(CS, OUTPUT);
  //digitalWrite(CS, HIGH); 
  //digitalWrite(ADO, LOW); 
  
  //pinMode(INTERRUPT_PIN, INPUT);
  //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

  I2Cone.begin(SDA,SCL); 
  I2Cone.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties   

  //复位MPU6000
  status = mpu6000IICWriteRegister(MPU_RA_PWR_MGMT_1, BIT_H_RESET);
  PcSerial.println(status);
  delay(50);
  status = mpu6000IICWriteRegister(MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
  PcSerial.println(status);
  delay(50);
  status = mpu6000IICWriteRegister(MPU_RA_PWR_MGMT_1, BIT_H_RESET);//复位两次增加传感器稳定性
  PcSerial.println(status);
  delay(50);
  status = mpu6000IICWriteRegister(MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
  PcSerial.println(status);
  delay(50);

  //读取ID
  uint8_t id = mpu6000IICreadByte(MPU_RA_WHO_AM_I);
  PcSerial.print(" ID:");
  PcSerial.println(id);
  //读取正常，初始化
  if(id == MPU6000_WHO_AM_I_CONST)
  {
    //设置X轴陀螺作为时钟 
    mpu6000IICWriteRegister(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROY);
    delay(15);
    
    //禁止I2C接口
    // mpu6000IICWriteRegister(MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
    //delay(15);
    mpu6000IICWriteRegister(MPU_RA_PWR_MGMT_2, 0x00);
    delay(15);
    
    //设置陀螺仪 +/- 2000 DPS量程
    mpu6000IICWriteRegister(MPU_RA_GYRO_CONFIG, FSR_2000DPS << 3);
    delay(15);
    
    //设置加速度 +/- 8 G 量程
    mpu6000IICWriteRegister(MPU_RA_ACCEL_CONFIG, FSR_8G << 3);
    delay(15);
    
    //设置中断引脚功能
    //mpu6000IICWriteRegister(MPU_RA_INT_PIN_CFG, 1 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);//中断引脚配置
    //delay(15);
    //mpu6000IICWriteRegister(MPU_RA_INT_ENABLE, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);//中断
    delay(15);

    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    //mpu6000IICWriteRegister(MPU_RA_SMPLRT_DIV, 0);//设置采样率
    //delay(15);
    
    //设置低通滤波带宽
    //mpu6000IICWriteRegister(MPU_RA_CONFIG, BITS_DLPF_CFG_98HZ);
    //delay(1);        

    MPU_Set_Rate(500);//采样频率分频器和滤波

    delay(1);       
    return true;
  }
  else
  {
    PcSerial.println("Undetected device");
    return false;
  }
  
}


bool mpu6000GyroRead(Axis3i16* gyroRaw)
{
  uint8_t buffer[6];
  mpu6000IICReadRegister(MPU_RA_GYRO_XOUT_H, 6, buffer);
  gyroRaw->x = (((int16_t) buffer[0]) << 8) | buffer[1];
  gyroRaw->y = (((int16_t) buffer[2]) << 8) | buffer[3];
  gyroRaw->z = (((int16_t) buffer[4]) << 8) | buffer[5];
  return true;
}

bool mpu6000AccRead(Axis3i16* accRaw)
{
  uint8_t buffer[6];
  mpu6000IICReadRegister(MPU_RA_ACCEL_XOUT_H, 6, buffer);
  accRaw->x = (((int16_t) buffer[0]) << 8) | buffer[1];
  accRaw->y = (((int16_t) buffer[2]) << 8) | buffer[3];
  accRaw->z = (((int16_t) buffer[4]) << 8) | buffer[5];
  return true;
}


void applySensorAlignment(int16_t * dest, int16_t * src, uint8_t rotation)
{
    const int16_t x = src[X];
    const int16_t y = src[Y];
    const int16_t z = src[Z];

    switch (rotation) {
    default:
    case CW0_DEG:
        dest[X] = x;
        dest[Y] = y;
        dest[Z] = z;
        break;
    case CW90_DEG:
        dest[X] = y;
        dest[Y] = -x;
        dest[Z] = z;
        break;
    case CW180_DEG:
        dest[X] = -x;
        dest[Y] = -y;
        dest[Z] = z;
        break;
    case CW270_DEG:
        dest[X] = -y;
        dest[Y] = x;
        dest[Z] = z;
        break;
    case CW0_DEG_FLIP:
        dest[X] = -x;
        dest[Y] = y;
        dest[Z] = -z;
        break;
    case CW90_DEG_FLIP:
        dest[X] = y;
        dest[Y] = x;
        dest[Z] = -z;
        break;
    case CW180_DEG_FLIP:
        dest[X] = x;
        dest[Y] = -y;
        dest[Z] = -z;
        break;
    case CW270_DEG_FLIP:
        dest[X] = -y;
        dest[Y] = -x;
        dest[Z] = -z;
        break;
    }
}




void gyroSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    gyroCalibration.cycleCount = calibrationCyclesRequired;
}

void performGyroCalibration(Axis3i16 gyroADCSample)
{
    for (int axis = 0; axis < 3; axis++) 
    {
        //PcSerial.print(" ok1:"); 
        //PcSerial.println(gyroCalibration.cycleCount);
        //复位数据
        if (gyroCalibration.cycleCount == CALIBRATING_GYRO_CYCLES) 
        {
            gyroCalibration.gyroSum.axis[axis] = 0;
            devClear(&gyroCalibration.var[axis]);
            
        }

        //陀螺数据累加
        gyroCalibration.gyroSum.axis[axis] += gyroADCSample.axis[axis];
        devPush(&gyroCalibration.var[axis], gyroADCSample.axis[axis]);

        //复位零偏 
        gyroCalibration.gyroZero.axis[axis] = 0;

        if (gyroCalibration.cycleCount == 1) 
        {
            const float stddev = devStandardDeviation(&gyroCalibration.var[axis]);
            //检测方差值是否大于陀螺仪受到移动的阈值
            //如果大于设定阈值则返回重新校准
            PcSerial.print(" axis:");  
            PcSerial.print(axis); 
            PcSerial.print(" stddev:");  
            PcSerial.println(stddev);   
            if ((stddev > GYRO_CALIBRATION_THRESHOLD) || (stddev == 0)) 
            {
                gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);//CALIBRATING_GYRO_CYCLES = 1000
                gyroCalibration.ok = 1;
                return;
            }
      
            //校准完成
            gyroCalibration.gyroZero.axis[axis] = (gyroCalibration.gyroSum.axis[axis] + (CALIBRATING_GYRO_CYCLES / 2)) / CALIBRATING_GYRO_CYCLES;
            gyroCalibration.ok = 0;
            EEPROM_write(4);
        }
    }
  
    gyroCalibration.cycleCount--;
}

void gyroUpdate(Axis3f *gyro,uint8_t sw)
{
    if(sw==0)
    {
        //读取原始数据
        mpu6000GyroRead(&gyroADCRaw);
      
        
        //传感器数据方向对齐
        applySensorAlignment(gyroADCRaw.axis, gyroADCRaw.axis, CW90_DEG);
        //陀螺仪校准
        if (gyroCalibration.ok==1) 
        {
          performGyroCalibration(gyroADCRaw);
          
          gyrof.x = 0.0f;
          gyrof.y = 0.0f;
          gyrof.z = 0.0f;
          *gyro = gyrof;
          return;
        }
      
        //计算gyroADC值，减去零偏
        gyroADC.x = gyroADCRaw.x - gyroCalibration.gyroZero.x;
        gyroADC.y = gyroADCRaw.y - gyroCalibration.gyroZero.y;
        gyroADC.z = gyroADCRaw.z - gyroCalibration.gyroZero.z; 
        
        //板对齐
        // applyBoardAlignment(gyroADC.axis);
        
        //转换为单位 °/s 
        gyrof.x = (float)gyroADC.x / GYRO_SCALE;
        gyrof.y = (float)gyroADC.y / GYRO_SCALE;
        gyrof.z = (float)gyroADC.z / GYRO_SCALE;
        
        //软件二阶低通滤波
        for (int axis = 0; axis < 3; axis++) 
        {
          gyrof.axis[axis] = biquadFilterApply(&gyroFilterLPF[axis], gyrof.axis[axis]);
        }   
        
        *gyro = gyrof;      
    }
    else if(sw==1)
    {

        IMU.getAGT();
        //读取原始数据
        gyroADCRaw.x= IMU.gyrX1();
        gyroADCRaw.y= IMU.gyrY1();
        gyroADCRaw.z= IMU.gyrZ1();
        //传感器数据方向对齐
        applySensorAlignment(gyroADCRaw.axis, gyroADCRaw.axis, CW180_DEG);
        //陀螺仪校准
        if (gyroCalibration.ok==1) 
        {
          performGyroCalibration(gyroADCRaw);
          
          gyrof.x = 0.0f;
          gyrof.y = 0.0f;
          gyrof.z = 0.0f;
          *gyro = gyrof;
          return;
        }
      
        //计算gyroADC值，减去零偏
        gyroADC.x = gyroADCRaw.x - gyroCalibration.gyroZero.x;
        gyroADC.y = gyroADCRaw.y - gyroCalibration.gyroZero.y;
        gyroADC.z = gyroADCRaw.z - gyroCalibration.gyroZero.z; 
        
        //板对齐
        // applyBoardAlignment(gyroADC.axis);
        
        //转换为单位 °/s 
        gyrof.x = (float)gyroADC.x / GYRO_SCALE;
        gyrof.y = (float)gyroADC.y / GYRO_SCALE;
        gyrof.z = (float)gyroADC.z / GYRO_SCALE;
        
        //软件二阶低通滤波
        for (int axis = 0; axis < 3; axis++) 
        {
          gyrof.axis[axis] = biquadFilterApply(&gyroFilterLPF[axis], gyrof.axis[axis]);
        }   
        
        *gyro = gyrof;      
    }                  
}

bool gyroInit(float gyroUpdateRate)//
{
  //初始化陀螺仪零偏校准
  gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
  gyroCalibration.ok = 0;// 1 校准陀螺仪
  //初始化二阶低通滤波
  for (int axis = 0; axis < 3; axis++)
  {
    biquadFilterInitLPF(&gyroFilterLPF[axis],GYRO_LPF_CUTOFF_FREQ, gyroUpdateRate);
    
  }
  return true;
}



//执行校准
static void performAcclerationCalibration(Axis3i16 accADCSample)
{
    for (int axis = 0; axis < 3; axis++) 
    {
        //复位数据
        if (accCalibration.cycleCount == CALIBRATING_ACC_CYCLES) 
        {
            accCalibration.accSum.axis[axis] = 0;
            devClear(&accCalibration.var[axis]);
            
        }

        //陀螺数据累加
        accCalibration.accSum.axis[axis] += accADCSample.axis[axis];
        
        //复位零偏 
        accCalibration.accZero.axis[axis] = 0;

        if (accCalibration.cycleCount == 1) 
        {    
            //校准完成
            accCalibration.accZero.axis[axis] = (accCalibration.accSum.axis[axis] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
            if(axis==2)
            accCalibration.accZero.axis[axis] -= (int)ACC_1G_ADC;
            accCalibration.ok = 0;   
            EEPROM_write(3);
        }
    }
    accCalibration.cycleCount--;
    
}



void accUpdate(Axis3f *acc, uint8_t sw)
{

  if(sw==0)
  {
  
      //读取原始数据
      mpu6000AccRead(&accADCRaw);
    
      
      //传感器数据方向对齐
      applySensorAlignment(accADCRaw.axis, accADCRaw.axis, CW90_DEG);
      
      //加速度计校准
      if (accCalibration.ok==1) 
      {
        performAcclerationCalibration(accADCRaw);
        return;
      }
      
      //计算accADC值，减去零偏
      accADC.x = accADCRaw.x - accCalibration.accZero.x;
      accADC.y = accADCRaw.y - accCalibration.accZero.y;
      accADC.z = accADCRaw.z - accCalibration.accZero.z;
      
      ////板对齐
      //applyBoardAlignment(accADC.axis);
      
      //转换为单位 g (9.8m/s^2)
      accf.x = (float)accADC.x / ACC_1G_ADC;
      accf.y = (float)accADC.y / ACC_1G_ADC;
      accf.z = (float)accADC.z / ACC_1G_ADC;
      
      //软件低通滤波
      for (int axis = 0; axis < 3; axis++) 
      {
        accf.axis[axis] = biquadFilterApply(&accFilterLPF[axis], accf.axis[axis]);
      }
      
      *acc = accf; 

  }
  else if(sw==1)
  {
  
      //读取原始数据
      accADCRaw.x= IMU.accX1();
      accADCRaw.y= IMU.accY1();
      accADCRaw.z= IMU.accZ1();  
      
      //传感器数据方向对齐
      applySensorAlignment(accADCRaw.axis, accADCRaw.axis, CW180_DEG);
      
      //加速度计校准
      if (accCalibration.ok==1) 
      {
        performAcclerationCalibration(accADCRaw);
        return;
      }
      
      //计算accADC值，减去零偏
      accADC.x = accADCRaw.x - accCalibration.accZero.x;
      accADC.y = accADCRaw.y - accCalibration.accZero.y;
      accADC.z = accADCRaw.z - accCalibration.accZero.z;
      
      ////板对齐
      //applyBoardAlignment(accADC.axis);
      
      //转换为单位 g (9.8m/s^2)
      accf.x = (float)accADC.x / ACC_1G_ADC;
      accf.y = (float)accADC.y / ACC_1G_ADC;
      accf.z = (float)accADC.z / ACC_1G_ADC;
      
      //软件低通滤波
      for (int axis = 0; axis < 3; axis++) 
      {
        accf.axis[axis] = biquadFilterApply(&accFilterLPF[axis], accf.axis[axis]);
      }
      
      *acc = accf; 

  }
   
}

void accSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    accCalibration.cycleCount = calibrationCyclesRequired;
}

bool accInit(float accUpdateRate)
{

  //初始化软件二阶低通滤波
  for (int axis = 0; axis < 3; axis++)
  {
    biquadFilterInitLPF(&accFilterLPF[axis], ACCEL_LPF_CUTOFF_FREQ , accUpdateRate);
   
  }
  accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
  accCalibration.ok = 0;
  
  return true;
}
