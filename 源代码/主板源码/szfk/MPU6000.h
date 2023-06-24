#ifndef __MPU6000_H
#define __MPU6000_H
#include<Wire.h> 
#include<Arduino.h> 
#include "SENSORS_TYPES.h"
#include "maths.h"
#include "filter.h"
#include "flash_memory.h"
#include "SerialDat.h"
#include "ICM42688.h"
/*传感器读取更新频率HZ*/
#define GYRO_UPDATE_RATE    500
#define ACC_UPDATE_RATE     500

//校准采样次数
#define CALIBRATING_ACC_CYCLES          100
#define CALIBRATING_GYRO_CYCLES         100

/*陀螺仪校准阈值*/
#define GYRO_CALIBRATION_THRESHOLD  15.0f


/*低通滤波参数*/
#define GYRO_LPF_CUTOFF_FREQ    200.0f

//mpu6000初始化陀螺仪量程为FSR_2000DPS，即
#define GYRO_SCALE  16.4f
/*传感器对齐定义的机体坐标系的安装方向*/
#define GYRO_ALIGN    CW180_DEG  //CW90_DEG

//软件二阶低通滤波参数（单位Hz）
#define ACCEL_LPF_CUTOFF_FREQ   200.0f

//传感器对齐定义的机体坐标系的安装方向
#define ACCEL_ALIGN   CW180_DEG  //CW90_DEG

//mpu6000初始化加速度量程为FSR_8G，ACC_1G_ADC = 65536 / (2 * 8) = 4096
#define ACC_1G_ADC  4096.0f 

#define MPU6000_WHO_AM_I_CONST 0x68


// RA = Register Address
#define MPU_RA_XG_OFFS_TC       0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_YG_OFFS_TC       0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_ZG_OFFS_TC       0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_X_FINE_GAIN      0x03    //[7:0] X_FINE_GAIN
#define MPU_RA_Y_FINE_GAIN      0x04    //[7:0] Y_FINE_GAIN
#define MPU_RA_Z_FINE_GAIN      0x05    //[7:0] Z_FINE_GAIN
#define MPU_RA_XA_OFFS_H        0x06    //[15:0] XA_OFFS
#define MPU_RA_XA_OFFS_L_TC     0x07
#define MPU_RA_YA_OFFS_H        0x08    //[15:0] YA_OFFS
#define MPU_RA_YA_OFFS_L_TC     0x09
#define MPU_RA_ZA_OFFS_H        0x0A    //[15:0] ZA_OFFS
#define MPU_RA_ZA_OFFS_L_TC     0x0B
#define MPU_RA_PRODUCT_ID       0x0C    // Product ID Register
#define MPU_RA_XG_OFFS_USRH     0x13    //[15:0] XG_OFFS_USR
#define MPU_RA_XG_OFFS_USRL     0x14
#define MPU_RA_YG_OFFS_USRH     0x15    //[15:0] YG_OFFS_USR
#define MPU_RA_YG_OFFS_USRL     0x16
#define MPU_RA_ZG_OFFS_USRH     0x17    //[15:0] ZG_OFFS_USR
#define MPU_RA_ZG_OFFS_USRL     0x18
#define MPU_RA_SMPLRT_DIV       0x19   //采样频率分频器
#define MPU_RA_CONFIG           0x1A   //配置寄存器
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_FF_THR           0x1D
#define MPU_RA_FF_DUR           0x1E
#define MPU_RA_MOT_THR          0x1F
#define MPU_RA_MOT_DUR          0x20
#define MPU_RA_ZRMOT_THR        0x21
#define MPU_RA_ZRMOT_DUR        0x22
#define MPU_RA_FIFO_EN          0x23
#define MPU_RA_I2C_MST_CTRL     0x24
#define MPU_RA_I2C_SLV0_ADDR    0x25
#define MPU_RA_I2C_SLV0_REG     0x26
#define MPU_RA_I2C_SLV0_CTRL    0x27
#define MPU_RA_I2C_SLV1_ADDR    0x28
#define MPU_RA_I2C_SLV1_REG     0x29
#define MPU_RA_I2C_SLV1_CTRL    0x2A
#define MPU_RA_I2C_SLV2_ADDR    0x2B
#define MPU_RA_I2C_SLV2_REG     0x2C
#define MPU_RA_I2C_SLV2_CTRL    0x2D
#define MPU_RA_I2C_SLV3_ADDR    0x2E
#define MPU_RA_I2C_SLV3_REG     0x2F
#define MPU_RA_I2C_SLV3_CTRL    0x30
#define MPU_RA_I2C_SLV4_ADDR    0x31
#define MPU_RA_I2C_SLV4_REG     0x32
#define MPU_RA_I2C_SLV4_DO      0x33
#define MPU_RA_I2C_SLV4_CTRL    0x34
#define MPU_RA_I2C_SLV4_DI      0x35
#define MPU_RA_I2C_MST_STATUS   0x36
#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_DMP_INT_STATUS   0x39
#define MPU_RA_INT_STATUS       0x3A
#define MPU_RA_ACCEL_XOUT_H     0x3B
#define MPU_RA_ACCEL_XOUT_L     0x3C
#define MPU_RA_ACCEL_YOUT_H     0x3D
#define MPU_RA_ACCEL_YOUT_L     0x3E
#define MPU_RA_ACCEL_ZOUT_H     0x3F
#define MPU_RA_ACCEL_ZOUT_L     0x40
#define MPU_RA_TEMP_OUT_H       0x41
#define MPU_RA_TEMP_OUT_L       0x42
#define MPU_RA_GYRO_XOUT_H      0x43
#define MPU_RA_GYRO_XOUT_L      0x44
#define MPU_RA_GYRO_YOUT_H      0x45
#define MPU_RA_GYRO_YOUT_L      0x46
#define MPU_RA_GYRO_ZOUT_H      0x47
#define MPU_RA_GYRO_ZOUT_L      0x48
#define MPU_RA_EXT_SENS_DATA_00 0x49
#define MPU_RA_MOT_DETECT_STATUS    0x61
#define MPU_RA_I2C_SLV0_DO      0x63
#define MPU_RA_I2C_SLV1_DO      0x64
#define MPU_RA_I2C_SLV2_DO      0x65
#define MPU_RA_I2C_SLV3_DO      0x66
#define MPU_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_MOT_DETECT_CTRL      0x69
#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_BANK_SEL         0x6D
#define MPU_RA_MEM_START_ADDR   0x6E
#define MPU_RA_MEM_R_W          0x6F
#define MPU_RA_DMP_CFG_1        0x70
#define MPU_RA_DMP_CFG_2        0x71
#define MPU_RA_FIFO_COUNTH      0x72
#define MPU_RA_FIFO_COUNTL      0x73
#define MPU_RA_FIFO_R_W         0x74
#define MPU_RA_WHO_AM_I         0x75



// Bits
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROY        0x02
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA         0x01
#define BIT_GYRO                    3
#define BIT_ACC                     2
#define BIT_TEMP                    1


#define ACC_GYRO_RAWDATA_LEN  14

enum IMUpidIndex
{
  IMU_Gyro_ROLL = 0,
  IMU_Gyro_PITCH,
  IMU_Gyro_YAW,
  IMU_Acc_ROLL,
  IMU_Acc_PITCH,
  IMU_Acc_YAW,
  IMU_PID_NUM
};


typedef struct accCalibration_s
{
  Axis3i32 accSum;
  Axis3i16 accZero;
  stdev_t var[3];
  uint16_t cycleCount;
  int ok;
  uint16_t accZeroFlashSign;
} accCalibration_t;

typedef struct gyroCalibration_s 
{
  Axis3i32 gyroSum;
  Axis3i16 gyroZero;
  stdev_t var[3];
  uint16_t cycleCount;
  int ok;
  uint16_t gyroZeroFlashSign;
} gyroCalibration_t;

enum mpu6000_dlpf_bw {
  MPU6000_DLPF_BW_256 = 0,
  MPU6000_DLPF_BW_188,
  MPU6000_DLPF_BW_98,
  MPU6000_DLPF_BW_42,
  MPU6000_DLPF_BW_20,
  MPU6000_DLPF_BW_10,
  MPU6000_DLPF_BW_5,
};

enum gyro_fsr_e {
    FSR_250DPS = 0,
    FSR_500DPS,
    FSR_1000DPS,
    FSR_2000DPS,
    NUM_GYRO_FSR
};

enum accel_fsr_e {
    FSR_2G = 0,
    FSR_4G,
    FSR_8G,
    FSR_16G,
    NUM_ACCEL_FSR
};

extern gyroCalibration_t gyroCalibration; //陀螺仪校准结构体参数
extern accCalibration_t accCalibration;  //加速度校准结构体参数

extern Axis3i16 gyroADCRaw;  //陀螺仪原始AD数据
extern Axis3i16 gyroADC;   //校准的ADC数据
extern Axis3f gyrof;     //转换单位为°/s的数据

extern Axis3i16 accADCRaw;    //加速度原始AD数据
extern Axis3i16 accADC;    //校准后的AD数据      
extern Axis3f accf;      //转换单位为G的数据


bool accInit(float accUpdateRate);
bool gyroInit(float gyroUpdateRate);
bool mpu6000Init(void);
uint8_t mpu6000IICreadByte(uint8_t registerAddress);
bool mpu6000GyroRead(Axis3i16* gyroRaw);
bool mpu6000AccRead(Axis3i16* accRaw);
void gyroUpdate(Axis3f *gyro,uint8_t sw);
void accUpdate(Axis3f *acc, uint8_t sw);
void accSetCalibrationCycles(uint16_t calibrationCyclesRequired);
void gyroSetCalibrationCycles(uint16_t calibrationCyclesRequired);

void ICM42688_Init(void);

#endif
