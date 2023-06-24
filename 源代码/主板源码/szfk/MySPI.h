#ifndef _MySPI_H_
#define _MySPI_H_
#include <Arduino.h>
#include "ESP32SPISlave.h"
#include "filter.h"
#include "SerialDat.h"
#include "mpu6000.h"
#include "pos_estimator.h"
#define HSPI_MISO 4
#define HSPI_MOSI 16
#define HSPI_SCLK 17
#define HSPI_CS   5

#define spiClk 5000000     //时钟信号

//气压数据结构
typedef struct
{
  uint32_t timestamp;
  double pressure;
  double local_pressure;
  double temperature;
  float       alt; //(cm)
  float       F_alt; //(cm)
  float       altBias; //(cm)
  float       prevalt; //(cm)
  float       epv;
  
} baro_t;

typedef struct
{
  
    unsigned char HighLen;
    unsigned char LowLen;
    float Len_mm;
    float begin_cm_bias;
    float begin_cm;
    float Len_cm;
    float H;
    int ok;  
    unsigned int hz;

}ultrasonic_t;

typedef struct{

    unsigned char packerflag = 0;//是否接收到一个完整的数据包标志
    //unsigned char rxbuf[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//接收数据的缓冲区
    int16_t flow_x_integral; // X 像素点累计时间内的累加位移(radians*10000) // [除以 10000 乘以高度(mm)后为实际位移(mm)] 
    int16_t flow_y_integral; // Y 像素点累计时间内的累加位移(radians*10000) // [除以 10000 乘以高度(mm)后为实际位移(mm)] 
    uint16_t integration_timespan; // 上一次发送光流数据到本次发送光流数据的累计时间（us）
    uint16_t ground_distance; // 预留。默认为 999（0x03E7）
    uint8_t valid; // 状态值:0(0x00)为光流数据不可用 //245(0xF5)为光流数据可用
    uint8_t Version; //版本号 
    double  radian_x;  /*!<光流弧度*/
    double  radian_y;  /*!<光流弧度 */
    double  LPFradian_x;  /*!<光流弧度*/
    double  LPFradian_y;  /*!<光流弧度 */
    float fgyroD[2];
    double speed_x;
    double speed_y;
    double sum_flow_x;
    double sum_flow_y;
    float Ts;
    float coefficient;
    unsigned long Time;
    int ok;    
    unsigned int hz;
    
}Light_t;

extern int hdn;
extern Light_t Light;//光流
extern baro_t baro;//气压
extern ultrasonic_t ultrasonic;//超声波
extern pt1Filter_t pt1FBaro;
extern biquadFilter_t biquadLight[6];//二阶低通滤波器

void SpiRxDat_init(void);
void SPI_queue();
void heightCalibration(void);
float SlidingDataX(float xdat,int n);
float SlidingDataY(float xdat,int n); //滑动数据 
#endif
