#ifndef _MyLc302_H_
#define _MyLc302_H_

#include <Arduino.h>

#define Lc302 Serial1
#define RXD1 26
#define TXD1 27

typedef struct{
  
    unsigned char recstatu = 0;//表示是否处于一个正在接收数据包的状态
    unsigned char ccnt = 0;//计数
    unsigned char packerflag = 0;//是否接收到一个完整的数据包标志
    unsigned char rxbuf[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};//接收数据的缓冲区
    
    int16_t flow_x_integral; // X 像素点累计时间内的累加位移(radians*10000) // [除以 10000 乘以高度(mm)后为实际位移(mm)] 
    int16_t flow_y_integral; // Y 像素点累计时间内的累加位移(radians*10000) // [除以 10000 乘以高度(mm)后为实际位移(mm)] 
    uint16_t integration_timespan; // 上一次发送光流数据到本次发送光流数据的累计时间（us）
    uint16_t ground_distance; // 预留。默认为 999（0x03E7）
    uint8_t valid; // 状态值:0(0x00)为光流数据不可用 //245(0xF5)为光流数据可用
    uint8_t Version; //版本号 
    double speed_x;
    double speed_y;
    double sum_flow_x;
    double sum_flow_y;
    float Ts;
    unsigned long Time;
    
}Light_t;

extern Light_t Light;

void Lc302_init();

void Lc302_Read(); //串口读数据

void flow_Decode(const unsigned char* f_buf);
 
#endif
