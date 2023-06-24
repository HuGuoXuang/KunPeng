#ifndef SerialDat_h
#define SerialDat_h

#include <Arduino.h>
#include "mpu6000.h"
#include "flash_memory.h"
#include "sensors.h"
#include "ibus.h"
#include "MySPI.h" 
#include "battery.h"
#include "state_control.h"


//#define PcSerial Serial1
#define PcSerial Serial
#define RXD1 22
#define TXD1 19

extern state_t  state;		/*四轴姿态*/


void Serialcommand(void);//读取串口0数据
void Serial_print(void);
void SerialcommandInit(void);


#endif
