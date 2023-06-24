#ifndef _MYultrasonic_H_
#define _MYultrasonic_H_

#include <Arduino.h>

#define csb Serial2
#define RXD2 18
#define TXD2 23


typedef struct
{
    unsigned char data[4];    
    unsigned char HighLen;
    unsigned char LowLen;
    float Len_mm;
    float Velocity;
    float last_Len_mm;
    unsigned long Time;
    unsigned long Time1;
    float Ts;
    int ok;    

}ultrasonic_t;

extern ultrasonic_t ultrasonic;

void csb_init();
void csb_Read();
 
#endif
