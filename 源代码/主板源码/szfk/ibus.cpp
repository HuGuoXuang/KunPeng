#include "ibus.h"

#define chMax 2000
#define chMin 1000

IBusBM IBus;    // IBus object

float Telecontrol_dead_zone = 5;

float Telecontrol_roll = 0;
float Telecontrol_pitch = 0;
float Telecontrol_yaw = 0;
float Telecontrol_thrust = 0;

int Telecontrol_VRA = 0;
int Telecontrol_VRB = 0;
int Telecontrol_SWA = 0;
int Telecontrol_SWB = 0;
int Telecontrol_SWC = 0;
int Telecontrol_SWD = 0;

int Telecontrol_dat[portNumber] = {0,0,0,0,0,0,0,0,0,0}; 

int ibuschx[portNumber] = {0,0,0,0,0,0,0,0,0,0}; 

int ibuschx_offset[portNumber] = {0,0,0,0,0,0,0,0,0,0}; 


void ibus_init(void)
{
    IBus.begin(Serial2,-1,RXD2,TXD2);    // iBUS object connected to serial2 RX2 pin 不使用定时器
    for(int i=0;i<portNumber;i++)
    {
        ibuschx_offset[i] = 0; 
    } 
}

int Deadband(int value, int deadband)
{
    if (abs(value) < deadband) 
    {
        value = 0;
    } 
    else if (value > 0) 
    {
        value -= deadband;
    } 
    else if (value < 0) 
    {
        value += deadband;
    }
    return value;
}


//读取遥控数据
void RXibus(void)
{    
    IBus.loop(); 

    for(int i=0;i<portNumber;i++)
    {
        ibuschx[i] = IBus.readChannel(i); 
    }

    ibuschx[0] = constrain(ibuschx[0], chMin, chMax);
    ibuschx[1] = constrain(ibuschx[1], chMin, chMax);
    ibuschx[2] = constrain(ibuschx[2], chMin, chMax);
    ibuschx[3] = constrain(ibuschx[3], chMin, chMax);
    ibuschx[4] = constrain(ibuschx[4], chMin, chMax);
    ibuschx[5] = constrain(ibuschx[5], chMin, chMax);
    ibuschx[6] = constrain(ibuschx[6], chMin, chMax);
    ibuschx[7] = constrain(ibuschx[7], chMin, chMax);
    ibuschx[8] = constrain(ibuschx[8], chMin, chMax);
    ibuschx[9] = constrain(ibuschx[9], chMin, chMax);

    for(int i=0;i<portNumber;i++)
    {
        Telecontrol_dat[i] = map(ibuschx[i], chMin, chMax, -500, 500);
        Telecontrol_dat[i] = Deadband(Telecontrol_dat[i], Telecontrol_dead_zone);//去除5死区
    }
    
    Telecontrol_yaw =  map(ibuschx[0], chMin, chMax, -400, 400) + ibuschx_offset[0];
    Telecontrol_pitch =  map(ibuschx[2], chMin, chMax, -60, 60) + ibuschx_offset[2];
    Telecontrol_thrust =  (float)map(ibuschx[1], chMin, chMax, -1500, 1500) + ibuschx_offset[1];
    Telecontrol_roll =  map(ibuschx[3], chMin, chMax, -60, 60) + ibuschx_offset[3];
    Telecontrol_SWA =  map(ibuschx[4], chMin, chMax, 0, 1) + ibuschx_offset[4];//解锁电机
    Telecontrol_SWB =  map(ibuschx[5], chMin, chMax, 0, 1) + ibuschx_offset[5];
    Telecontrol_SWC =  map(ibuschx[6], chMin, chMax, 0, 2) + ibuschx_offset[6];
    Telecontrol_SWD =  map(ibuschx[7], chMin, chMax, 0, 1) + ibuschx_offset[7];

      
    Telecontrol_VRA =  map(ibuschx[8], chMin, chMax, 0, 7) + ibuschx_offset[8];
    Telecontrol_VRB =  map(ibuschx[9], chMin, chMax, 0, 7) + ibuschx_offset[9];    
    
}



void TelecontrolPromptTone(void)//遥控器
{
    static unsigned long ms[portNumber]={0,0,0,0,0,0,0,0,0,0}; 
    static int last_Telecontrol_VRA = 0;
    static int last_Telecontrol_VRB = 0;
    static int last_Telecontrol_SWA = 0;
    static int last_Telecontrol_SWD = 0;
    
    if(Telecontrol_VRA != last_Telecontrol_VRA)
    {
        ledcWriteTone(2, 0); 
        if((millis()-ms[0])>=100)
        {
            last_Telecontrol_VRA = Telecontrol_VRA;
            ledcWriteTone(2, 10000);             
        }
    }
    else
    {
        ms[0] = millis();  
    } 


    if(Telecontrol_VRB != last_Telecontrol_VRB)
    {
        ledcWriteTone(2, 0); 
        if((millis()-ms[1])>=100)
        {
            last_Telecontrol_VRB = Telecontrol_VRB;
            ledcWriteTone(2, 10000);             
        }
    }
    else
    {
        ms[1] = millis();  
    }   

    if(Telecontrol_SWA != last_Telecontrol_SWA)
    {
        ledcWriteTone(2, 0); 
        if((millis()-ms[2])>=100)
        {
            last_Telecontrol_SWA = Telecontrol_SWA;
            ledcWriteTone(2, 10000);             
        }
    }
    else
    {
        ms[2] = millis();  
    } 

    if(Telecontrol_SWD != last_Telecontrol_SWD)
    {
        ledcWriteTone(2, 0); 
        if((millis()-ms[3])>=100)
        {
            last_Telecontrol_SWD = Telecontrol_SWD;
            ledcWriteTone(2, 10000);             
        }
    }
    else
    {
        ms[3] = millis();  
    } 
 
}



// 遥控器数据微调
void ibus_data_offset(void)
{

}
