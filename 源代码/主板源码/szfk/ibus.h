#ifndef ibus_h
#define ibus_h

//#include "sbus.h"
#include "IBusBM.h"
#include "myBUZZER.h"


#define RXD2 35
#define TXD2 32
#define portNumber 10

extern int ibuschx[portNumber]; 
extern int ibuschx_offset[portNumber]; 


extern float Telecontrol_dead_zone;
extern float Telecontrol_roll;
extern float Telecontrol_pitch;
extern float Telecontrol_yaw;
extern float Telecontrol_thrust;

extern int Telecontrol_VRA;
extern int Telecontrol_VRB;
extern int Telecontrol_SWA;
extern int Telecontrol_SWB;
extern int Telecontrol_SWC;
extern int Telecontrol_SWD;

extern int Telecontrol_dat[portNumber]; 

void ibus_init(void);

//读取遥控数据
void RXibus(void);
// 遥控器数据微调
void ibus_data_offset(void);
void TelecontrolPromptTone(void);

#endif
