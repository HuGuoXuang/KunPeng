#ifndef flash_memory_h
#define flash_memory_h
#include "mpu6000.h"
#include "EEPROM.h"
#include "myservo.h"
#include "SerialDat.h"


void EEPROM_init(void);
void EEPROM_write(int x);
void EEPROM_read(int x);

#endif
