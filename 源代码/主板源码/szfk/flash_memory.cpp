#include "flash_memory.h"


void EEPROM_init(void)
{
    if (!EEPROM.begin(1000)) 
    {
    PcSerial.println("Failed to initialise EEPROM");
    PcSerial.println("Restarting...");
    delay(1000);
    ESP.restart();
    }

    EEPROM_read(1);//舵机1
    EEPROM_read(2);//舵机2
    EEPROM_read(3);//
    EEPROM_read(4);//
    EEPROM_read(5);//
    EEPROM_read(6);//

}

//
void EEPROM_write(int x)
{

   switch (x)
    {
        case 1:
                EEPROM.writeInt(30,111);  
                vTaskDelay(10);
                EEPROM.writeInt(34,Servo1_Angle_offset);  
                vTaskDelay(10);
                EEPROM.commit();
                vTaskDelay(10);
                Servo1_Angle_offset = EEPROM.readInt(34);
                PcSerial.print(" Servo1_Angle_offset:");
                PcSerial.println(Servo1_Angle_offset); 
                break; 
                
        case 2:
                EEPROM.writeInt(40,222);  
                vTaskDelay(10);
                EEPROM.writeInt(44,Servo2_Angle_offset);  
                vTaskDelay(10);
                EEPROM.commit();
                vTaskDelay(10);
                Servo2_Angle_offset = EEPROM.readInt(44);
                PcSerial.print(" Servo2_Angle_offset:");
                PcSerial.println(Servo2_Angle_offset); 
                break;   


        case 3:
                accCalibration.accZeroFlashSign = 123;
                EEPROM.writeShort(50,accCalibration.accZero.x);  
                vTaskDelay(10);
                EEPROM.writeShort(52,accCalibration.accZero.y);  
                vTaskDelay(10);
                EEPROM.writeShort(54,accCalibration.accZero.z);  
                vTaskDelay(10);
                EEPROM.writeShort(56,accCalibration.accZeroFlashSign);  
                vTaskDelay(10);
                EEPROM.commit();
                vTaskDelay(10);
                accCalibration.accZero.x = EEPROM.readShort(50);
                accCalibration.accZero.y = EEPROM.readShort(52);
                accCalibration.accZero.z = EEPROM.readShort(54);
                accCalibration.accZeroFlashSign = EEPROM.readShort(56);
                PcSerial.println("acc calibration ok");
                break;  

    
        case 4:
                gyroCalibration.gyroZeroFlashSign = 321;
                EEPROM.writeShort(60,gyroCalibration.gyroZero.x);  
                vTaskDelay(10);
                EEPROM.writeShort(62,gyroCalibration.gyroZero.y);  
                vTaskDelay(10);
                EEPROM.writeShort(64,gyroCalibration.gyroZero.z);  
                vTaskDelay(10);
                EEPROM.writeShort(66,gyroCalibration.gyroZeroFlashSign);  
                vTaskDelay(10);
                EEPROM.commit();
                vTaskDelay(10);
                gyroCalibration.gyroZero.x = EEPROM.readShort(60);
                gyroCalibration.gyroZero.y = EEPROM.readShort(62);
                gyroCalibration.gyroZero.z = EEPROM.readShort(64);
                gyroCalibration.gyroZeroFlashSign = EEPROM.readShort(66);
                PcSerial.println("gyro calibration ok");        
                break;  

        case 5:
                calibratedGravity.CMSSFlashSign = 333;
                EEPROM.writeFloat(70,calibratedGravity.CMSS.x);  
                vTaskDelay(10);
                EEPROM.writeFloat(74,calibratedGravity.CMSS.y);  
                vTaskDelay(10);
                EEPROM.writeFloat(80,calibratedGravity.CMSS.z);  
                vTaskDelay(10);
                EEPROM.writeShort(84,calibratedGravity.CMSSFlashSign);  
                vTaskDelay(10);
                EEPROM.commit();
                vTaskDelay(10);
                calibratedGravity.CMSS.x = EEPROM.readFloat(70);
                calibratedGravity.CMSS.y = EEPROM.readFloat(74);
                calibratedGravity.CMSS.z = EEPROM.readFloat(80);
                calibratedGravity.CMSSFlashSign = EEPROM.readShort(84);  
                PcSerial.println("calibratedGravity.CMSS ok");        
                break;  

        case 6:
                battery_FlashSign = 111;
                EEPROM.writeInt(90,battery_FlashSign);  
                vTaskDelay(10);
                EEPROM.writeFloat(94,battery_voltageS);  
                vTaskDelay(10);
                EEPROM.commit();
                vTaskDelay(10);
                battery_FlashSign = EEPROM.readInt(90);
                battery_voltageS = EEPROM.readFloat(94);
                PcSerial.println("battery_FlashSign ok");        
                break;  


        default:  

                break;
    }

}

void EEPROM_read(int x)
{
   int FlashSign = 0;
   switch (x)
    {

        case 1:
                FlashSign = EEPROM.readShort(30);
                if(FlashSign == 111)
                {
                    Servo1_Angle_offset = EEPROM.readInt(34);  
                    PcSerial.print(" Servo1_Angle_offset:");
                    PcSerial.println(Servo1_Angle_offset);  
                }
                else
                {
                    PcSerial.println("Servo1 EEPROM no");   
                }
                break; 
                
        case 2:
                FlashSign = EEPROM.readShort(40);
                if(FlashSign == 222)
                {
                    Servo2_Angle_offset = EEPROM.readInt(44);
                    PcSerial.print(" Servo2_Angle_offset:");
                    PcSerial.println(Servo2_Angle_offset);   
                } 
                else
                {
                     PcSerial.println("Servo2 EEPROM no");   
                }

                break;   


        case 3:
                accCalibration.accZeroFlashSign = EEPROM.readShort(56);
                if(accCalibration.accZeroFlashSign==123)
                {
                    accCalibration.accZero.x = EEPROM.readShort(50);
                    accCalibration.accZero.y = EEPROM.readShort(52);
                    accCalibration.accZero.z = EEPROM.readShort(54);   
                    PcSerial.print(" accCalibration.accZero.x:");
                    PcSerial.print(accCalibration.accZero.x);
                    PcSerial.print(" accCalibration.accZero.y:");
                    PcSerial.print(accCalibration.accZero.y);
                    PcSerial.print(" accCalibration.accZero.z:");
                    PcSerial.println(accCalibration.accZero.z);
                    PcSerial.println("acc EEPROM ok");
                                   
                }
                else
                {
                    PcSerial.println("acc EEPROM no");  
                }
                
                break;  

    
        case 4:
                gyroCalibration.gyroZeroFlashSign = EEPROM.readShort(66);
                if(gyroCalibration.gyroZeroFlashSign==321)
                {
                    gyroCalibration.gyroZero.x = EEPROM.readShort(60);
                    gyroCalibration.gyroZero.y = EEPROM.readShort(62);
                    gyroCalibration.gyroZero.z = EEPROM.readShort(64);  

                    PcSerial.print(" gyroCalibration.gyroZero.x:");
                    PcSerial.print(gyroCalibration.gyroZero.x);
                    PcSerial.print(" gyroCalibration.gyroZero.y:");
                    PcSerial.print(gyroCalibration.gyroZero.y);
                    PcSerial.print(" gyroCalibration.gyroZero.z:");
                    PcSerial.println(gyroCalibration.gyroZero.z);                     
                    PcSerial.println("gyro EEPROM ok");
                                   
                }
                else
                {
                    PcSerial.println("gyro EEPROM no");  
                }        
                break;  

        case 5:
                calibratedGravity.CMSSFlashSign = EEPROM.readShort(84);
                if(calibratedGravity.CMSSFlashSign==333)
                {
                    calibratedGravity.CMSS.x = EEPROM.readFloat(70);
                    calibratedGravity.CMSS.y = EEPROM.readFloat(74);
                    calibratedGravity.CMSS.z = EEPROM.readFloat(80);


                    PcSerial.print(" calibratedGravity.CMSS.x:");
                    PcSerial.print(calibratedGravity.CMSS.x);  
                    PcSerial.print(" calibratedGravity.CMSS.y:");  
                    PcSerial.print(calibratedGravity.CMSS.y);  
                    PcSerial.print(" calibratedGravity.CMSS.z:");  
                    PcSerial.println(calibratedGravity.CMSS.z);    
                    PcSerial.println(" calibratedGravity.CMSS EEPROM ok");
                                   
                }
                else
                {
                    PcSerial.println("calibratedGravity.CMSS EEPROM no");  
                }        
                break;  

        case 6:      

                battery_FlashSign = EEPROM.readInt(90);
                if(battery_FlashSign==111)
                {
                    battery_voltageS = EEPROM.readFloat(94);


                    PcSerial.print(" battery_voltageS:");   
                    PcSerial.println(battery_voltageS);
                    PcSerial.println("battery EEPROM ok");  
                                   
                }
                else
                {
                    PcSerial.println("battery EEPROM no");  
                }        
                break;                  


                                         
        default:  

                break;
    }
}
