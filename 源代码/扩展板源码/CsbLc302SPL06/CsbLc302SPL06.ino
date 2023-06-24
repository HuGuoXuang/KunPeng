#include "MyUltrasonic.h" 
#include "MyLc302.h" 
#include <WiFi.h>
#include "MySPI.h" 
#include "SPL06-006.h"
#include <Wire.h>




void setup()
{   
    Serial.begin(500000);  
    delay(333);
    Serial.println("\nGoertek-SPL06-007 Demo\n");
    SPL_init(); // 设置初始SPL芯片寄存器     
    csb_init();
    Lc302_init();
    SpiTxDat_init();
    
 
}

void loop()
{
    csb_Read();
    Lc302_Read(); //串口读数据
    SPL06_read();  
    
    if(Light.packerflag==1)
    {   
          Light.packerflag = 0;
          flow_Decode(Light.rxbuf);
          SpiTxDat();
          
    /*      Serial.print(" x:");
          Serial.print(Light.flow_x_integral);
          Serial.print(" y:");
          Serial.print(Light.flow_y_integral);
          Serial.print(" us:");
          Serial.print(Light.integration_timespan);
          Serial.print(" ");
          Serial.print(Light.ground_distance);
          Serial.print(" ");
          Serial.print(Light.valid);
          Serial.print(" ");
          Serial.print(Light.Version);
          Serial.println(" ");  
  */
  /*
          Light.speed_x = (double)Light.flow_x_integral/10000*ultrasonic.Len_mm/(Light.integration_timespan*0.000001);
          Light.speed_y = (double)Light.flow_y_integral/10000*ultrasonic.Len_mm/(Light.integration_timespan*0.000001);

          //Light.sum_flow_x += (double)Light.speed_x*(Light.integration_timespan*0.000001);
          //Light.sum_flow_y += (double)Light.speed_y*(Light.integration_timespan*0.000001)    

          Serial.print(" mm:");  
          Serial.print(ultrasonic.Len_mm); 
                
          Serial.print(Light.speed_x);
          Serial.print(",");
          
          Serial.print(Light.speed_y); 
          Serial.print(",");

          //Serial.print(Light.sum_flow_x);
          //Serial.print(",");
          
          //Serial.print(Light.sum_flow_y); 
          //Serial.print(",");

          
          //Serial.print(Light.Ts,3); 
          //Serial.print(","); 
          
          Serial.print('\n');  
   */        
      }    
    
     
}
