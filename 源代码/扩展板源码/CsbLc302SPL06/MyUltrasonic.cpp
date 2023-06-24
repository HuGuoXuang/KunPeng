#include "MYultrasonic.h" 

ultrasonic_t ultrasonic;


void csb_init()
{
    csb.begin(9600, SERIAL_8N1, RXD2, TXD2);
    while(csb.read() >= 0);     // 
    ultrasonic.Time = millis();  
}


void csb_Read()
{ 
    if(csb.available() >= 4)                   
    {
        ultrasonic.data[0] = csb.read();  
        ultrasonic.data[1] = csb.read();  
        ultrasonic.data[2] = csb.read();  
        ultrasonic.data[3] = csb.read();  
        if(ultrasonic.data[0]==0xff)
        {
            int sum = (ultrasonic.data[0] + ultrasonic.data[1] + ultrasonic.data[2]) & 0x00ff;
            if(sum==ultrasonic.data[3]) 
            {

              ultrasonic.HighLen = ultrasonic.data[1];
              ultrasonic.LowLen = ultrasonic.data[2];
              float mm  = (float)(ultrasonic.HighLen*256 + ultrasonic.LowLen);    
              //if((mm > 50) && (mm < 2500)) 
              //{
                  ultrasonic.Len_mm  = mm;
                  ultrasonic.ok = 1;                 
              //}
              //else
              //{
              //    ultrasonic.ok = 0;    
              //}       
 
            }
          
        }
        else
        {
            ultrasonic.data[0] = 0;  
            ultrasonic.data[1] = 0;  
            ultrasonic.data[2] = 0;  
            ultrasonic.data[3] = 0;  
            ultrasonic.ok = 0;            
        }

    }  
            
}
