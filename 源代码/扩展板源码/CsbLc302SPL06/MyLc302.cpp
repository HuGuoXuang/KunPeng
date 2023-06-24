#include "MyLc302.h" 

Light_t Light;

void Lc302_init()
{
    Lc302.begin(19200, SERIAL_8N1, RXD1, TXD1);  
    Light.Time = millis();
}


unsigned char getXor()
{
       unsigned char temp = Light.rxbuf[2];
       for(int i=3;i<12;i++)
       {
          temp^=Light.rxbuf[i];
       }
       return temp;
}



void Lc302_Read() //串口读数据
{
  while(Lc302.available() > 0) 
  {
     unsigned char dat = Lc302.read();  
     //Serial.println(dat);  
     if((Light.ccnt==0)&&(dat == 0xfe))
     {
       Light.rxbuf[Light.ccnt] = dat;
       Light.ccnt = 1;
     }
     else if((Light.ccnt==1)&&(dat == 0x0a))
     {
       Light.rxbuf[Light.ccnt] = dat;
       Light.recstatu = 1;
       Light.ccnt = 2; 
     }
     else if((Light.recstatu == 1)&&(Light.ccnt>1)) //表示是否处于一个正在接收数据包的状态
     {
          Light.rxbuf[Light.ccnt] = dat; 
          Light.ccnt++;
          if(Light.ccnt==14)
          {  
              if(getXor()==Light.rxbuf[12])
              {
                  Light.rxbuf[0] = 0;
                  Light.rxbuf[1] = 0;
                  Light.recstatu = 0;
                  Light.ccnt = 0; 
                  Light.packerflag = 1;//用于告知系统已经接收成功 
              }
              else
              {
                  Light.rxbuf[0] = 0;
                  Light.rxbuf[1] = 0;
                  Light.recstatu = 0;
                  Light.packerflag = 0;//用于告知系统已经接收失败
                  Light.ccnt = 0;       
                  Serial.println("on2..............................");                     
              }                        
          }  
      }
      else
      {
          Light.rxbuf[0] = 0;
          Light.rxbuf[1] = 0;
          Light.recstatu = 0;
          Light.packerflag = 0;//用于告知系统已经接收失败
          Light.ccnt = 0;
          Serial.println("on1.............................."); 
      }         
  }
}



void flow_Decode(const unsigned char* f_buf)
{   
    Light.flow_x_integral       = f_buf[2] + (f_buf[3]<<8);// X 像素点累计时间内的累加位移(radians*10000) // [除以 10000 乘以高度(mm)后为实际位移(mm)] 
    Light.flow_y_integral       = f_buf[4] + (f_buf[5]<<8);// Y 像素点累计时间内的累加位移(radians*10000) // [除以 10000 乘以高度(mm)后为实际位移(mm)] 
    Light.integration_timespan  = f_buf[6] + (f_buf[7]<<8);// 上一次发送光流数据到本次发送光流数据的累计时间（us）
    Light.ground_distance       = f_buf[8] + (f_buf[9]<<8);// 预留。默认为 999（0x03E7）
    Light.valid                 = f_buf[10] ;              // 状态值:0(0x00)为光流数据不可用 //245(0xF5)为光流数据可用
    Light.Version               = f_buf[11] ; //实际为新版本中的valid
}
