#include "ws2812.h"

//设置led数量，声明rmt对象。
#define ledPin 12
#define NR_OF_LEDS  4  //196颗灯  有一半110颗灯 86
#define NR_OF_ALL_BITS (24*NR_OF_LEDS)
//rmt数据
rmt_data_t led_data[NR_OF_ALL_BITS];
//rmt对象
rmt_obj_t* rmt_send = NULL;
//每一个led的颜色状态。
int allColor[NR_OF_LEDS][3] =  {0};



void ledint(void)
{
  
  //初始化rmt对象
  //12：针脚，true:输出或输入，RMT_MEM_192：内存数量
  if ((rmt_send = rmtInit(ledPin, true, RMT_MEM_192)) == NULL) {
    Serial.println("init sender failed\n");
  }

  //设置rmt的时间单位，这里为100ns，返回实际设置的时间单位。
  float realTick = rmtSetTick(rmt_send, 100);
  Serial.printf("real tick set to: %fns\n", realTick);

  allLed(0, 0, 0); //灯灭
  
}



void ShowColor()
{
  int led, col, bit;
  int i = 0;
  for (led = 0; led < NR_OF_LEDS; led++) {
    for (col = 0; col < 3; col++ ) {
      for (bit = 0; bit < 8; bit++) {
        //位为1时。
        if  (allColor[led][col]  & (1 << (8 - bit))) {
          //先输出高电平
          led_data[i].level0 = 1;
          //高电平输出时间为8*100ns=0.8微秒
          led_data[i].duration0 = 8;
          //再输出低电平
          led_data[i].level1 = 0;
          //低电平输出时间为4*100ns=0.4微秒
          led_data[i].duration1 = 4;
        } else {
          //位为0时。
          led_data[i].level0 = 1;
          led_data[i].duration0 = 4;
          led_data[i].level1 = 0;
          led_data[i].duration1 = 8;
        }
        i++;
      }
    }
  }

  //输出波形
  rmtWrite(rmt_send, led_data, NR_OF_ALL_BITS);
  vTaskDelay(8);                                   //延时8ms；
}

void ledColor(int x, int r, int g, int b)
{

  allColor[x][0] = r;
  allColor[x][1] = g;
  allColor[x][2] = b;
  ShowColor();
}

void allLed(int r, int g, int b) //灯灭
{
  for (int i = 0; i < NR_OF_LEDS; i++)
  {
    ledColor(i, r, g, b);
  }
  //ShowColor();
}

void ledMs(int x,int r, int g, int b,int ms,int c)
{
  if(x >= 1)
  for (int i = 0; i < c; i++)
  {
    ledColor(x-1, r, g, b);
    vTaskDelay(ms); 
    ledColor(x-1, 0, 0, 0);
    vTaskDelay(ms); 
  }
           
}

void breathingLight(int r, int g, int b,int ms)
{
    float sinVal = 0; 
    int ledVal = 0;
    for(int x=0; x<90; x++)
    {
      //当用sin函数时转化角度单位到弧度单位
      sinVal = (sin(x*(3.1412/180)));
      ledVal = int(sinVal*255);//<span style="font-family: Arial, Helvetica, sans-serif;"> 将正弦采样值转换到合适的亮度值</span>

      if((r==1)&&(g==1)&&(b==1))
        allLed(ledVal, ledVal, ledVal);
      else if((r==1)&&(g==0)&&(b==0))
        allLed(ledVal, 0, 0);
      else if((r==0)&&(g==1)&&(b==0))
        allLed(0, ledVal, 0);
      else if((r==0)&&(g==0)&&(b==1))
        allLed(0, 0, ledVal);
      else if((r==0)&&(g==1)&&(b==1))
        allLed(0, ledVal, ledVal);
      else if((r==1)&&(g==0)&&(b==1))
        allLed(ledVal, 0, ledVal);
      else if((r==1)&&(g==1)&&(b==0))
        allLed(ledVal, ledVal, 0);
                      
      vTaskDelay(ms);
      if(Telecontrol_VRB!=3)
        break; 
    }  
}


void ledShow(void)
{
    switch (Telecontrol_VRB)
    {
          
        case 1:
                ledMs( 1 , 222 , 0 , 0 , 88 , 2 );//
                ledMs( 2 , 0 , 0 , 222 , 88 , 2 );//
                ledMs( 3 , 222 , 222 , 222 , 88 , 2 );//
                break;  
                
        case 2:
                ledMs( 1 , 222 , 0 , 0 , 88 , 2 );//
                ledMs( 2 , 0 , 0 , 222 , 88 , 2 );//
                ledMs( 3 , 0 , 222 , 0 , 88 , 2 );//
                if(Telecontrol_VRB!=2)
                   break; 
                ledMs( 1 , 222 , 222 , 0 , 88 , 2 );//
                ledMs( 2 , 0 , 222 , 222 , 88 , 2 );//
                ledMs( 3 , 222 , 222 , 0 , 88 , 2 );//
                if(Telecontrol_VRB!=2)
                   break; 
                ledMs( 1 , 222 , 0 , 222 , 88 , 2 );//
                ledMs( 2 , 222 , 222 , 222 , 88 , 2 );//  
                ledMs( 3 , 222 , 222 , 222 , 88 , 2 );//
                break;   

        case 3:

                breathingLight(0,0,1,10);
                break;  
        case 4:


                allLed(222, 222, 222); 
                break;  
                                         
        default:  

                allLed(0, 0, 0);   
                break;
    }
    
}
