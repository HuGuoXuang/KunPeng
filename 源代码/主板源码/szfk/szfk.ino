#include "mpu6000.h"
#include "MySPI.h"
#include "SerialDat.h"
#include "flash_memory.h"
#include "sensors.h"
#include "ibus.h"
#include "dshot600.h"
#include "oled.h"
#include "myBUZZER.h"
#include "ws2812.h"
#include "myservo.h"
#include "state_control.h"
unsigned long us=0;
unsigned long us1=0; 
float dt = 0;
int unlock_motor = 0;  //解锁电机
int t1 = 0;

void setup() 
{
  SerialcommandInit();
  SpiRxDat_init();
  PcSerial.println(" ");
  dshot_init();
  PcSerial.println("dshot_init");
  ledint();
  PcSerial.println("ledint");
  ibus_init();
  ibus_data_offset();
  PcSerial.println("ibus_init");
  sensorsInit();
  PcSerial.println("sensorsInit");
  EEPROM_init(); 
  PcSerial.println("EEPROM_init");

  PcSerial.print("core count:");
  PcSerial.println(portNUM_PROCESSORS);  //输出可用的核心数量，应是2
  PcSerial.println(ESP.getFreeHeap());   //自由堆

  Servo_Initialization();  // 舵机初始化
  PcSerial.println("Servo_Initialization");

  allPidInit();
  PcSerial.println("allPidInit");
  
  BUZZER_init();
  //BuzzerMs(3,111);


  //创建任务
  //核心1上
  xTaskCreatePinnedToCore(
    Task1,     //任务对应的函数
    "Task1",   //任务名
    1024 * 5,  //栈大小。(This stack size can be checked & adjusted by reading the Stack Highwater)
    NULL,      //传给任务函数的参数
    1,         //任务优先级。(Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.)
    NULL,      //用来返回任务handle
    1          //指定CPU核心,tskNO_AFFINITY表示不指定，数字代表核心ID
  );


  xTaskCreatePinnedToCore(
    Task2,     //任务对应的函数
    "Task2",   //任务名
    1024 * 5,  //栈大小。(This stack size can be checked & adjusted by reading the Stack Highwater)
    NULL,      //传给任务函数的参数
    1,         //任务优先级。(Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.)
    NULL,      //用来返回任务handle
    1          //指定CPU核心,tskNO_AFFINITY表示不指定，数字代表核心ID
  );
  
  us1=micros();
  
}

void loop() 
{   
    static int Mx = 0;    
    Serialcommand();  //读取串口0数据  PC端数据
    RXibus();  //读取遥控器数据
    SPI_queue();//读取扩展板数据
    us=micros();
    dt = (us-us1)*0.000001;
    if(dt>=0.002)
    {
        us1 = us; 
        sensorsTask(); 

        t1++;
        if(t1>=5)
        {
            t1 = 0;  
            Serial_print(); 
        }
        if(Telecontrol_SWD == 1)
          Servo_Angle(motorPWM.Servo1, motorPWM.Servo2);  //设置左右舵机角度-900~900对应-90~90度
        else if(Telecontrol_SWD == 0)
          Servo_Angle(0,0);  //设置左右舵机角度-900~900对应-90~90度
          
        //姿态控制
        stateControl();

        if ((Telecontrol_SWD == 1) && (Telecontrol_dat[1] <= -444) && (unlock_motor == 0))  //电机解锁条件
          unlock_motor = 1;

        if (Telecontrol_SWD == 0)  //关闭电机条件
          unlock_motor = 0;


        if (unlock_motor == 1)  //解锁电机
        {
          Mx = 0;
          if (Telecontrol_thrust < Accelerator_Min)  //油门
            Telecontrol_thrust = Accelerator_Min;
            
          if(RemoteControl.RemoteControlSign == 1) //接收机物理线连接标志
          {
              dshotOutput1((uint16_t)motorPWM.m1, 0);  //左边电机
              dshotOutput2((uint16_t)motorPWM.m2, 0);  //右边电机 
          } 

          //Servo_Angle(motorPWM.Servo1, motorPWM.Servo2);  //设置左右舵机角度-900~900对应-90~90度

        } 
        else
        {
              if((Telecontrol_SWD == 0)&&(Mx == 0))
              {
                  static int x = 0;
                  x++;
                  if(x>=5)
                  {
                      x = 0;  
                      Mx = 1;
                  }
                  dshotOutput1(0, 0);  //左边电机
                  dshotOutput2(0, 0);  //右边电机      
              }
              
     
        }
          
                
    }

}



void Task1(void *pvParameters)  // This is a task.
{

  for (;;)  //任务不能自己执行结束，否则会造成崩溃，需要在任务中调用vTaskDelete删除
  {
    ledShow();
    vTaskDelay(20);
  }
}

void Task2(void *pvParameters)  // This is a task.
{
  battery_init();  //电池初始化
  OledInit();

  for (;;)  //任务不能自己执行结束，否则会造成崩溃，需要在任务中调用vTaskDelete删除
  {
    voltage_measure();  //电压测量
    oled_show();
    TelecontrolPromptTone();
    vTaskDelay(20);

    if (battery_voltage <= 14.5) 
    {
      BuzzerMs(3, 111);
      //PcSerial.print(" battery_voltage:");
      //PcSerial.println(battery_voltage);  
    }
  }
}
