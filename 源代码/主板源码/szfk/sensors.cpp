#include "sensors.h"

sensorData_t  sensorData1;  /*传感器数据*/
sensorData_t  sensorData;	/*传感器数据*/
state_t  state;		/*四轴姿态*/


/* 传感器器件初始化 */
void sensorsInit(void)
{
  //mpu6000Init();
  ICM42688_Init();
  gyroInit(GYRO_UPDATE_RATE);
  accInit(ACC_UPDATE_RATE);
  //Gravityinit();
  
}

/*传感器任务*/
void sensorsTask(void)
{  

     //int ss = micros();
     //gyroUpdate(&sensorData.gyro,0);//gyro读取原始数据
     //accUpdate(&sensorData.acc,0);//acc读取原始数据   

       // Serial.print(" s1:");  
       // Serial.print(micros()-ss);
       // Serial.print(" s2:");      
       // ss = micros();
            
     gyroUpdate(&sensorData.gyro,1);//gyro读取原始数据
     accUpdate(&sensorData.acc,1);//acc读取原始数据     

        //Serial.print(micros()-ss);
       // Serial.println("\t");   

        //Serial.print(sensorData.gyro.x);
        //Serial.print("\t");
        //Serial.print(sensorData.gyro.y);
        //Serial.print("\t");
        //Serial.println(sensorData.gyro.z);
   
               
    if ((accCalibration.ok==0)&&(gyroCalibration.ok==0)) //陀螺仪校准完成 
      imuUpdateAttitude(&sensorData, &state, 0.002);	    

    
    updatePositionEstimator(&sensorData, &state, 0.002);//更新预估器，由主循环调用
}
