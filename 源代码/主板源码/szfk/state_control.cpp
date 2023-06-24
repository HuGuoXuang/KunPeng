#include "state_control.h"

#define fixed_point_lowest_height     10  //cm

float sensitivity = 0.3;

float altThrustAdj;
float altHoldThrust;

motorPWM_t motorPWM;
PidObject pid[PID_NUM];
control_t control;

//所有PID初始化
void allPidInit(void)
{
  //角度PID（roll\pitch\yaw） 
  pidInit(&pid[ANGLE_ROLL], 5, 0, 0, PID_ANGLE_ROLL_INTEGRATION_LIMIT , PID_ANGLE_ROLL_OUTPUT_LIMIT, PID_DT, true, PID_RATE_LPF_CUTOFF_FREQ);//5, 10, 0,
  pidInit(&pid[ANGLE_PITCH], 4, 0, 0, PID_ANGLE_PITCH_INTEGRATION_LIMIT, PID_ANGLE_PITCH_OUTPUT_LIMIT, PID_DT, true, PID_RATE_LPF_CUTOFF_FREQ);

  //角速度PID（roll\pitch\yaw） 
  pidInit(&pid[RATE_ROLL], 0.8, 0, 0.001, PID_RATE_ROLL_INTEGRATION_LIMIT, PID_RATE_ROLL_OUTPUT_LIMIT, PID_DT, true, PID_RATE_LPF_CUTOFF_FREQ);
  pidInit(&pid[RATE_PITCH], 1.5, 0, 0.001, PID_RATE_PITCH_INTEGRATION_LIMIT, PID_RATE_PITCH_OUTPUT_LIMIT, PID_DT, true, PID_RATE_LPF_CUTOFF_FREQ);
  pidInit(&pid[RATE_YAW], 0.8, 1, 0, PID_RATE_YAW_INTEGRATION_LIMIT, PID_ANGLE_YAW_OUTPUT_LIMIT, PID_DT, true, PID_RATE_LPF_CUTOFF_FREQ);

  //位移速度PID
  pidInit(&pid[VELOCITY_Z], 4, 5, 0, PID_VZ_INTEGRATION_LIMIT, PID_VZ_OUTPUT_LIMIT, PID_DT, true, PID_RATE_LPF_CUTOFF_FREQ);   
  pidInit(&pid[VELOCITY_X], 0.2, 0.0, 0, PID_VX_INTEGRATION_LIMIT,PID_VX_OUTPUT_LIMIT, PID_DT, true, PID_RATE_LPF_CUTOFF_FREQ); 
  pidInit(&pid[VELOCITY_Y], 0.2, 0.0, 0, PID_VY_INTEGRATION_LIMIT,PID_VY_OUTPUT_LIMIT, PID_DT, true, PID_RATE_LPF_CUTOFF_FREQ); 
  
  //位置PID
  pidInit(&pid[POSHOLD_Z], 0, 0, 0, 0,PID_POS_Z_OUTPUT_LIMIT, PID_DT, true, PID_RATE_LPF_CUTOFF_FREQ); 
  pidInit(&pid[POSHOLD_X], 1, 0, 0, PID_POS_X_INTEGRATION_LIMIT,PID_POS_X_OUTPUT_LIMIT, PID_DT, true, PID_RATE_LPF_CUTOFF_FREQ); 
  pidInit(&pid[POSHOLD_Y], 1, 0, 0, PID_POS_Y_INTEGRATION_LIMIT,PID_POS_Y_OUTPUT_LIMIT, PID_DT, true, PID_RATE_LPF_CUTOFF_FREQ); 

  
}



//姿态控制
void stateControl(void)
{
    static float posLight_last[4]={0,0,0,0};
    static float last_pos_Z = state.Position.z;
    static float ZposError = 0;
    static float ZvelocityError = 0;
    static float VelocityZ = 0;
    static float VelocityX = 0;
    static float VelocityY = 0;
    static int height_select_select = 0;
    static float roll_target_angle = 0;
    static float pitch_target_angle = 0;


    //定高傳感器選擇
    if((ultrasonic.hz >= 6)&&(posUltrasonic.est.pos.z<200)&&(state.Position.z<200)&&(ultrasonic.Len_cm<200))//是否使用超声波定高
    {
        height_select_select = 1;//使用超声波
    }
    else
    {
        height_select_select = 0;//使用气压计 
    }
    height_select_select = 0;//使用气压计 

    //記錄定高的高度
    if(Telecontrol_dat[1]!=0)//油门不归零
    {
       //记录上一次的高度
       if(height_select_select == 1)
       {
          last_pos_Z = posUltrasonic.est.pos.z;     
       }
       else
       {
          last_pos_Z = state.Position.z;             
       }

    }
    //位置PID////////////////////////////////////////////////////////////////////// 

    //定高PID              
    if(height_select_select == 1)
    {
      ZposError = last_pos_Z - posUltrasonic.est.pos.z;  //超声波高度误差   
    }
    else
    {
      ZposError = last_pos_Z - state.Position.z;   //气压计高度误差          
    }  
    ZposError = constrain(ZposError, -50, 50);
    if((Telecontrol_dat[3] == 0)&& (state.Position.z>fixed_point_lowest_height))
      VelocityZ = pidUpdate(&pid[POSHOLD_Z], ZposError);    
    else
      VelocityZ = 0;
      
    if(VelocityZ<0)
    {
        VelocityZ = VelocityZ/2.5; 
    }    
    
  
  
    ///////////////////////////////////////位移數度PID////////////////////////////////////////////////////
    
    //X軸速度PID
    if((ultrasonic.hz >= 6)&&(Light.hz>=33)&&(Telecontrol_dat[3]==0)&&(posUltrasonic.est.pos.z>=fixed_point_lowest_height)&&(Telecontrol_SWA==1))
    {
        float xv = 0;
        float xp = 0;
        
        if(Telecontrol_SWB==1)
        {
            xv = constrain(accpos[Y], -80.0f, 80.0f);//限制      
            xp = posLight.est.pos.y - posLight_last[0];    
        }
        else if(Telecontrol_SWB==0)
        {
            xv = constrain(Light.speed_x, -50.0f, 50.0f);//限制     
            xp = Light.sum_flow_x - posLight_last[1];    
        }
        if(abs(xv)<50)
            VelocityX = pidUpdate(&pid[POSHOLD_X],0 - xp); 
        else
        {
            posLight_last[0] = posLight.est.pos.y;  
            posLight_last[1] = Light.sum_flow_x;          
        }    
        roll_target_angle = pidUpdate(&pid[VELOCITY_X], xv - VelocityX); 
  
    }
    else
    {
        roll_target_angle = Telecontrol_roll;  
        pidReset(&pid[VELOCITY_X]);  
        posLight_last[0] = posLight.est.pos.y;  
        posLight_last[1] = Light.sum_flow_x;
    }

    //Y軸速度PID
    if((ultrasonic.hz >= 6)&&(Light.hz>=33)&&(Telecontrol_dat[2]==0)&&(posUltrasonic.est.pos.z>=fixed_point_lowest_height)&&(Telecontrol_SWA==1))
    {     
        float yv = 0;
        float yp = 0;
        
        if(Telecontrol_SWB==1)
        {
            yv = constrain(accpos[X], -80.0f, 80.0f);//限制  
            yp = posLight.est.pos.x - posLight_last[2];     
        }
        else if(Telecontrol_SWB==0)
        {
            yv = constrain(Light.speed_y, -50.0f, 50.0f);//限制  
            yp = Light.sum_flow_y - posLight_last[3];       
        }
        if(abs(yv)<50)
            VelocityY = pidUpdate(&pid[POSHOLD_Y], 0 - yp);      
        else
            {
                posLight_last[2] = posLight.est.pos.x;  
                posLight_last[3] = Light.sum_flow_y;                 
            }
        pitch_target_angle = pidUpdate(&pid[VELOCITY_Y], VelocityY - yv); 
 
    }
    else
    {
        pitch_target_angle = Telecontrol_pitch;  
        pidReset(&pid[VELOCITY_Y]);
        posLight_last[2] = posLight.est.pos.x;  
        posLight_last[3] = Light.sum_flow_y;          
    }

    
    //Z軸速度PID
    static float TelecontrolThrust = 0;//油门手感
    if(Telecontrol_thrust<0)
    {
        TelecontrolThrust = Telecontrol_thrust/10; 
    }
    else
    {
        TelecontrolThrust = Telecontrol_thrust/4;    
    }

    if((Telecontrol_dat[1] == 0)&&(posUltrasonic.est.pos.z>=fixed_point_lowest_height))//油门归零且氣壓計高度大於50cm
    {
        ZvelocityError = VelocityZ - state.Velocity.z;  
        //pid[VELOCITY_Z].integ_start = 0;   
    }
    else
    {
        ZvelocityError = TelecontrolThrust - state.Velocity.z;  
        //pid[VELOCITY_Z].integ_start = 1;
        pidReset(&pid[POSHOLD_Z]);       
    }
   
    altThrustAdj = pidUpdate(&pid[VELOCITY_Z], ZvelocityError);
  
    pid[VELOCITY_Z].integ = constrain(pid[VELOCITY_Z].integ, -100, pid[VELOCITY_Z].iLimit);
    altThrustAdj = constrain(altThrustAdj, 0, 1800);
    pid[VELOCITY_Z].output = constrain(pid[VELOCITY_Z].output, 0, 1800);
    
      
    
    //角度PID（外环）////////////////////////////////////////////////////////////////////////////////////////////////////
    //roll_target_angle = constrain(roll_target_angle, -60.0f, 60.0f);//限制60°
    //pitch_target_angle = constrain(pitch_target_angle, -60.0f, 60.0f);//限制60°
    if(Telecontrol_SWC==0)
    {
        sensitivity = 0.3;  
    }
    else if(Telecontrol_SWC==1)
    {
        sensitivity = 0.6;  
    }
    else if(Telecontrol_SWC==2)
    {
        sensitivity = 1.0;  
    }
    else 
    {
        sensitivity = 0.3;        
    }

    
    float AngleRoll = pidUpdate(&pid[ANGLE_ROLL], roll_target_angle*sensitivity - state.attitude.roll);//
    float AnglePitch = pidUpdate(&pid[ANGLE_PITCH], pitch_target_angle*sensitivity - state.attitude.pitch);//
   
    //角速度PID（内环）///////////////////////////////////////////////////////////////////////////////////////////////////
    float RATE_ROLL_error = AngleRoll - sensorData.gyro.x;   
    //RATE_ROLL_error = constrain(RATE_ROLL_error, -300.0f, 300.0f);//限制
    control.roll  = pidUpdate(&pid[RATE_ROLL],RATE_ROLL_error );//

    float RATE_PITCH_error = AnglePitch - sensorData.gyro.y;    
    //RATE_PITCH_error = constrain(RATE_PITCH_error, -300.0f, 300.0f);//限制
    control.pitch = pidUpdate(&pid[RATE_PITCH], RATE_PITCH_error );//

    float RATE_YAW_error =  -Telecontrol_yaw - sensorData.gyro.z;  
    //RATE_YAW_error = constrain(RATE_YAW_error, -300.0f, 300.0f);//限制
    control.yaw = pidUpdate(&pid[RATE_YAW],RATE_YAW_error);//
  
    //油门+横滚角
    float m1 = altThrustAdj + control.roll;
    float m2 = altThrustAdj - control.roll;
    motorPWM.m1 = int(m1);
    motorPWM.m2 = int(m2);
    
    //
    float Servo1 = Servo1_Angle  - control.pitch + control.yaw; //设置左右舵机角度-900~900对应-90~90度
    float Servo2 = Servo2_Angle  + control.pitch + control.yaw;
    motorPWM.Servo1 = int(Servo1);
    motorPWM.Servo2 = int(Servo2); 

   

    motorPWM.m1 = constrain(motorPWM.m1, 0, 1900);
    motorPWM.m2 = constrain(motorPWM.m2, 0, 1900);
    
    //
    motorPWM.Servo1 = constrain(motorPWM.Servo1, -900, 900);
    motorPWM.Servo2 = constrain(motorPWM.Servo2, -900, 900);

    PIDErrorAbnormal();      
   ////PID清除积分和微分
   if (unlock_motor == 0)
   {
      
      pidReset(&pid[ANGLE_ROLL]);   
      pidReset(&pid[RATE_ROLL]);  

      pidReset(&pid[ANGLE_PITCH]);  
      pidReset(&pid[ANGLE_ROLL]);  

      pidReset(&pid[RATE_YAW]);  
          
      pidReset(&pid[VELOCITY_Z]);
      pidReset(&pid[VELOCITY_X]);
      pidReset(&pid[VELOCITY_Y]);
      
      pidReset(&pid[POSHOLD_Z]); 
      pidReset(&pid[POSHOLD_X]); 
      pidReset(&pid[POSHOLD_Y]); 

      motorPWM.m1 = 0;
      motorPWM.m2 = 0;

      //motorPWM.Servo1 = 0; //设置左右舵机角度-900~900对应-90~90度
      //motorPWM.Servo2 = 0;      
      
   }    

}




void PIDErrorAbnormal(void)
{
    //檢查傳感器是否異常
    if((abs(pid[ANGLE_ROLL].error)>700)||(abs(pid[ANGLE_PITCH].error)>700))//700°
    {
        //角度誤差
        PcSerial.print("  pid[ANGLE_ROLL].error:");
        PcSerial.print(pid[ANGLE_ROLL].error);
        PcSerial.print("  pid[ANGLE_PITCH].error:");
        PcSerial.println(pid[ANGLE_PITCH].error);  
        unlock_motor = 0;    
    }
    if((abs(pid[RATE_ROLL].error)>1500)||(abs(pid[RATE_PITCH].error)>1500)||(abs(pid[RATE_YAW].error)>1500))//1500°/s
    {
        //角速度誤差
        PcSerial.print("  pid[RATE_ROLL].error:");  // pid[RATE_ROLL].error:1700.03  pid[RATE_PITCH].error:1904.38  pid[RATE_YAW].error:2399.54
        PcSerial.print(pid[RATE_ROLL].error);
        PcSerial.print("  pid[RATE_PITCH].error:");
        PcSerial.print(pid[RATE_PITCH].error);
        PcSerial.print("  pid[RATE_YAW].error:");
        PcSerial.println(pid[RATE_YAW].error);  
        unlock_motor = 0;     
    }

    if((abs(pid[VELOCITY_Z].error)>700)||(abs(pid[VELOCITY_X].error)>700)||(abs(pid[VELOCITY_Y].error)>700))//700cm/s
    {
        //位移速度誤差
        PcSerial.print("  pid[VELOCITY_Z].error:");
        PcSerial.print(pid[VELOCITY_Z].error);
        PcSerial.print("  pid[VELOCITY_X].error:");
        PcSerial.print(pid[VELOCITY_X].error);
        PcSerial.print("  pid[VELOCITY_Y].error:");
        PcSerial.print(pid[VELOCITY_Y].error);    
        unlock_motor = 0; 
    }
    if(abs(pid[POSHOLD_Z].error)>300)//300cm
    {
        //位置誤差
        PcSerial.print("  pid[POSHOLD_Z].error:");
        PcSerial.println(pid[POSHOLD_Z].error);   
        unlock_motor = 0;    
    }      
}
