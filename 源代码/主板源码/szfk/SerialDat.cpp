#include "SerialDat.h"

int fl[2]={200,150};

int Serial_switch = 0;

//PC端数据
String comdata = "";    //串口0接收数据
char terminator = '=';

void SerialcommandInit(void)
{
    //PcSerial.begin(500000, SERIAL_8N1, RXD1, TXD1);
    PcSerial.begin(500000);
}

void Serialcommand(void)//读取串口0数据
{
    if(PcSerial.available()>0)
    {   
        comdata =PcSerial.readStringUntil(terminator);
        if(comdata=="sw")
        {
          Serial_switch = (int)PcSerial.parseInt();
          PcSerial.print("Serial_switch");
          PcSerial.print('=');
          PcSerial.println(Serial_switch);
        }  

        if(comdata=="lbk")
        {
          pt1FBaro.k = PcSerial.parseFloat();
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(pt1FBaro.k);
        } 

        //
        if(comdata=="hdn")
        {
          hdn = (int)PcSerial.parseInt();
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(hdn);
        } 

        //
        if(comdata=="bfl")
        {
          int xs = (int)PcSerial.parseInt();
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(xs);

          //biquadFilterInitLPF(&biquadLight[4],xs, 500);
          //biquadFilterInitLPF(&biquadLight[5],xs, 500);          
          biquadFilterInitLPF(&BiquadPosAcc[X],xs, 500);
          biquadFilterInitLPF(&BiquadPosAcc[Y],xs, 500);
          //biquadFilterInitLPF(&BiquadPosAcc[Z],xs, 500);      
          //biquadFilterInitLPF(&biquadLight[2],xs, 500);
          //biquadFilterInitLPF(&biquadLight[3],xs, 500);              
        } 


        if(comdata=="xbz")
        {
          fl[0] = (int)PcSerial.parseInt();
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(fl[0]);
          PcSerial.println(fl[1]);
      
          //二阶陷波器初始化(结构体变量 采样频率 中心频率 截止频率)
          biquadFilterInitNotch(&AccNotchingFilter[X], 500, fl[0], fl[1]);
          biquadFilterInitNotch(&AccNotchingFilter[Y], 500, fl[0], fl[1]);
          biquadFilterInitNotch(&AccNotchingFilter[Z], 500, fl[0], fl[1]);             
        } 

        if(comdata=="xbj")
        {
          fl[1] = (int)PcSerial.parseInt();
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(fl[0]);
          PcSerial.println(fl[1]);
      
          //二阶陷波器初始化(结构体变量 采样频率 中心频率 截止频率)
          biquadFilterInitNotch(&AccNotchingFilter[X], 500, fl[0], fl[1]);
          biquadFilterInitNotch(&AccNotchingFilter[Y], 500, fl[0], fl[1]);
          biquadFilterInitNotch(&AccNotchingFilter[Z], 500, fl[0], fl[1]);             
        }         

        //
        if(comdata=="csb")
        {
          csbok = (int)PcSerial.parseInt();
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(csbok);
        } 


        if(comdata=="vcc")
        {
          float vcc = PcSerial.parseFloat();
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(vcc);
          battery_voltageS = battery_ADC/vcc;
          EEPROM_write(6);
        } 


        //
        if(comdata=="accqz")
        {
          W_ACC_BIAS = PcSerial.parseFloat();
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(W_ACC_BIAS);
          baro.altBias = 0;
        } 

        if(comdata=="glxs")
        {
          Light.coefficient = PcSerial.parseFloat();
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(Light.coefficient);
        } 
        
        //光流权重
        if(comdata=="gl")
        {
          W_XY_Light_P = PcSerial.parseFloat();
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(W_XY_Light_P,6);
        }   

        //光流权重
        if(comdata=="glv")
        {
          W_XY_Light_V = PcSerial.parseFloat();
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(W_XY_Light_V,6);
        }                
        
        //气压权重
        if(comdata=="qy")
        {
          W_Z_BARO_P = PcSerial.parseFloat();
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(W_Z_BARO_P);
        } 

        //设置舵机角度
        if(comdata=="ad")
        {
          Servo1_Angle = (int)PcSerial.parseInt();
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(Servo1_Angle);
        } 
        if(comdata=="bd")
        {
          Servo2_Angle = (int)PcSerial.parseInt();
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(Servo2_Angle);
        }

        //设置舵机微调
        if(comdata=="awt")
        {
          Servo1_Angle_offset = PcSerial.parseInt();
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(Servo1_Angle_offset);
          EEPROM_write(1);
        }    
        if(comdata=="bwt")
        {
          Servo2_Angle_offset = PcSerial.parseInt();
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(Servo2_Angle_offset);
          EEPROM_write(2);
        }


        if(comdata=="pos")
        {
          int ok = (int)PcSerial.parseInt();
          if(ok == 1)
            posEstimator.imu.gravityCalibrationComplete = 1;
          PcSerial.print("posEstimator.imu.gravityCalibrationComplete");
          PcSerial.print('=');
          PcSerial.println(posEstimator.imu.gravityCalibrationComplete);

          baro.local_pressure = baro.pressure;
        } 

        if(comdata=="acc")
        {
          accCalibration.ok = (int)PcSerial.parseInt();
          PcSerial.print("accCalibration.ok");
          PcSerial.print('=');
          PcSerial.println(accCalibration.ok);
          accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
        } 

        if(comdata=="gyro")
        {
          gyroCalibration.ok = (int)PcSerial.parseInt();
          PcSerial.print("gyroCalibration.ok");
          PcSerial.print('=');
          PcSerial.println(gyroCalibration.ok);
          gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);
        }   

        //横滚角速度PID
        if(comdata=="xskp")
        {
          float c  = PcSerial.parseFloat();
          pid[RATE_ROLL].kp = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(pid[RATE_ROLL].kp);
        }   

        if(comdata=="xski")
        {
          float c = PcSerial.parseFloat();
          pid[RATE_ROLL].ki = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }  

        if(comdata=="xskd")
        {
          float c = PcSerial.parseFloat();
          pid[RATE_ROLL].kd = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }  

        //横滚角度PID
        if(comdata=="xkp")
        {
          float c = PcSerial.parseFloat();
          pid[ANGLE_ROLL].kp = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }   

        if(comdata=="xki")
        {
          float c = PcSerial.parseFloat();
          pid[ANGLE_ROLL].ki = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }  

        if(comdata=="xkd")
        {
          float c = PcSerial.parseFloat();
          pid[ANGLE_ROLL].kd = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }  


        //俯仰角速度PID
        if(comdata=="yskp")
        {
          float c = PcSerial.parseFloat();
          pid[RATE_PITCH].kp = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }   

        if(comdata=="yski")
        {
          float c = PcSerial.parseFloat();
          pid[RATE_PITCH].ki = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }  

        if(comdata=="yskd")
        {
          float c = PcSerial.parseFloat();
          pid[RATE_PITCH].kd = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }  

        //俯仰角度PID
        if(comdata=="ykp")
        {
          float c = PcSerial.parseFloat();
          pid[ANGLE_PITCH].kp = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }   

        if(comdata=="yki")
        {
          float c = PcSerial.parseFloat();
          pid[ANGLE_PITCH].ki = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }  

        if(comdata=="ykd")
        {
          float c = PcSerial.parseFloat();
          pid[ANGLE_PITCH].kd = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }  

        //航向角速度PID
        if(comdata=="zskp")
        {
          float c = PcSerial.parseFloat();
          pid[RATE_YAW].kp = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }   

        if(comdata=="zski")
        {
          float c = PcSerial.parseFloat();
          pid[RATE_YAW].ki = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }  

        if(comdata=="zskd")
        {
          float c = PcSerial.parseFloat();
          pid[RATE_YAW].kd = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }  

        //X位移速度PID
        if(comdata=="xvkp")
        {
          float c = PcSerial.parseFloat();
          pid[VELOCITY_X].kp = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }   

        if(comdata=="xvki")
        {
          float c = PcSerial.parseFloat();
          pid[VELOCITY_X].ki = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }  

        if(comdata=="xvkd")
        {
          float c = PcSerial.parseFloat();
          pid[VELOCITY_X].kd = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }            


        //Y位移速度PID
        if(comdata=="yvkp")
        {
          float c = PcSerial.parseFloat();
          pid[VELOCITY_Y].kp = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }   

        if(comdata=="yvki")
        {
          float c = PcSerial.parseFloat();
          pid[VELOCITY_Y].ki = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }  

        if(comdata=="yvkd")
        {
          float c = PcSerial.parseFloat();
          pid[VELOCITY_Y].kd = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }


        //高度速度PID
        if(comdata=="hskp")
        {
          float c = PcSerial.parseFloat();
          pid[VELOCITY_Z].kp = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }   

        if(comdata=="hski")
        {
          float c = PcSerial.parseFloat();
          pid[VELOCITY_Z].ki = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }  

        if(comdata=="hskd")
        {
          float c = PcSerial.parseFloat();
          pid[VELOCITY_Z].kd = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }                  

        //高度位置PID
        if(comdata=="hkp")
        {
          float c = PcSerial.parseFloat();
          pid[POSHOLD_Z].kp = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }   

        if(comdata=="hki")
        {
          float c = PcSerial.parseFloat();
          pid[POSHOLD_Z].ki = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }  

        if(comdata=="hkd")
        {
          float c = PcSerial.parseFloat();
          pid[POSHOLD_Z].kd = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }   

        //位置PIDX
        if(comdata=="pxkp")
        {
          float c = PcSerial.parseFloat();
          pid[POSHOLD_X].kp = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }   

        if(comdata=="pxki")
        {
          float c = PcSerial.parseFloat();
          pid[POSHOLD_X].ki = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }  

        if(comdata=="pxkd")
        {
          float c = PcSerial.parseFloat();
          pid[POSHOLD_X].kd = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }   

        //位置PIDY
        if(comdata=="pykp")
        {
          float c = PcSerial.parseFloat();
          pid[POSHOLD_Y].kp = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }   

        if(comdata=="pyki")
        {
          float c = PcSerial.parseFloat();
          pid[POSHOLD_Y].ki = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }  

        if(comdata=="pykd")
        {
          float c = PcSerial.parseFloat();
          pid[POSHOLD_Y].kd = c;
          PcSerial.print(comdata);
          PcSerial.print('=');
          PcSerial.println(c);
        }   
        
            
    }
    comdata = "";    
}

void Serial_print(void)
{
    //Serial_switch = 0;
    switch (Serial_switch)
    {
          
        case 0:


                break; 

        case 1:
                PcSerial.print(state.attitude.roll);
                PcSerial.print("\t");
                PcSerial.print(state.attitude.pitch);
                PcSerial.print("\t");
                PcSerial.println(state.attitude.yaw);    
                             
                break; 
          
        case 2:
                PcSerial.print(sensorData.gyro.x);
                PcSerial.print("\t");
                PcSerial.print(sensorData.gyro.y);
                PcSerial.print("\t");
                PcSerial.println(sensorData.gyro.z);       
                
                break;   

        case 3:
                PcSerial.print(sensorData.acc.x*GRAVITY_CMSS);
                PcSerial.print("\t");
                PcSerial.print(sensorData.acc.y*GRAVITY_CMSS);
                PcSerial.print("\t");
                PcSerial.println(sensorData.acc.z*GRAVITY_CMSS-GRAVITY_CMSS);  
                break;  

    
        case 4:        
                PcSerial.print(state.Position.x);
                PcSerial.print("\t");
                PcSerial.print(state.Position.y);
                PcSerial.print("\t");
                PcSerial.println(state.Position.z);           
       
                break;  

        case 5:   
                PcSerial.print(state.Velocity.x);
                PcSerial.print("\t");
                PcSerial.print(state.Velocity.y);
                PcSerial.print("\t");
                PcSerial.println(state.Velocity.z);   
                                   
                break;  


        case 6:
                PcSerial.print(" ch1:"); 
                PcSerial.print(ibuschx[0]);   
                PcSerial.print(" ch2:"); 
                PcSerial.print(ibuschx[1]);    
                PcSerial.print(" ch3:"); 
                PcSerial.print(ibuschx[2]);   
                PcSerial.print(" ch4:"); 
                PcSerial.print(ibuschx[3]);   
                PcSerial.print(" ch5:"); 
                PcSerial.print(ibuschx[4]);   
                PcSerial.print(" ch6:"); 
                PcSerial.print(ibuschx[5]);   
                PcSerial.print(" ch7:"); 
                PcSerial.print(ibuschx[6]);   
                PcSerial.print(" ch8:");   
                PcSerial.print(ibuschx[7]);  
                PcSerial.print(" ch9:"); 
                PcSerial.print(ibuschx[8]);   
                PcSerial.print(" ch10:");   
                PcSerial.println(ibuschx[9]);                       
                break;  


        case 7:
                
                PcSerial.print(" alt:");  
                PcSerial.print(baro.alt); 
                
                PcSerial.print(" pressure:");  
                PcSerial.print(baro.pressure); 
                
                PcSerial.print(" temperature:");  
                PcSerial.print(baro.temperature); 
                  
                PcSerial.print(" mmok:");  
                PcSerial.print(ultrasonic.ok); 
                
                PcSerial.print(" mm:");  
                PcSerial.print(ultrasonic.Len_mm); 

                PcSerial.print(" cm:");  
                PcSerial.print(ultrasonic.Len_cm); 

                PcSerial.print(" Light.us:");  
                PcSerial.print(Light.integration_timespan); 
                
                PcSerial.print(" Light.Ts:");  
                PcSerial.print(Light.Ts,4); 
                
                PcSerial.print(" x");
                PcSerial.print(Light.sum_flow_x);
                
                PcSerial.print(" y");
                PcSerial.println(Light.sum_flow_y);   

                   
                break;  

        case 8:  
                PcSerial.print("  battery_ADC:");
                PcSerial.print(battery_ADC);
                PcSerial.print("  S:");
                PcSerial.print(battery_voltageS);
                PcSerial.print("  V:");
                PcSerial.println(battery_voltage);
 
                break;  

        case 9:  
                PcSerial.print("    RATE_ROLL_KP:");
                PcSerial.print(pid[RATE_ROLL].kp);
                PcSerial.print("    RATE_ROLL_KI:");
                PcSerial.print(pid[RATE_ROLL].ki);   
                PcSerial.print("    RATE_ROLL_KD:");
                PcSerial.print(pid[RATE_ROLL].kd,3);  
                PcSerial.print("    outP:");
                PcSerial.print(pid[RATE_ROLL].outP);  
                PcSerial.print("    outD:");
                PcSerial.print(pid[RATE_ROLL].outD);    
                PcSerial.print("    motorPWM.m1:");
                PcSerial.print(motorPWM.m1);   
                PcSerial.print("    motorPWM.m2:");
                PcSerial.println(motorPWM.m2);  
                
                                   
                break;  

        case 10:  
                PcSerial.print("    ANGLE_ROLL_KP:");
                PcSerial.print(pid[ANGLE_ROLL].kp);
                PcSerial.print("    ANGLE_ROLL_KI:");
                PcSerial.print(pid[ANGLE_ROLL].ki);   
                PcSerial.print("    ANGLE_ROLL_KD:");
                PcSerial.print(pid[ANGLE_ROLL].kd,3); 
                PcSerial.print("    outD:");
                PcSerial.print(pid[ANGLE_ROLL].outD);  
                PcSerial.print("    outI:");
                PcSerial.print(pid[ANGLE_ROLL].outI);        
                PcSerial.print("    outP:");
                PcSerial.println(pid[ANGLE_ROLL].outP);                     
                break;  

        case 11:  
                PcSerial.print("    RATE_PITCH_KP:");
                PcSerial.print(pid[RATE_PITCH].kp);
                PcSerial.print("    RATE_PITCH_KI:");
                PcSerial.print(pid[RATE_PITCH].ki);   
                PcSerial.print("    RATE_PITCH_KD:");
                PcSerial.print(pid[RATE_PITCH].kd,3);     
                PcSerial.print("    motorPWM.Servo1:");
                PcSerial.print(motorPWM.Servo1);  
                PcSerial.print("    motorPWM.Servo2:");
                PcSerial.println(motorPWM.Servo2);                                   
                break;  

        case 12:  
                PcSerial.print("    ANGLE_PITCH_KP:");
                PcSerial.print(pid[ANGLE_PITCH].kp);
                PcSerial.print("    ANGLE_PITCH_KI:");
                PcSerial.print(pid[ANGLE_PITCH].ki);   
                PcSerial.print("    ANGLE_PITCH_KD:");
                PcSerial.print(pid[ANGLE_PITCH].kd);  
                PcSerial.print("    pid[ANGLE_PITCH].integ:");
                PcSerial.print(pid[ANGLE_PITCH].integ);   
                PcSerial.print("    pid[ANGLE_PITCH].outI:");
                PcSerial.print(pid[ANGLE_PITCH].outI);   
                PcSerial.print("    ANGLE_PITCH_output:");
                PcSerial.println(pid[ANGLE_PITCH].output);                 
                break;  

        case 13: 
                PcSerial.print("    RATE_YAW _KP:");
                PcSerial.print(pid[RATE_YAW ].kp);
                PcSerial.print("    RATE_YAW _KI:");
                PcSerial.print(pid[RATE_YAW ].ki);   
                PcSerial.print("    RATE_YAW _KD:");
                PcSerial.print(pid[RATE_YAW ].kd,3);   
                PcSerial.print("    pid[RATE_YAW].integ:");
                PcSerial.print(pid[RATE_YAW].integ);   
                PcSerial.print("    pid[ANGLE_PITCH].outI:");
                PcSerial.print(pid[RATE_YAW].outI);  
                PcSerial.print("    RATE_YAW_output:");
                PcSerial.println(pid[RATE_YAW].output);           
                break; 
                
        case 14: 
                PcSerial.print("    yaw:");
                PcSerial.print(Telecontrol_yaw);
                PcSerial.print("    pitch:");
                PcSerial.print(Telecontrol_pitch);   
                PcSerial.print("    roll:");
                PcSerial.print(Telecontrol_roll); 
                PcSerial.print("    thrust:");
                PcSerial.print(Telecontrol_thrust);     
                PcSerial.print("    SWA:");
                PcSerial.print(Telecontrol_SWA);  
                PcSerial.print("    SWB:");
                PcSerial.print(Telecontrol_SWB);   
                PcSerial.print("    SWC:");
                PcSerial.print(Telecontrol_SWC);  
                PcSerial.print("    SWD:");
                PcSerial.print(Telecontrol_SWD);   
                PcSerial.print("    VRA:");
                PcSerial.print(Telecontrol_VRA);  
                PcSerial.print("    VRB:");
                PcSerial.println(Telecontrol_VRB);        

                 
                break; 
                
        case 15:
                PcSerial.print("    pid[ANGLE_ROLL].error:");
                PcSerial.println(pid[ANGLE_ROLL].error);             
                break;        

        case 16:
                PcSerial.print("    pid[ANGLE_PITCH].error:");
                PcSerial.println(pid[ANGLE_PITCH].error);  
                break;   

        case 17:
                PcSerial.print("    state.acc.x:");
                PcSerial.print(state.acc.x);
                PcSerial.print("    state.acc.y:");
                PcSerial.print(state.acc.y);  
                PcSerial.print("    state.acc.z:");
                PcSerial.println(state.acc.z);    
                break;       

        case 18:
                PcSerial.print("    state.Velocity.x:");
                PcSerial.print(state.Velocity.x);
                PcSerial.print("    state.Velocity.y:");
                PcSerial.print(state.Velocity.y);  
                PcSerial.print("    state.Velocity.z:");
                PcSerial.println(state.Velocity.z);    
                break;   

        case 19:
                PcSerial.print("    state.Position.x:");
                PcSerial.print(state.Position.x);
                PcSerial.print("    state.Position.y:");
                PcSerial.print(state.Position.y);  
                PcSerial.print("    state.Position.z:");
                PcSerial.println(state.Position.z);    
                break;                                

        case 20:
                PcSerial.print(" ultrasonic:");  
                PcSerial.print(ultrasonic.Len_cm); 
                PcSerial.print(" baro.pressure:");  
                PcSerial.print(baro.pressure); 
                PcSerial.print(" baro.local_pressure:");  
                PcSerial.print(baro.local_pressure); 
                PcSerial.print(" alt:");  
                PcSerial.print(baro.alt); 
                PcSerial.print("    state.Position.z:");
                PcSerial.println(state.Position.z);   
                
                break;     

        case 21:
                
                PcSerial.print(" ultrasonic:");  
                PcSerial.print(ultrasonic.Len_cm); 
                PcSerial.print("    state.Position.z:");
                PcSerial.println(state.Position.z);   
   
                break;    

        case 22:
                
                PcSerial.print("    VELOCITY_Z _KP:");
                PcSerial.print(pid[VELOCITY_Z ].kp);
                PcSerial.print("    VELOCITY_Z _KI:");
                PcSerial.print(pid[VELOCITY_Z ].ki);   
                PcSerial.print("    VELOCITY_Z _KD:");
                PcSerial.print(pid[VELOCITY_Z ].kd);   
                PcSerial.print("    integ_start:");
                PcSerial.print(pid[VELOCITY_Z].integ_start);  
                PcSerial.print("    integ:");
                PcSerial.print(pid[VELOCITY_Z].integ);   
                PcSerial.print("    outI:");
                PcSerial.print(pid[VELOCITY_Z].outI);  
                PcSerial.print("    pid[VELOCITY_Z].output:");
                PcSerial.println(pid[VELOCITY_Z].output);    
                break;    

        case 23:
                
                PcSerial.print("    pid[VELOCITY_Z].error:");
                PcSerial.print(pid[VELOCITY_Z].error);   
                PcSerial.print("    pid[VELOCITY_Z].output:");
                PcSerial.println(pid[VELOCITY_Z].output);    
                break;    


        case 24:
                PcSerial.print(" ultrasonic:");  
                PcSerial.print(ultrasonic.Len_cm); 
                PcSerial.print(" alt:");  
                PcSerial.print(baro.alt); 
                PcSerial.print("    state.Position.z:");
                PcSerial.print(state.Position.z);   
                PcSerial.print("    state.Velocity.z:");
                PcSerial.println(state.Velocity.z);   
                
                break;   

        case 25:
               
                PcSerial.print(" alt:");  
                PcSerial.print(baro.alt); 
                PcSerial.print("    baro.F_alt:");
                PcSerial.print(baro.F_alt);                  
                PcSerial.print("    state.Position.z:");
                PcSerial.println(state.Position.z);   
                break; 


        case 26:
                PcSerial.print("  state.Position.z:");
                PcSerial.print(state.Position.z); 
                PcSerial.print(" PosUltrasonic:");  
                PcSerial.println(posUltrasonic.est.pos.z);  
                
                break; 

        case 27:
               
                PcSerial.print(" RemoteControl.RemoteControlSign:");  
                PcSerial.println(RemoteControl.RemoteControlSign);  
                
                break;      

        case 28:
               
                PcSerial.print(" Light.speed_x:");  
                PcSerial.print(Light.speed_x);  
                PcSerial.print(" Light.speed_y:");  
                PcSerial.println(Light.speed_y);  
                //PcSerial.print(" Light.hz:");  
                //PcSerial.println(Light.hz); 
                
                break;                                 

        case 29:

                PcSerial.print(" posUltrasonic.est.pos.z:");  
                PcSerial.print(posUltrasonic.est.pos.z);  
                PcSerial.print(" ultrasonic.Len_cm:");  
                PcSerial.println(ultrasonic.Len_cm);  

                break; 

        case 30:
                PcSerial.print(" error:");
                PcSerial.print(pid[POSHOLD_Z ].error);
                PcSerial.print(" POSHOLD_Z_KP:");
                PcSerial.print(pid[POSHOLD_Z ].kp);
                PcSerial.print(" POSHOLD_Z_KI:");
                PcSerial.print(pid[POSHOLD_Z ].ki);   
                PcSerial.print(" POSHOLD_Z_KD:");
                PcSerial.print(pid[POSHOLD_Z ].kd);   
                PcSerial.print(" outD:");
                PcSerial.print(pid[POSHOLD_Z].outD); 
                PcSerial.print(" integ:");
                PcSerial.print(pid[POSHOLD_Z].integ);   
                PcSerial.print(" outI:");
                PcSerial.print(pid[POSHOLD_Z].outI);  
                PcSerial.print(" pid[POSHOLD_Z].outP:");
                PcSerial.print(pid[POSHOLD_Z].outP);    
                PcSerial.print(" output:");
                PcSerial.print(pid[POSHOLD_Z].output);   
                PcSerial.print(" m1:");
                PcSerial.print(motorPWM.m1);   
                PcSerial.print(" m2:");
                PcSerial.println(motorPWM.m2);   
                break;  

        case 31:
        
                PcSerial.print(" Light.hz:");  
                PcSerial.print(Light.hz); 
                PcSerial.print(" ultrasonic.hz:");  
                PcSerial.println(ultrasonic.hz); 
               
                break;  

        case 32:
                PcSerial.print(" ch1:"); 
                PcSerial.print(Telecontrol_dat[0]);   
                PcSerial.print(" ch2:"); 
                PcSerial.print(Telecontrol_dat[1]);  
                PcSerial.print(" ch3:"); 
                PcSerial.print(Telecontrol_dat[2]);   
                PcSerial.print(" ch4:"); 
                PcSerial.print(Telecontrol_dat[3]);   
                PcSerial.print(" ch5:"); 
                PcSerial.print(Telecontrol_dat[4]);   
                PcSerial.print(" ch6:"); 
                PcSerial.print(Telecontrol_dat[5]);   
                PcSerial.print(" ch7:"); 
                PcSerial.print(Telecontrol_dat[6]);   
                PcSerial.print(" ch8:");   
                PcSerial.print(Telecontrol_dat[7]);  
                PcSerial.print(" ch9:"); 
                PcSerial.print(Telecontrol_dat[8]);   
                PcSerial.print(" ch10:");   
                PcSerial.println(Telecontrol_dat[9]);                       
                break;                  

        case 33:
                
                PcSerial.print("    VELOCITY_X _KP:");
                PcSerial.print(pid[VELOCITY_X ].kp);
                PcSerial.print("    VELOCITY_X _KI:");
                PcSerial.print(pid[VELOCITY_X ].ki);   
                PcSerial.print("    VELOCITY_X _KD:");
                PcSerial.print(pid[VELOCITY_X ].kd);   
                PcSerial.print("    integ_start:");
                PcSerial.print(pid[VELOCITY_X].integ_start);  
                PcSerial.print("    integ:");
                PcSerial.print(pid[VELOCITY_X].integ);   
                PcSerial.print("    outI:");
                PcSerial.print(pid[VELOCITY_X].outI);  
                PcSerial.print("    outP:");
                PcSerial.print(pid[VELOCITY_X].outP);  
                PcSerial.print("    pid[VELOCITY_X].output:");
                PcSerial.println(pid[VELOCITY_X].output);    
                break;  
                

        case 34:
                
                PcSerial.print("    VELOCITY_Y _KP:");
                PcSerial.print(pid[VELOCITY_Y ].kp);
                PcSerial.print("    VELOCITY_Y _KI:");
                PcSerial.print(pid[VELOCITY_Y ].ki);   
                PcSerial.print("    VELOCITY_Y _KD:");
                PcSerial.print(pid[VELOCITY_Y ].kd);   
                PcSerial.print("    integ_start:");
                PcSerial.print(pid[VELOCITY_Y].integ_start);  
                PcSerial.print("    integ:");
                PcSerial.print(pid[VELOCITY_Y].integ);   
                PcSerial.print("    outI:");
                PcSerial.print(pid[VELOCITY_Y].outI);  
                PcSerial.print("    outP:");
                PcSerial.print(pid[VELOCITY_Y].outP);  
                PcSerial.print("    pid[VELOCITY_Y].output:");
                PcSerial.println(pid[VELOCITY_Y].output);    
                break;  

        case 35:
                
                PcSerial.print(" Light.speed_x:");  
                PcSerial.print(Light.speed_x);  
                PcSerial.print(" Light.speed_y:");  
                PcSerial.print(Light.speed_y); 
                PcSerial.print(" ultrasonic.Len_cm:");  
                PcSerial.print(ultrasonic.Len_cm);  
                PcSerial.print(" baro.alt:");  
                PcSerial.println(baro.alt); 
                
                break;  

        case 36:
                PcSerial.print("    accx:");
                PcSerial.print(sensorData.acc.x);
                PcSerial.print("    accy:");
                PcSerial.print(sensorData.acc.y);
                PcSerial.print("    accz:");
                PcSerial.print(sensorData.acc.z); 
                PcSerial.print("    roll:"); 
                PcSerial.print(state.attitude.roll);
                PcSerial.print("    pitch:"); 
                PcSerial.print(state.attitude.pitch);
                PcSerial.print("    ------    "); 
                PcSerial.print("    ax:");
                PcSerial.print(sin_approx(DEGREES_TO_RADIANS(state.attitude.pitch)));
                PcSerial.print("    ay:");
                PcSerial.println(sin_approx(DEGREES_TO_RADIANS(state.attitude.roll)));
                
                break;  

        case 37:
                //PcSerial.print("    px:");
                //PcSerial.print(posLight.est.pos.x);
                PcSerial.print("    py:");
                PcSerial.print(posLight.est.pos.y);      
                PcSerial.print("    lpx:");
                PcSerial.println(Light.sum_flow_x);
                //PcSerial.print("    lpy:");
                //PcSerial.println(Light.sum_flow_y);   
                          
                break;  


        case 38:
 
                PcSerial.print(" Light.speed_x:");  
                PcSerial.print(Light.speed_x);  
                //PcSerial.print(" Light.speed_y:");  
                //PcSerial.print(Light.speed_y); 
                //cSerial.print("    sx:");
                //PcSerial.print(posLight.est.vel.x);
                PcSerial.print("    sy:");
                PcSerial.println(accpos[Y]);   
                          
                break;  

        case 39:
                PcSerial.print("    lrx:");
                PcSerial.print(Light.radian_x);
                PcSerial.print("    lry:");
                PcSerial.println(Light.radian_y);   
                          
                break;  
        case 40:        
        
                PcSerial.print(calibratedGravity.CMSSFlashSign);
                PcSerial.print("\t");
                PcSerial.print(posEstimator.imu.gravityCalibrationComplete);
                PcSerial.print("\t");
                PcSerial.print(posEstimator.imu.accelNEU.x);
                PcSerial.print("\t");
                PcSerial.print(posEstimator.imu.accelNEU.y);
                PcSerial.print("\t");
                PcSerial.println(posEstimator.imu.accelNEU.z);           
       
                break;  

        case 41:

                PcSerial.print("    sx:");
                PcSerial.print(posLight.est.vel.x);
                PcSerial.print("    sy:");
                PcSerial.println(posLight.est.vel.y);   
                          
                break;  

        case 42:
 
                PcSerial.print(" Light.speed_x:");  
                PcSerial.print(Light.speed_x);  
                PcSerial.print(" Light.speed_y:");  
                PcSerial.println(Light.speed_y); 
                break;  

        case 43:
 
                PcSerial.print(" acc_x:");  
                PcSerial.print(posLight.imu.accelNEU.x);  
                PcSerial.print(" acc_y:");  
                PcSerial.println(posLight.imu.accelNEU.y); 
                break;  
                
        case 44:
                PcSerial.print(" error:");
                PcSerial.print(pid[POSHOLD_X ].error);
                PcSerial.print(" POSHOLD_X_KP:");
                PcSerial.print(pid[POSHOLD_X ].kp);
                PcSerial.print(" POSHOLD_X_KI:");
                PcSerial.print(pid[POSHOLD_X ].ki);   
                PcSerial.print(" POSHOLD_X_KD:");
                PcSerial.print(pid[POSHOLD_X ].kd);   
                PcSerial.print(" outD:");
                PcSerial.print(pid[POSHOLD_X].outD); 
                PcSerial.print(" integ:");
                PcSerial.print(pid[POSHOLD_X].integ);   
                PcSerial.print(" outI:");
                PcSerial.print(pid[POSHOLD_X].outI);  
                PcSerial.print(" pid[POSHOLD_X].outP:");
                PcSerial.print(pid[POSHOLD_X].outP);    
                PcSerial.print(" output:");
                PcSerial.println(pid[POSHOLD_X].output);   
                break;  

        case 45:
                PcSerial.print(" error:");
                PcSerial.print(pid[POSHOLD_Y ].error);
                PcSerial.print(" POSHOLD_Y_KP:");
                PcSerial.print(pid[POSHOLD_Y ].kp);
                PcSerial.print(" POSHOLD_Y_KI:");
                PcSerial.print(pid[POSHOLD_Y ].ki);   
                PcSerial.print(" POSHOLD_Y_KD:");
                PcSerial.print(pid[POSHOLD_Y ].kd);   
                PcSerial.print(" outD:");
                PcSerial.print(pid[POSHOLD_Y].outD); 
                PcSerial.print(" integ:");
                PcSerial.print(pid[POSHOLD_Y].integ);   
                PcSerial.print(" outI:");
                PcSerial.print(pid[POSHOLD_Y].outI);  
                PcSerial.print(" pid[POSHOLD_Y].outP:");
                PcSerial.print(pid[POSHOLD_Y].outP);    
                PcSerial.print(" output:");
                PcSerial.println(pid[POSHOLD_Y].output);   
                break;  

        case 46:
                PcSerial.print("  Xerror:");
                PcSerial.print(pid[POSHOLD_X ].error);
                PcSerial.print(" Yerror:");
                PcSerial.println(pid[POSHOLD_Y ].error); 
                break;  
                
                
        default:  

                
                break;
    }

}
