#include "MySPI.h" 

ESP32SPISlave slave;   //定义一个从机对象

pt1Filter_t pt1FBaro;
pt1Filter_t pt1FLight[2];
biquadFilter_t biquadLight[6];//二阶低通滤波器

int hdn = 4;

Light_t Light;//光流
ultrasonic_t ultrasonic;//超声波
baro_t baro;//气压

//定义发送和接收的缓冲区，最大支64字节
static constexpr uint32_t BUFFER_SIZE {36};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];
uint8_t spi_slave_tx_buf1[BUFFER_SIZE];
uint8_t spi_slave_rx_buf1[BUFFER_SIZE];
uint8_t spi_slave_tx_buf2[BUFFER_SIZE];
uint8_t spi_slave_rx_buf2[BUFFER_SIZE];


void SpiRxDat_init(void)
{
  //设置时钟极性和相位
  slave.setDataMode(SPI_MODE0);
  //设置SPI相关引脚并占用总线
  slave.begin(HSPI,HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_CS); //SCLK, MISO, MOSI, SS  

  slave.queue(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);
  slave.queue(spi_slave_rx_buf1, spi_slave_tx_buf1, BUFFER_SIZE);
  slave.queue(spi_slave_rx_buf2, spi_slave_tx_buf2, BUFFER_SIZE);
  Light.Time = millis();

  ultrasonic.begin_cm_bias = 0;
  baro.local_pressure = 1010.94;
  pt1FBaro.k = 0.1;
  pt1FLight[0].k = 0.9;
  pt1FLight[1].k = 0.9;
  Light.coefficient = 0.0195;
  biquadFilterInitLPF(&biquadLight[0],200, 500);
  biquadFilterInitLPF(&biquadLight[1],200, 500);
  biquadFilterInitLPF(&biquadLight[2],80, 500);
  biquadFilterInitLPF(&biquadLight[3],80, 500);

  biquadFilterInitLPF(&biquadLight[4],30, 500);
  biquadFilterInitLPF(&biquadLight[5],30, 500);  
  
}

double Get_altitude(double pressure, double seaLevelhPa) {
  double altitude;

  altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return altitude;
}

unsigned int CRC16_2(unsigned char *buf, int len)
{
    unsigned int crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++)
    {
        crc ^= (unsigned int)buf[pos]; // XOR byte into least sig. byte of crc
        for (int i = 8; i != 0; i--)   // Loop over each bit
        {
            if ((crc & 0x0001) != 0)   // If the LSB is set
            {
                crc >>= 1; // Shift right and XOR 0xA001
                crc ^= 0xA001;
            }
            else // Else LSB is not set
            {
                crc >>= 1;    // Just shift right
            }
        }
    }
 
    //高低字节转换
    crc = ((crc & 0x00ff) << 8) | ((crc & 0xff00) >> 8);
    return crc;
}


boolean crchs(unsigned char buffer[])
{
  uint16_t crc1=CRC16_2(&buffer[1],33); //获取CRC校验位
  unsigned char dat34=crc1/256;//寄存器个数高位
  unsigned char dat35=crc1%256;//寄存器个数低位

  if ((dat34 == buffer[34])&&(dat35 == buffer[35]))
    return true;
  else
    return false;
}

void SPI_Decode(const unsigned char* f_buf)
{   
    static uint16_t cs = 0;
    static uint32_t tt = millis();
    
    ultrasonic.ok = f_buf[1]; 
    if(ultrasonic.ok == 1)
    {
        ultrasonic.HighLen = f_buf[2];                   //High byte of distance
        ultrasonic.LowLen  = f_buf[3];                   //Low byte of distance
        float mm  = (float)(ultrasonic.HighLen*256 + ultrasonic.LowLen);  
        if(abs(mm)<3000)
          ultrasonic.Len_mm  = mm;   
        ultrasonic.Len_cm  = ultrasonic.Len_mm*0.1;  //相對
        ultrasonic.H = abs(ultrasonic.Len_cm)*cos_approx(DEGREES_TO_RADIANS(state.attitude.roll))*cos_approx(DEGREES_TO_RADIANS(state.attitude.pitch));  
        
        ultrasonic.begin_cm = ultrasonic.Len_cm - ultrasonic.begin_cm_bias;//相對起飛的高度
        cs++;
    }

    if((millis()-tt)>=500)
    {
        ultrasonic.hz = cs*2;
        cs = 0;  
        tt = millis();
    }
       

        
    Light.flow_x_integral       = f_buf[6] + (f_buf[7]<<8);// X 像素点累计时间内的累加位移(radians*10000) // [除以 10000 乘以高度(mm)后为实际位移(mm)] 
    Light.flow_y_integral       = f_buf[8] + (f_buf[9]<<8);// Y 像素点累计时间内的累加位移(radians*10000) // [除以 10000 乘以高度(mm)后为实际位移(mm)] 
    Light.integration_timespan  = f_buf[10] + (f_buf[11]<<8);// 上一次发送光流数据到本次发送光流数据的累计时间（us）
    Light.ground_distance       = f_buf[12] + (f_buf[13]<<8);// 预留。默认为 999（0x03E7）
    Light.valid                 = f_buf[14] ;              // 状态值:0(0x00)为光流数据不可用 //245(0xF5)为光流数据可用
    Light.Version               = f_buf[15] ; //实际为新版本中的valid
    Light.packerflag = 1;   

    //气压计
    ((uint8_t *)&baro.pressure)[0]= f_buf[18];
    ((uint8_t *)&baro.pressure)[1]= f_buf[19];
    ((uint8_t *)&baro.pressure)[2]= f_buf[20];
    ((uint8_t *)&baro.pressure)[3]= f_buf[21];
    ((uint8_t *)&baro.pressure)[4]= f_buf[22];
    ((uint8_t *)&baro.pressure)[5]= f_buf[23];
    ((uint8_t *)&baro.pressure)[6]= f_buf[24];
    ((uint8_t *)&baro.pressure)[7]= f_buf[25];

    float  barocm = (float)Get_altitude(baro.pressure,baro.local_pressure)*100 - baro.altBias;//cm
    if(abs(barocm)<3000000)
      baro.alt = barocm;

    baro.F_alt = pt1FilterApply(&pt1FBaro,baro.alt);

    //温度
    ((uint8_t *)&baro.temperature)[0]= f_buf[26];
    ((uint8_t *)&baro.temperature)[1]= f_buf[27];
    ((uint8_t *)&baro.temperature)[2]= f_buf[28];
    ((uint8_t *)&baro.temperature)[3]= f_buf[29];
    ((uint8_t *)&baro.temperature)[4]= f_buf[30];
    ((uint8_t *)&baro.temperature)[5]= f_buf[31];
    ((uint8_t *)&baro.temperature)[6]= f_buf[32];
    ((uint8_t *)&baro.temperature)[7]= f_buf[33];

    heightCalibration();
}

void heightCalibration(void)
{
   if (unlock_motor == 0) 
   {
      baro.local_pressure = baro.pressure;
      ultrasonic.begin_cm_bias = ultrasonic.Len_cm ;
      Light.sum_flow_x = 0;
      Light.sum_flow_y = 0;  
      Light.speed_x = 0;//
      Light.speed_y = 0;//      
         
   }
}


float SlidingDataX(float xdat,int n) //滑动数据
{  
  static  float dat[8];                       
  for(int i=1;i<8;i++)
  {
      dat[i-1] = dat[i];
  }  
  dat[7] = xdat;
  
  return dat[n];
}

float SlidingDataY(float xdat,int n) //滑动数据
{  
  static  float dat[8];                       
  for(int i=1;i<8;i++)
  {
      dat[i-1] = dat[i];
  }  
  dat[7] = xdat;
  
  return dat[n];
}

void LightCompensation()
{
    static uint16_t cs = 0;
    static uint32_t tt = millis();
    Light.Ts = (millis()-Light.Time)*0.001;
    Light.Time = millis();  
                
    if(Light.valid == 245)
    {

        float gx = sensorData.gyro.x*Light.coefficient;
        float gy = -sensorData.gyro.y*Light.coefficient;

        gx = SlidingDataX(gx,hdn);
        gy = SlidingDataY(gy,hdn);


       // if(abs(gx)<600)
       //   gx = biquadFilterApply(&biquadLight[2], gx);
          
       // if(abs(gy)<600)
       //   gy = biquadFilterApply(&biquadLight[3], gy);
        
        
        double radianx = (double)Light.flow_x_integral/10000/(Light.integration_timespan*0.000001);//弧度
        double radiany = (double)Light.flow_y_integral/10000/(Light.integration_timespan*0.000001); 


        //if(abs(radianx)<60)
        //  radianx = biquadFilterApply(&biquadLight[0], radianx);
          
        //if(abs(radiany)<60)
        //  radiany = biquadFilterApply(&biquadLight[1], radiany);
                
        Light.radian_x = radianx-gx; 
        Light.radian_y = radiany-gy; 

                //PcSerial.print(" gx:");  
                //PcSerial.print(gx);  
                //PcSerial.print(" gy:");  
                //PcSerial.println(gy); 
                //PcSerial.print(" lx:");  
                //PcSerial.print(Light.radian_x); 
                //PcSerial.print(" gy:");  
                //PcSerial.print(gy);  
                //PcSerial.print(" radianx:");  
                //PcSerial.println(radianx);  
                //PcSerial.print(" radiany:");  
                //PcSerial.println(radiany);      


        float Hcm = ultrasonic.Len_cm;//state.Position.z;//
        if(Hcm<0)
          Hcm = 0;
        double speedx = Light.radian_x*Hcm;//弧度乘以高 cm/s
        double speedy = Light.radian_y*Hcm;   

        if(abs(speedx)<333)
          Light.speed_x = constrain(speedx, -333.0f, 333.0f);//限制X轴的速度为150cm/s
        if(abs(speedy)<333)
          Light.speed_y = constrain(speedy, -333.0f, 333.0f);//限制X轴的速度为150cm/s

                //PcSerial.print(" x:");  
                //PcSerial.print(Light.speed_x); 
        Light.speed_x = pt1FilterApply(&pt1FLight[0],Light.speed_x);
        Light.speed_y = pt1FilterApply(&pt1FLight[1],Light.speed_y);
 
                //PcSerial.print(" fx:");  
                //PcSerial.println(Light.speed_x);    
        
        //Light.speed_x = biquadFilterApply(&biquadLight[4],Light.speed_x);
        //Light.speed_y = biquadFilterApply(&biquadLight[5],Light.speed_y);

        //Light.speed_x = applyDeadband(Light.speed_x, 5);//去除5(cm/s)死区
        //Light.speed_y = applyDeadband(Light.speed_y, 5);//去除5(cm/s)死区
        
        Light.ok = 1;
        cs++;

        Light.sum_flow_x += (double)Light.speed_x*(Light.integration_timespan*0.000001);
        Light.sum_flow_y += (double)Light.speed_y*(Light.integration_timespan*0.000001);        
    }
    else
    {
        Light.ok = 0;     
  
    }



    if((millis()-tt)>=500)
    {
        Light.hz = cs*2;
        cs = 0;  
        tt = millis();
    }
  
}


void handle1(uint8_t* rx_buf, uint8_t* tx_buf, uint32_t size1)
{
  if(crchs(rx_buf))
  {
    
    SPI_Decode(rx_buf);
    
    LightCompensation();
  
    /*
    for(int i=0; i<size1;i++) 
    {
      tx_buf[i] = rx_buf[i];
      Serial.print(tx_buf[i]);
      Serial.print(" "); 
    }
    Serial.println();   
    */   
  }
}

void SPI_queue()
{
  static int index = 0;  //目前未接收传输事务的缓冲区编号

   while (slave.available()) {
    int len = slave.size();
    //Serial.println(len);
    switch(index)
    {
      case 0:  
        slave.queue(spi_slave_rx_buf, spi_slave_tx_buf, BUFFER_SIZE);
        handle1(spi_slave_rx_buf, spi_slave_tx_buf, len);
        break;
      case 1:  
        slave.queue(spi_slave_rx_buf1, spi_slave_tx_buf1, BUFFER_SIZE);
        handle1(spi_slave_rx_buf1, spi_slave_tx_buf1, len);
        break;
      case 2:
        slave.queue(spi_slave_rx_buf2, spi_slave_tx_buf2, BUFFER_SIZE);
        handle1(spi_slave_rx_buf2, spi_slave_tx_buf2, len);
        break;
    }

    index = (index+1)%3;
    slave.pop();
  }
}
