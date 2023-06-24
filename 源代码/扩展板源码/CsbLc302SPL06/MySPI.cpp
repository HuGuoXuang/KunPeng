#include "MySPI.h" 


auto hspi = SPIClass(HSPI); //定义SPI对象
static constexpr uint32_t BUFFER_SIZE {36};
uint8_t spi_txbuf[BUFFER_SIZE];
uint8_t spi_rxbuf[BUFFER_SIZE];

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

void SpiTxDat_init(void)
{
  hspi.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_CS); //SCLK, MISO, MOSI, SS  
  pinMode(HSPI_CS, OUTPUT); //HSPI SS  
  hspi.beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));    //设置SPI的传输参数
}

void SpiTxDat(void)
{
    uint16_t crc1;//计算的CRC校验位
    
    //头字节
    spi_txbuf[0]=121;
    spi_txbuf[1]=ultrasonic.ok;
    
    //超声波数据
    if(ultrasonic.ok == 1)
    {
        spi_txbuf[2]=ultrasonic.HighLen;//高位
        spi_txbuf[3]=ultrasonic.LowLen; //低位                  
        ultrasonic.ok = 0;
    }

    //光流数据  
    spi_txbuf[4]=Light.rxbuf[0];
    spi_txbuf[5]=Light.rxbuf[1];
    spi_txbuf[6]=Light.rxbuf[2];
    spi_txbuf[7]=Light.rxbuf[3];
    spi_txbuf[8]=Light.rxbuf[4];
    spi_txbuf[9]=Light.rxbuf[5];
    spi_txbuf[10]=Light.rxbuf[6];
    spi_txbuf[11]=Light.rxbuf[7];
    spi_txbuf[12]=Light.rxbuf[8];
    spi_txbuf[13]=Light.rxbuf[9];
    spi_txbuf[14]=Light.rxbuf[10];
    spi_txbuf[15]=Light.rxbuf[11];
    spi_txbuf[16]=Light.rxbuf[12];
    spi_txbuf[17]=Light.rxbuf[13];  

    //气压计
    spi_txbuf[18]=((uint8_t *)&AirPressure)[0]; //
    spi_txbuf[19]=((uint8_t *)&AirPressure)[1]; 
    spi_txbuf[20]=((uint8_t *)&AirPressure)[2]; 
    spi_txbuf[21]=((uint8_t *)&AirPressure)[3];     
    spi_txbuf[22]=((uint8_t *)&AirPressure)[4]; 
    spi_txbuf[23]=((uint8_t *)&AirPressure)[5]; 
    spi_txbuf[24]=((uint8_t *)&AirPressure)[6]; 
    spi_txbuf[25]=((uint8_t *)&AirPressure)[7];   

    //温度 
    spi_txbuf[26]=((uint8_t *)&Temperature)[0]; //
    spi_txbuf[27]=((uint8_t *)&Temperature)[1]; 
    spi_txbuf[28]=((uint8_t *)&Temperature)[2]; 
    spi_txbuf[29]=((uint8_t *)&Temperature)[3];     
    spi_txbuf[30]=((uint8_t *)&Temperature)[4]; 
    spi_txbuf[31]=((uint8_t *)&Temperature)[5]; 
    spi_txbuf[32]=((uint8_t *)&Temperature)[6]; 
    spi_txbuf[33]=((uint8_t *)&Temperature)[7];                     
    
    crc1=CRC16_2(&spi_txbuf[1],33); //获取CRC校验位
    
    spi_txbuf[34]=crc1/256;//寄存器个数高位
    spi_txbuf[35]=crc1%256;//寄存器个数低位

    digitalWrite(HSPI_CS, LOW); //片选
    hspi.transferBytes(spi_txbuf, spi_rxbuf, BUFFER_SIZE);             //传输数据
    digitalWrite(HSPI_CS, HIGH); //片选拉高
    hspi.endTransaction();           //结束传输
    /*
    for(int i = 0; i<BUFFER_SIZE;i++)
    {
        Serial.print(spi_txbuf[i]);
        Serial.print(" ");  
    }
    Serial.println();
    */
}
