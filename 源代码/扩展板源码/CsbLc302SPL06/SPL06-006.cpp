#include "SPL06-006.h"
#include "Wire.h"

//TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

double AirPressure = 0;
double Temperature = 0;


unsigned long us=0;
unsigned long us1=0;
float dt = 0;


uint8_t SPL_CHIP_ADDRESS = 0x76;

void SPL_init()
{
  I2Ctwo.begin(33,25, 400000);//SDA1,SCL1
    
  i2c_eeprom_write_uint8_t(SPL_CHIP_ADDRESS, 0X06, 0x43); // Pressure 8x oversampling 配置压力测量速率(PM_RATE)和分辨率(PM_PRC)。

  i2c_eeprom_write_uint8_t(SPL_CHIP_ADDRESS, 0X07, 0Xc3); // Temperature 8x oversampling 配置温度测量速率(TMP_RATE)和分辨率(TMP_PRC)。

	i2c_eeprom_write_uint8_t(SPL_CHIP_ADDRESS, 0X08, 0B0111);	// continuous temp and pressure measurement 设置测量模式 连续温度和压力测量

	i2c_eeprom_write_uint8_t(SPL_CHIP_ADDRESS, 0X09, 0X30);	// FIFO Pressure measurement  

  us1=micros();
}

uint8_t get_spl_id()
{
	return i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0x0D);	
}

uint8_t get_spl_prs_cfg()
{
	return i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0x06);
}

uint8_t get_spl_tmp_cfg()
{
	return i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0x07);
}

uint8_t get_spl_meas_cfg()
{
	return i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0x08);
}

uint8_t get_spl_cfg_reg()
{
	return i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0x09);
}

uint8_t get_spl_int_sts()
{
	return i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0x0A);
}

uint8_t get_spl_fifo_sts()
{
	return i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0x0B);
}



double get_altitude(double pressure, double seaLevelhPa) {
	double altitude;

	altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

	return altitude;
}

double get_altitude_f(double pressure, double seaLevelhPa)
{
	double altitude;

	altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

	return altitude * 3.281;
}


double get_traw_sc()
{
	int32_t traw = get_traw();
	return (double(traw)/get_temperature_scale_factor());
}


double get_temp_c()
{
	int16_t c0,c1;
	c0 = get_c0();
	c1 = get_c1();
	double traw_sc = get_traw_sc();
	return (double(c0) * 0.5f) + (double(c1) * traw_sc);
}


double get_temp_f()
{
	int16_t c0,c1;
	c0 = get_c0();
	c1 = get_c1();
	double traw_sc = get_traw_sc();
	return (((double(c0) * 0.5f) + (double(c1) * traw_sc)) * 9/5) + 32;
}


double get_temperature_scale_factor()
{
  
  double k;

  uint8_t tmp_Byte;
  tmp_Byte = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X07); // MSB

  //Serial.print("tmp_Byte: ");
  //Serial.println(tmp_Byte);

  //tmp_Byte = tmp_Byte >> 4; //Focus on bits 6-4
  tmp_Byte = tmp_Byte & 0B00000111;
  //Serial.print("tmp_Byte: ");
  //Serial.println(tmp_Byte);

  switch (tmp_Byte) 
  {
    case 0B000:
      k = 524288.0d;
    break;

    case 0B001:
      k = 1572864.0d;
    break;

    case 0B010:
      k = 3670016.0d;
    break;

    case 0B011:
      k = 7864320.0d;
    break;

    case 0B100:
      k = 253952.0d;
    break;

    case 0B101:
      k = 516096.0d;
    break;

    case 0B110:
      k = 1040384.0d;
    break;

    case 0B111:
      k = 2088960.0d;
    break;
  }

  return k;
}


int32_t get_traw()
{
  int32_t tmp;
  uint8_t tmp_MSB,tmp_LSB,tmp_XLSB;
  tmp_MSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X03); // MSB

  tmp_LSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X04); // LSB

  tmp_XLSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X05); // XLSB

  tmp = (tmp_MSB << 8) | tmp_LSB;
  tmp = (tmp << 8) | tmp_XLSB;


  if(tmp & (1 << 23))
    tmp = tmp | 0XFF000000; // Set left bits to one for 2's complement conversion of negitive number
  
  
  return tmp;
}

double get_praw_sc()
{
	int32_t praw = get_praw();
	return (double(praw)/get_pressure_scale_factor());
}

double get_pcomp()
{
	int32_t c00,c10;
	int16_t c01,c11,c20,c21,c30;
	c00 = get_c00();
	c10 = get_c10();
	c01 = get_c01();
	c11 = get_c11();
	c20 = get_c20();
	c21 = get_c21();
	c30 = get_c30();
	double traw_sc = get_traw_sc();
	double praw_sc = get_praw_sc();
	return double(c00) + praw_sc * (double(c10) + praw_sc * (double(c20) + praw_sc * double(c30))) + traw_sc * double(c01) + traw_sc * praw_sc * ( double(c11) + praw_sc * double(c21));
}

double get_pressure()
{
	double pcomp = get_pcomp();
	return pcomp / 100; // 转换为MB convert to mb
}



double get_pressure_scale_factor()
{
	double k;

	uint8_t tmp_Byte;
	tmp_Byte = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X06); // MSB

	tmp_Byte = tmp_Byte & 0B00000111; // Focus on 2-0 oversampling rate 


	switch (tmp_Byte) // oversampling rate
	{
		case 0B000:
			k = 524288.0d;
		break;

		case 0B001:
			k = 1572864.0d;
		break;

		case 0B010:
			k = 3670016.0d;
		break;

		case 0B011:
			k = 7864320.0d;
		break;

		case 0B100:
			k = 253952.0d;
		break;

		case 0B101:
			k = 516096.0d;
		break;

		case 0B110:
			k = 1040384.0d;
		break;

		case 0B111:
			k = 2088960.0d;
		break;
	}

	return k;
}




int32_t get_praw()
{
  int32_t tmp;
  uint8_t tmp_MSB,tmp_LSB,tmp_XLSB;
  tmp_MSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X00); // MSB


  tmp_LSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X01); // LSB


  tmp_XLSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X02); // XLSB

  
  tmp = (tmp_MSB << 8) | tmp_LSB;
  tmp = (tmp << 8) | tmp_XLSB;



  if(tmp & (1 << 23))
    tmp = tmp | 0XFF000000; // Set left bits to one for 2's complement conversion of negitive number
  
  
  return tmp;
}

int16_t get_c0()
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X10); 
  tmp_LSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X11); 



  tmp_LSB = tmp_LSB >> 4;


  tmp = (tmp_MSB << 4) | tmp_LSB;

  if(tmp & (1 << 11)) // Check for 2's complement negative number
    tmp = tmp | 0XF000; // Set left bits to one for 2's complement conversion of negitive number
  
  return tmp;
}


int16_t get_c1()
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X11); 
  tmp_LSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X12); 


  tmp_MSB = tmp_MSB & 0XF;


  tmp = (tmp_MSB << 8) | tmp_LSB;

  if(tmp & (1 << 11)) // Check for 2's complement negative number
    tmp = tmp | 0XF000; // Set left bits to one for 2's complement conversion of negitive number
  
  return tmp;
}

int32_t get_c00()
{
  int32_t tmp; 
  uint8_t tmp_MSB,tmp_LSB,tmp_XLSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X13); 
  tmp_LSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X14); 
  tmp_XLSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X15);


  
  tmp_XLSB = tmp_XLSB >> 4;

   
  tmp = (tmp_MSB << 8) | tmp_LSB;
  tmp = (tmp << 4) | tmp_XLSB;

  tmp = (uint32_t)tmp_MSB << 12 | (uint32_t)tmp_LSB << 4 | (uint32_t)tmp_XLSB >> 4;

  if(tmp & (1 << 19))
    tmp = tmp | 0XFFF00000; // Set left bits to one for 2's complement conversion of negitive number
    

  return tmp;
}

int32_t get_c10()
{
  int32_t tmp; 
  uint8_t tmp_MSB,tmp_LSB,tmp_XLSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X15); // 4 bits
  tmp_LSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X16); // 8 bits
  tmp_XLSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X17); // 8 bits


  tmp_MSB = tmp_MSB & 0b00001111;



  tmp = (tmp_MSB << 4) | tmp_LSB;
  tmp = (tmp << 8) | tmp_XLSB;



  tmp = (uint32_t)tmp_MSB << 16 | (uint32_t)tmp_LSB << 8 | (uint32_t)tmp_XLSB;

  if(tmp & (1 << 19))
    tmp = tmp | 0XFFF00000; // Set left bits to one for 2's complement conversion of negitive number

  return tmp;
}



int16_t get_c01()
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X18); 
  tmp_LSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X19); 

  tmp = (tmp_MSB << 8) | tmp_LSB;
  return tmp;
}

int16_t get_c11()
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X1A); 
  tmp_LSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X1B); 

  tmp = (tmp_MSB << 8) | tmp_LSB;
  return tmp;
}

int16_t get_c20()
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X1C); 
  tmp_LSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X1D); 


  tmp = (tmp_MSB << 8) | tmp_LSB;
  return tmp;
}

int16_t get_c21()
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X1E); 
  tmp_LSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X1F); 

  tmp = (tmp_MSB << 8) | tmp_LSB;
  return tmp;
}

int16_t get_c30()
{
  int16_t tmp; 
  uint8_t tmp_MSB,tmp_LSB;
  
  tmp_MSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X20); 
  tmp_LSB = i2c_eeprom_read_uint8_t(SPL_CHIP_ADDRESS, 0X21); 

  tmp = (tmp_MSB << 8) | tmp_LSB;
  return tmp;
  Serial.print("tmp: ");
  Serial.println(tmp);
}

void i2c_eeprom_write_uint8_t(  uint8_t deviceaddress, uint8_t eeaddress, uint8_t data ) 
{
    uint8_t rdata = data;
    delay(5); // Make sure to delay log enough for EEPROM I2C refresh time
    I2Ctwo.beginTransmission(deviceaddress);
    I2Ctwo.write((uint8_t)(eeaddress));
    I2Ctwo.write(rdata);
    I2Ctwo.endTransmission();
}



uint8_t i2c_eeprom_read_uint8_t(  uint8_t deviceaddress, uint8_t eeaddress ) 
{
    uint8_t rdata = 0xFF;
    I2Ctwo.beginTransmission(deviceaddress);
    I2Ctwo.write(eeaddress); 
    I2Ctwo.endTransmission(false); // false to not release the line
    
    I2Ctwo.requestFrom(deviceaddress,1);
    if (I2Ctwo.available()) rdata = I2Ctwo.read();
    return rdata;
}


void SPL06_read(void)
{
    us=micros();
    dt = (us-us1)*0.000001;
    if(dt>=0.0625)
    {
        us1 = us; 
        if(get_spl_int_sts()==3)
        {
            AirPressure = get_pressure();
            Temperature = get_temp_c();
           /*
            Serial.print(AirPressure,3);
            Serial.print("  "); 
            double local_pressure = 1003.899 ; // Look up local sea level pressure on google // Local pressure from airport website 8/22
            Serial.print(get_altitude(AirPressure,local_pressure),3);
            Serial.print(" m  "); 
        
            Serial.print(Temperature,3);
            Serial.println(" C"); 
            */
            
        }   
    }    
}
