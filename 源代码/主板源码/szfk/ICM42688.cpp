#include "Arduino.h"
#include "ICM42688.h"
#include "registers.h"


//IMU IO
#define SCL   33//13
#define SDA   25//15
#define INT1   27//0
#define ADO   23//2
#define INT2   18
#define CS   26


using namespace ICM42688reg;

/* ICM42688 object, input the I2C bus and address */
ICM42688::ICM42688(TwoWire &bus, uint8_t address) {
  _i2c = &bus; // I2C bus
  _address = address; // I2C address
  _useSPI = false; // set to use I2C
}

/* ICM42688 object, input the SPI bus and chip select pin */
ICM42688::ICM42688(SPIClass &bus, uint8_t csPin, uint32_t SPI_HS_CLK) {
  _spi = &bus; // SPI bus
  _csPin = csPin; // chip select pin
  _useSPI = true; // set to use SPI
  SPI_HS_CLOCK = SPI_HS_CLK;
}

/* starts communication with the ICM42688 */
int ICM42688::begin() {
  if( _useSPI ) { // using SPI for communication
    // use low speed SPI for register setting
    _useSPIHS = false;
    // setting CS pin to output
    pinMode(_csPin,OUTPUT);
    // setting CS pin high
    digitalWrite(_csPin,HIGH);
    // begin SPI communication
    _spi->begin();
  } else { // using I2C for communication
    // starting the I2C bus
    //_i2c->begin();
    // setting the I2C clock
   // _i2c->setClock(I2C_CLK);
      _i2c->begin(SDA, SCL);
      _i2c->setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties   
      pinMode(ADO, OUTPUT);
      pinMode(INT2, OUTPUT);
      pinMode(CS, OUTPUT);
      digitalWrite(INT2, LOW); 
      digitalWrite(ADO, LOW); 
      digitalWrite(CS, HIGH);
        
  }

  // reset the ICM42688
  reset();

  // check the WHO AM I byte
  if(whoAmI() != WHO_AM_I) {
    return -3;
  }

  // turn on accel and gyro in Low Noise (LN) Mode在低噪声(LN)模式下打开加速和陀螺
  if(writeRegister(UB0_REG_PWR_MGMT0, 0x0F) < 0) {
    return -4;
  }

  // 16G is default -- do this to set up accel resolution scaling
  int ret = setAccelFS(gpm16);
  if (ret < 0) return ret;

  // 2000DPS is default -- do this to set up gyro resolution scaling
  ret = setGyroFS(dps2000);
  if (ret < 0) return ret;

  // // disable inner filters (Notch filter, Anti-alias filter, UI filter block)
  // if (setFilters(false, false) < 0) {
  //   return -7;
  // }

  // estimate gyro bias
  //if (calibrateGyro() < 0) {
  ///  return -8;
  //}
  // successful init, return 1
  return 1;
}

/* sets the accelerometer full scale range to values other than default *///将加速度计满量程设置为默认值*/以外的值
int ICM42688::setAccelFS(AccelFS fssel) 
{
  // use low speed SPI for register setting
  _useSPIHS = false;

  setBank(0);

  // read current register value
  uint8_t reg;
  if (readRegisters(UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0) return -1;

  // only change FS_SEL in reg
  reg = (fssel << 5) | (reg & 0x1F);

  if (writeRegister(UB0_REG_ACCEL_CONFIG0, reg) < 0) return -2;

  _accelScale = static_cast<float>(1 << (4 - fssel)) / 32768.0f;
  _accelFS = fssel;

  return 1;
}

/* sets the gyro full scale range to values other than default */
int ICM42688::setGyroFS(GyroFS fssel) {
  // use low speed SPI for register setting
  _useSPIHS = false;

  setBank(0);

  // read current register value
  uint8_t reg;
  if (readRegisters(UB0_REG_GYRO_CONFIG0, 1, &reg) < 0) return -1;

  // only change FS_SEL in reg
  reg = (fssel << 5) | (reg & 0x1F);

  if (writeRegister(UB0_REG_GYRO_CONFIG0, reg) < 0) return -2;

  _gyroScale = (2000.0f / static_cast<float>(1 << fssel)) / 32768.0f;
  _gyroFS = fssel;

  return 1;
}


void ICM42688:: setGyroNotchFilterFHz(double freq, uint8_t axis) //設置陷波器
{
  uint8_t bank = 1;
  writeReg(REG_BANK_SEL, &bank, 1);
  double fdesired = freq * 1000;
  double coswz = cos(2 * 3.14 * fdesired / 32);
  int16_t nfCoswz;
  uint8_t nfCoswzSel;
  if (abs(coswz) <= 0.875) {
    nfCoswz = round(coswz * 256);
    nfCoswzSel = 0;
  } else {
    nfCoswzSel = 1;
    if (coswz > 0.875) {
      nfCoswz = round(8 * (1 - coswz) * 256);
    } else if (coswz < -0.875) {
      nfCoswz = round(-8 * (1 + coswz) * 256);
    }
  }
  if (axis == X_AXIS) {
    gyroConfigStatic9.gyroNFCoswzSelX = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzX8 = nfCoswz >> 8;
    writeReg(UB1_REG_GYRO_CONFIG_STATIC6, &nfCoswz, 1);
    writeReg(UB1_REG_GYRO_CONFIG_STATIC9, &gyroConfigStatic9, 1);
  } else if (axis == Y_AXIS) {
    gyroConfigStatic9.gyroNFCoswzSelY = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzY8 = nfCoswz >> 8;
    writeReg(UB1_REG_GYRO_CONFIG_STATIC7, &nfCoswz, 1);
    writeReg(UB1_REG_GYRO_CONFIG_STATIC9, &gyroConfigStatic9, 1);
  } else if (axis == Z_AXIS) {
    gyroConfigStatic9.gyroNFCoswzSelZ = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzZ8 = nfCoswz >> 8;
    writeReg(UB1_REG_GYRO_CONFIG_STATIC8, &nfCoswz, 1);
    writeReg(UB1_REG_GYRO_CONFIG_STATIC9, &gyroConfigStatic9, 1);
  } else if (axis == ALL)
  {
    gyroConfigStatic9.gyroNFCoswzSelX = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzX8 = nfCoswz >> 8;
    gyroConfigStatic9.gyroNFCoswzSelY = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzY8 = nfCoswz >> 8;
    gyroConfigStatic9.gyroNFCoswzSelZ = nfCoswzSel;
    gyroConfigStatic9.gyroNFCoswzZ8 = nfCoswz >> 8;
    writeReg(UB1_REG_GYRO_CONFIG_STATIC6, &nfCoswz, 1);
    writeReg(UB1_REG_GYRO_CONFIG_STATIC7, &nfCoswz, 1);
    writeReg(UB1_REG_GYRO_CONFIG_STATIC8, &nfCoswz, 1);
    writeReg(UB1_REG_GYRO_CONFIG_STATIC9, &gyroConfigStatic9, 1);
  }
  bank = 0;
  writeReg(REG_BANK_SEL, &bank, 1);
}



void ICM42688::setGyroNFbandwidth(uint8_t bw)
{
  uint8_t bank = 1;
  writeReg(REG_BANK_SEL, &bank, 1);
  uint8_t bandWidth = (bw << 4) | 0x01;
  writeReg(UB1_REG_GYRO_CONFIG_STATIC10, &bandWidth, 1);
  bank = 0;
  writeReg(REG_BANK_SEL, &bank, 1);
}



void ICM42688::setGyroNotchFilter(bool mode)
{
  if (mode) {
    gyroConfigStatic2.gyroNFDis = 0;
  } else {
    gyroConfigStatic2.gyroNFDis = 1;
  }
  uint8_t bank = 1;
  writeReg(REG_BANK_SEL, &bank, 1);
  writeReg(UB1_REG_GYRO_CONFIG_STATIC2, &gyroConfigStatic2, 1);
  bank = 0;
  writeReg(REG_BANK_SEL, &bank, 1);
}

void ICM42688::setAAFBandwidth(uint8_t who, uint8_t BWIndex) //設置二階濾波器帶寬
{
  uint8_t bank = 0;
  uint16_t AAFDeltsqr = BWIndex * BWIndex;
  if (who == GYRO) {
    bank = 1;
    writeReg(REG_BANK_SEL, &bank, 1);
    writeReg(UB1_REG_GYRO_CONFIG_STATIC3, &BWIndex, 1);
    writeReg(UB1_REG_GYRO_CONFIG_STATIC4, &AAFDeltsqr, 1);
    gyroConfigStatic5.gyroAAFDeltsqr = AAFDeltsqr >> 8;
    if (BWIndex == 1) {
      gyroConfigStatic5.gyroAAFBitshift = 15;
    } else if (BWIndex == 2) {
      gyroConfigStatic5.gyroAAFBitshift = 13;
    } else if (BWIndex == 3) {
      gyroConfigStatic5.gyroAAFBitshift = 12;
    } else if (BWIndex == 4) {
      gyroConfigStatic5.gyroAAFBitshift = 11;
    } else if (BWIndex == 5 || BWIndex == 6) {
      gyroConfigStatic5.gyroAAFBitshift = 10;
    } else if (BWIndex > 6 && BWIndex < 10) {
      gyroConfigStatic5.gyroAAFBitshift = 9;
    } else if (BWIndex > 9 && BWIndex < 14) {
      gyroConfigStatic5.gyroAAFBitshift = 8;
    } else if (BWIndex > 13 && BWIndex < 19) {
      gyroConfigStatic5.gyroAAFBitshift = 7;
    } else if (BWIndex > 18 && BWIndex < 27) {
      gyroConfigStatic5.gyroAAFBitshift = 6;
    } else if (BWIndex > 26 && BWIndex < 37) {
      gyroConfigStatic5.gyroAAFBitshift = 5;
    } else if (BWIndex > 36 && BWIndex < 53) {
      gyroConfigStatic5.gyroAAFBitshift = 4;
    } else if (BWIndex > 53 && BWIndex <= 63) {
      gyroConfigStatic5.gyroAAFBitshift = 3;
    }
    writeReg(UB1_REG_GYRO_CONFIG_STATIC5, &gyroConfigStatic5, 1);
    bank = 0;
    writeReg(REG_BANK_SEL, &bank, 1);
  } else if (who == ACCEL) {
    bank = 2;
    writeReg(REG_BANK_SEL, &bank, 1);
    accelConfigStatic2.accelAAFDelt = BWIndex;
    writeReg(UB2_REG_ACCEL_CONFIG_STATIC2, &accelConfigStatic2, 1);
    writeReg(UB2_REG_ACCEL_CONFIG_STATIC3, &AAFDeltsqr, 1);
    accelConfigStatic4.accelAAFDeltsqr = AAFDeltsqr >> 8;
    if (BWIndex == 1) {
      accelConfigStatic4.accelAAFBitshift = 15;
    } else if (BWIndex == 2) {
      accelConfigStatic4.accelAAFBitshift = 13;
    } else if (BWIndex == 3) {
      accelConfigStatic4.accelAAFBitshift = 12;
    } else if (BWIndex == 4) {
      accelConfigStatic4.accelAAFBitshift = 11;
    } else if (BWIndex == 5 || BWIndex == 6) {
      accelConfigStatic4.accelAAFBitshift = 10;
    } else if (BWIndex > 6 && BWIndex < 10) {
      accelConfigStatic4.accelAAFBitshift = 9;
    } else if (BWIndex > 9 && BWIndex < 14) {
      accelConfigStatic4.accelAAFBitshift = 8;
    } else if (BWIndex > 13 && BWIndex < 19) {
      accelConfigStatic4.accelAAFBitshift = 7;
    } else if (BWIndex > 18 && BWIndex < 27) {
      accelConfigStatic4.accelAAFBitshift = 6;
    } else if (BWIndex > 26 && BWIndex < 37) {
      accelConfigStatic4.accelAAFBitshift = 5;
    } else if (BWIndex > 36 && BWIndex < 53) {
      accelConfigStatic4.accelAAFBitshift = 4;
    } else if (BWIndex > 53 && BWIndex <= 63) {
      accelConfigStatic4.accelAAFBitshift = 3;
    }
    writeReg(UB2_REG_ACCEL_CONFIG_STATIC4, &accelConfigStatic4, 1);

    bank = 0;
    writeReg(REG_BANK_SEL, &bank, 1);
  } else if (who == ALL) {
    bank = 1;
    writeReg(REG_BANK_SEL, &bank, 1);
    writeReg(UB1_REG_GYRO_CONFIG_STATIC3, &BWIndex, 1);
    writeReg(UB1_REG_GYRO_CONFIG_STATIC4, &AAFDeltsqr, 1);
    gyroConfigStatic5.gyroAAFDeltsqr = AAFDeltsqr >> 8;
    if (BWIndex == 1) {
      gyroConfigStatic5.gyroAAFBitshift = 15;
    } else if (BWIndex == 2) {
      gyroConfigStatic5.gyroAAFBitshift = 13;
    } else if (BWIndex == 3) {
      gyroConfigStatic5.gyroAAFBitshift = 12;
    } else if (BWIndex == 4) {
      gyroConfigStatic5.gyroAAFBitshift = 11;
    } else if (BWIndex == 5 || BWIndex == 6) {
      gyroConfigStatic5.gyroAAFBitshift = 10;
    } else if (BWIndex > 6 && BWIndex < 10) {
      gyroConfigStatic5.gyroAAFBitshift = 9;
    } else if (BWIndex > 9 && BWIndex < 14) {
      gyroConfigStatic5.gyroAAFBitshift = 8;
    } else if (BWIndex > 13 && BWIndex < 19) {
      gyroConfigStatic5.gyroAAFBitshift = 7;
    } else if (BWIndex > 18 && BWIndex < 27) {
      gyroConfigStatic5.gyroAAFBitshift = 6;
    } else if (BWIndex > 26 && BWIndex < 37) {
      gyroConfigStatic5.gyroAAFBitshift = 5;
    } else if (BWIndex > 36 && BWIndex < 53) {
      gyroConfigStatic5.gyroAAFBitshift = 4;
    } else if (BWIndex > 53 && BWIndex <= 63) {
      gyroConfigStatic5.gyroAAFBitshift = 3;
    }
    writeReg(UB1_REG_GYRO_CONFIG_STATIC5, &gyroConfigStatic5, 1);
    bank = 2;
    writeReg(REG_BANK_SEL, &bank, 1);
    accelConfigStatic2.accelAAFDelt = BWIndex;
    writeReg(UB2_REG_ACCEL_CONFIG_STATIC2, &accelConfigStatic2, 1);
    writeReg(UB2_REG_ACCEL_CONFIG_STATIC3, &AAFDeltsqr, 1);
    accelConfigStatic4.accelAAFDeltsqr = AAFDeltsqr >> 8;
    if (BWIndex == 1) {
      accelConfigStatic4.accelAAFBitshift = 15;
    } else if (BWIndex == 2) {
      accelConfigStatic4.accelAAFBitshift = 13;
    } else if (BWIndex == 3) {
      accelConfigStatic4.accelAAFBitshift = 12;
    } else if (BWIndex == 4) {
      accelConfigStatic4.accelAAFBitshift = 11;
    } else if (BWIndex == 5 || BWIndex == 6) {
      accelConfigStatic4.accelAAFBitshift = 10;
    } else if (BWIndex > 6 && BWIndex < 10) {
      accelConfigStatic4.accelAAFBitshift = 9;
    } else if (BWIndex > 9 && BWIndex < 14) {
      accelConfigStatic4.accelAAFBitshift = 8;
    } else if (BWIndex > 13 && BWIndex < 19) {
      accelConfigStatic4.accelAAFBitshift = 7;
    } else if (BWIndex > 18 && BWIndex < 27) {
      accelConfigStatic4.accelAAFBitshift = 6;
    } else if (BWIndex > 26 && BWIndex < 37) {
      accelConfigStatic4.accelAAFBitshift = 5;
    } else if (BWIndex > 36 && BWIndex < 53) {
      accelConfigStatic4.accelAAFBitshift = 4;
    } else if (BWIndex > 53 && BWIndex <= 63) {
      accelConfigStatic4.accelAAFBitshift = 3;
    }
    writeReg(UB2_REG_ACCEL_CONFIG_STATIC4, &accelConfigStatic4, 1);
    bank = 0;
    writeReg(REG_BANK_SEL, &bank, 1);
  }
}



void ICM42688::setAAF(uint8_t who, bool mode)
{
  uint8_t bank = 0;
  if (who == GYRO) {
    if (mode) {
      gyroConfigStatic2.gyroAAFDis = 0;
    } else {
      gyroConfigStatic2.gyroAAFDis = 1;
    }
    bank = 1;
    writeReg(REG_BANK_SEL, &bank, 1);
    writeReg(UB1_REG_GYRO_CONFIG_STATIC2, &gyroConfigStatic2, 1);
  } else if (who == ACCEL) {
    if (mode) {
      accelConfigStatic2.accelAAFDis = 0;
    } else {
      accelConfigStatic2.accelAAFDis = 1;
    }
    bank = 2;
    writeReg(REG_BANK_SEL, &bank, 1);
    writeReg(UB2_REG_ACCEL_CONFIG_STATIC2, &accelConfigStatic2, 1);
  } else if (who == ALL) {
    if (mode) {
      gyroConfigStatic2.gyroAAFDis = 0;
      accelConfigStatic2.accelAAFDis = 0;
    } else {
      gyroConfigStatic2.gyroAAFDis = 1;
      accelConfigStatic2.accelAAFDis = 1;
    }
    bank = 1;
    writeReg(REG_BANK_SEL, &bank, 1);
    writeReg(UB1_REG_GYRO_CONFIG_STATIC2, &gyroConfigStatic2, 1);
    bank = 2;
    writeReg(REG_BANK_SEL, &bank, 1);
    writeReg(UB2_REG_ACCEL_CONFIG_STATIC2, &accelConfigStatic2, 1);
  }
  bank = 0;
  writeReg(REG_BANK_SEL, &bank, 1);
}


bool ICM42688::setUIFilter(uint8_t who, uint8_t filterOrder , uint8_t UIFilterIndex)
{
  bool ret = true;
  setBank(0);
  if (filterOrder > 3 || UIFilterIndex > 15) {
    ret = false;
  } else {
    if (who == GYRO) {
      gyroConfig1.gyroUIFiltODR = filterOrder;
      writeReg(UB0_REG_GYRO_CONFIG1, &gyroConfig1, 1);
      gyroAccelConfig0.gyroUIFiltBW = UIFilterIndex;
      writeReg(UB0_REG_GYRO_ACCEL_CONFIG0, &gyroAccelConfig0, 1);
    } else if (who == ACCEL) {
      accelConfig1.accelUIFiltORD = filterOrder;
      writeReg(UB0_REG_ACCEL_CONFIG1, &accelConfig1, 1);
      gyroAccelConfig0.accelUIFiltBW = UIFilterIndex;
      writeReg(UB0_REG_GYRO_ACCEL_CONFIG0, &gyroAccelConfig0, 1);
    } else if (who == ALL) {
      gyroConfig1.gyroUIFiltODR = filterOrder;
      writeReg(UB0_REG_GYRO_CONFIG1, &gyroConfig1, 1);
      accelConfig1.accelUIFiltORD = filterOrder;
      writeReg(UB0_REG_ACCEL_CONFIG1, &accelConfig1, 1);
      gyroAccelConfig0.gyroUIFiltBW = UIFilterIndex;
      gyroAccelConfig0.accelUIFiltBW = UIFilterIndex;
      writeReg(UB0_REG_GYRO_ACCEL_CONFIG0, &gyroAccelConfig0, 1);
    }
  }
  return ret;
}


bool ICM42688::setODRAndFSR(uint8_t who, uint8_t ODR, uint8_t FSR)
{
  bool ret = true;
  
  setBank(0);
  
  if (who == GYRO) {
    if (ODR > 15 || FSR > FSR_7) 
    {
      ret = false;
    } 
    else 
    {
      gyroConfig0.gyroODR = ODR;
      gyroConfig0.gyroFsSel = FSR;
      writeReg(UB0_REG_GYRO_CONFIG0, &gyroConfig0, 1);
   
      switch (FSR) 
      {
        case FSR_0:
          _gyroRange = 4000 / 65535.0;
          break;
        case FSR_1:
          _gyroRange = 2000 / 65535.0;
          _gyroScale = _gyroRange;
          _gyroFS = dps2000;
          break;
        case FSR_2:
          _gyroRange = 1000 / 65535.0;
          _gyroScale = _gyroRange;
          _gyroFS = dps1000;
          break;
        case FSR_3:
          _gyroRange = 500 / 65535.0;
          setGyroFS(dps500);
          break;
        case FSR_4:
          _gyroRange = 250 / 65535.0;
          _gyroScale = _gyroRange;
          _gyroFS = dps250;
          break;
        case FSR_5:
          _gyroRange = 125 / 65535.0;
          _gyroScale = _gyroRange;
          _gyroFS = dps125;
          break;
        case FSR_6:
          _gyroRange = 62.5 / 65535.0;
          _gyroScale = _gyroRange;
          _gyroFS = dps62_5;
          break;
        case FSR_7:
          _gyroRange = 31.25 / 65535.0;
          _gyroScale = _gyroRange;
          _gyroFS = dps31_25;
          break;
      }

    }
  } 
  else if (who == ACCEL) 
  {
    if (ODR > 15 || FSR > FSR_3) 
    {
      ret = false;
    } else {
      accelConfig0.accelODR = ODR;
      accelConfig0.accelFsSel = FSR;
      writeReg(UB0_REG_ACCEL_CONFIG0, &accelConfig0, 1);
      
      switch (FSR) {
        case FSR_0:
          _accelRange = 0.488f;
          setAccelFS(gpm16);
          break;
        case FSR_1:
          _accelRange = 0.2441f;
          setAccelFS(gpm8);
          break;
        case FSR_2:
          _accelRange = 0.122f;
          setAccelFS(gpm4);
          break;
        case FSR_3:
          _accelRange = 0.061f;
          setAccelFS(gpm2);
          break;
      }
    }
  }
  return ret;
}





int ICM42688::setAccelODR(ODR odr) {
  // use low speed SPI for register setting
  _useSPIHS = false;

  setBank(0);

  // read current register value
  uint8_t reg;
  if (readRegisters(UB0_REG_ACCEL_CONFIG0, 1, &reg) < 0) return -1;

  // only change ODR in reg
  reg = odr | (reg & 0xF0);

  if (writeRegister(UB0_REG_ACCEL_CONFIG0, reg) < 0) return -2;

  return 1;
}

int ICM42688::setGyroODR(ODR odr) {
  // use low speed SPI for register setting
  _useSPIHS = false;

  setBank(0);

  // read current register value
  uint8_t reg;
  if (readRegisters(UB0_REG_GYRO_CONFIG0, 1, &reg) < 0) return -1;

  // only change ODR in reg
  reg = odr | (reg & 0xF0);

  if (writeRegister(UB0_REG_GYRO_CONFIG0, reg) < 0) return -2;

  return 1;
}

int ICM42688::setFilters(bool gyroFilters, bool accFilters) {
  if (setBank(1) < 0) return -1;

  if (gyroFilters == true) {
    if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_ENABLE | GYRO_AAF_ENABLE) < 0) {
      return -2;
    }
  }
  else {
    if (writeRegister(UB1_REG_GYRO_CONFIG_STATIC2, GYRO_NF_DISABLE | GYRO_AAF_DISABLE) < 0) {
      return -3;
    }
  }
  
  if (setBank(2) < 0) return -4;

  if (accFilters == true) {
    if (writeRegister(UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_ENABLE) < 0) {
      return -5;
    }
  }
  else {
    if (writeRegister(UB2_REG_ACCEL_CONFIG_STATIC2, ACCEL_AAF_DISABLE) < 0) {
      return -6;
    }
  }
  if (setBank(0) < 0) return -7;
  return 1;
}

int ICM42688::enableDataReadyInterrupt() {
  // use low speed SPI for register setting
  _useSPIHS = false;

  // push-pull, pulsed, active HIGH interrupts
  if (writeRegister(UB0_REG_INT_CONFIG, 0x18 | 0x03) < 0) return -1;

  // need to clear bit 4 to allow proper INT1 and INT2 operation
  uint8_t reg;
  if (readRegisters(UB0_REG_INT_CONFIG1, 1, &reg) < 0) return -2;
  reg &= ~0x10;
  if (writeRegister(UB0_REG_INT_CONFIG1, reg) < 0) return -3;

  // route UI data ready interrupt to INT1
  if (writeRegister(UB0_REG_INT_SOURCE0, 0x18) < 0) return -4;

  return 1;
}

int ICM42688::disableDataReadyInterrupt() {
  // use low speed SPI for register setting
  _useSPIHS = false;

  // set pin 4 to return to reset value
  uint8_t reg;
  if (readRegisters(UB0_REG_INT_CONFIG1, 1, &reg) < 0) return -1;
  reg |= 0x10;
  if (writeRegister(UB0_REG_INT_CONFIG1, reg) < 0) return -2;

  // return reg to reset value
  if (writeRegister(UB0_REG_INT_SOURCE0, 0x10) < 0) return -3;

  return 1;
}

/* reads the most current data from ICM42688 and stores in buffer */
int ICM42688::getAGT() {
  _useSPIHS = true; // use the high speed SPI for data readout
  // grab the data from the ICM42688
  if (readRegisters(UB0_REG_TEMP_DATA1, 14, _buffer) < 0) return -1;

  // combine bytes into 16 bit values
  int16_t rawMeas[7]; // temp, accel xyz, gyro xyz
  for (size_t i=0; i<7; i++) {
    rawMeas[i] = ((int16_t)_buffer[i*2] << 8) | _buffer[i*2+1];
  }

  _t = (static_cast<float>(rawMeas[0]) / TEMP_DATA_REG_SCALE) + TEMP_OFFSET;

  _acc[0] = ((rawMeas[1] * _accelScale) - _accB[0]) * _accS[0];
  _acc[1] = ((rawMeas[2] * _accelScale) - _accB[1]) * _accS[1];
  _acc[2] = ((rawMeas[3] * _accelScale) - _accB[2]) * _accS[2];

  _acc1[0] = (rawMeas[1] - _accB[0]) * _accS[0];
  _acc1[1] = (rawMeas[2] - _accB[1]) * _accS[1];
  _acc1[2] = (rawMeas[3] - _accB[2]) * _accS[2];

  _gyr[0] = (rawMeas[4] * _gyroScale) - _gyrB[0];
  _gyr[1] = (rawMeas[5] * _gyroScale) - _gyrB[1];
  _gyr[2] = (rawMeas[6] * _gyroScale) - _gyrB[2];

  _gyr1[0] = rawMeas[4] - _gyrB[0];
  _gyr1[1] = rawMeas[5] - _gyrB[1];
  _gyr1[2] = rawMeas[6] - _gyrB[2];


  return 1;
}



/* configures and enables the FIFO buffer  */
int ICM42688_FIFO::enableFifo(bool accel,bool gyro,bool temp) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  if(writeRegister(FIFO_EN,(accel*FIFO_ACCEL)|(gyro*FIFO_GYRO)|(temp*FIFO_TEMP_EN)) < 0) {
    return -2;
  }
  _enFifoAccel = accel;
  _enFifoGyro = gyro;
  _enFifoTemp = temp;
  _fifoFrameSize = accel*6 + gyro*6 + temp*2;
  return 1;
}

/* reads data from the ICM42688 FIFO and stores in buffer */
int ICM42688_FIFO::readFifo() {
  _useSPIHS = true; // use the high speed SPI for data readout
  // get the fifo size
  readRegisters(UB0_REG_FIFO_COUNTH, 2, _buffer);
  _fifoSize = (((uint16_t) (_buffer[0]&0x0F)) <<8) + (((uint16_t) _buffer[1]));
  // read and parse the buffer
  for (size_t i=0; i < _fifoSize/_fifoFrameSize; i++) {
    // grab the data from the ICM42688
    if (readRegisters(UB0_REG_FIFO_DATA, _fifoFrameSize, _buffer) < 0) {
      return -1;
    }
    if (_enFifoAccel) {
      // combine into 16 bit values
      int16_t rawMeas[3];
      rawMeas[0] = (((int16_t)_buffer[0]) << 8) | _buffer[1];
      rawMeas[1] = (((int16_t)_buffer[2]) << 8) | _buffer[3];
      rawMeas[2] = (((int16_t)_buffer[4]) << 8) | _buffer[5];
      // transform and convert to float values
      _axFifo[i] = ((rawMeas[0] * _accelScale) - _accB[0]) * _accS[0];
      _ayFifo[i] = ((rawMeas[1] * _accelScale) - _accB[1]) * _accS[1];
      _azFifo[i] = ((rawMeas[2] * _accelScale) - _accB[2]) * _accS[2];
      _aSize = _fifoSize / _fifoFrameSize;
    }
    if (_enFifoTemp) {
      // combine into 16 bit values
      int16_t rawMeas = (((int16_t)_buffer[0 + _enFifoAccel*6]) << 8) | _buffer[1 + _enFifoAccel*6];
      // transform and convert to float values
      _tFifo[i] = (static_cast<float>(rawMeas) / TEMP_DATA_REG_SCALE) + TEMP_OFFSET;
      _tSize = _fifoSize/_fifoFrameSize;
    }
    if (_enFifoGyro) {
      // combine into 16 bit values
      int16_t rawMeas[3];
      rawMeas[0] = (((int16_t)_buffer[0 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[1 + _enFifoAccel*6 + _enFifoTemp*2];
      rawMeas[1] = (((int16_t)_buffer[2 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[3 + _enFifoAccel*6 + _enFifoTemp*2];
      rawMeas[2] = (((int16_t)_buffer[4 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[5 + _enFifoAccel*6 + _enFifoTemp*2];
      // transform and convert to float values
      _gxFifo[i] = (rawMeas[0] * _gyroScale) - _gyrB[0];
      _gyFifo[i] = (rawMeas[1] * _gyroScale) - _gyrB[1];
      _gzFifo[i] = (rawMeas[2] * _gyroScale) - _gyrB[2];
      _gSize = _fifoSize/_fifoFrameSize;
    }
  }
  return 1;
}

/* returns the accelerometer FIFO size and data in the x direction, m/s/s */
void ICM42688_FIFO::getFifoAccelX_mss(size_t *size,float* data) {
  *size = _aSize;
  memcpy(data,_axFifo,_aSize*sizeof(float));
}

/* returns the accelerometer FIFO size and data in the y direction, m/s/s */
void ICM42688_FIFO::getFifoAccelY_mss(size_t *size,float* data) {
  *size = _aSize;
  memcpy(data,_ayFifo,_aSize*sizeof(float));
}

/* returns the accelerometer FIFO size and data in the z direction, m/s/s */
void ICM42688_FIFO::getFifoAccelZ_mss(size_t *size,float* data) {
  *size = _aSize;
  memcpy(data,_azFifo,_aSize*sizeof(float));
}

/* returns the gyroscope FIFO size and data in the x direction, dps */
void ICM42688_FIFO::getFifoGyroX(size_t *size,float* data) {
  *size = _gSize;
  memcpy(data,_gxFifo,_gSize*sizeof(float));
}

/* returns the gyroscope FIFO size and data in the y direction, dps */
void ICM42688_FIFO::getFifoGyroY(size_t *size,float* data) {
  *size = _gSize;
  memcpy(data,_gyFifo,_gSize*sizeof(float));
}

/* returns the gyroscope FIFO size and data in the z direction, dps */
void ICM42688_FIFO::getFifoGyroZ(size_t *size,float* data) {
  *size = _gSize;
  memcpy(data,_gzFifo,_gSize*sizeof(float));
}

/* returns the die temperature FIFO size and data, C */
void ICM42688_FIFO::getFifoTemperature_C(size_t *size,float* data) {
  *size = _tSize;
  memcpy(data,_tFifo,_tSize*sizeof(float));
}

/* estimates the gyro biases *///估计陀螺偏差
int ICM42688::calibrateGyro() {
  // set at a lower range (more resolution) since IMU not moving设置在较低的范围(更高的分辨率)，因为IMU不移动
  const GyroFS current_fssel = _gyroFS;
  if (setGyroFS(dps250) < 0) return -1;

  // take samples and find bias抽取样本，找出偏差
  _gyroBD[0] = 0;
  _gyroBD[1] = 0;
  _gyroBD[2] = 0;
  for (size_t i=0; i < NUM_CALIB_SAMPLES; i++) {
    getAGT();
    _gyroBD[0] += (gyrX() + _gyrB[0]) / NUM_CALIB_SAMPLES;
    _gyroBD[1] += (gyrY() + _gyrB[1]) / NUM_CALIB_SAMPLES;
    _gyroBD[2] += (gyrZ() + _gyrB[2]) / NUM_CALIB_SAMPLES;
    delay(1);
  }
  _gyrB[0] = _gyroBD[0];
  _gyrB[1] = _gyroBD[1];
  _gyrB[2] = _gyroBD[2];

  // recover the full scale setting
  if (setGyroFS(current_fssel) < 0) return -4;
  return 1;
}

/* returns the gyro bias in the X direction, dps */
float ICM42688::getGyroBiasX() {
  return _gyrB[0];
}

/* returns the gyro bias in the Y direction, dps */
float ICM42688::getGyroBiasY() {
  return _gyrB[1];
}

/* returns the gyro bias in the Z direction, dps */
float ICM42688::getGyroBiasZ() {
  return _gyrB[2];
}

/* sets the gyro bias in the X direction to bias, dps */
void ICM42688::setGyroBiasX(float bias) {
  _gyrB[0] = bias;
}

/* sets the gyro bias in the Y direction to bias, dps */
void ICM42688::setGyroBiasY(float bias) {
  _gyrB[1] = bias;
}

/* sets the gyro bias in the Z direction to bias, dps */
void ICM42688::setGyroBiasZ(float bias) {
  _gyrB[2] = bias;
}

/* finds bias and scale factor calibration for the accelerometer,
this should be run for each axis in each direction (6 total) to find
the min and max values along each 
查找加速度计的偏差和比例因子校准，这应该在每个方向(总共6个)的每个轴上运行，以找到每个轴上的最小值和最大值
*/
int ICM42688::calibrateAccel() {
  // set at a lower range (more resolution) since IMU not moving设置在较低的范围(更高的分辨率)，因为IMU不移动
  const AccelFS current_fssel = _accelFS;
  if (setAccelFS(gpm2) < 0) return -1;

  // take samples and find min / max取样本，找出最小/最大值
  _accBD[0] = 0;
  _accBD[1] = 0;
  _accBD[2] = 0;
  for (size_t i=0; i < NUM_CALIB_SAMPLES; i++) {
    getAGT();
    _accBD[0] += (accX()/_accS[0] + _accB[0]) / NUM_CALIB_SAMPLES;
    _accBD[1] += (accY()/_accS[1] + _accB[1]) / NUM_CALIB_SAMPLES;
    _accBD[2] += (accZ()/_accS[2] + _accB[2]) / NUM_CALIB_SAMPLES;
    delay(1);
  }
  if (_accBD[0] > 0.9f) 
  {
    _accMax[0] = _accBD[0];
  }
  if (_accBD[1] > 0.9f) 
  {
    _accMax[1] = _accBD[1];
  }
  if (_accBD[2] > 0.9f) 
  {
    _accMax[2] = _accBD[2];
  }
  if (_accBD[0] < -0.9f) 
  {
    _accMin[0] = _accBD[0];
  }
  if (_accBD[1] < -0.9f) 
  {
    _accMin[1] = _accBD[1];
  }
  if (_accBD[2] < -0.9f) 
  {
    _accMin[2] = _accBD[2];
  }
  Serial.println("\t");
  Serial.print(_accBD[0],6);
  Serial.print("\t");
  Serial.print(_accBD[1],6);
  Serial.print("\t");
  Serial.print(_accBD[2],6);
  Serial.print("\t");
  Serial.print(_accMax[0],6);
  Serial.print("\t");
  Serial.print(_accMax[1],6);
  Serial.print("\t");
  Serial.print(_accMax[2],6);
  Serial.print("\t");
  Serial.print(_accMin[0],6);
  Serial.print("\t");
  Serial.print(_accMin[1],6);
  Serial.print("\t");
  Serial.println(_accMin[2],6);

  
  // find bias and scale factor找出偏差和比例因子
  if ((abs(_accMin[0]) > 0.9f) && (abs(_accMax[0]) > 0.9f)) 
  {
    _accB[0] = (_accMin[0] + _accMax[0]) / 2.0f;
    _accS[0] = 1/((abs(_accMin[0]) + abs(_accMax[0])) / 2.0f);
  }
  if ((abs(_accMin[1]) > 0.9f) && (abs(_accMax[1]) > 0.9f)) 
  {
    _accB[1] = (_accMin[1] + _accMax[1]) / 2.0f;
    _accS[1] = 1/((abs(_accMin[1]) + abs(_accMax[1])) / 2.0f);
  }
  if ((abs(_accMin[2]) > 0.9f) && (abs(_accMax[2]) > 0.9f)) 
  {
    _accB[2] = (_accMin[2] + _accMax[2]) / 2.0f;
    _accS[2] = 1/((abs(_accMin[2]) + abs(_accMax[2])) / 2.0f);
  }

  Serial.print("\t");
  Serial.print(_accB[0],6);
  Serial.print("\t");
  Serial.print(_accS[0],6);
  Serial.print("\t");
  Serial.print(_accB[1],6);
  Serial.print("\t");
  Serial.print(_accS[10],6);
  Serial.print("\t");
  Serial.print(_accB[2],6);
  Serial.print("\t");
  Serial.println(_accS[2],6);

  // recover the full scale setting恢复满量程设置
  if (setAccelFS(current_fssel) < 0) return -4;
  return 1;
}

/* returns the accelerometer bias in the X direction, m/s/s */
float ICM42688::getAccelBiasX_mss() {
  return _accB[0];
}

/* returns the accelerometer scale factor in the X direction */
float ICM42688::getAccelScaleFactorX() {
  return _accS[0];
}

/* returns the accelerometer bias in the Y direction, m/s/s */
float ICM42688::getAccelBiasY_mss() {
  return _accB[1];
}

/* returns the accelerometer scale factor in the Y direction */
float ICM42688::getAccelScaleFactorY() {
  return _accS[1];
}

/* returns the accelerometer bias in the Z direction, m/s/s */
float ICM42688::getAccelBiasZ_mss() {
  return _accB[2];
}

/* returns the accelerometer scale factor in the Z direction */
float ICM42688::getAccelScaleFactorZ() {
  return _accS[2];
}

/* sets the accelerometer bias (m/s/s) and scale factor in the X direction */
void ICM42688::setAccelCalX(float bias,float scaleFactor) {
  _accB[0] = bias;
  _accS[0] = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Y direction */
void ICM42688::setAccelCalY(float bias,float scaleFactor) {
  _accB[1] = bias;
  _accS[1] = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Z direction */
void ICM42688::setAccelCalZ(float bias,float scaleFactor) {
  _accB[2] = bias;
  _accS[2] = scaleFactor;
}

/* writes a byte to ICM42688 register given a register address and data */
int ICM42688::writeRegister(uint8_t subAddress, uint8_t data) {
  /* write data to device */
  if( _useSPI ) {
    _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
    digitalWrite(_csPin,LOW); // select the ICM42688 chip
    _spi->transfer(subAddress); // write the register address
    _spi->transfer(data); // write the data
    digitalWrite(_csPin,HIGH); // deselect the ICM42688 chip
    _spi->endTransaction(); // end the transaction
  }
  else{
    _i2c->beginTransmission(_address); // open the device
    _i2c->write(subAddress); // write the register address
    _i2c->write(data); // write the data
    _i2c->endTransmission();
  }

  delay(10);

  /* read back the register */
  readRegisters(subAddress, 1, _buffer);
  /* check the read back register against the written register */
  if(_buffer[0] == data) {
    return 1;
  }
  else{
    return -1;
  }
}

void ICM42688::writeReg(uint8_t reg, void* pBuf, size_t size)
{
  if (pBuf == NULL) {
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;
  _i2c->beginTransmission(_address);
  _i2c->write(&reg, 1);
  for (uint16_t i = 0; i < size; i++) {
    _i2c->write(_pBuf[i]);
  }
  _i2c->endTransmission();
}



/* reads registers from ICM42688 given a starting register address, number of bytes, and a pointer to store data */
int ICM42688::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest) {
  if( _useSPI ) {
    // begin the transaction
    if(_useSPIHS) {
      _spi->beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
    }
    else{
      _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
    }
    digitalWrite(_csPin,LOW); // select the ICM42688 chip
    _spi->transfer(subAddress | 0x80); // specify the starting register address
    for(uint8_t i = 0; i < count; i++) {
      dest[i] = _spi->transfer(0x00); // read the data
    }
    digitalWrite(_csPin,HIGH); // deselect the ICM42688 chip
    _spi->endTransaction(); // end the transaction
    return 1;
  }
  else{
    _i2c->beginTransmission(_address); // open the device
    _i2c->write(subAddress); // specify the starting register address
    _i2c->endTransmission(false);
    _numBytes = _i2c->requestFrom(_address, count); // specify the number of bytes to receive
    if (_numBytes == count) {
      for(uint8_t i = 0; i < count; i++) {
        dest[i] = _i2c->read();
      }
      return 1;
    } else {
      return -1;
    }
  }
}

int ICM42688::setBank(uint8_t bank) {
  // if we are already on this bank, bail
  if (_bank == bank) return 1;

  _bank = bank;

  return writeRegister(REG_BANK_SEL, bank);
}

void ICM42688::reset() {
  setBank(0);

  writeRegister(UB0_REG_DEVICE_CONFIG, 0x01);

  // wait for ICM42688 to come back up
  delay(1);
}

/* gets the ICM42688 WHO_AM_I register value */
uint8_t ICM42688::whoAmI() {
  setBank(0);

  // read the WHO AM I register
  if (readRegisters(UB0_REG_WHO_AM_I, 1, _buffer) < 0) {
    return -1;
  }
  // return the register value
  return _buffer[0];
}
