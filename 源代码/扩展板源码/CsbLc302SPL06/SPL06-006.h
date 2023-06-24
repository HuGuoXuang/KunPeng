#include "Arduino.h"

extern double AirPressure;
extern double Temperature;

void SPL_init();

uint8_t get_spl_id();		// Get ID Register 		0x0D
uint8_t get_spl_prs_cfg();	// Get PRS_CFG Register	0x06 配置压力测量速率(PM_RATE)和分辨率(PM_PRC)。
uint8_t get_spl_tmp_cfg();	// Get TMP_CFG Register	0x07 配置温度测量速率(TMP_RATE)和分辨率(TMP_PRC)。
uint8_t get_spl_meas_cfg();	// Get MEAS_CFG Register	0x08 设置测量模式
uint8_t get_spl_cfg_reg();	// Get CFG_REG Register	0x09 配置中断，测量数据移位和FIFO启用
uint8_t get_spl_int_sts();	// Get INT_STS Register	0x0A  中断状态寄存器。寄存器在读取时被清除
uint8_t get_spl_fifo_sts();	// Get FIFO_STS Register	0x0B

double get_altitude(double pressure, double seaLevelhPa);	// get altitude in meters
double get_altitude_f(double pressure, double seaLevelhPa);	// get altitude in feet

int32_t get_traw();
double get_traw_sc();
double get_temp_c();
double get_temp_f();
double get_temperature_scale_factor();

int32_t get_praw();
double get_praw_sc();
double get_pcomp();
double get_pressure_scale_factor();
double get_pressure();

int16_t get_c0();
int16_t get_c1();
int32_t get_c00();
int32_t get_c10();
int16_t get_c01();
int16_t get_c11();
int16_t get_c20();
int16_t get_c21();
int16_t get_c30();

void i2c_eeprom_write_uint8_t(  uint8_t deviceaddress, uint8_t eeaddress, uint8_t data );
uint8_t i2c_eeprom_read_uint8_t(  uint8_t deviceaddress, uint8_t eeaddress );
void SPL06_read(void);
