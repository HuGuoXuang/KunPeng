#ifndef _MySPI_H_
#define _MySPI_H_
#include <Arduino.h>
#include <SPI.h>
#include "MyUltrasonic.h" 
#include "MyLc302.h" 
#include "SPL06-006.h"

#define HSPI_MISO 4
#define HSPI_MOSI 16
#define HSPI_SCLK 17
#define HSPI_CS   5

#define spiClk 5000000     //时钟信号

void SpiTxDat_init(void);
void SpiTxDat(void);

 
#endif
