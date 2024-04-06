#ifndef _BSP_SPI_H_
#define _BSP_SPI_H_

#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"


void SPI1_Init(void);
uint8_t SPI_ReadWriteByte(SPI_TypeDef* SPIx,uint8_t data);
void SPI_WriteData(SPI_TypeDef* SPIx,uint8_t* data,uint16_t len);

#endif
