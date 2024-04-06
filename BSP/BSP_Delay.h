#ifndef _BSP_DELAY_H
#define _BSP_DELAY_H

#include "stm32f4xx.h"


void Systick_tim_init(void);
void Delay_ms(unsigned long Time);
void Delay_IIC(u32 s);
void Delay_us(u32 nus);
void BMI088_delay_us(u32 time);
void BMI088_delay_ms(u32 time);

#endif
