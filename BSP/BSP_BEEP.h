#ifndef _BSP_BEEP_H
#define _BSP_BEEP_H

#include "stm32f4xx.h"

#define BEEP_PIN GPIO_Pin_8     //蜂鸣器引脚号

#define BEEP_ON() GPIO_SetBits(GPIOF,BEEP_PIN)  
#define BEEP_OFF() GPIO_ResetBits(GPIOF,BEEP_PIN) 

void Beep_init(void);   //蜂鸣器引脚初始化




#endif
