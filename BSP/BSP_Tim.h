#ifndef _BSP_TIM_H_
#define _BSP_TIM_H_

#include "stm32f4xx.h"







int get_led_pwm(void);
void TIM3_Init(void);
void TIM5_ch1__ic_init(void);

#endif
