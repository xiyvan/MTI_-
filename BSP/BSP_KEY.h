#ifndef _BSP_KEY_H
#define _BSP_KEY_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

#define KEY_1_PIN GPIO_Pin_2  //GPIOE
#define KEY_2_PIN GPIO_Pin_3
#define KEY_3_PIN GPIO_Pin_4
#define KEY_4_PIN GPIO_Pin_0  //GPIOA










void KEY_init(void);
uint8_t get_key_val(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void Exti_key0_init(void);

#endif
