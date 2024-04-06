#ifndef _BSP_ADC_H_
#define _BSP_ADC_H_

#include "stm32f4xx.h"




void ADC_init_I(void);
uint16_t Get_adc_val(ADC_TypeDef* adc,u8 ch);
uint16_t Get_adc_aveval(ADC_TypeDef* adc,u8 ch,uint8_t time);

#endif
