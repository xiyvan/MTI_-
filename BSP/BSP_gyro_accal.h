#ifndef _BSP_GYRO_ACCAL_H_
#define _BSP_GYRO_ACCAL_H_
#include "stm32f4xx.h"



void BMI088_read(float gyro[3],float accel[3],float* temperate);
u8 BMI088_init(void);


#endif
