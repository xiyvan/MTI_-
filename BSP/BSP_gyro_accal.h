#ifndef _BSP_GYRO_ACCAL_H_
#define _BSP_GYRO_ACCAL_H_
#include "stm32f4xx.h"


typedef struct 
{
    float acc[3];
    float gyro[3];
    float temp;

}bmi088_G_A_type;



void BMI088_read(float gyro[3],float accel[3],float* temperate);


#endif
