#ifndef _GYRO_TASK_H_
#define _GYRO_TASK_H_

#include "stm32f4xx.h"
#include "BSP_gyro_accal.h"
#include "PID.h"

#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS        2


#define GYRO_TEMP_PID_KP 100.0f
#define GYRO_TEMP_PID_KI 1.0f
#define GYRO_TEMP_PID_KD 0.0f
#define GYRO_TEMP_PID_MAXOUT 1000.0f
#define GYRO_TEMP_PID_MAXIOUT 100


typedef struct BMI088_REAL_DATA
{
    uint8_t status;
    float accel[3];
    float temp;
    float gyro[3];
    float time;
} bmi088_real_data_t;


typedef struct __attribute__((packed))
{
    u8 status;
    s16 accel[3];
    s16 temp;
    s16 gyro[3];
} bmi088_raw_data_t;



typedef struct ist8310_real_data_t
{
    u8 status;
    float mag[3];
} ist8310_real_data_t;




void GYRO_task(void *pvParameters);



#endif
