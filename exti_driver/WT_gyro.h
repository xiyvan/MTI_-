
#ifndef _WT_GYRO_H_
#define _WT_GYRO_H_


#include "stm32f4xx.h"


#define WT_DATA_TYPE_ACCLE      0X51    //加速度 数据
#define WT_DATA_TYPE_ANG_SPEED  0x52    //角速度数据  
#define WT_DATA_TYPE_ANGLE      0X53    //角度数据
#define WT_DATA_TYPE_MAG_FIELD  0X54    //磁场数据
#define WT_DATA_TYPE_QUATE      0X58    //四元数数据
#define WT_DATA_TYPE_TIME       0X50    //时间数据


#define WT_DECODE_COEFFIENT 32768.0f   // 解码系数





/// @brief 维特陀螺仪结构体数据
typedef struct 
{
    float ax;
    float ay;
    float az;
    short temputer;

    float wx;
    float wy;
    float wz;

    float roll;
    float pitch;
    float yaw;
}WT_gyro_t;




void WT_gyro_decode(u8* data,WT_gyro_t* wt_data);   //维特陀螺仪数据解码



#endif

