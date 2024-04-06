
#ifndef _FOC_H_
#define _FOC_H_

#include "stm32f4xx.h"


#define PI 3.1415926
#define voltage_power_max 24  //输入电压的最大值
#define encoder_max 8191      //电机编码器最大值


typedef struct 
{
    const int* the_ta;//电机实时角度信息指针
    
    int the_ta_t;     //机械角度与电角度转换后的信息

    int iq;             //设定力矩
    float i_alph;       
    float i_bei_ta;

    
    float ia;           //a相电流
    float ib;           //b
    float ic;           //c

    int p;              //电机的极对数

}FOC_type_def;



float sin(float x);
float cos(float x);
void FOC_Init(FOC_type_def* foc_init_t,int p,int* the_ta);
void FOC_calc(FOC_type_def* foc_calc,int set_iq);


#endif
