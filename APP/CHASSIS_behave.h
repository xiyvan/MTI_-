
#ifndef _CHASSIS_BEHAVE_H_
#define _CHASSIS_BEHAVE_H_

#include "stm32f4xx.h"
#include "CHASSIS_task.h"



// 根号2的值
#define CHASSIS_BEHAVE_SQRT_2 1.414213f

#define CHASSIS_BEHAVE_PI_2 1.57079f



/// 舵轮解算需要的结构体变量
typedef struct 
{
    float vxm[4];       // 单个轮子 旋转速度分解后的 x方向 值
    float vym[4];       // 同上

    float angle[4];     // 最终解算出来的轮子角度
    float speed[4];     // 最终解算出来的轮子的速度
}chassis_solve_duo_t;




// 底盘跟随云台底盘结算
void chassis_follow_gym_solve(CHASSIS_struct_t* chassis);

// 零电流底盘解算
void chassis_zero_solve(CHASSIS_struct_t* chassis);

// 小陀螺底盘解算
void chassis_revolve_solve(CHASSIS_struct_t* chassis);

// 底盘跟随底盘模式结算
void chassis_follow_chassis_solve(CHASSIS_struct_t* chassis);

// 底盘跟随底盘模式结算（舵轮）
void chassis_follow_chassis_solve_D(CHASSIS_struct_t* chassis,chassis_solve_duo_t* date);


#endif
