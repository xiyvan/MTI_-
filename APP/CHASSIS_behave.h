
#ifndef _CHASSIS_BEHAVE_H_
#define _CHASSIS_BEHAVE_H_

#include "stm32f4xx.h"
#include "CHASSIS_task.h"



// 根号2的值
#define CHASSIS_BEHAVE_SQRT_2 1.414213f

// PI/2
#define CHASSIS_BEHAVE_PI_2 1.57079f

#define CHASSIS_BEHAVE_PI_4 0.785395f

// 舵轮的轮子方向死区角度
#define CHASSIS_DUO_DEAB_ANGLE 0.1f






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
