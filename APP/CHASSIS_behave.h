
#ifndef _CHASSIS_BEHAVE_H_
#define _CHASSIS_BEHAVE_H_

#include "stm32f4xx.h"
#include "CHASSIS_task.h"





// 底盘跟随云台底盘结算
void chassis_follow_gym_solve(CHASSIS_struct_t* chassis);

// 零电流底盘解算
void chassis_zero_solve(CHASSIS_struct_t* chassis);

// 小陀螺底盘解算
void chassis_revolve_solve(CHASSIS_struct_t* chassis);

// 底盘跟随底盘模式结算
void chassis_follow_chassis_solve(CHASSIS_struct_t* chassis);


#endif
