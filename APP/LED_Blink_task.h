#ifndef _LED_BLINK_TASK_H_
#define _LED_BLINK_TASK_H_

#include "stm32f4xx.h"

#define normal 1
#define abnormal 0

#define TIMEOUT_CHECK 1     /*掉线超时秒数 0- 240*/


enum
{
    Remote_RECEVER,
    CHASSIS_MOTOR_D_0,
    CHASSIS_MOTOR_D_1,
    CHASSIS_MOTOR_D_2,
    CHASSIS_MOTOR_D_3,
    CHASSIS_MOTOR_1_D_0,
    CHASSIS_MOTOR_1_D_1,
    HOOK_ALL,
};





/// @brief LED状态显示存储结构体
typedef struct 
{
    u8 Remote_time_out_d;
    u8 chassis_motor_d[4];         //关节电机
    u8 chassis_motor_1_d[2];       //底盘运动电机
}status_display_t;




void led_task(void *pvParameters);
void VLEDBlink_ofdetection_update(u8* time);


#endif
