#ifndef _CK_TIMEOUT_TASK_H_
#define _CK_TIMEOUT_TASK_H_

#include "stm32f4xx.h"
#include "mk_task.h"


enum
{
    TIMEOUT_REMOTE_TIMEOUT,
    TIMEOUT_WHEEL_SPEED_MOTOR1,
    TIMEOUT_WHEEL_SPEED_MOTOR2,
    TIMEOUT_WHEEL_SPEED_MOTOR3,
    TIMEOUT_WHEEL_SPEED_MOTOR4,
#if (CAR_TYPE == CAR_TYPE_Caster_wheel)
    TIMEOUT_WHEEL_ANGLE_MOTOR1,
    TIMEOUT_WHEEL_ANGLE_MOTOR2,
    TIMEOUT_WHEEL_ANGLE_MOTOR3,
    TIMEOUT_WHEEL_ANGLE_MOTOR4,
#endif
    TIMEOUT_ALL
};



typedef struct 
{
    u32 all_time;
    u32 driver_time[TIMEOUT_ALL];
    u8 TimeOut_num;
}CK_Timeout_struct_t;




void CK_Timeout_task(void *pvParameters);
void CkTime_DriverTimeNew(u8 num);
u8 Get_TimeOutNum(void);



#endif
