#ifndef _REMOTE_TASK_H_
#define _REMOTE_TASK_H_

#include "stm32f4xx.h"

// 帧头
#define REMOTE_HEAD 0XA5
// 帧尾
#define REMOTE_WEI1 0xff
#define REMOTE_WEI2 0xaa


#define REMOVE_S_DOWN 2
#define REMOVE_S_MID 3
#define REMOVE_S_UP 1

// CAN角度ID
#define REMOTE_ANGLE_HUAN 0X555
// CAN 速度设置ID
#define REMOTE_SPEED_SET 0X556
// CAN 功率限制ID
#define REMOTE_POWER_LIMTED 0X557



typedef struct __attribute__((packed))
{
    u8 sof;
    float angle;        // 云台相对角度
    u16 huan;           // 缓冲功率
    u8 mode_set;        // 模式设置
}Remote_angle_t;


typedef struct __attribute__((packed))
{
    u8 sof;
    u16 power_limted;        // 功率限制
    u8 K[5];                 // 预留位
}Remote_power_limted_t;


typedef struct __attribute__((packed))
{
    u8 sof;
    s16 vx_set;           // vx速度设置
    s16 vy_set;           // vy速度设置
    s16 wz_set;
    u8 mode_set;        // 模式设置
}Remote_speed_t;





typedef struct 
{
    u8 step;
    u8 date[20];
    u8 date_p;
}remote_solve_one_t;




void remote_task(void *pvParameters);
Remote_speed_t* get_speed_set_p(void);
Remote_angle_t* get_angle_p(void);



#endif
