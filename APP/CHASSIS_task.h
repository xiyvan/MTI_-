
#ifndef _CHASSIS_TASK_H_
#define _CHASSIS_TASK_H_

#include "BSP_motor.h"
#include "PID.h"
#include "REMOTE_task.h"
#include "RM_motor.h"
#include "Z_math.h"
#include "GYRO_task.h"
#include "mk_task.h"

#define PI 3.1415926f


// 舵轮转向电机零点修正
#define chassis_duo_motor_cali1 3.75134993f
#define chassis_duo_motor_cali2 3.01734018f
#define chassis_duo_motor_cali3 5.87974834f
#define chassis_duo_motor_cali4 4.07655382f

// 底盘轮子速度PID
#define chassis_whell_speed_pid_kp 9000.0f
#define chassis_whell_speed_pid_ki 0.0f
#define chassis_whell_speed_pid_kd 2.0f
#define chassis_whell_speed_pid_maxout 16800.0f
#define chassis_whell_speed_pid_maxiout 0.0f

// 底盘整体角度PID
#define chassis_angle_pid_kp 2.0f
#define chassis_angle_pid_ki 0.0f
#define chassis_angle_pid_kd 75.0f
#define chassis_angle_pid_maxout 3.0f
#define chassis_angle_pid_maxiout 0.0f


// 舵轮底盘转向电机角度PID
#define chassis_angle_angle_kp 5.0f
#define chassis_angle_angle_ki 0.0f
#define chassis_angle_angle_kd 0.0f
#define chassis_angle_angle_maxout 3.0f
#define chassis_angle_angle_maxiout 0.0f

// 舵轮底盘转向电机速度PID
#define chassis_angle_speed_kp 7000.0f
#define chassis_angle_speed_ki 0.0f
#define chassis_angle_speed_kd 0.0f
#define chassis_angle_speed_maxout 30000.0f
#define chassis_angle_speed_maxiout 0.0f

// 遥控器值转换为实际值需要的系数
#define CHASSIS_REMOTE_CHANGE_VX 0.003
#define CHASSIS_REMOTE_CHANGE_VY 0.003
#define CHASSIS_REMOTE_CHANGE_WZ 0.003




//运动解算的时候，wz（旋转）加入速度的比例
#define CHASSIS_WZ_MOTION_PARSE_COEF 0.5f

//开环状态下，轮子速度转换为电流的大小 (待定)
#define CHASSIS_OPEN_LOOP_COE 1000

//轮子半径（待定 单位m）
#define CHASSIS_WHEEL_RADIUS 0.0676f
#define CHASSIS_WHEEL_MAX_SPEED 2.5f   /*单个轮子的最大速度 （m/s）*/



/*小陀螺速度设置*/
#define CHASSIS_MODE_TOP_SPEED 0.5         


/// 舵轮解算需要的结构体变量
typedef struct 
{
    float vxm[4];       // 单个轮子 旋转速度分解后的 x方向 值
    float vym[4];       // 同上

    float angle_cali[4];
    float angle[4];     // 最终解算出来的轮子角度
    float speed[4];     // 最终解算出来的轮子的速度

    u8 state;   // 用来标志轮子是不是全部到位
}chassis_solve_duo_t;


/// @brief 底盘控制模式
enum
{
    CHASSIS_FLOW_GIMBAL = 0,     /*底盘跟随云台模式*/
    CHASSIS_FLOW_CHASSIS,       /*底盘跟随底盘模式*/
    CHASSIS_ZERO_CU,            /*底盘零电流模式*/
    CHASSIS_NO_FLOW_CHASSIS,    /*底盘不跟随云台模式*/
    CHASSIS_OPEN_LOOP,          /*底盘开环控制模式*/
    CHASSIS_revolve_mode,       /*底盘小陀螺模式*/
};



typedef struct 
{
    float speed_change_set[2];  /* 用来保存速度设置（通过改变后的 vx vy）*/
    int wz_jq;                  /*wz 方向进行计圈*/
    u8 mode_set;                    /* 模式设置 */
    u8 mode_last;                   /*上一次的模式*/
    step_slope_msg_t vx_speed;  /* vx 方向的阶跃转斜坡*/
    step_slope_msg_t vy_speed;  /* vy 方向的阶跃转斜坡*/
    float vx_set;
    float vy_set;
    float wz_set;
    float wz_SetAngle;              /* 底盘跟随底盘的时候的角度设置 */
    float wheel_speed_set[4];   /* 四个轮子的速度设置*/
#if (CAR_TYPE != CAR_TYPE_Caster_wheel)
    s16 current_set[4];             /*六个电机最终给与电流*/
#else
    s16 current_set[8];             /*8个电机最终给与电流*/
#endif
}chassis_set_msg_t;





typedef struct 
{
    Remote_speed_t* speed_set;          // 速度设置与模式设置
    Remote_angle_t* gym_angle;          // 保存云台角度与底盘缓冲与模式设置
}chassis_remote_msg_t;



typedef struct 
{
    float *YawAngle;
    s32* Yaw_qvan;
    float yaw_all_angle;
}Gym_ins_struct_t;



/// @brief 底盘总结构体
typedef struct 
{
    chassis_set_msg_t chassis_set_msg;          /* 底盘设置信息 */
    chassis_remote_msg_t chassis_remote;        /* 遥控信息 */

    Gym_ins_struct_t Ins_msg;                   /* 陀螺仪信息 */
    float wheel_speed_msg[4];               /* 四个轮子的速度反馈 */
    motor_return_msg_t motor_msg[4];        // 保存四个电机的数据
    PID_type_def wheel_speed_pid[4];        // 四个轮子速度PID
    PID_type_def angle_pid;                 // 底盘角度PID

// 如果是舵轮的话就创建这两个变量
#if (CAR_TYPE == CAR_TYPE_Caster_wheel)
    // 保存轮子角度的角度环PID（舵轮）
    PID_type_def wheel_angle_angle_pid[4];
    // 保存轮子角度的速度环pid
    PID_type_def wheel_angle_speed_pid[4];
    // 保存四个转向电机的数据
    motor_return_msg_t angle_motor_msg[4];
    // 舵轮解算用数据
    chassis_solve_duo_t duo_solve_date;
#endif
}CHASSIS_struct_t;




void Chassis_task(void *pvParameters);






#endif
