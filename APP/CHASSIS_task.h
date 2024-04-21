#ifndef _CHASSIS_TASK_H_
#define _CHASSIS_TASK_H_

#include "BSP_motor.h"
#include "PID.h"
#include "REMOTE_task.h"
#include "RM_motor.h"

#define PI 3.1415926


#define chassis_whell_speed_pid_kp 0.0f
#define chassis_whell_speed_pid_ki 0.0f
#define chassis_whell_speed_pid_kd 0.0f
#define chassis_whell_speed_pid_maxout 0.0f
#define chassis_whell_speed_pid_maxiout 0.0f

#define chassis_angle_pid_kp 0.0f
#define chassis_angle_pid_ki 0.0f
#define chassis_angle_pid_kd 0.0f
#define chassis_angle_pid_maxout 0.0f
#define chassis_angle_pid_maxiout 0.0f



// 遥控器值转换为实际值需要的系数
#define CHASSIS_REMOTE_CHANGE_VX 0.01
#define CHASSIS_REMOTE_CHANGE_VY 0.01
#define CHASSIS_REMOTE_CHANGE_WZ 0.01



#define CHASSIS_LEG_SET_coefficient 6600.0f

//运动解算的时候，wz（旋转）加入速度的比例
#define CHASSIS_WZ_MOTION_PARSE_COEF 1

//开环状态下，轮子速度转换为电流的大小 (待定)
#define CHASSIS_OPEN_LOOP_COE 1000

//轮子半径（待定 单位m）
#define CHASSIS_WHEEL_RADIUS 0.0676
#define CHASSIS_WHEEL_MAX_SPEED 1.0f   /*单个轮子的最大速度 （m/s）*/
#define CHASSIS_WHEEL_SPEED_CHANGE_COEFFICIENT 1.0f /*轮子反馈的速度转换为角速度的系数*/


#define CHASSIS_MODE_TOP_SPEED 0.5          /*小陀螺速度设置*/


#define CHASSIS_SPORT_MOTOR_MAX 2000.0f     /*腿部末端运动电机的最大电流*/



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
    float wheel_speed_set[4];   /* 四个轮子的速度设置*/
    float vx_set;
    float vy_set;
    float wz_set;
    s16 current_set[4];             /*六个电机最终给与电流*/
    u8 mode_last;                   /*上一次的模式*/

}chassis_set_msg_t;





typedef struct 
{
    Remote_speed_t* speed_set;          // 速度设置与模式设置
    Remote_angle_t* gym_angle;          // 保存云台角度与底盘缓冲与模式设置
}chassis_remote_msg_t;



/// @brief 底盘总结构体
typedef struct 
{
    chassis_set_msg_t chassis_set_msg;          /* 底盘设置信息 */
    chassis_remote_msg_t chassis_remote;        /* 遥控信息 */
    float wheel_speed_msg[4];                   /* 四个轮子的速度反馈 */
    motor_return_msg_t motor_msg[4];    // 保存四个电机的数据
    PID_type_def wheel_speed_pid[4];    // 四个轮子速度PID
    PID_type_def angle_pid;             // 底盘角度PID
    
}CHASSIS_struct_t;




void Chassis_task(void *pvParameters);






#endif
