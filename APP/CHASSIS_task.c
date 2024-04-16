/******************************************
*   2023/9/8                              
*   作者：韩昂轩
*   
*   1.0  添加了基本的麦轮运动系统    23.9.8
*
*
*   底盘任务                  
*******************************************
*/



#include "CHASSIS_task.h"
#include "BSP_CAN.h"
#include "LED_Blink_task.h"
#include "Z_math.h"
#include "arm_math.h"
#include "CHASSIS_behave.h"
#include "FreeRTOS.h"
#include "task.h"
#include "RM_motor.h"


///************************************************************************* 函数声明  *************************************************************************///

static void chassis_init(CHASSIS_struct_t* chassis);
static void chassis_cale(CHASSIS_struct_t* chassis);
static void chassis_ctrl_set(CHASSIS_struct_t* chassis);
static void chassis_motion_parse(CHASSIS_struct_t* chassis);
static void chassis_msg_update(CHASSIS_struct_t* chassis);


///************************************************************************ 变量声明  ************************************************************************///


/*底盘主结构体*/
CHASSIS_struct_t Main_chassis;

///*********************************************************************** 底盘任务 **************************************************************************///

/// @brief 地盘任务 
void Chassis_task(void *pvParameters)
{
    chassis_init(&Main_chassis);
    while (1)
    {
        chassis_ctrl_set(&Main_chassis);        /*控制量设置*/
        chassis_msg_update(&Main_chassis);      /*底盘信息更新*/
        chassis_motion_parse(&Main_chassis);  /*电机运动解算*/
        chassis_cale(&Main_chassis);          /*底盘控制电流计算*/
        RM_motor_send(CAN1,Main_chassis.chassis_set_msg.current_set[0],Main_chassis.chassis_set_msg.current_set[1],
                                Main_chassis.chassis_set_msg.current_set[2],Main_chassis.chassis_set_msg.current_set[3],CAN_3508_ID14);
        vTaskDelay(5);
        //Main_chassis.chassis_set_msg.mode_last = Main_chassis.chassis_set_msg.chassis_mode_set;     /*更新地盘数据*/
    }
}








/// @brief 底盘初始化
/// @param chassis 底盘结构体
static void chassis_init(CHASSIS_struct_t* chassis)
{
    float temp[3] = {0};

    // 获取信息指针
    chassis->chassis_remote.speed_set = get_speed_set_p();
    chassis->chassis_remote.gym_angle = get_angle_p();

    // 四个轮子速度PID初始化
    temp[0]=chassis_whell_speed_pid_kp;temp[1]=chassis_whell_speed_pid_ki;temp[2]=chassis_whell_speed_pid_kd;
    for(u8 i = 0;i < 4;i++)
    {
        PID_Init(&chassis->wheel_speed_pid[i],temp,chassis_whell_speed_pid_maxout,chassis_whell_speed_pid_maxiout);
    }
    
    // 底盘角度PID
    temp[0]=chassis_angle_pid_kp;temp[1]=chassis_angle_pid_ki;temp[2]=chassis_angle_pid_kd;
    PID_Init(&chassis->angle_pid,temp,chassis_angle_pid_maxout,chassis_angle_pid_maxiout);
}






/// @brief 底盘控制量设置
/// @param chassis 底盘结构体
static void chassis_ctrl_set(CHASSIS_struct_t* chassis)
{
    chassis->chassis_set_msg.vx_set =  chassis->chassis_remote.speed_set->vx_set * CHASSIS_REMOTE_CHANGE_VX;
    chassis->chassis_set_msg.vy_set = chassis->chassis_remote.speed_set->vy_set * CHASSIS_REMOTE_CHANGE_VY;
    chassis->chassis_set_msg.wz_set = chassis->chassis_remote.speed_set->wz_set * CHASSIS_REMOTE_CHANGE_WZ;
}



/// @brief 底盘信息更细
/// @param chassis 底盘结构体变量指针
static void chassis_msg_update(CHASSIS_struct_t* chassis)
{
    chassis->wheel_speed_msg[0] = chassis->motor_msg[0].speed_s * 2 * PI * CHASSIS_WHEEL_RADIUS;
    chassis->wheel_speed_msg[1] = chassis->motor_msg[1].speed_s * 2 * PI * CHASSIS_WHEEL_RADIUS;
    chassis->wheel_speed_msg[2] = chassis->motor_msg[2].speed_s * 2 * PI * CHASSIS_WHEEL_RADIUS;
    chassis->wheel_speed_msg[3] = chassis->motor_msg[3].speed_s * 2 * PI * CHASSIS_WHEEL_RADIUS;
}



/// @brief 底盘运动解算
/// @param chassis 底盘结构体 (待定)
static void chassis_motion_parse(CHASSIS_struct_t* chassis)
{
    switch(chassis->chassis_remote.speed_set->mode_set)
    {
        case CHASSIS_ZERO_CU:
        {
            /// 零电流模式的情况下
            chassis_zero_solve(chassis);
        }break;

        case CHASSIS_FLOW_GIMBAL: 
        {
            /// 底盘跟随云台
            chassis_follow_gym_solve(chassis);
        }

        case CHASSIS_revolve_mode:
        {
            /// 小陀螺
            chassis_revolve_solve(chassis);
        }break;
    }
}






/// @brief 底盘输出计算
/// @param chassis 底盘结构体
static void chassis_cale(CHASSIS_struct_t* chassis)
{
    switch (chassis->chassis_remote.speed_set->mode_set)
    {
        case CHASSIS_ZERO_CU:
        {
            // 零电流已经在前面响应过了
        }break;
        case CHASSIS_revolve_mode:
        case CHASSIS_FLOW_CHASSIS:
        case CHASSIS_FLOW_GIMBAL:
        {
            // 底盘跟随云台模式
            chassis->chassis_set_msg.current_set[0] = PID_cale(&chassis->wheel_speed_pid[0],chassis->chassis_set_msg.wheel_speed_set[0],chassis->wheel_speed_msg[0]);
            chassis->chassis_set_msg.current_set[1] = PID_cale(&chassis->wheel_speed_pid[1],chassis->chassis_set_msg.wheel_speed_set[1],chassis->wheel_speed_msg[1]);
            chassis->chassis_set_msg.current_set[2] = PID_cale(&chassis->wheel_speed_pid[2],chassis->chassis_set_msg.wheel_speed_set[2],chassis->wheel_speed_msg[2]);
            chassis->chassis_set_msg.current_set[3] = PID_cale(&chassis->wheel_speed_pid[3],chassis->chassis_set_msg.wheel_speed_set[3],chassis->wheel_speed_msg[3]);
        }break;
        default:break;
    }
}




















/// @brief 地盘用速度等比例限幅
static void chassis_speed_limt(float* speed)
{
    float max_speed = speed[0];
    for(char x = 1;x<2;x++)         /*把四个轮子中最大速度选出来*/
    {
        if(FZ_math_absolute(max_speed) < FZ_math_absolute(speed[x]))
        {
            max_speed = speed[x];
        }
    }
    if(max_speed > CHASSIS_SPORT_MOTOR_MAX || max_speed < -CHASSIS_SPORT_MOTOR_MAX)         /*如果最大速度超过限制速度*/
    {
        max_speed = FZ_math_absolute(CHASSIS_SPORT_MOTOR_MAX / max_speed);    /*就让限制速度除以最大速度得到一个系数*/
        for(char x = 0;x < 2;x++)                           /*让四个轮子都乘上这个系数*/
        {
            speed[x] = speed[x]*max_speed;
        }
    }
}






/**********************************
 * @brief 正负限幅函数
 * @param max 最大值
 * @param min 最小值
 * @param xyz 角度输入
 * @param input 力矩输出输入
************************************/
static float Chassis_speed_sp(float max,float min,float xyz,float input)
{
    if(xyz >= max && input > 0)
    {
        return 0;
    }
    if(xyz <= min && input < 0)
    {
        return 0;
    }
    return input;
}
