/******************************************
*   2023/9/8                              
*   作者：韩昂轩
*   
*   1.0  添加了基本的麦轮运动系统    23.9.8
*   2.0  添加了舵轮运动解析并把运动解析分家单独成立一个文件，并添加了条件选择编译  24.5.29
*
*   底盘任务                  
*******************************************
*/



#include "CHASSIS_task.h"
#include "BSP_CAN.h"
#include "LED_Blink_task.h"
#include "Z_math.h"
#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "RM_motor.h"
#include "mk_task.h"


///************************************************************************* 函数声明  *************************************************************************///

static void chassis_init(CHASSIS_struct_t* chassis);
static void chassis_mode_set(CHASSIS_struct_t* chassis);
static void chassis_cale(CHASSIS_struct_t* chassis);
static void chassis_ctrl_set(CHASSIS_struct_t* chassis);
static void chassis_motion_parse(CHASSIS_struct_t* chassis);
static void chassis_msg_update(CHASSIS_struct_t* chassis);
static void chassis_speed_limt(float* speed);


///************************************************************************ 变量声明  ************************************************************************///


/*底盘主结构体*/
CHASSIS_struct_t Main_chassis;

extern float INS_angle[3];
extern s32 ins_qvan[3];
///*********************************************************************** 底盘任务 **************************************************************************///

/// @brief 地盘任务 
void Chassis_task(void *pvParameters)
{
    chassis_init(&Main_chassis);
    while (1)
    {
        chassis_mode_set(&Main_chassis);
        chassis_msg_update(&Main_chassis);      /*底盘信息更新*/
        chassis_ctrl_set(&Main_chassis);        /*控制量设置*/
        chassis_motion_parse(&Main_chassis);  /*电机运动解算*/
        chassis_cale(&Main_chassis);          /*底盘控制电流计算*/
        RM_motor_send(CAN2,Main_chassis.chassis_set_msg.current_set[0],Main_chassis.chassis_set_msg.current_set[1],
                                Main_chassis.chassis_set_msg.current_set[2],Main_chassis.chassis_set_msg.current_set[3],CAN_3508_ID14);
    #if (CAR_TYPE == CAR_TYPE_Caster_wheel)
        RM_motor_send(CAN1,Main_chassis.chassis_set_msg.current_set[4],Main_chassis.chassis_set_msg.current_set[5],
                                Main_chassis.chassis_set_msg.current_set[6],Main_chassis.chassis_set_msg.current_set[7],CAN_6010_ID14);
    #endif
        vTaskDelay(5);
    }
}


  





/// @brief 底盘初始化
/// @param chassis 底盘结构体
static void chassis_init(CHASSIS_struct_t* chassis)
{
    float temp[3] = {0};
    float temp_1[3] = {0};
    float temp_2[3] = {0};  
    float temp_3[4] = {0};  // 舵轮矫正角度

    // 获取信息指针
    chassis->chassis_remote.speed_set = get_speed_set_p();
    chassis->chassis_remote.gym_angle = get_angle_p();

    // 四个轮子速度PID初始化
    temp[0]=chassis_whell_speed_pid_kp;temp[1]=chassis_whell_speed_pid_ki;temp[2]=chassis_whell_speed_pid_kd;
    temp_1[0]=chassis_angle_angle_kp;temp_1[1]=chassis_angle_angle_ki;temp_1[2]=chassis_angle_angle_kd;
    temp_2[0]=chassis_angle_speed_kp;temp_2[1]=chassis_angle_speed_ki;temp_2[2]=chassis_angle_speed_kd;
    temp_3[0]=chassis_duo_motor_cali1;temp_3[1]=chassis_duo_motor_cali2;temp_3[2]=chassis_duo_motor_cali3;temp_3[3]=chassis_duo_motor_cali4;
    for(u8 i = 0;i < 4;i++)
    {
        PID_Init(&chassis->wheel_speed_pid[i],temp,chassis_whell_speed_pid_maxout,chassis_whell_speed_pid_maxiout);
    #if (CAR_TYPE == CAR_TYPE_Caster_wheel)
        PID_Init(&chassis->wheel_angle_angle_pid[i],temp_1,chassis_angle_angle_maxout,chassis_angle_angle_maxiout);
        PID_Init(&chassis->wheel_angle_speed_pid[i],temp_2,chassis_angle_speed_maxout,chassis_angle_speed_maxiout);
        chassis->duo_solve_date.angle_cali[i] = temp_3[i];
    #endif
    }

    // 底盘角度PID
    temp[0]=chassis_angle_pid_kp;temp[1]=chassis_angle_pid_ki;temp[2]=chassis_angle_pid_kd;
    PID_Init(&chassis->angle_pid,temp,chassis_angle_pid_maxout,chassis_angle_pid_maxiout);

    chassis->Ins_msg.YawAngle = &INS_angle[0];
    chassis->Ins_msg.Yaw_qvan = &ins_qvan[0];
}



/// @brief 底盘模式设置
/// @param chassis 底盘信息结构体
static void chassis_mode_set(CHASSIS_struct_t* chassis)
{
    switch (chassis->chassis_remote.speed_set->mode_set)
    {
    case REMOVE_S_DOWN:
        {
            // 波妞在下面的时候
            chassis->chassis_set_msg.mode_set = CHASSIS_ZERO_CU;
        }break;
    
    case REMOVE_S_MID:
        {
            // 波妞在中间的时候
            chassis->chassis_set_msg.mode_set = CHASSIS_NO_FLOW_CHASSIS;
        }break;

    case REMOVE_S_UP:
        {
            // 波妞在上面的时候
            chassis->chassis_set_msg.mode_set = CHASSIS_revolve_mode;
        }break;

    default:
        {
            // 波妞信息错误的情况
            chassis->chassis_set_msg.mode_set = CHASSIS_ZERO_CU;
        }break;
    }

///********************************************** 发生在模式切换时的操作  *************************************************************///

    // 当从零电流切换到底盘跟随底盘的时候 让yaw轴的圈数置零防止溢出
    if(chassis->chassis_set_msg.mode_set == CHASSIS_NO_FLOW_CHASSIS && chassis->chassis_set_msg.mode_last == CHASSIS_ZERO_CU)
    {
        *chassis->Ins_msg.Yaw_qvan = 0;
        chassis->chassis_set_msg.wz_SetAngle = *chassis->Ins_msg.YawAngle;
    }


    chassis->chassis_set_msg.mode_last = chassis->chassis_set_msg.mode_set;
}



/// @brief 底盘控制量设置
/// @param chassis 底盘结构体
static void chassis_ctrl_set(CHASSIS_struct_t* chassis)
{
    float vx_set,vy_set,wz_set;

    // 各个速度设置
    vx_set = chassis->chassis_remote.speed_set->vx_set * CHASSIS_REMOTE_CHANGE_VX ;
    vy_set = chassis->chassis_remote.speed_set->vy_set * CHASSIS_REMOTE_CHANGE_VY ;
    wz_set = chassis->chassis_remote.speed_set->wz_set * CHASSIS_REMOTE_CHANGE_WZ ;

    // wz 速度设置
    chassis->chassis_set_msg.wz_set = wz_set;
    // vx 速度设置
    if(vx_set != 0)
    {
        chassis->chassis_set_msg.vx_set =  FZ_math_StepToSlope_cale(&chassis->chassis_set_msg.vx_speed,
                                                                    vx_set,
                                                                    0.015); 
    }
    else
    {
        chassis->chassis_set_msg.vx_set = 0;
        chassis->chassis_set_msg.vx_speed.out = 0;
    }

    // vy 速度设置
    if(vy_set != 0)
    {
        chassis->chassis_set_msg.vy_set = FZ_math_StepToSlope_cale(&chassis->chassis_set_msg.vy_speed,
                                                                    vy_set,
                                                                    0.015);
    }
    else
    {
        chassis->chassis_set_msg.vy_set = 0;
        chassis->chassis_set_msg.vy_speed.out = 0;
    }
///*********************************************  底盘跟随底盘情况下的角度设定  *********************************************///
    if(chassis->chassis_set_msg.mode_set == CHASSIS_NO_FLOW_CHASSIS)
    {
        chassis->chassis_set_msg.wz_SetAngle -= chassis->chassis_set_msg.wz_set * 0.01f;		// 测得是反的所以这里加负号（根据实际情况来）
    }

}



/// @brief 底盘信息更细
/// @param chassis 底盘结构体变量指针
static void chassis_msg_update(CHASSIS_struct_t* chassis)
{
    chassis->wheel_speed_msg[0] = chassis->motor_msg[0].speed_s * 2 * PI * CHASSIS_WHEEL_RADIUS;
    chassis->wheel_speed_msg[1] = chassis->motor_msg[1].speed_s * 2 * PI * CHASSIS_WHEEL_RADIUS;
    chassis->wheel_speed_msg[2] = chassis->motor_msg[2].speed_s * 2 * PI * CHASSIS_WHEEL_RADIUS;
    chassis->wheel_speed_msg[3] = chassis->motor_msg[3].speed_s * 2 * PI * CHASSIS_WHEEL_RADIUS;

    chassis->Ins_msg.yaw_all_angle = *chassis->Ins_msg.Yaw_qvan * 6.2831853071f + *chassis->Ins_msg.YawAngle;
}



/// @brief 底盘运动解算
/// @param chassis 底盘结构体 (待定)
static void chassis_motion_parse(CHASSIS_struct_t* chassis)
{
    switch(chassis->chassis_set_msg.mode_set)
    {
        case CHASSIS_ZERO_CU:
        {
            /// 零电流模式的情况下
            chassis_zero_solve(chassis);
        }break;
// 脉轮或者全向轮情况下
#if (CAR_TYPE != CAR_TYPE_Caster_wheel)
        case CHASSIS_NO_FLOW_CHASSIS:
        {
            // 底盘跟随底盘（不跟随云台模式）
            chassis_follow_chassis_solve(chassis);
        }break;

// 舵轮情况下
#elif (CAR_TYPE == CAR_TYPE_Caster_wheel)
        case CHASSIS_NO_FLOW_CHASSIS:
        {
            chassis_follow_chassis_solve_D(chassis,&chassis->duo_solve_date);
        }break;
#endif

        case CHASSIS_FLOW_GIMBAL: 
        {
            /// 底盘跟随云台
            chassis_follow_gym_solve(chassis);
        }break;

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
        case CHASSIS_NO_FLOW_CHASSIS:
        case CHASSIS_revolve_mode:
        case CHASSIS_FLOW_CHASSIS:
        case CHASSIS_FLOW_GIMBAL:
    #if (CAR_TYPE != CAR_TYPE_Caster_wheel)
        {
            // 底盘跟随云台模式
            chassis_speed_limt(chassis->chassis_set_msg.wheel_speed_set);
            chassis->chassis_set_msg.current_set[0] = PID_cale(&chassis->wheel_speed_pid[0],chassis->chassis_set_msg.wheel_speed_set[0],chassis->wheel_speed_msg[0]);
            chassis->chassis_set_msg.current_set[1] = PID_cale(&chassis->wheel_speed_pid[1],-chassis->chassis_set_msg.wheel_speed_set[1],chassis->wheel_speed_msg[1]);
            chassis->chassis_set_msg.current_set[2] = PID_cale(&chassis->wheel_speed_pid[2],-chassis->chassis_set_msg.wheel_speed_set[2],chassis->wheel_speed_msg[2]);
            chassis->chassis_set_msg.current_set[3] = PID_cale(&chassis->wheel_speed_pid[3],chassis->chassis_set_msg.wheel_speed_set[3],chassis->wheel_speed_msg[3]);
        }break;
    #else
        {
            // 当每个轮子的角度达到设定值，才开始给轮子添加速度
            if(chassis->duo_solve_date.state == 0)
            {
                chassis_speed_limt(chassis->duo_solve_date.speed);      // 速度限幅
                // 轮子速度计算
                chassis->chassis_set_msg.current_set[0] = PID_cale(&chassis->wheel_speed_pid[0],chassis->duo_solve_date.speed[0],chassis->wheel_speed_msg[0]);
                chassis->chassis_set_msg.current_set[1] = PID_cale(&chassis->wheel_speed_pid[1],-chassis->duo_solve_date.speed[1],chassis->wheel_speed_msg[1]);
                chassis->chassis_set_msg.current_set[2] = PID_cale(&chassis->wheel_speed_pid[2],-chassis->duo_solve_date.speed[2],chassis->wheel_speed_msg[2]);
                chassis->chassis_set_msg.current_set[3] = PID_cale(&chassis->wheel_speed_pid[3],chassis->duo_solve_date.speed[3],chassis->wheel_speed_msg[3]);
            }
            else
            {
                chassis->chassis_set_msg.current_set[0] = 0;
                chassis->chassis_set_msg.current_set[1] = 0;
                chassis->chassis_set_msg.current_set[2] = 0;
                chassis->chassis_set_msg.current_set[3] = 0;
            }
            // 舵轮的角度环PID计算
            PID_cale(&chassis->wheel_angle_angle_pid[0],chassis->duo_solve_date.angle[0],chassis->angle_motor_msg[0].all_angle);
            PID_cale(&chassis->wheel_angle_angle_pid[1],chassis->duo_solve_date.angle[1],chassis->angle_motor_msg[1].all_angle);
            PID_cale(&chassis->wheel_angle_angle_pid[2],chassis->duo_solve_date.angle[2],chassis->angle_motor_msg[2].all_angle);
            PID_cale(&chassis->wheel_angle_angle_pid[3],chassis->duo_solve_date.angle[3],chassis->angle_motor_msg[3].all_angle);
            // 速度环pid计算
            chassis->chassis_set_msg.current_set[4] = PID_cale(&chassis->wheel_angle_speed_pid[0],
                                                                    chassis->wheel_angle_angle_pid[0].out,
                                                                    chassis->angle_motor_msg[0].speed_s);
            chassis->chassis_set_msg.current_set[5] = PID_cale(&chassis->wheel_angle_speed_pid[1],
                                                                    chassis->wheel_angle_angle_pid[1].out,
                                                                    chassis->angle_motor_msg[1].speed_s);
            chassis->chassis_set_msg.current_set[6] = PID_cale(&chassis->wheel_angle_speed_pid[2],
                                                                    chassis->wheel_angle_angle_pid[2].out,
                                                                    chassis->angle_motor_msg[2].speed_s);
            chassis->chassis_set_msg.current_set[7] = PID_cale(&chassis->wheel_angle_speed_pid[3],
                                                                    chassis->wheel_angle_angle_pid[3].out,
                                                                    chassis->angle_motor_msg[3].speed_s)
        }break;
    #endif
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
    if(max_speed > CHASSIS_WHEEL_MAX_SPEED || max_speed < -CHASSIS_WHEEL_MAX_SPEED)         /*如果最大速度超过限制速度*/
    {
        max_speed = FZ_math_absolute(CHASSIS_WHEEL_MAX_SPEED / max_speed);    /*就让限制速度除以最大速度得到一个系数*/
        for(char x = 0;x < 2;x++)                           /*让四个轮子都乘上这个系数*/
        {
            speed[x] = speed[x] * max_speed;
        }
    }
}

