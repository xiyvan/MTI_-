/*********************************
 * @author 韩昂轩（Han Angxvan）
 * 创建时间 ： 24.4.23
 * 
 *   底盘运动结算行为包
*********************************/



#include "CHASSIS_behave.h"
#include "arm_math.h"
#include "Z_math.h"


/// @brief 底盘跟随云台结算
/// @param chassis 底盘信息结构体指针
void chassis_follow_gym_solve(CHASSIS_struct_t* chassis)
{
    float sin_x,cos_x;
    arm_sin_cos_f32(chassis->chassis_remote.gym_angle->angle,&sin_x,&cos_x);
    // 底盘跟随云台模式下
    PID_cale(&chassis->angle_pid,0,chassis->chassis_remote.gym_angle->angle);
    chassis->chassis_set_msg.wz_set = chassis->angle_pid.out;

    // 设置变化后的vx值与vy值，从而设置
    chassis->chassis_set_msg.speed_change_set[0] = chassis->chassis_set_msg.vx_set * cos_x - chassis->chassis_set_msg.vy_set * sin_x;
    chassis->chassis_set_msg.speed_change_set[1] = chassis->chassis_set_msg.vx_set * sin_x + chassis->chassis_set_msg.vy_set * cos_x;

    // 四个轮子的速度设置(未测定)
    chassis->chassis_set_msg.wheel_speed_set[0] = chassis->chassis_set_msg.speed_change_set[0] - chassis->chassis_set_msg.speed_change_set[1] + chassis->chassis_set_msg.wz_set * CHASSIS_WZ_MOTION_PARSE_COEF;
    chassis->chassis_set_msg.wheel_speed_set[1] = chassis->chassis_set_msg.speed_change_set[0] + chassis->chassis_set_msg.speed_change_set[1] - chassis->chassis_set_msg.wz_set * CHASSIS_WZ_MOTION_PARSE_COEF;
    chassis->chassis_set_msg.wheel_speed_set[2] = chassis->chassis_set_msg.speed_change_set[0] - chassis->chassis_set_msg.speed_change_set[1] - chassis->chassis_set_msg.wz_set * CHASSIS_WZ_MOTION_PARSE_COEF;
    chassis->chassis_set_msg.wheel_speed_set[3] = chassis->chassis_set_msg.speed_change_set[0] + chassis->chassis_set_msg.speed_change_set[1] + chassis->chassis_set_msg.wz_set * CHASSIS_WZ_MOTION_PARSE_COEF;
}




/// @brief 零电流底盘解算
/// @param chassis 底盘信息结构体指针
void chassis_zero_solve(CHASSIS_struct_t* chassis)
{
#if (CAR_TYPE != CAR_TYPE_Caster_wheel)
    for(u8 i = 0;i < 4;i++)
    {
        chassis->chassis_set_msg.current_set[i] = 0;
    }
#else
    for(u8 i = 0;i < 8;i++)
    {
        chassis->chassis_set_msg.current_set[i] = 0;
    }
#endif
}




/// @brief 小陀螺结算
/// @param chassis 底盘信息结构体指针
void chassis_revolve_solve(CHASSIS_struct_t* chassis)
{
    float sin_x,cos_x;
    arm_sin_cos_f32(chassis->chassis_remote.gym_angle->angle,&sin_x,&cos_x);

    // 小陀螺转速设置
    chassis->chassis_set_msg.wz_set = CHASSIS_MODE_TOP_SPEED;

    // 设置变化后的vx值与vy值，从而设置
    chassis->chassis_set_msg.speed_change_set[0] = chassis->chassis_set_msg.vx_set * cos_x - chassis->chassis_set_msg.vy_set * sin_x;
    chassis->chassis_set_msg.speed_change_set[1] = chassis->chassis_set_msg.vx_set * sin_x + chassis->chassis_set_msg.vy_set * cos_x;

    // 四个轮子的速度设置
    chassis->chassis_set_msg.wheel_speed_set[0] = chassis->chassis_set_msg.speed_change_set[0] - chassis->chassis_set_msg.speed_change_set[1] + chassis->chassis_set_msg.wz_set * CHASSIS_WZ_MOTION_PARSE_COEF;
    chassis->chassis_set_msg.wheel_speed_set[1] = chassis->chassis_set_msg.speed_change_set[0] + chassis->chassis_set_msg.speed_change_set[1] - chassis->chassis_set_msg.wz_set * CHASSIS_WZ_MOTION_PARSE_COEF;
    chassis->chassis_set_msg.wheel_speed_set[2] = chassis->chassis_set_msg.speed_change_set[0] - chassis->chassis_set_msg.speed_change_set[1] - chassis->chassis_set_msg.wz_set * CHASSIS_WZ_MOTION_PARSE_COEF;
    chassis->chassis_set_msg.wheel_speed_set[3] = chassis->chassis_set_msg.speed_change_set[0] + chassis->chassis_set_msg.speed_change_set[1] + chassis->chassis_set_msg.wz_set * CHASSIS_WZ_MOTION_PARSE_COEF;
}





/// @brief 底盘跟随底盘模式结算
/// @param chassis 底盘信息结构体指针
void chassis_follow_chassis_solve(CHASSIS_struct_t* chassis)
{
    float wzv_set = 0.0f;
    wzv_set = -PID_cale(&chassis->angle_pid,chassis->chassis_set_msg.wz_SetAngle,chassis->Ins_msg.yaw_all_angle);

    chassis->chassis_set_msg.wheel_speed_set[0] = chassis->chassis_set_msg.vx_set - chassis->chassis_set_msg.vy_set + wzv_set * CHASSIS_WZ_MOTION_PARSE_COEF;
    chassis->chassis_set_msg.wheel_speed_set[1] = chassis->chassis_set_msg.vx_set + chassis->chassis_set_msg.vy_set - wzv_set * CHASSIS_WZ_MOTION_PARSE_COEF;
    chassis->chassis_set_msg.wheel_speed_set[2] = chassis->chassis_set_msg.vx_set - chassis->chassis_set_msg.vy_set - wzv_set * CHASSIS_WZ_MOTION_PARSE_COEF;
    chassis->chassis_set_msg.wheel_speed_set[3] = chassis->chassis_set_msg.vx_set + chassis->chassis_set_msg.vy_set + wzv_set * CHASSIS_WZ_MOTION_PARSE_COEF;
}



#if CAR_TYPE == CAR_TYPE_Caster_wheel

/// @brief  舵轮底盘运动解算（底盘跟随底盘）
/// @param chassis 底盘信息结构体指针
/// @param date 舵轮底盘解算结构体
/// @note 解算方法 先把旋转速度分解到 x 与 y 方向上面，然后根据两个方向的速度合成求解轮子的角度与速度
void chassis_follow_chassis_solve_D(CHASSIS_struct_t* chassis,chassis_solve_duo_t* date)
{
    float wzv_set = 0.0f;
    float wheel_angle_cale[4] = {0};    // 用来保存矫正过后的轮子角度
    int16_t state = 0;

    // 计算校正后的反馈值
    wheel_angle_cale[0] = chassis->angle_motor_msg[0].all_angle - date->angle_cali[0];
    wheel_angle_cale[1] = chassis->angle_motor_msg[1].all_angle - date->angle_cali[1];
    wheel_angle_cale[2] = chassis->angle_motor_msg[2].all_angle - date->angle_cali[2];
    wheel_angle_cale[3] = chassis->angle_motor_msg[3].all_angle - date->angle_cali[3];

    wzv_set = -PID_cale(&chassis->angle_pid,chassis->chassis_set_msg.wz_SetAngle,chassis->Ins_msg.yaw_all_angle);
	wzv_set = FZ_math_deadzone_limt(0.05f,wzv_set,0);
    //wzv_set = chassis->chassis_set_msg.wz_set;
    ///*****************************  把旋转速度分解到vx与vy上面  *********************************************///

    date->vxm[0] = chassis->chassis_set_msg.vx_set + wzv_set / CHASSIS_BEHAVE_SQRT_2;
    date->vym[0] = chassis->chassis_set_msg.vy_set + wzv_set / CHASSIS_BEHAVE_SQRT_2;

    date->vxm[1] = chassis->chassis_set_msg.vx_set - wzv_set / CHASSIS_BEHAVE_SQRT_2;
    date->vym[1] = chassis->chassis_set_msg.vy_set + wzv_set / CHASSIS_BEHAVE_SQRT_2;

    date->vxm[2] = chassis->chassis_set_msg.vx_set - wzv_set / CHASSIS_BEHAVE_SQRT_2;
    date->vym[2] = chassis->chassis_set_msg.vy_set - wzv_set / CHASSIS_BEHAVE_SQRT_2;

    date->vxm[3] = chassis->chassis_set_msg.vx_set + wzv_set / CHASSIS_BEHAVE_SQRT_2;
    date->vym[3] = chassis->chassis_set_msg.vy_set - wzv_set / CHASSIS_BEHAVE_SQRT_2;

    ///****************************  解算各个轮子的角度与速度  ***********************************************///

    if(!(chassis->chassis_set_msg.vy_set == 0 && chassis->chassis_set_msg.vx_set == 0 && wzv_set == 0))
    {
        date->angle[0] = atan2f(chassis->chassis_set_msg.vy_set + wzv_set / CHASSIS_BEHAVE_SQRT_2,
                        chassis->chassis_set_msg.vx_set + wzv_set / CHASSIS_BEHAVE_SQRT_2);
        date->angle[1] = atan2f(chassis->chassis_set_msg.vy_set + wzv_set / CHASSIS_BEHAVE_SQRT_2,
                                chassis->chassis_set_msg.vx_set - wzv_set / CHASSIS_BEHAVE_SQRT_2);
        date->angle[2] = atan2f(chassis->chassis_set_msg.vy_set - wzv_set / CHASSIS_BEHAVE_SQRT_2,
                                chassis->chassis_set_msg.vx_set - wzv_set / CHASSIS_BEHAVE_SQRT_2);
        date->angle[3] = atan2f(chassis->chassis_set_msg.vy_set - wzv_set / CHASSIS_BEHAVE_SQRT_2,
                                chassis->chassis_set_msg.vx_set + wzv_set / CHASSIS_BEHAVE_SQRT_2);
    }

    // 如果vxm与vym都是0的话
    if(chassis->chassis_set_msg.vy_set == 0 && chassis->chassis_set_msg.vx_set == 0 && wzv_set == 0)
    {
        date->angle[0] = 0.78539816339f;
        date->angle[1] = 0.78539816339f + 1.570796326794f;
        date->angle[2] = 0.78539816339f;
        date->angle[3] = 0.78539816339f + 1.570796326794f;
    }

    for(u8 i = 0;i < 4;i++)
    {
        // 利用勾股定理求解速度大小
        arm_sqrt_f32((date->vym[i] * date->vym[i] + date->vxm[i] * date->vxm[i]),&date->speed[i]);

        // 判断是不是大于180°，若大于下面对应的算法就应该是 -180 否则 +180
        if(FZ_math_absolute(date->angle[i]) < 0)
        {
            state = 1;
        }
        else if(FZ_math_absolute(date->angle[i]) > 0)
        {
            state = -1;
        }

        // 如果 |设定值减去当前值| 小于90°
        if(FZ_math_absolute(date->angle[i] - wheel_angle_cale[i]) < 1.570796326794f)
        {
            // 就按照这个角度进行角度设置
            date->angle[i] = date->angle[i];
        }
        else if(FZ_math_absolute(2*PI - (date->angle[i] - wheel_angle_cale[i])) < 1.570796326794f)
        {
            // 如果让 |2PI - （设定值 - 当前值）| < 90°的话同理
            date->angle[i] = date->angle[i];
        }
        else if(FZ_math_absolute((date->angle[i] + state*PI) - wheel_angle_cale[i]) < 1.570796326794f)
        {
            // 如果 |设定值+180° - 当前值| < 90°的话
            // 就上设定值改为 +180 并且让速度反向
            date->angle[i] += state*PI;
            date->speed[i] = - date->speed[i];
        }
        else if(FZ_math_absolute(2*PI-((date->angle[i] + state*PI) - wheel_angle_cale[i])) < 1.570796326794f)
        {
            date->angle[i] += state*PI;
            date->speed[i] = -date->speed[i];
        }
    }

//******************************* 角度设定值校正 ******************************************///

    for(u8 i = 0;i < 4;i++)
    {
        // 角度修正 把角度转换到 0 的位置
        date->angle[i] += date->angle_cali[i];
    }
}

#endif
