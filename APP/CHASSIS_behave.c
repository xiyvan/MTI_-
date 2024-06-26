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
    wzv_set = -PID_cale(&chassis->angle_pid,chassis->chassis_set_msg.wz_SetAngle,chassis->Ins_msg.yaw_all_angle);
	//wzv_set = 0;
    //wzv_set = FZ_math_deadzone_limt(0.1f,wzv_set,0);
    ///*****************************  把旋转速度分解到vx与vy上面  *********************************************///
    date->vxm[0] = FZ_math_deadzone_limt(0.15f,chassis->chassis_set_msg.vx_set + wzv_set / CHASSIS_BEHAVE_SQRT_2,0);
    date->vym[0] = FZ_math_deadzone_limt(0.15f,chassis->chassis_set_msg.vy_set + wzv_set / CHASSIS_BEHAVE_SQRT_2,0);

    date->vxm[1] = FZ_math_deadzone_limt(0.15f,chassis->chassis_set_msg.vx_set - wzv_set / CHASSIS_BEHAVE_SQRT_2,0);
    date->vym[1] = FZ_math_deadzone_limt(0.15f,chassis->chassis_set_msg.vy_set + wzv_set / CHASSIS_BEHAVE_SQRT_2,0);

    date->vxm[2] = FZ_math_deadzone_limt(0.15f,chassis->chassis_set_msg.vx_set - wzv_set / CHASSIS_BEHAVE_SQRT_2,0);
    date->vym[2] = FZ_math_deadzone_limt(0.15f,chassis->chassis_set_msg.vy_set - wzv_set / CHASSIS_BEHAVE_SQRT_2,0);

    date->vxm[3] = FZ_math_deadzone_limt(0.15f,chassis->chassis_set_msg.vx_set + wzv_set / CHASSIS_BEHAVE_SQRT_2,0);
    date->vym[3] = FZ_math_deadzone_limt(0.15f,chassis->chassis_set_msg.vy_set - wzv_set / CHASSIS_BEHAVE_SQRT_2,0);
	
//		date->vxm[0] = chassis->chassis_set_msg.vx_set + wzv_set / CHASSIS_BEHAVE_SQRT_2;
//		date->vym[0] = chassis->chassis_set_msg.vy_set + wzv_set / CHASSIS_BEHAVE_SQRT_2;
//	
//		date->vxm[1] = chassis->chassis_set_msg.vx_set - wzv_set / CHASSIS_BEHAVE_SQRT_2;
//    date->vym[1] = chassis->chassis_set_msg.vy_set + wzv_set / CHASSIS_BEHAVE_SQRT_2;

//    date->vxm[2] = chassis->chassis_set_msg.vx_set - wzv_set / CHASSIS_BEHAVE_SQRT_2;
//    date->vym[2] = chassis->chassis_set_msg.vy_set - wzv_set / CHASSIS_BEHAVE_SQRT_2;

//    date->vxm[3] = chassis->chassis_set_msg.vx_set + wzv_set / CHASSIS_BEHAVE_SQRT_2;
//    date->vym[3] = chassis->chassis_set_msg.vy_set - wzv_set / CHASSIS_BEHAVE_SQRT_2;
	
	

    ///****************************  解算各个轮子的角度与速度  ***********************************************///
    for(u8 i = 0;i < 4;i++)
    {
        if((date->vxm[i] != 0) && (date->vym[i] != 0))
        {
            // 当两个方向的速度都不是 0 的时候
            // 计算轮子角度
            date->angle[i] = atan(date->vym[i] / date->vxm[i]);
            // 计算轮子速度
            arm_sqrt_f32((date->vxm[i] * date->vxm[i] + date->vym[i] * date->vym[i]),&date->speed[i]);
            if(date->vxm[i] < 0)
            {
                // 如果 x方向上的速度是负数数的话
                date->speed[i] = -date->speed[i];
            }
        }
        else if((date->vxm[i] == 0) && (date->vym[i] != 0))
        {
            // 当 x 速度等于 0 的时候
            if(date->vym[i] > 0)
            {
                date->angle[i] = CHASSIS_BEHAVE_PI_2;
            }
            else
            {
                date->angle[i] = -CHASSIS_BEHAVE_PI_2;
            }
            if(date->vym[i] > 0)
            {
                date->speed[i] = date->vym[i];
            }
            else if(date->vym[i] < 0)
            {
                date->speed[i] = -date->vym[i];
            }
        }
        else if((date->vxm[i] != 0) && (date->vym[i] == 0))
        {
            // 当 y 速度等于 0 的时候
            date->angle[i] = 0;
            date->speed[i] = date->vxm[i];
        }
        else
        {
            // x 与 y 方向都是 0 得时候
						date->angle[0] = CHASSIS_BEHAVE_PI_4;
					date->angle[1] = -CHASSIS_BEHAVE_PI_4;
					date->angle[2] = CHASSIS_BEHAVE_PI_4;
					date->angle[3] = -CHASSIS_BEHAVE_PI_4;
					//date->angle[i] = 0;
            date->speed[i] = 0;
        }
    }
//******************************* 判断轮子角度是不是全部到位 ******************************************///
    date->state = 0;
    for(u8 i = 0;i < 4;i++)
    {
        // 角度修正 把角度转换到 0 的位置
        date->angle[i] += date->angle_cali[i];
        if(((date->angle[i] + CHASSIS_DUO_DEAB_ANGLE) >= chassis->angle_motor_msg[i].all_angle) 
        && ((date->angle[i] - CHASSIS_DUO_DEAB_ANGLE) < chassis->angle_motor_msg[i].all_angle))
        {
            
        }
        else
        {
            date->state ++;
        }
    }
}

#endif
