
#include "CHASSIS_behave.h"



// 底盘跟随云台底盘结算
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
    chassis->chassis_set_msg.wheel_speed_set[0] = chassis->chassis_set_msg.speed_change_set[0] + chassis->chassis_set_msg.speed_change_set[1] + chassis->chassis_set_msg.wz_set * CHASSIS_WZ_MOTION_PARSE_COEF;
    chassis->chassis_set_msg.wheel_speed_set[1] = chassis->chassis_set_msg.speed_change_set[0] + chassis->chassis_set_msg.speed_change_set[1] + chassis->chassis_set_msg.wz_set * CHASSIS_WZ_MOTION_PARSE_COEF;
    chassis->chassis_set_msg.wheel_speed_set[2] = chassis->chassis_set_msg.speed_change_set[0] + chassis->chassis_set_msg.speed_change_set[1] + chassis->chassis_set_msg.wz_set * CHASSIS_WZ_MOTION_PARSE_COEF;
    chassis->chassis_set_msg.wheel_speed_set[3] = chassis->chassis_set_msg.speed_change_set[0] + chassis->chassis_set_msg.speed_change_set[1] + chassis->chassis_set_msg.wz_set * CHASSIS_WZ_MOTION_PARSE_COEF;
}




// 零电流底盘解算
void chassis_zero_solve(CHASSIS_struct_t* chassis)
{
    for(u8 i = 0;i < 4;i++)
    {
        chassis->chassis_set_msg.current_set[i] = 0;
    }
}




// 小陀螺底盘解算
void chassis_revolve_solve(CHASSIS_struct_t* chassis)
{
    float sin_x,cos_x;
    arm_sin_cos_f32(chassis->chassis_remote.gym_angle->angle,&sin_x,&cos_x);

    // 小陀螺转速设置
    chassis->chassis_set_msg.wz_set = CHASSIS_MODE_TOP_SPEED;

    // 设置变化后的vx值与vy值，从而设置
    chassis->chassis_set_msg.speed_change_set[0] = chassis->chassis_set_msg.vx_set * cos_x - chassis->chassis_set_msg.vy_set * sin_x;
    chassis->chassis_set_msg.speed_change_set[1] = chassis->chassis_set_msg.vx_set * sin_x + chassis->chassis_set_msg.vy_set * cos_x;

    // 四个轮子的速度设置(未测定)
    chassis->chassis_set_msg.wheel_speed_set[0] = chassis->chassis_set_msg.speed_change_set[0] + chassis->chassis_set_msg.speed_change_set[1] + chassis->chassis_set_msg.wz_set * CHASSIS_WZ_MOTION_PARSE_COEF;
    chassis->chassis_set_msg.wheel_speed_set[1] = chassis->chassis_set_msg.speed_change_set[0] + chassis->chassis_set_msg.speed_change_set[1] + chassis->chassis_set_msg.wz_set * CHASSIS_WZ_MOTION_PARSE_COEF;
    chassis->chassis_set_msg.wheel_speed_set[2] = chassis->chassis_set_msg.speed_change_set[0] + chassis->chassis_set_msg.speed_change_set[1] + chassis->chassis_set_msg.wz_set * CHASSIS_WZ_MOTION_PARSE_COEF;
    chassis->chassis_set_msg.wheel_speed_set[3] = chassis->chassis_set_msg.speed_change_set[0] + chassis->chassis_set_msg.speed_change_set[1] + chassis->chassis_set_msg.wz_set * CHASSIS_WZ_MOTION_PARSE_COEF;
}





// 底盘跟随底盘模式结算
void chassis_follow_chassis_solve(CHASSIS_struct_t* chassis)
{
    // 符号未测定
    chassis->chassis_set_msg.wheel_speed_set[0] = chassis->chassis_set_msg.vx_set + chassis->chassis_set_msg.vy_set + chassis->chassis_set_msg.wz_set;
    chassis->chassis_set_msg.wheel_speed_set[1] = chassis->chassis_set_msg.vx_set + chassis->chassis_set_msg.vy_set + chassis->chassis_set_msg.wz_set;
    chassis->chassis_set_msg.wheel_speed_set[2] = chassis->chassis_set_msg.vx_set + chassis->chassis_set_msg.vy_set + chassis->chassis_set_msg.wz_set;
    chassis->chassis_set_msg.wheel_speed_set[3] = chassis->chassis_set_msg.vx_set + chassis->chassis_set_msg.vy_set + chassis->chassis_set_msg.wz_set;
}
