
#ifndef _LK_MOTOR_H_
#define _LK_MOTOR_H_

#include "stm32f4xx.h"

#define LK_MOTOR_READ_PID_PARAMETER_COM         0X30    //读取PID参数命令
#define LK_MOTOR_WRITE_PID_PARAMETER_TO_RAM_COM 0X31    //写入PID参数到RAM命令
#define LK_MOTOR_WRITE_PID_PARAMETER_TO_ROM_COM 0X32    //写入PID参数到ROM命令
#define LK_MOTOR_READ_ACCEL_COM                 0X33    //读取加速度命令
#define LK_MOTOR_WRITE_ACCEL_TO_RAM_COM         0x34    //写入加速度到RAM命令
#define LK_MOTOR_READ_ENCODER_COM               0X90    //读取编码器命令
#define LK_MOTOR_WRITE_ENCODER_VAL_TO_ROM_COM   0X91    //写入值到ROM作为零点命令
#define LK_MOTOR_WRITE_LOCATION_VAL_TO_ROM_COM  0X19    //写入当前位置的值到ROM作为零点命令
#define LK_MOTOR_READ_MORE_CIRCLE_VAL_COM       0X92    //读取多圈角度命令
#define LK_MOTOR_READ_ONE_CIRCLE_VAL_COM        0X94    //读取单圈角度命令
#define LK_MOTOR_CLEAR_MOTOR_ANGLE_COM          0X95    //清除电机角度命令
#define LK_MOTOR_READ_MOTOR_STA1_REEOR_COM      0X9A    //读取电机状态与错误标志命令
#define LK_MOTOR_CLEAR_MOTOR_ERRO_COM           0X9B    //清除电机错误标志命令
#define LK_MOTOR_READ_MOTOR_STA2_COM            0X9C    //读取电机状态2
#define LK_MOTOR_READ_MOTOR_STA3_COM            0X9D    //读取电机状态3
#define LK_MOTOR_MOTOR_OFF_COM                  0X80    //电机关闭指令
#define LK_MOTOR_MOTOR_STOP_COM                 0X81    //电机停止指令
#define LK_MOTOR_MOTOR_STAR_COM                 0X88    //电机运行指令
#define LK_MOTOR_TORQUE_OPEN_COM                0XA0    //转矩开环指令
#define LK_MOTOR_TORQUE_CLOSE_COM               0XA1    //    闭环指令
#define LK_MOTOR_SPEED_CLOSE_COM                0XA2    //速度闭环指令
#define LK_MOTOR_POSITION_CLOSE1_COM            0XA3    //位置闭环指令1
#define LK_MOTOR_POSITION_CLOSE2_COM            0XA4    //位置闭环指令2
#define LK_MOTOR_POSITION_CLOSE3_COM            0XA5    //位置闭环指令3
#define LK_MOTOR_POSITION_CLOSE4_COM            0XA6    //位置闭环指令4
#define LK_MOTOR_POSITION_CLOSE5_COM            0XA7    //位置闭环指令5
#define LK_MOTOR_POSITION_CLOSE6_COM            0XA8    //位置闭环指令6



#define LK_MOTOR_OUT_MAX    2000        //电机扭矩最大设定值
#define LK_MOTOR_OUT_MIN    -2000       //        最小设定值

#define LK_MOTOR_MORE_MOTOR_CTRL_ID 0X280   //多电机转矩控制时包ID
#define LK_MOTOR_MORE_MOTOR_RETURN_ID   0X140   //多电机控制时电机反馈的ID号为 0x140+ID

#define LK_MOTOR_ONE_MOTOR_CTRL_ID  0X140   //单电机控制时包ID+电机ID


typedef struct 
{
    s8 temperature;     //温度
    s16 iq;             //电流
    s16 speed;          //速度
    float speed_g;      //弧度制速度
    u16 encoder;        //编码器值
    u16 last_encoder;   //上一次编码器的值
    float encoder_g;    // 转换后的的编码器值 弧度制
    int qvan;           //保存圈数
}LK_MOTOR_MSG_t;

void LK_MOTOR_send(CAN_TypeDef* CAN,u8 ID,s16 motor1);                      //瓴控电机发送速度函数
void LK_MOTOR_decode(u32 ID,u8* data,LK_MOTOR_MSG_t* motor);                 //瓴控电机解码函数

#endif
