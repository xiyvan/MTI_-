
#ifndef _HT_8115_H_
#define _HT_8115_H_

#include "stm32f4xx.h"

#define HT_MOTOR_VX_MAX 45.0f       //rad/s
#define HT_MOTOR_VX_MIN -45.0f          
#define HT_MOTOR_P_MAX  95.5f       //Radians
#define HT_MOTOR_P_MIN -95.5f
#define HT_MOTOR_T_MIN -18.0f        //N.M
#define HT_MOTOR_T_MAX 18.0f

#define HT_MOROT_COM_MOTOR 0XFC  /*海泰电机进入FOC模式命令*/
#define HT_MOROT_COM_EXTI 0XFD   /*海泰电机退出FOC模式命令*/
#define HT_MOROT_COM_ZERO 0XFE   /*海泰电机零电流指令*/


/// @brief 海泰电机反馈信息结构体
typedef struct 
{
    s8 ID;
    float angle;            //角度
    float speed;            //速度
    float moment;           //力矩
}HT_motor_return_msg_t;


enum
{
    HT_MOTOR_CAN_ID_1 = 0X01,       /*海泰电机ID*/
    HT_MOTOR_CAN_ID_2,
    HT_MOTOR_CAN_ID_3,
    HT_MOTOR_CAN_ID_4,

    HT_MOTOR_COM,               /*发送命令标志*/
    HT_MOTOR_DAT,               /*发送数据标志*/
};




void HT_motor_star(void);   //海泰启动函数
void HT_motor_over(void);   //海泰关闭函数

void HT_motor_can_send(float motor1,float motor2,float motor3,float motor4); //海泰多组发送(数据)
void CAN1_TX_HT(s8 ID,float moment,s8 command,s8 com_or_dat);       //海泰CAN发送函数

void HT_motor_decode(u8 ID,u8* data,HT_motor_return_msg_t* motor);  //海泰电机接收数据解码

uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);    //float转uint
float uint_to_float(int x_int, float x_min, float x_max, int bits);         //uint转float


#endif
