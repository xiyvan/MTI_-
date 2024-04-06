#ifndef _RM_MOTOR_H_
#define _RM_MOTOR_H_


#include "stm32f4xx.h"

#define CAN_3508_ID14 0X200
#define CAN_3508_ID58 0X1FF
#define CAN_3508_RETURN 0x200   /* 0x200 + 电调ID*/

#define CAN_6010_ID14 0X1FF
#define CAN_6010_ID57 0X2FF
#define CAN_6010_RETURN 0X204  /* 0X204 + 电调ID*/

#define CAN_RM_CHANGE_CE  0.000766990393942820614       /* rm 电机编码器角度转弧度制角度系数*/


/// @brief RM电机反馈信息结构体
typedef struct 
{
    u16 angle;
    u16 last_angle;     /* 上一次的角度 用来记录圈数*/
    float all_angle;    /* 总角度 */
    s16 speed;
    s16 current;
    u8 temperature;
    float speed_s;   /*转换后的速度，单位 转/秒*/
    s32 qvan;       /* 保存电机圈数 可用可不用 */
}motor_return_msg_t;



void RM_motor_send(CAN_TypeDef* CAN,s16 motor1,s16 motor2,s16 motor3,s16 motor4,u32 ID);
void motor_msg_decode_3508(u8 ID,u8* data,motor_return_msg_t* motor_msg);
void motor_msg_decode_6020(u8 ID,u8* data,motor_return_msg_t* motor_msg);


#endif
