/*
    时间：23.10.8
    作者：韩昂轩
    内容：瓴控电机驱动包
*/


#include "LK_motor.h"
#include "LED_Blink_task.h"             //移植可删

extern status_display_t LED_state_dis;     /*掉线统计结构体*/   //移植可删

/// @brief 瓴控电机速度控制函数
/// @param CAN can的结构体指针
/// @param ID 电机的ID
/// @param motor1 扭矩 2000~ -2000
void LK_MOTOR_send(CAN_TypeDef* CAN,u8 ID,s16 motor1)
{
    uint8_t mbox;
    CanTxMsg TxMessage;
    TxMessage.StdId = LK_MOTOR_ONE_MOTOR_CTRL_ID + ID;
    TxMessage.IDE = CAN_Id_Standard;        /*标准标识符*/
    TxMessage.RTR = CAN_RTR_Data;           /*数据帧*/
    TxMessage.DLC = 8;                      /*设置数据长度为8个字节*/
    TxMessage.Data[0] = LK_MOTOR_TORQUE_CLOSE_COM;
    TxMessage.Data[1] = 0;
    TxMessage.Data[2] = 0;
    TxMessage.Data[3] = 0;
    TxMessage.Data[4] = motor1 & 0xff;
    TxMessage.Data[5] = (motor1 >> 8) & 0xff;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;
    mbox = CAN_Transmit(CAN,&TxMessage);          /*开启发送*/
    while((CAN_TransmitStatus(CAN,mbox) == CAN_TxStatus_Failed));
}


/// @brief 瓴控电机多电机控制解码函数
/// @param ID 反馈的ID
/// @param data 原始数据
/// @param motor 电机数据结构体指针
void LK_MOTOR_decode(u32 ID,u8* data,LK_MOTOR_MSG_t* motor)
{
    VLEDBlink_ofdetection_update(&LED_state_dis.chassis_motor_1_d[ID-LK_MOTOR_MORE_MOTOR_RETURN_ID-2]); //移植可删
    motor[ID-LK_MOTOR_MORE_MOTOR_RETURN_ID-2].temperature = data[1];
    motor[ID-LK_MOTOR_MORE_MOTOR_RETURN_ID-2].iq = ((s16)data[3] << 8) | (data[2]);
    motor[ID-LK_MOTOR_MORE_MOTOR_RETURN_ID-2].speed = ((s16)data[5] << 8) | (data[4]);
    motor[ID-LK_MOTOR_MORE_MOTOR_RETURN_ID-2].encoder = ((u16)data[7] << 8) | data[6];
    motor[ID-LK_MOTOR_MORE_MOTOR_RETURN_ID-2].speed_g =  motor[ID-LK_MOTOR_MORE_MOTOR_RETURN_ID-2].speed * 0.0174532925199432957;
    motor[ID-LK_MOTOR_MORE_MOTOR_RETURN_ID-2].encoder_g = motor[ID-LK_MOTOR_MORE_MOTOR_RETURN_ID-2].encoder * 0.00009636787f;
    if(motor[ID-LK_MOTOR_MORE_MOTOR_RETURN_ID-2].encoder - motor[ID-LK_MOTOR_MORE_MOTOR_RETURN_ID-2].last_encoder > 20000)
    {
        motor[ID-LK_MOTOR_MORE_MOTOR_RETURN_ID-2].qvan --;
    }
    else if(motor[ID-LK_MOTOR_MORE_MOTOR_RETURN_ID-2].encoder - motor[ID-LK_MOTOR_MORE_MOTOR_RETURN_ID-2].last_encoder < -20000)
    {
        motor[ID-LK_MOTOR_MORE_MOTOR_RETURN_ID-2].qvan ++;
    }
    motor[ID-LK_MOTOR_MORE_MOTOR_RETURN_ID-2].last_encoder = motor[ID-LK_MOTOR_MORE_MOTOR_RETURN_ID-2].encoder;     // 更新数据
}




