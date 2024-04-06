/*  
    作者：韩昂轩
    内容：RM系列电机驱动CAN
    1.0.0       添加了基本的驱动    23.10.7
    1.0.1       修改了发送函数，把can序号作为形参传入，可以实现can1或者can2发送的选择   23.10.28

*/


#include "RM_motor.h"




/// @brief CAN 发送函数
/// @param CAN 号 电机1---电机4
void RM_motor_send(CAN_TypeDef* CAN,s16 motor1,s16 motor2,s16 motor3,s16 motor4,u32 ID)
{
    u32 mbox;
    CanTxMsg TxMessage;
    TxMessage.StdId = ID;
    TxMessage.IDE = CAN_Id_Standard;        /*标准标识符*/
    TxMessage.RTR = CAN_RTR_Data;           /*数据帧*/
    TxMessage.DLC = 0x08;                   /*设置数据长度为8个字节*/
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;
    mbox = CAN_Transmit(CAN,&TxMessage);          /*开启发送*/
    while((CAN_TransmitStatus(CAN,mbox) == CAN_TxStatus_Failed));
}




/// @brief RM系列电机解码函数
/// @param ID 电机ID
/// @param data CAN接收原始数据
/// @param motor_msg 电机数据结构体数组（注意，这里这个变量很危险，不要超边界，不要超边界，不要超边界）
void motor_msg_decode_6020(u8 ID,u8* data,motor_return_msg_t* motor_msg)
{
    motor_msg[ID-1].angle = (s16)data[0] << 8 | data[1];
    motor_msg[ID-1].speed = (s16)data[2] << 8 | data[3];
    motor_msg[ID-1].current = (s16)data[4] <<8 | data[5];
    motor_msg[ID-1].temperature = data[6];
    motor_msg[ID-1].speed_s = motor_msg[ID-1].speed/60.0f;

    if(motor_msg[ID-1].angle - motor_msg[ID-1].last_angle > 8000)                 // 计算电机转过的圈数
    {
        motor_msg[ID-1].qvan --;
    }
    else if(motor_msg[ID-1].angle - motor_msg[ID-1].last_angle < -8000)
    {
        motor_msg[ID-1].qvan ++;
    }

    motor_msg[ID-1].all_angle = motor_msg[ID-1].angle*CAN_RM_CHANGE_CE + motor_msg[ID-1].qvan * 6.283185307179586476;       // 计算总的角度(弧度制角度)
    motor_msg[ID-1].last_angle = motor_msg[ID-1].angle;
}



/// @brief RM系列电机解码函数 3508
/// @param ID 电机ID
/// @param data CAN接收原始数据
/// @param motor_msg 电机数据结构体数组（注意，这里这个变量很危险，不要超边界，不要超边界，不要超边界）
void motor_msg_decode_3508(u8 ID,u8* data,motor_return_msg_t* motor_msg)
{
    motor_msg[ID-1].angle = (s16)data[0] << 8 | data[1];
    motor_msg[ID-1].speed = (s16)data[2] << 8 | data[3];
    motor_msg[ID-1].current = (s16)data[4] <<8 | data[5];
    motor_msg[ID-1].temperature = data[6];
    motor_msg[ID-1].speed_s = motor_msg[ID-1].speed/60.0f/19.0f;/*速度转换 3508减速电机才能用到，若不是3508应该调整19.0这个是减速箱的减速比*/
}

