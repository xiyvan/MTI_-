/*  
    作者：韩昂轩
    内容：海泰-04电机驱动CAN
    1.0.0       添加了基本的驱动    23.10.7

*/


#include "HT_8115.h"
#include "LED_Blink_task.h"         //移植可删
#include "Z_math.h"

extern status_display_t LED_state_dis;     /*掉线统计结构体*/   //移植可删

/// @brief 海泰电机启动函数
void HT_motor_star(void)
{
    CAN1_TX_HT(HT_MOTOR_CAN_ID_1,0,HT_MOROT_COM_MOTOR,HT_MOTOR_COM);
    vTaskDelay(1);
    CAN1_TX_HT(HT_MOTOR_CAN_ID_1,0,0,HT_MOTOR_DAT);
		vTaskDelay(1);
		CAN1_TX_HT(HT_MOTOR_CAN_ID_2,0,HT_MOROT_COM_MOTOR,HT_MOTOR_COM);
		vTaskDelay(1);
		CAN1_TX_HT(HT_MOTOR_CAN_ID_2,0,0,HT_MOTOR_DAT);
		vTaskDelay(1);
		CAN1_TX_HT(HT_MOTOR_CAN_ID_3,0,HT_MOROT_COM_MOTOR,HT_MOTOR_COM);
		vTaskDelay(1);
		CAN1_TX_HT(HT_MOTOR_CAN_ID_3,0,0,HT_MOTOR_DAT);
		vTaskDelay(1);
		CAN1_TX_HT(HT_MOTOR_CAN_ID_4,0,HT_MOROT_COM_MOTOR,HT_MOTOR_COM);
		vTaskDelay(1);
		CAN1_TX_HT(HT_MOTOR_CAN_ID_4,0,0,HT_MOTOR_DAT);
		vTaskDelay(1);
	
}



/// @brief 海泰电机关闭函数
void HT_motor_over(void)
{
    CAN1_TX_HT(HT_MOTOR_CAN_ID_2,0,HT_MOROT_COM_EXTI,HT_MOTOR_COM);
    vTaskDelay(100);
}


/// @brief 海泰电机CAN发送函数
/// @param ID 电机ID
/// @param moment 力矩
/// @param command 命令
/// @param com_or_dat 选择命令还是力矩
void CAN1_TX_HT(s8 ID,float moment,s8 command,s8 com_or_dat)
{
    s16 temp = float_to_uint(moment,HT_MOTOR_T_MIN,HT_MOTOR_T_MAX,12);
    uint8_t mbox;
    CanTxMsg TxMessage;
    TxMessage.StdId = ID;
    TxMessage.IDE = CAN_Id_Standard;        /*标准标识符*/
    TxMessage.RTR = CAN_RTR_Data;           /*数据帧*/
    TxMessage.DLC = 8;                      /*设置数据长度为8个字节*/
    if(HT_MOTOR_DAT == com_or_dat)          /*发送数据*/
    {
        for(s8 i = 0;i < 6;i++)
        {
            TxMessage.Data[i] = 0x00;
        }
        TxMessage.Data[6] = (temp >> 8)&0x0f;
        TxMessage.Data[7] = temp;
    }
    else if(com_or_dat == HT_MOTOR_COM)         /*如果是发送命令时*/
    {
        for(s8 i = 0;i < 7;i++)
        {
            TxMessage.Data[i] = 0xff;
        }
        TxMessage.Data[7] = command;
    }
    mbox = CAN_Transmit(CAN1,&TxMessage);          /*开启发送*/
    while((CAN_TransmitStatus(CAN1,mbox) == CAN_TxStatus_Failed));
}



/// @brief 海泰发送函数封装
/// @param motor1 电机1的力矩
/// @param motor2 电机2的力矩
/// @param motor3 电机3的力矩
/// @param motor4 电机4的力矩
void HT_motor_can_send(float motor1,float motor2,float motor3,float motor4)
{
    CAN1_TX_HT(HT_MOTOR_CAN_ID_1,motor1,0,HT_MOTOR_DAT);
    vTaskDelay(1);
    CAN1_TX_HT(HT_MOTOR_CAN_ID_2,motor2,0,HT_MOTOR_DAT);
    vTaskDelay(1);
    CAN1_TX_HT(HT_MOTOR_CAN_ID_3,motor3,0,HT_MOTOR_DAT);
    vTaskDelay(1);
    CAN1_TX_HT(HT_MOTOR_CAN_ID_4,motor4,0,HT_MOTOR_DAT);
}




/// @brief 海泰电机解码函数
/// @param ID 电机ID
/// @param data can原始数据
/// @param motor 电机信息结构体指针
void HT_motor_decode(u8 ID,u8* data,HT_motor_return_msg_t* motor)
{
    VLEDBlink_ofdetection_update(&LED_state_dis.chassis_motor_d[ID-1]);     //移植可删
    motor[ID-1].angle = uint_to_float((((s16)data[1] << 8) | data[2]),HT_MOTOR_P_MIN,HT_MOTOR_P_MAX,16);
    motor[ID-1].ID = data[0];
    motor[ID-1].speed = uint_to_float((((s16)data[3] << 4) | ((data[4] >> 4) & 0x0f)),HT_MOTOR_VX_MIN,HT_MOTOR_VX_MAX,12);
    motor[ID-1].moment = uint_to_float(((((s16)data[4] & 0x0f) << 8 ) | data[5]),HT_MOTOR_T_MIN,HT_MOTOR_T_MAX,12);
    
    if(ID-1 == 0)
    {
        motor[ID-1].angle = FZ_math_absolute(motor[ID-1].angle -= 2.94507599);
    }
    else if(ID-1 == 1)
    {
        motor[ID-1].angle = FZ_math_absolute(motor[ID-1].angle -= 1.35523224) ;
    }
    else if(ID-1 == 2)
    {
        motor[ID-1].angle = FZ_math_absolute(motor[ID-1].angle -= 3.168029785) ;
    }
    else if(ID-1 == 3)
    {
        motor[ID-1].angle = FZ_math_absolute(motor[ID-1].angle += 0.1166168515);
    }
}





/// @brief float 转换为uint
/// @param x 待转换数字
/// @param x_min x的最小值
/// @param x_max x的最大值
/// @param bits 要转换的位数
/// @return 转换后的数字
uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    
    return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}






/// @brief uint 转换为 float
/// @param x_int 要转换的数字
/// @param x_min x的最小值
/// @param x_max x的最大值
/// @param bits 被转换的位数
/// @return 转换后的数字
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

