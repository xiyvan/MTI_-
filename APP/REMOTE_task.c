/******************************************
    2024.4.6                              
    作者：韩昂轩                       
    

    与云台通信任务包                  
*******************************************
*/

#include "REMOTE_task.h"
#include "BSP_Usart.h"
#include "LED_Blink_task.h"
#include "BSP_CAN.h"
#include "string.h"
#include "stdio.h"


static void solve_gym_msg(u16 cmd_id,u8* data);




Remote_angle_t remote_angle;
// Remote_power_limted_t remote_power_limted;
Remote_speed_t Remote_speed;




/*CAN中断接收函数*/
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;
    if(CAN_GetITStatus(CAN1,CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);                               /*清除中断标志位*/
        CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);                               /*CAN接收函数*/
        if(RxMessage.StdId == REMOTE_ANGLE_HUAN || RxMessage.StdId == REMOTE_SPEED_SET)
        {
            solve_gym_msg(RxMessage.StdId,RxMessage.Data);
        }
    }
}






static void solve_gym_msg(u16 cmd_id,u8* data)
{
    switch (cmd_id)
    {
    case REMOTE_ANGLE_HUAN:
        {
            memcpy(&remote_angle,data,sizeof(Remote_angle_t));
        }break;
        
    case REMOTE_SPEED_SET:
        {
            memcpy(&Remote_speed,data,sizeof(Remote_speed_t));
        }break;
/*
    case REMOTE_POWER_LIMTED:
        {
            memcpy(&remote_power_limted,data,sizeof(Remote_power_limted_t));
        }break;
*/
    default:break;
    }
}




Remote_angle_t* get_angle_p(void)
{
    return &remote_angle;
}


Remote_speed_t* get_speed_set_p(void)
{
    return &Remote_speed;
}
