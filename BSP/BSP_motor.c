/******************************************
    2023/9/3                              
    作者：韩昂轩                       
    
    1.0  添加了CAN1的接收中断函数，电机的解码程序（使用的指针有点危险，注意不要超出结构体数组的边界）。     23.9.3

    电机接收解码包                  
*******************************************
*/


#include "BSP_motor.h"
#include "BSP_CAN.h"
#include "CHASSIS_task.h"
#include "RM_motor.h"
#include "LED_Blink_task.h"


extern status_display_t LED_state_dis;
extern CHASSIS_struct_t Main_chassis;



void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;
    if(CAN_GetITStatus(CAN2,CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2,CAN_IT_FMP0);
        CAN_Receive(CAN2,CAN_FIFO0,&RxMessage);
        VLEDBlink_ofdetection_update(&LED_state_dis.chassis_motor_d[RxMessage.StdId - CAN_3508_RETURN]);    //电机掉线更新
        motor_msg_decode_3508(RxMessage.StdId - CAN_3508_RETURN,RxMessage.Data,Main_chassis.motor_msg);
        VLEDBlink_ofdetection_update(&LED_state_dis.chassis_motor_d[RxMessage.StdId - CAN_3508_RETURN-1]);
    }
}


/*CAN中断接收函数*/
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;
    if(CAN_GetITStatus(CAN1,CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);                               /*清除中断标志位*/
        CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);                               /*CAN接收函数*/
        
    }
}

