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



extern CHASSIS_struct_t Main_chassis;



void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;
    if(CAN_GetITStatus(CAN2,CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2,CAN_IT_FMP0);
        CAN_Receive(CAN2,CAN_FIFO0,&RxMessage);
        
    }
}

