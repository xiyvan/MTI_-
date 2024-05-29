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
#include "CK_Timeout_task.h"
#include "mk_task.h"


extern CHASSIS_struct_t Main_chassis;



void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;
    if(CAN_GetITStatus(CAN2,CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2,CAN_IT_FMP0);
        CAN_Receive(CAN2,CAN_FIFO0,&RxMessage);
        motor_msg_decode_3508(RxMessage.StdId - CAN_3508_RETURN,RxMessage.Data,Main_chassis.motor_msg);
        CkTime_DriverTimeNew(RxMessage.StdId - CAN_3508_RETURN -1 + TIMEOUT_WHEEL_SPEED_MOTOR1);
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
    #if (CAR_TYPE == CAR_TYPE_Caster_wheel)
        motor_msg_decode_6020(RxMessage.StdId - CAN_6010_RETURN,RxMessage.Data,Main_chassis.angle_motor_msg);
        CkTime_DriverTimeNew(RxMessage.StdId - CAN_6010_RETURN -1 + TIMEOUT_WHEEL_ANGLE_MOTOR1);
    #endif
    }
}

