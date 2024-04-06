/******************************************
    2023/8/28                              
    作者：韩昂轩
    
    1.0  添加了基本舵机驱动    23.8.29
    1.0.1 添加了遥控控制舵机    23.9.16

    舵机控制任务                  
*******************************************
*/


#include "SERVOS_task.h"
#include "stm32f4xx.h"


void static servos_Init(servos_type_t* servos);

servos_type_t servos_main;



void Servos_task(void *pvParameters)
{
    servos_Init(&servos_main);
    while (1)
    {
        if(servos_main.remote_msg->channel_4 > 600)
        {
            TIM_SetCompare1(TIM1, SERVOS_ON_PE);
        }
        else if(servos_main.remote_msg->channel_4 < 600 && servos_main.remote_msg->channel_4 > 0)
        {
            TIM_SetCompare1(TIM1, SERVOS_OFF_PE);
        }
        vTaskDelay(20);
    }
}




/// @brief 初始化
/// @param servos 舵机结构体指针
void static servos_Init(servos_type_t* servos)
{
    servos->remote_msg = get_remote_msg_p();
}
