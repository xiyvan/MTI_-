/******************************************
    2023/8/28                              
    作者：韩昂轩
    
    1.0  添加了基本的蜂鸣器鸣叫    23.8.28
    1.0.1 说明：该任务的目的是响应掉线的序号  24.1.07

    蜂鸣器鸣叫任务                  
*******************************************
*/


#include "BUZZER_task.h"
#include "BSP_pwm.h"
#include "stm32f4xx.h"
#include "LED_Blink_task.h"
#include "CK_Timeout_task.h"

u8 BUZZER_OUTTIME_NUM = 0;


void buzzer_task(void *pvParameters)
{
    
    while (1)
    {
        BUZZER_OUTTIME_NUM = Get_TimeOutNum();
        if(BUZZER_OUTTIME_NUM < 255)
        {
            for(char i = 0; i < BUZZER_OUTTIME_NUM+1;i++)
            {
                TIM_SetCompare3(TIM4, 10000);
                vTaskDelay(100);
                TIM_SetCompare3(TIM4, 0);
                vTaskDelay(100);
            }
        }
        vTaskDelay(500);
    }
}


