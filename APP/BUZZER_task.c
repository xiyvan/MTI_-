/******************************************
    2023/8/28                              
    作者：韩昂轩
    
    1.0  添加了基本的蜂鸣器鸣叫    23.8.28

    蜂鸣器鸣叫任务                  
*******************************************
*/



#include "BUZZER_task.h"
#include "BSP_pwm.h"
#include "stm32f4xx.h"
#include "LED_Blink_task.h"

extern u8 LED_Blink_outime_num;

void buzzer_task(void *pvParameters)
{
    u8 BUZZER_OUTTIME_NUM = 0;
    while (1)
    {
        BUZZER_OUTTIME_NUM = LED_Blink_outime_num;
        if(BUZZER_OUTTIME_NUM < 230)
        {
            for(char i = 0; i < BUZZER_OUTTIME_NUM+1;i++)
            {
                TIM_SetCompare3(TIM4, 10000);
                vTaskDelay(200);
                TIM_SetCompare3(TIM4, 0);
                vTaskDelay(200);
            }
        }
        vTaskDelay(1000);
    }
}


