/******************************************
    2023/8/28                              
    作者：韩昂轩                       
    
    指示灯任务
    如果红灯常亮说明有设备掉线
*******************************************
*/


#include "LED_Blink_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "BSP_LED.h"
#include "CK_Timeout_task.h"


/// @brief led指示灯任务
/// @param pvParameters 
void led_task(void *pvParameters)
{
	while(1)
	{
        if(Get_TimeOutNum() == 255)
        {
            LED_G_ON();
            vTaskDelay(500);
            LED_G_OFF();
            LED_B_ON();
            vTaskDelay(500);
            LED_B_OFF();
        }
        else
        {
            LED_R_ON();
            LED_B_OFF();
            LED_G_OFF();
						vTaskDelay(500);
						LED_R_OFF();
						vTaskDelay(500);
        }
	}
}



