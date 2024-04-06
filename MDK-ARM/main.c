#include "stm32f4xx.h"
#include "BSP_LED.h"
#include "BSP_pwm.h"
#include "BSP_Usart.h"
#include "BSP_Tim.h"
#include "BSP_Delay.h"
#include "BSP_CAN.h"
#include "mk_task.h"
#include "FreeRTOS.h"
#include "task.h"



int main(void)
{
	/*初始化用*/

 	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	LED_init();
	//TIM4_PWM_init();
	//TIM1_PWM_init();
	//TIM10_PWM_init();
	USART1_Init();
	//USART3_Init();
	CAN1_init(); 
	CAN2_init();  
	/*创建任务*/
	
	create_task();
	/*进入任务调度*/
	vTaskStartScheduler();
	while(1)
	{
		 
 	}
}




