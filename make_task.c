/******************************************
    2023/6/21                              
    作者：韩昂轩
    
    1.0  添加了任务添加函数ADC电压检测    23.6.21
	1.0.1 添加了LED 与 蜂鸣器   		 23.8.28
	1.0.2  添加了姿态解算任务				23.9.10
	1.0.3		添加了云台任务与发射任务				23.10.23

    RTOS任务创建包
*******************************************
*/
#include "mk_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "LED_Blink_task.h"
#include "BUZZER_task.h"
#include "GYRO_task.h"
#include "CHASSIS_task.h"
#include "CK_Timeout_task.h"


void create_task(void)
{
//*********************************************************************************
//								LED指示灯任务创建函数
	TaskHandle_t FREETASK_Handler;
	xTaskCreate((TaskFunction_t)led_task,							//任务函数名称
							 (const char*  ) "led_task",			
							 (uint16_t     ) 64,					//任务堆栈大小		
							 (void*        )NULL,					//任务函数变量
							 (UBaseType_t  )1,						//任务优先级
							 (TaskHandle_t*)&FREETASK_Handler		//任务结构体变量指针
							);
//**********************************************************************************
//								蜂鸣器任务创建
	TaskHandle_t BEZZER_Handler;
	xTaskCreate((TaskFunction_t)buzzer_task,						//任务函数名称
							 (const char*  ) "buzzer_task",			
							 (uint16_t     ) 32,					//任务堆栈大小
							 (void*        )NULL,					//任务函数变量
							 (UBaseType_t  )1,						//任务优先级
							 (TaskHandle_t*)&BEZZER_Handler			//任务结构体变量指针
							);
//**********************************************************************************
//							   底盘任务创建
	TaskHandle_t CHASSIS_Handler;
	xTaskCreate((TaskFunction_t)Chassis_task,						//任务函数名称
							(const char*  ) "Chassis_task",			
							(uint16_t     ) 512,					//任务堆栈大小
							(void*        )NULL,					//任务函数变量
							(UBaseType_t  )3,						//任务优先级
							(TaskHandle_t*)&CHASSIS_Handler			//任务结构体变量指针
					);
//**********************************************************************************
//							   底盘任务创建
	TaskHandle_t REMOTE_Handler;
	xTaskCreate((TaskFunction_t)remote_task,						//任务函数名称
							(const char*  ) "remote_task",			
							(uint16_t     ) 128,					//任务堆栈大小
							(void*        )NULL,					//任务函数变量
							(UBaseType_t  )4,						//任务优先级
							(TaskHandle_t*)&REMOTE_Handler			//任务结构体变量指针
					);
//**********************************************************************************
//							   姿态解算任务创建
	TaskHandle_t GYRO_Handler;
	xTaskCreate((TaskFunction_t)remote_task,						//任务函数名称
							(const char*  ) "GYRO_task",			
							(uint16_t     ) 256,					//任务堆栈大小
							(void*        )NULL,					//任务函数变量
							(UBaseType_t  )3,						//任务优先级
							(TaskHandle_t*)&GYRO_Handler			//任务结构体变量指针
					);
//**********************************************************************************
//							   掉线检测任务创建
	TaskHandle_t CK_Handler;
	xTaskCreate((TaskFunction_t)CK_Timeout_task,						//任务函数名称
							(const char*  ) "CK_Timeout_task",			
							(uint16_t     ) 128,					//任务堆栈大小
							(void*        )NULL,					//任务函数变量
							(UBaseType_t  )2,						//任务优先级
							(TaskHandle_t*)&CK_Handler			//任务结构体变量指针
					);
}
