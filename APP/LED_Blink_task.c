/******************************************
    2023/8/28                              
    作者：韩昂轩                       
    
    1.0  添加了指示灯任务，程序运行指示灯       23.8.28
    1.0.1 添加了掉线检测功能，有设备掉线的时候LED红灯亮起蜂鸣器叫  23.9.2

    指示灯任务
    
    使用说明：添加新的需要检测的设备的时候 需要在头文件结构体里添加相应的变量
                然后再头文件的枚举里面添加相应的变量，并再该文件内的初始化函
                数里面添加相应的指针,    注意记得再需要更新的地方添加更新函数
*******************************************
*/


#include "LED_Blink_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "BSP_LED.h"


static void Now_Time_updata(u8* time);
static void LED_Blink_init(void);
static u8 Return_fail_number(void);


status_display_t LED_state_dis;     /*掉线统计结构体*/
u8 Now_Time = 0;                    /*当前时间*/
u8 LED_Blink_outime_num = 240;      /*记录掉线ID*/
static u8* time_list[HOOK_ALL];   /*创建一个指针数组，用来遍历所有注册了的外设更新时间*/

/// @brief led指示灯任务
/// @param pvParameters 
void led_task(void *pvParameters)
{
    
    LED_Blink_init();
	while(1)
	{
        LED_G_ON();
        LED_Blink_outime_num = Return_fail_number();
        vTaskDelay(500);
        LED_G_OFF();
        LED_B_ON();
        LED_Blink_outime_num = Return_fail_number();
        vTaskDelay(500);
        Now_Time_updata(&Now_Time);                      /*更新系统时间*/
        LED_B_OFF();
        if(LED_Blink_outime_num != 240)
        {
            LED_R_ON();
        }
        else
        {
            LED_R_OFF();
        }
	}
}




/// @brief 外设更新时间
/// @param time外设时间
void VLEDBlink_ofdetection_update(u8* time)
{
    *time = Now_Time;
}





/// @brief 总计时更新
/// @param time 
static void Now_Time_updata(u8* time)
{
    time[0]++;
    if(*time >= 240)
    {
        *time = 0;
    }
}



/// @brief 初始化指针数组里面的指针(添加了新设备 要在这里进行注册 注册先后决定了报警优先级)
static void LED_Blink_init(void)
{
    time_list[0] = &LED_state_dis.Remote_time_out_d;
    time_list[1] = &LED_state_dis.chassis_motor_d[0];
    time_list[2] = &LED_state_dis.chassis_motor_d[1];
    time_list[3] = &LED_state_dis.chassis_motor_d[2];
    time_list[4] = &LED_state_dis.chassis_motor_d[3];
}



/// @brief 遍历所有注册的外设时间
/// @return 无掉线的就返回 240  有掉线的就返回掉线的注册序号
static u8 Return_fail_number(void)
{
    for(int x = 0;x < HOOK_ALL;x++)
    {
        if((Now_Time - *time_list[x] > TIMEOUT_CHECK) || (Now_Time - *time_list[x] < -TIMEOUT_CHECK))
        {
            return x;
        }
    }
    return 240;
}


