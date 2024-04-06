/******************************************
    2023/6/3                              
    作者：韩昂轩                       
    
    1.0  添加了基本功能
    1.0.1 优化了代码并移植到了TYPE-C板上面   23.8.28

    LED板级支持包                  
*******************************************
*/

#include "BSP_LED.h"
#include "stm32f4xx_gpio.h"




/// @brief 初始化LED所需要的引脚；首先定义一个用于初始化的结构体变量；然后初始化引脚所需要的时钟
/// @param  无
void LED_init(void)
{
    GPIO_InitTypeDef gpio_define;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);

    gpio_define.GPIO_Mode = GPIO_Mode_OUT;      //输出模式
    gpio_define.GPIO_OType = GPIO_OType_PP;     //推挽输出
    gpio_define.GPIO_PuPd = GPIO_PuPd_UP;       //上拉
    gpio_define.GPIO_Speed = GPIO_Low_Speed;    //低速
    gpio_define.GPIO_Pin = LED_R_PIN;          //引脚号
    
    GPIO_Init(GPIOH,&gpio_define);

    gpio_define.GPIO_Pin = LED_G_PIN;          //引脚号
    GPIO_Init(GPIOH,&gpio_define);

    gpio_define.GPIO_Pin = LED_B_PIN;
    GPIO_Init(GPIOH,&gpio_define);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    LED_R_OFF();  //设置红、蓝色LED引脚为高电平  保证初始化结束后灯是灭的
    LED_G_OFF();
    LED_B_OFF();
}           

