/******************************************
    最新更新时间：2023/6/3                              
    作者：韩昂轩

    1.0  添加了基本功能

    BEEP板级支持包   GPIOF                 
*******************************************
*/

#include "BSP_BEEP.h"
#include "stm32f4xx_gpio.h"



/// @brief 蜂鸣器引脚初始化，原理同LED引脚初始化
/// @param  无
void Beep_init(void)
{
    GPIO_InitTypeDef gpio_define;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);

    gpio_define.GPIO_Mode = GPIO_Mode_OUT;      //输出模式
    gpio_define.GPIO_OType = GPIO_OType_PP;     //推挽输出
    gpio_define.GPIO_PuPd = GPIO_PuPd_DOWN;      //下拉
    gpio_define.GPIO_Speed = GPIO_Low_Speed;    //低速
    gpio_define.GPIO_Pin = BEEP_PIN;          //引脚号

    GPIO_Init(GPIOF,&gpio_define);
}

