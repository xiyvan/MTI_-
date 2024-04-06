/******************************************
    最新更新时间：2023/6/3                              
    作者：韩昂轩

    1.0  添加了基本功能    23.6.3
    1.0.1 添加了exti中断示例   23.6.17

    BEEP板级支持包   GPIOA | GPIOE
*******************************************
*/



#include "BSP_KEY.h"
#include "BSP_LED.h"
#include "BSP_Delay.h"



/// @brief 按键初始化，，初始化原理与LED初始化相同，，区别点：输入模式，高速，不用设置输出模式
/// @param  
void KEY_init(void)
{
    GPIO_InitTypeDef gpio_define;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

    gpio_define.GPIO_Mode = GPIO_Mode_IN;       //输入模式
    gpio_define.GPIO_PuPd = GPIO_PuPd_UP;       //上拉
    gpio_define.GPIO_Speed = GPIO_Fast_Speed;    //中高速
    gpio_define.GPIO_Pin = KEY_1_PIN;            //引脚号

    GPIO_Init(GPIOE,&gpio_define);
//---------------------------------------------------------------------------------
    gpio_define.GPIO_Mode = GPIO_Mode_IN;       //输入模式
    gpio_define.GPIO_PuPd = GPIO_PuPd_UP;       //上拉
    gpio_define.GPIO_Speed = GPIO_Fast_Speed;    //中高速
    gpio_define.GPIO_Pin = KEY_2_PIN;            //引脚号

    GPIO_Init(GPIOE,&gpio_define);
//---------------------------------------------------------------------------------
    gpio_define.GPIO_Mode = GPIO_Mode_IN;       //输入模式
    gpio_define.GPIO_PuPd = GPIO_PuPd_UP;       //上拉
    gpio_define.GPIO_Speed = GPIO_Fast_Speed;    //中高速
    gpio_define.GPIO_Pin = KEY_3_PIN;            //引脚号

    GPIO_Init(GPIOE,&gpio_define);
//---------------------------------------------------------------------------------
    gpio_define.GPIO_Mode = GPIO_Mode_IN;       //输入模式
    gpio_define.GPIO_PuPd = GPIO_PuPd_UP;       //上拉
    gpio_define.GPIO_Speed = GPIO_Fast_Speed;    //中高速
    gpio_define.GPIO_Pin = KEY_4_PIN;            //引脚号

    GPIO_Init(GPIOA,&gpio_define);
}




/// @brief 获取按键的电平
/// @param GPIOx 哪一组引脚
/// @param GPIO_Pin 那个引脚
/// @return 返回1的话就是对应引脚为高电平，0的话就是低电平
uint8_t get_key_val(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	u8 i = 0;
    if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin))
    {
        for(i = 0; i < 100;i++);
        if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin))
        {
            return 1;
        }
        return 0;
    }
    return 0;
}






void Exti_key0_init(void)
{
    EXTI_InitTypeDef exti_init;
    NVIC_InitTypeDef EXTI_NVIC_INIT;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);            //开启syscfg时钟

    KEY_init();                                                     //初始化key的IO口
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE,EXTI_PinSource2);    //链接exti的中断线

    exti_init.EXTI_Line = EXTI_Line2;                           //exti中断线2
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;                  //exti为中断模式
    exti_init.EXTI_Trigger = EXTI_Trigger_Rising_Falling;       //exti触发方式为高低电平均触发
    exti_init.EXTI_LineCmd = ENABLE;                            //exti中断线使能
    EXTI_Init(&exti_init);                                      //exti初始化

    EXTI_NVIC_INIT.NVIC_IRQChannel = EXTI2_IRQn;                //nvic中断源
    EXTI_NVIC_INIT.NVIC_IRQChannelPreemptionPriority = 4;       //抢占优先级为4
    EXTI_NVIC_INIT.NVIC_IRQChannelSubPriority = 0;
    EXTI_NVIC_INIT.NVIC_IRQChannelCmd = ENABLE;                 //nvic使能
    NVIC_Init(&EXTI_NVIC_INIT);

}





/// @brief 按键0的中断函数  功能按下的时候 绿灯亮
void EXTI2_IRQHandler()
{
    static u8 xi = 0;
    EXTI_ClearITPendingBit(EXTI_Line2);     //清楚中断标志位
    if(xi %2 == 0)
    {
        LED_B_ON();
        xi++;
    }
    else if(xi %2 == 1)
    {
        LED_B_OFF();
        xi++;
    }
    if(xi == 10)
    {
        xi = 0;
    }
}

