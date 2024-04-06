/******************************************
    2023/6/19                              
    作者：韩昂轩                       
    
    1.0  添加了TIM14的PWM初始化函数用来LED R的测试   23.6.19
    1.0.1 修改了TIM14的pwm初始化函数用来适配  TYPE-C 板  TIM4用来驱动蜂鸣器  23.8.28
    1.0.2 添加了TIM1的pwm初始化函数用来驱动舵机  23.8.30            注意TIM1与TIM8需要pwm输出使能
    1.0.3 添加了TIM1的PWM初始化多通道输出        23.8.31            TIM1是高级定时器有多个配置需要配置否则会出错

    PWM板级支持包   GPIOF                  
*******************************************
*/

#include "BSP_pwm.h"
#include "BSP_LED.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"




/// @brief PWM初始化函数
/// @param  无
void TIM4_PWM_init(void)
{
    GPIO_InitTypeDef pwm_gpio_init;
    TIM_TimeBaseInitTypeDef pwm_tim_init;
    TIM_OCInitTypeDef tim_pwm_init;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);    //使能定时器时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);    //使能IO口时钟

    pwm_gpio_init.GPIO_Pin = GPIO_Pin_14;        //引脚9
    pwm_gpio_init.GPIO_Mode = GPIO_Mode_AF;     //GPIO模式为复用模式
    pwm_gpio_init.GPIO_OType = GPIO_OType_PP;   //推挽输出
    pwm_gpio_init.GPIO_PuPd = GPIO_PuPd_UP;     //上拉
    pwm_gpio_init.GPIO_Speed = GPIO_High_Speed; //高速
    GPIO_Init(GPIOD,&pwm_gpio_init);

    GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);      //IO口复用

    pwm_tim_init.TIM_Prescaler = 15;                         //预分频系数为84M
    pwm_tim_init.TIM_Period = (21000-1);                    //自动重装载值为500
    pwm_tim_init.TIM_ClockDivision = TIM_CKD_DIV1;
    pwm_tim_init.TIM_CounterMode = TIM_CounterMode_Up;      //计数模式为向上计数
    TIM_TimeBaseInit(TIM4,&pwm_tim_init);

    tim_pwm_init.TIM_OCMode = TIM_OCMode_PWM1;              //设置PWM为模式一   在此模式下若计数低于比较值的话为有效值   模式二与此相反
    tim_pwm_init.TIM_OCPolarity = TIM_OCPolarity_High;       //设置PWM有效的是偶为低电平
    tim_pwm_init.TIM_OutputState = ENABLE;                  //使能

    TIM_OC3Init(TIM4,&tim_pwm_init);

    TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);        //使能预装载寄存器

    TIM_ARRPreloadConfig(TIM4,ENABLE);                     //ARPE使能

    TIM_Cmd(TIM4,ENABLE);                                  //定时器使能

}




/// @brief tim1用作pwm输出主要是控制舵机  50hz的频率1-4个通道
void TIM1_PWM_init(void)
{
    GPIO_InitTypeDef pwm_gpio_init;
    TIM_TimeBaseInitTypeDef pwm_tim_init;
    TIM_OCInitTypeDef tim_pwm_init;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);    //使能定时器时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);    //使能IO口时钟

    GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1);      //IO口复用
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1);      //IO口复用
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1);      //IO口复用
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1);      //IO口复用

    pwm_gpio_init.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_14;
    pwm_gpio_init.GPIO_Mode = GPIO_Mode_AF;     //GPIO模式为复用模式
    pwm_gpio_init.GPIO_OType = GPIO_OType_PP;   //推挽输出
    pwm_gpio_init.GPIO_PuPd = GPIO_PuPd_UP;     //上拉
    pwm_gpio_init.GPIO_Speed = GPIO_High_Speed; //高速
    GPIO_Init(GPIOE,&pwm_gpio_init);

    pwm_tim_init.TIM_Prescaler = (168 - 1);                   //预分频系数为167M  ----->1Mhz
    pwm_tim_init.TIM_Period = (20000-1);                    //自动重装载值为20000 ------>1M/20000 = 50HZ
    pwm_tim_init.TIM_ClockDivision = TIM_CKD_DIV1;
    pwm_tim_init.TIM_CounterMode = TIM_CounterMode_Up;      //计数模式为向上计数
    pwm_tim_init.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1,&pwm_tim_init);

    tim_pwm_init.TIM_OCMode = TIM_OCMode_PWM2;              //设置PWM为模式一   在此模式下若计数低于比较值的话为有效值   模式二与此相反
    tim_pwm_init.TIM_OCPolarity = TIM_OCPolarity_Low;       //设置PWM有效的是偶为低电平
    tim_pwm_init.TIM_OutputState = ENABLE;                  //使能
    tim_pwm_init.TIM_OutputNState = DISABLE;
    tim_pwm_init.TIM_OCNPolarity = TIM_OCPolarity_Low;
    tim_pwm_init.TIM_OCIdleState = 0X00000000U;
    tim_pwm_init.TIM_OCNIdleState = 0X00000000U;
    tim_pwm_init.TIM_Pulse = 2000;
    TIM_OC1Init(TIM1,&tim_pwm_init);
    TIM_OC2Init(TIM1,&tim_pwm_init);
    TIM_OC3Init(TIM1,&tim_pwm_init);
    TIM_OC4Init(TIM1,&tim_pwm_init);


    TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1,ENABLE);                     //ARPE使能

    TIM_Cmd(TIM1,ENABLE);                                  //定时器使能

    TIM_CtrlPWMOutputs(TIM1,ENABLE);                       //pwm输出使能
}


void TIM10_PWM_init(void)
{
    GPIO_InitTypeDef pwm_gpio_init;
    TIM_TimeBaseInitTypeDef pwm_tim_init;
    TIM_OCInitTypeDef tim_pwm_init;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);    //使能定时器时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);    //使能IO口时钟

    pwm_gpio_init.GPIO_Pin = GPIO_Pin_6;        //引脚9
    pwm_gpio_init.GPIO_Mode = GPIO_Mode_AF;     //GPIO模式为复用模式
    pwm_gpio_init.GPIO_OType = GPIO_OType_PP;   //推挽输出
    pwm_gpio_init.GPIO_PuPd = GPIO_PuPd_UP;     //上拉
    pwm_gpio_init.GPIO_Speed = GPIO_High_Speed; //高速
    GPIO_Init(GPIOF,&pwm_gpio_init);

    GPIO_PinAFConfig(GPIOF,GPIO_PinSource6,GPIO_AF_TIM10);      //IO口复用

    pwm_tim_init.TIM_Prescaler = 168;                        //预分频  1M
    pwm_tim_init.TIM_Period = (2000-1);                     //自动重装载值为 2000
    pwm_tim_init.TIM_ClockDivision = TIM_CKD_DIV1;
    pwm_tim_init.TIM_CounterMode = TIM_CounterMode_Up;      //计数模式为向上计数
    TIM_TimeBaseInit(TIM10,&pwm_tim_init);

    tim_pwm_init.TIM_OCMode = TIM_OCMode_PWM1;              //设置PWM为模式一   在此模式下若计数低于比较值的话为有效值   模式二与此相反
    tim_pwm_init.TIM_OCPolarity = TIM_OCPolarity_High;       //设置PWM有效的是偶为低电平
    tim_pwm_init.TIM_OutputState = ENABLE;                  //使能

    TIM_OC1Init(TIM10,&tim_pwm_init);

    TIM_OC1PreloadConfig(TIM10,TIM_OCPreload_Enable);        //使能预装载寄存器

    TIM_ARRPreloadConfig(TIM10,ENABLE);                     //ARPE使能

    TIM_Cmd(TIM10,ENABLE);                                  //定时器使能

}