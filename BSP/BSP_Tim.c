/******************************************
    2023/6/18                              
    作者：韩昂轩                       
    
    1.0  添加了基本功能     23.6.18
    1.0.1 添加了TIM中断与PWM协同作业  23.6.19
    1.0.2 添加了TIM5输入捕获    23.6.20

    定时器板级支持包
*******************************************
*/
#include "BSP_Tim.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "BSP_LED.h"


long tim5_ic_time = 0;
u8 tim5_ic_state = 0;

/// @brief TIM3初始化 
/// @param  无
void TIM3_Init(void)
{
    
    TIM_TimeBaseInitTypeDef tim3_init;
    NVIC_InitTypeDef TIM3_NVIC;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

    tim3_init.TIM_Prescaler = (8400-1);                 //预分频系数   频率是在apb1时钟的基准下倍频之后的为84M.因为APB1的预分频系数不是1所以要APB1频率*2才是定时器3用的时钟频率
    tim3_init.TIM_CounterMode = TIM_CounterMode_Up;     //计数模式为向上计数
    tim3_init.TIM_Period = (10-1);                    //自动重装载值
    tim3_init.TIM_ClockDivision = TIM_CKD_DIV1;         //不分频
    TIM_TimeBaseInit(TIM3,&tim3_init);

    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);            //设置定时器的触发方式为更新中断
    TIM_Cmd(TIM3,ENABLE);                           //使能TIM3，此时tim3才开始计时

    TIM3_NVIC.NVIC_IRQChannel = TIM3_IRQn;           //NVIC的初始化请看NVIC文件里面有介绍
    TIM3_NVIC.NVIC_IRQChannelPreemptionPriority = 0;
    TIM3_NVIC.NVIC_IRQChannelSubPriority = 0;
    TIM3_NVIC.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&TIM3_NVIC);
}





/// @brief 定时器3的中断服务函数
/// @param  无
void TIM3_IRQHandler(void)
{
    
    if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET)        //判断是否为更新中断
    {
        TIM_ClearITPendingBit(TIM3,TIM_IT_Update);          //清除中断标志位
    }
    
}


/// @brief 返回LEDPWM的值
/// @param  无
/// @return pwm值  0-500
int get_led_pwm(void)
{
    return 0;
}

//*******************************************************************************************************************************************



/// @brief 定时器5通道1输入捕获
/// @param 无 
void TIM5_ch1__ic_init(void)
{
    
    GPIO_InitTypeDef GPIO_T;
    TIM_TimeBaseInitTypeDef TIMER_T;
    TIM_ICInitTypeDef TIMER_IC_T;
    NVIC_InitTypeDef NVIC_T;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

    GPIO_T.GPIO_Pin = GPIO_Pin_0;
    GPIO_T.GPIO_Mode = GPIO_Mode_AF;
    GPIO_T.GPIO_OType = GPIO_OType_PP;
    GPIO_T.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_T.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA,&GPIO_T);

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5);   //IO口复用映射到TIM5

    TIMER_T.TIM_Prescaler = (84-1);
    TIMER_T.TIM_Period = (0XFFFFFFFF);
    TIMER_T.TIM_ClockDivision = TIM_CKD_DIV1;
    TIMER_T.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5,&TIMER_T);

    TIMER_IC_T.TIM_Channel = TIM_Channel_1;             //捕获通道1
    TIMER_IC_T.TIM_ICPolarity = TIM_ICPolarity_Rising;  //上升沿触发捕获
    TIMER_IC_T.TIM_ICSelection = TIM_ICSelection_DirectTI; //把通道1映射到TI1上面  +++++++++++++++++++++++++++++++++++++++++
    TIMER_IC_T.TIM_ICPrescaler = TIM_ICPSC_DIV1;        //不分频   时钟频率为84M
    TIMER_IC_T.TIM_ICFilter = 0X00;                     //输入捕获滤波---几个边沿触发一次捕获
    TIM_ICInit(TIM5,&TIMER_IC_T);
    TIM_ITConfig(TIM5,TIM_IT_Update|TIM_IT_CC1,ENABLE); //允许溢出中断与捕获中断
    TIM_Cmd(TIM5,ENABLE);           //时钟使能

    NVIC_T.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_T.NVIC_IRQChannelCmd = ENABLE;
    NVIC_T.NVIC_IRQChannelPreemptionPriority = 2; 
    NVIC_T.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_T);
}



/// @brief tim5输入捕获中断函数   本函数用到了一个 tim5_ic_state 标志位，这个标志为分成三个部分。最高位为1的话就是捕获到了低电平，次高位为1的话就是捕获到了高电平。
/// @param  
void TIM5_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM5,TIM_IT_Update) != RESET)    //判断是不是定时器溢出中断
    {
        if(tim5_ic_state & 0x40)                        //判断是不是捕获低电平状态
        {
            tim5_ic_state++;                            //如果为捕获低电平状态的话，就让 tim5_ic_state 的值+1，用来记录溢出次数。本设计没有考虑state溢出情况，因为让其溢出需要的时间很长
        }
    }

    if(TIM_GetITStatus(TIM5,TIM_IT_CC1) != RESET)       //判断是不是捕获到了电平
    {
        if(tim5_ic_state & 0x40)                        //如果现在为捕获低电平状态
        {
            tim5_ic_state |= 0x80;                      //把状态改为捕获高电平状态
            tim5_ic_time = TIM_GetCapture1(TIM5);       //把定时器现在的值赋值给time
            TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Rising);
        }
        else
        {
            tim5_ic_state = 0;                          //先把标志位清零
            tim5_ic_state |= 0x40;                      //再把标志位设置为捕获低电平状态
            tim5_ic_time = 0;                           //把记录时间清零
            TIM_Cmd(TIM5,DISABLE);                      //失能定时器，为后面设置定时器的值做准备
            TIM_SetCounter(TIM5,0);                     //把定时器的值设置为0
            TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Falling);
            TIM_Cmd(TIM5,ENABLE);                       //使能定时器5
        }
    }
    TIM_ClearITPendingBit(TIM5,TIM_IT_CC1|TIM_IT_Update);//清除中断标志位
}



