/******************************************
    2023/6/9                              
    作者：韩昂轩                       
    
    1.0  添加了基本功能
    1.0.1 添加了IIC用延时函数（不精准）  23.6.22

    Delay 板级支持包        
*******************************************
*/


#include "BSP_Delay.h"
#include "core_cm4.h"

volatile static unsigned long systick_time;  //延时时间保存变量





/// @brief systick 初始化函数
/// @param  无
/*void Systick_tim_init(void)
{
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);   //选择时钟源
    SysTick_Config(SystemCoreClock_ts/1000); //设置SYSTICK定时器中断时间为1ms，并打开中断
}
*/




/// @brief 毫秒延时函数
/// @param Time 延时时间
void Delay_ms(unsigned long Time)
{
    systick_time = Time;
    while(systick_time)
		{
		}
}


/// @brief IIC用毫秒延时  不精准
/// @param s 延时毫秒数
void Delay_IIC(u32 s)
{
    long x = 0;
    for(x = 0;x < (168*s);x++);
}


//延时nus
//nus为要延时的us数.     
//注意:nus的值,不要大于798915us(最大值即2^24/fac_us@fac_us=21)
void Delay_us(u32 nus)
{            
       u32 temp;                  
       SysTick->LOAD=nus*168; //时间加载，fac_us与时钟频率有关，例如：168Mhz时
                                 //fac_us=168，168/168M=1us               
       SysTick->VAL=0x00;         //清空计数器
       SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ; //开始倒数      
       do
       {
              temp=SysTick->CTRL;    //读取控制及状态寄存器的值
 
       }while((temp&0x01)&&!(temp&(1<<16))); //等待时间到达，使能且位16为0（未计到0）
       SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; //关闭计数器
       SysTick->VAL =0X00;                      //清空计数器
 
}


void BMI088_delay_us(u32 time)
{
    Delay_us(time);
}

void BMI088_delay_ms(u32 time)
{
    for(int x = 0;x < 1000;x++)
    {
        Delay_us(time);
    }
    
}

//@brief systick定时器中断服务函数
//@param  无
//void SysTick_Handler(void)
//{
//    if(systick_time != 0x00)
//    {
//        systick_time--;
//    }
//}