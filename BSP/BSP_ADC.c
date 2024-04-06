/******************************************
    2023/6/22                              
    作者：韩昂轩                       
    
    1.0  添加了adc初始化与adc通道值获取函数  23.6.22

    ADC板级支持包                
*******************************************
*/

#include "stm32f4xx_adc.h"
#include "BSP_ADC.h"

void ADC_init_I(void)
{
    GPIO_InitTypeDef gpio_define;
    ADC_CommonInitTypeDef adc_c_define;
    ADC_InitTypeDef adc_define;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3,ENABLE);

    gpio_define.GPIO_Mode = GPIO_Mode_AN;      //模拟输入模式
    gpio_define.GPIO_PuPd = GPIO_PuPd_NOPULL;     //不上拉下拉
    gpio_define.GPIO_Speed = GPIO_Fast_Speed;    //高速
    gpio_define.GPIO_Pin = GPIO_Pin_10;          //引脚号
    GPIO_Init(GPIOF,&gpio_define);

    adc_c_define.ADC_Mode = ADC_Mode_Independent;                        //ADC模式为独立模式
    adc_c_define.ADC_Prescaler = ADC_Prescaler_Div4;                     //ADC分频为4分频    84M/4 = 21M  ADC频率不能超过36M
    adc_c_define.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;         //ADC不使用DMA
    adc_c_define.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;    //ADC两次采样之间的间隔  5个时钟周期
    ADC_CommonInit(&adc_c_define);  //ADC基本配置

    adc_define.ADC_Resolution = ADC_Resolution_12b;                     //ADC采样精度  12位
    adc_define.ADC_ScanConvMode = DISABLE;                              //adc扫描模式使能
    adc_define.ADC_ContinuousConvMode = DISABLE;                        //adc连续转换使能
    adc_define.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;    //禁止触发检测，使用软件触发
    adc_define.ADC_DataAlign = ADC_DataAlign_Right;                      //ADC数据对齐方式为右对齐
    adc_define.ADC_NbrOfConversion = 1;                                 //规则序列中存在一个转换
    ADC_Init(ADC1,&adc_define);

    ADC_Cmd(ADC1,ENABLE);       //adc使能
}


/// @brief 获取adc通道值
/// @param adc adc号
/// @param ch 通道号
/// @return 通道值
uint16_t Get_adc_val(ADC_TypeDef* adc,u8 ch)
{

    ADC_RegularChannelConfig(adc,ch,1,ADC_SampleTime_480Cycles);  //设置ADC通道
    ADC_SoftwareStartConv(adc);                                 //软件开启ADC转换
    while(!ADC_GetFlagStatus(adc,ADC_FLAG_EOC));
    return ADC_GetConversionValue(adc);
}





/// @brief adc通道值平均值获取
/// @param adc adc号
/// @param ch 通道号
/// @param time 取几次平均值
/// @return 平均值
uint16_t Get_adc_aveval(ADC_TypeDef* adc,u8 ch,uint8_t time)
{
    long temp = 0;
    for(int x = 0;x < time;x++)
    {
        temp += Get_adc_val(adc,ch);
    }
    return temp/time;
}
