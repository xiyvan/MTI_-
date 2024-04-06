/******************************************
    2023/8/31                              
    作者：韩昂轩                       
    
    1.0  添加了CAN1的基础配置与CAN1的发送函数   23.8.31
    1.0.1 终于修复了CAN1的BUG，TQ不对导致发送与接收的问题，很奇怪为什么按照官方的来却不一样。   23.9.2
    can板级支持包                  
*******************************************
*/



#include "BSP_CAN.h"
#include "stm32f4xx_can.h"




void CAN1_init(void)
{
    CAN_InitTypeDef CAN_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1);   /*复用为CAN1*/
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;               /*复用模式*/
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;             
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOD,&GPIO_InitStruct);

    CAN_InitStruct.CAN_Prescaler = 3; //3                  /*CAN的分频系数*/
    CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;          /*CAN的模式 ---->普通模式*/
    CAN_InitStruct.CAN_SJW = 1;
    CAN_InitStruct.CAN_BS1 = 8; //8
    CAN_InitStruct.CAN_BS2 = 3; //3
    CAN_InitStruct.CAN_ABOM = DISABLE;              /*失能自动总线关闭功能*/
    CAN_InitStruct.CAN_AWUM = DISABLE;              /*失能自动唤醒模式*/
    CAN_InitStruct.CAN_NART = ENABLE;               /*自动重传失能 提高总线的传输实时性*/
    CAN_InitStruct.CAN_RFLM = DISABLE;              /*使用FIFO溢出模式，如果使能之后FIFO满了之后将不会接收新的数据*/
    CAN_InitStruct.CAN_TTCM = DISABLE;              /*失能时间触发发送*/
    CAN_InitStruct.CAN_TXFP = DISABLE;              /*标准FIFO优先级*/
    CAN_Init(CAN1,&CAN_InitStruct);

    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    CAN_FilterInitStructure.CAN_FilterNumber = 0;                       // 使用过滤器0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;     // 使用标识符屏蔽位模式
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;    // 使用32位屏蔽
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;                  // 设置标识符高位
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;                   // 设置标识符低位
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;              // 设置屏蔽高位
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;               // 设置屏蔽低位
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;       // 将过滤器分配给FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);


    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);         //     /*设置中断标志位*/

    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}


void CAN2_init(void)
{
    CAN_InitTypeDef CAN_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_CAN2);   /*复用为CAN2*/
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_CAN2);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;               /*复用模式*/
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;             
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_Init(GPIOB,&GPIO_InitStruct);

    CAN_InitStruct.CAN_Prescaler = 3; //3                  /*CAN的分频系数*/
    CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;          /*CAN的模式 ---->普通模式*/
    CAN_InitStruct.CAN_SJW = 1;
    CAN_InitStruct.CAN_BS1 = 8; //8
    CAN_InitStruct.CAN_BS2 = 3; //3
    CAN_InitStruct.CAN_ABOM = DISABLE;              /*失能自动总线关闭功能*/
    CAN_InitStruct.CAN_AWUM = DISABLE;              /*失能自动唤醒模式*/
    CAN_InitStruct.CAN_NART = ENABLE;               /*自动重传失能 提高总线的传输实时性*/
    CAN_InitStruct.CAN_RFLM = DISABLE;              /*使用FIFO溢出模式，如果使能之后FIFO满了之后将不会接收新的数据*/
    CAN_InitStruct.CAN_TTCM = DISABLE;              /*失能时间触发发送*/
    CAN_InitStruct.CAN_TXFP = DISABLE;              /*标准FIFO优先级*/
    CAN_Init(CAN2,&CAN_InitStruct);

    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    CAN_FilterInitStructure.CAN_FilterNumber = 15;                       // 使用过滤器0
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;     // 使用标识符屏蔽位模式
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;    // 使用32位屏蔽
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;                  // 设置标识符高位
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;                   // 设置标识符低位
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;              // 设置屏蔽高位
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;               // 设置屏蔽低位
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;       // 将过滤器分配给FIFO0
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);         //     /*设置中断标志位*/

    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = CAN2_RX0_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}







