/******************************************
    2023/6/16                              
    作者：韩昂轩                       
    
    1.0     添加了基本功能(存在BUG，使用串口发送的时候)                                           23.6.16
    1.0.1   添加了串口多数据发送函数（花了40块钱找别人找bug成功修复bug，库函数版本与教程不符）      23.6.18
    1.0.2   添加了串口DMA发送配置并添加了串口一DMA传输函数                                        23.6.22
    1.0.3   添加了串口DMA接收的配置，并把中断改为空闲中断，两个配合可以用来接收不定长数据           23.8.30           
    1.0.4   解决BUG用示波器看了才知道，上一次根本就不是库函数的问题，很可能是频率不对的问题，
            这次研读库函数代码发现，移植库函数的时候注意要把外晶振频率改掉，                       23.9.5

    串口板级支持包  
*******************************************
*/

#include "BSP_Usart.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_dma.h"
#include "misc.h"
#include "WT_gyro.h"


static u8 usart1_recever_buff = 0;                              //串口一接收缓冲
uint8_t usart1_dma_tx_buff[USART1_DMA_TX_BUFF_SIZE] = {0};      //串口一DMA发送缓冲
uint8_t usart1_dma_rx_buff[USART1_DMA_RX_BUFF_SIZE];

void USART1_Init(void)
{
    DMA_InitTypeDef usart1_dma_init;
    GPIO_InitTypeDef usart1_init1;
    USART_InitTypeDef usart1_init_f;
    NVIC_InitTypeDef usart1_nvic_init;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);

    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //IO口复用映射   PA9 -> USART1
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);//上同           PB7 -> USART1

    usart1_init1.GPIO_Mode = GPIO_Mode_AF;       //复用模式
    usart1_init1.GPIO_OType = GPIO_OType_PP;
    usart1_init1.GPIO_Speed = GPIO_Speed_50MHz;   //高速
    usart1_init1.GPIO_PuPd = GPIO_PuPd_UP;
    usart1_init1.GPIO_Pin = GPIO_Pin_9;          /*PA9设置*/
    GPIO_Init(GPIOA,&usart1_init1);
    usart1_init1.GPIO_Pin = GPIO_Pin_7;         /*PB7设置*/
    GPIO_Init(GPIOB,&usart1_init1);

    usart1_init_f.USART_BaudRate = 115200;                  //设置波特率
    usart1_init_f.USART_WordLength = USART_WordLength_8b;   //设置数据长度为8位
    usart1_init_f.USART_StopBits = USART_StopBits_1;        //设置停止位为1位
    usart1_init_f.USART_Parity = USART_Parity_No;           //无奇偶校验位
    usart1_init_f.USART_Mode = USART_Mode_Rx|USART_Mode_Tx; //又发送又接收
    usart1_init_f.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件流控制
    USART_Init(USART1,&usart1_init_f);
    USART_Cmd(USART1,ENABLE);           //使能串口

    USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);    //设置串口1的中断方式为空闲中断
    usart1_nvic_init.NVIC_IRQChannel = USART1_IRQn;         //设置中断源
    usart1_nvic_init.NVIC_IRQChannelPreemptionPriority = 0; //设置抢占优先级
    usart1_nvic_init.NVIC_IRQChannelSubPriority = 0;        //设置响应优先级
    usart1_nvic_init.NVIC_IRQChannelCmd = ENABLE;           //使能这个串口中断
    NVIC_Init(&usart1_nvic_init);

    usart1_dma_init.DMA_Channel = DMA_Channel_4;                            //DMA通道数
    usart1_dma_init.DMA_PeripheralBaseAddr = (u32)&(USART1->DR);            //外设地址
    usart1_dma_init.DMA_Memory0BaseAddr = (u32)usart1_dma_tx_buff;          //发送缓冲地址
    usart1_dma_init.DMA_DIR = DMA_DIR_MemoryToPeripheral;                   //DMA传输方向
    usart1_dma_init.DMA_BufferSize = 50;                                   //数据传输量
    usart1_dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;          //DMA外设数据指针递增不使能
    usart1_dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;                   //DMA存储器数据指针递增使能
    usart1_dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;   //外设数据格式为1字节8位
    usart1_dma_init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;           //存储器数据格式位1字节8位
    usart1_dma_init.DMA_Mode = DMA_Mode_Normal;                             //DMA模式位普通
    usart1_dma_init.DMA_Priority = DMA_Priority_Medium;                     //DMA优先级
    usart1_dma_init.DMA_FIFOMode = DMA_FIFOMode_Disable;                    //不使用FIFO因为数据可以对齐
    usart1_dma_init.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;             //
    usart1_dma_init.DMA_MemoryBurst = DMA_MemoryBurst_Single;               //储存器突发单次传输
    usart1_dma_init.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;       //外设突发单词传输
    DMA_Init(DMA2_Stream7,&usart1_dma_init);

    usart1_dma_init.DMA_Channel = DMA_Channel_4;                            //DMA通道数
    usart1_dma_init.DMA_DIR = DMA_DIR_PeripheralToMemory;                   //DMA传输方向
    usart1_dma_init.DMA_Memory0BaseAddr = (u32)usart1_dma_rx_buff;          //接收缓冲地址
    DMA_Init(DMA2_Stream2,&usart1_dma_init);

    USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);                //使能串口DMA传输
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
}



//void USART3_Init(void)
//{
//    DMA_InitTypeDef usart1_dma_init;
//    GPIO_InitTypeDef usart1_init1;
//    USART_InitTypeDef usart1_init_f;
//    NVIC_InitTypeDef usart1_nvic_init;
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);

//    GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3); //IO口复用映射   PC11 -> USART3

//    usart1_init1.GPIO_Mode = GPIO_Mode_AF;       //复用模式
//    usart1_init1.GPIO_OType = GPIO_OType_PP;
//    usart1_init1.GPIO_Speed = GPIO_Speed_100MHz;   //高速
//    usart1_init1.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    usart1_init1.GPIO_Pin = GPIO_Pin_11;          /*PA9设置*/
//    GPIO_Init(GPIOC,&usart1_init1);

//    usart1_init_f.USART_BaudRate = 100000;                  //设置波特率
//    usart1_init_f.USART_WordLength = USART_WordLength_8b;   //设置数据长度为8位
//    usart1_init_f.USART_StopBits = USART_StopBits_1;        //设置停止位为1位
//    usart1_init_f.USART_Parity = USART_Parity_No;           //无奇偶校验位
//    usart1_init_f.USART_Mode = USART_Mode_Rx;               //接收
//    usart1_init_f.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件流控制
//    USART_Init(USART3,&usart1_init_f);
//    USART_Cmd(USART3,ENABLE);                           //使能串口

//    USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);    //设置串口3的中断方式为空闲中断
//    usart1_nvic_init.NVIC_IRQChannel = USART3_IRQn;         //设置中断源
//    usart1_nvic_init.NVIC_IRQChannelPreemptionPriority = 1; //设置抢占优先级
//    usart1_nvic_init.NVIC_IRQChannelSubPriority = 0;        //设置响应优先级
//    usart1_nvic_init.NVIC_IRQChannelCmd = ENABLE;           //使能这个串口中断
//    NVIC_Init(&usart1_nvic_init);

//    usart1_dma_init.DMA_Channel = DMA_Channel_4;                            //DMA通道数
//    usart1_dma_init.DMA_PeripheralBaseAddr = (u32)&(USART3->DR);            //外设地址
//    usart1_dma_init.DMA_Memory0BaseAddr = (u32)usart3_rx_data_buf;          //接收缓冲地址
//    usart1_dma_init.DMA_DIR = DMA_DIR_PeripheralToMemory;                   //DMA传输方向
//    usart1_dma_init.DMA_BufferSize = 20;                                    //数据传输量
//    usart1_dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;           //DMA外设数据指针递增不使能
//    usart1_dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;                   //DMA存储器数据指针递增使能
//    usart1_dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;   //外设数据格式为1字节8位
//    usart1_dma_init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;           //存储器数据格式位1字节8位
//    usart1_dma_init.DMA_Mode = DMA_Mode_Normal;                             //DMA模式位普通
//    usart1_dma_init.DMA_Priority = DMA_Priority_Medium;                     //DMA优先级
//    usart1_dma_init.DMA_FIFOMode = DMA_FIFOMode_Disable;                    //不使用FIFO因为数据可以对齐
//    usart1_dma_init.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;             //
//    usart1_dma_init.DMA_MemoryBurst = DMA_MemoryBurst_Single;               //储存器突发单次传输
//    usart1_dma_init.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;       //外设突发单词传输
//    DMA_Init(DMA1_Stream1,&usart1_dma_init);

//    USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);                //使能串口DMA传输
//}
//***********************************************************************************************************************************

/// @brief 串口发送函数
/// @param usart 串口号
/// @param data 数据数组
/// @param len 数据长度
void usart_send_date(USART_TypeDef* usart,unsigned char* data,int len)
{
    int x = 0;
    for(x = 0;x < len;x++)
    {
        while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
        USART_SendData(usart,data[x]);
        while (USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);
    }
}


/// @brief 串口一DMA传输函数
/// @param data 数据
/// @param len 数据长度
void usart1_DMA_send_date(unsigned char* data,int len)
{
    DMA_Cmd(DMA2_Stream7,DISABLE);
    while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);  //确保DMA处于可配置状态，dma此时没有数据传输
    DMA_ClearITPendingBit(DMA2_Stream7,DMA_FLAG_TCIF7); //清除传输完成标志位
    DMA2_Stream7->M0AR = (u32)data;                     //修改DMA存储器地址
    DMA_SetCurrDataCounter(DMA2_Stream7,len);           //DMA数据发送数量填充
    DMA_Cmd(DMA2_Stream7,ENABLE);           //使能DMA
}




