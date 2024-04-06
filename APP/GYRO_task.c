/******************************************
    2023/10/19                              
    作者：韩昂轩
    
    1.0  添加了陀螺仪信息接收并解码的功能   23.10.19

    陀螺仪信息解算任务                  
*******************************************
*/


#include "GYRO_task.h"
#include "WT_gyro.h"
#include "BSP_Usart.h"
#include "CHASSIS_task.h"

extern CHASSIS_struct_t Main_chassis;

uint8_t usart1_dma_rx_buff[USART1_DMA_RX_BUFF_SIZE] = {0};


/// @brief 串口一中断函数，关于串口1的所有中断，如空闲中断、接收非空中断等-----本函数名称在启动文件的中断向量表中寻找
/// @param  无
void USART1_IRQHandler(void)
{
    volatile uint8_t rc_tmp;                                /*编译器认为这个变量没有被使用，，加上volatile防止被优化掉而导致报错*/
    if(USART_GetITStatus(USART1,USART_IT_IDLE))             //判断是不是串口空闲中断
    {
        DMA_Cmd(DMA2_Stream2,DISABLE);
        rc_tmp=USART1->SR;
        rc_tmp=USART1->DR;                                  /*访问SR & DR是为了清除中断标志位*/
        //data_len = USART1_DMA_RX_DATA_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);       /*设置的接受长度 - 剩余的长度 = 本次接收的长度*/
        DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);              /*DMA传输完成标志位*/
        DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TEIF2);              /*DMA传输错误标志位*/
        WT_gyro_decode(usart1_dma_rx_buff,&Main_chassis.chassis_msg.GYRO_msg);           /*数据处理函数*/
    }
    DMA_SetCurrDataCounter(DMA2_Stream2,USART1_DMA_RX_DATA_LEN);            /*重新装填DMA数据长度*/
    DMA_Cmd(DMA2_Stream2,ENABLE);                   /*使能DMA*/
}



