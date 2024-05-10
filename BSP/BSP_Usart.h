#ifndef _BSP_USART_H_
#define _BSP_USART_H_

#include "stm32f4xx.h"


#define USART1_DMA_TX_BUFF_SIZE 1       //串口一DMA发送缓冲大小(象征性,因为dma的发送函数会把发送地址重置)
#define USART1_DMA_RX_BUFF_SIZE 50      //串口一DMA接收缓冲大小
#define USART3_DMA_RX_BUFF_SIZE 20      /*串口3DMA接收缓冲大小*/

#define USART1_DMA_RX_DATA_LEN 50       /*串口1DMA接收数据长度设置  要小于接收缓冲大小*/
#define USART3_DMA_RX_DATA_LEN 20       /*串口3DMA接收数据长度设置  */


void USART1_Init(void);
void usart_send_date(USART_TypeDef* usart,unsigned char* data,int len);
void usart1_DMA_send_date(unsigned char* data,int len);
void USART3_Init(void);

#endif
