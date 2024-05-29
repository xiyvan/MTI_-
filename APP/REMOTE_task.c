/******************************************
    2024.4.6                              
    作者：韩昂轩                       
    

    与云台通信任务包                  
*******************************************
*/

#include "REMOTE_task.h"
#include "BSP_Usart.h"
#include "CK_Timeout_task.h"
#include "BSP_CAN.h"
#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Fifo.h"


#define USART1_DMA_RX_DATA_LEN 20
#define REMOTE_FIFO_SIZE 512


static void solve_gym_msg(u16 cmd_id,u8* data);
static void one_byte_solve(u8 date,remote_solve_one_t* one_where);




FIFO_t remote_fifo;
remote_solve_one_t remote_step;
u8 remote_fifo_queue[REMOTE_FIFO_SIZE] = {0};
Remote_angle_t remote_angle;
//Remote_power_limted_t remote_power_limted;
Remote_speed_t Remote_speed;

extern uint8_t usart1_dma_rx_buff[USART1_DMA_RX_BUFF_SIZE];



void remote_task(void *pvParameters)
{
    remote_fifo.max = REMOTE_FIFO_SIZE;
    remote_fifo.num = 0;
    remote_fifo.queue = remote_fifo_queue;
    while (1)
    {
        while (remote_fifo.num)
        {
            one_byte_solve(Fifo_Get(&remote_fifo),&remote_step);
        }
        vTaskDelay(2);
    }
}




/// @brief 串口一中断函数，关于串口1的所有中断，如空闲中断、接收非空中断等-----本函数名称在启动文件的中断向量表中寻找
/// @param  无
void USART1_IRQHandler(void)
{
    volatile uint8_t rc_tmp;                                /*编译器认为这个变量没有被使用，，加上volatile防止被优化掉而导致报错*/
    u16 data_len;                                           // 保存本次发送的数据长度
    if(USART_GetITStatus(USART1,USART_IT_IDLE))             //判断是不是串口空闲中断
    {
        DMA_Cmd(DMA2_Stream2,DISABLE);
        rc_tmp=USART1->SR;
        rc_tmp=USART1->DR;                                  /*访问SR & DR是为了清除中断标志位*/
        data_len = USART1_DMA_RX_DATA_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);       /*设置的接受长度 - 剩余的长度 = 本次接收的长度*/
		Fifo_AddNum(&remote_fifo,usart1_dma_rx_buff,data_len);               /*数据处理函数*/
        DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);              /*DMA传输完成标志位*/
        DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TEIF2);              /*DMA传输错误标志位*/
    }
    DMA_SetCurrDataCounter(DMA2_Stream2,USART1_DMA_RX_DATA_LEN);            /*重新装填DMA数据长度*/
    DMA_Cmd(DMA2_Stream2,ENABLE);                   /*使能DMA*/
}

/// @brief 接受信息解码
/// @param cmd_id 命令码
/// @param data 原始数据
static void solve_gym_msg(u16 cmd_id,u8* data)
{
    switch (cmd_id)
    {
    case REMOTE_ANGLE_HUAN:
        {
            memcpy(&remote_angle,data,sizeof(Remote_angle_t));
						CkTime_DriverTimeNew(TIMEOUT_REMOTE_TIMEOUT);     // 掉线检测回调
        }break;
        
    case REMOTE_SPEED_SET:
        {
            memcpy(&Remote_speed,data,sizeof(Remote_speed_t));
            CkTime_DriverTimeNew(TIMEOUT_REMOTE_TIMEOUT);     // 掉线检测回调
        }break;
/*
    case REMOTE_POWER_LIMTED:
        {
            memcpy(&remote_power_limted,data,sizeof(Remote_power_limted_t));
        }break;
*/
    default:break;
    }
}



/// @brief 单字节解包
/// @param date 从fifo里面取出来的数据
/// @param one_where 解包步骤记录变量指针
static void one_byte_solve(u8 date,remote_solve_one_t* one_where)
{
    switch (one_where->step)
    {
    case 0:
    {
        // 当当前字节为帧头的时候
        if(date == REMOTE_HEAD)
        {
            one_where->date_p = 0;
            one_where->step ++;
        }
        else
        {
            one_where->step = 0;
            memset(one_where->date,0,20);
        }
    }break;
    
    case 1:
    {
        // 当当前步骤为id_cmd的时候
        one_where->date[one_where->date_p] = date;  // 0
        one_where->date_p ++;
        one_where->step ++;
    }break;

    case 2:
    {
        one_where->date[one_where->date_p] = date;  // 1
        one_where->date_p ++;
        one_where->step ++;
    }break;

    case 3:
    {
        if(date == REMOTE_HEAD)
        {
            // 如果捕获到数据帧帧头的话
            one_where->step ++;
            one_where->date[one_where->date_p] = date;
            one_where->date_p ++;
        }
        else
        {
            one_where->step = 0;
            memset(one_where->date,0,20);
        }
    }break;

    case 4:
    {
        if(date != REMOTE_WEI1)
        {
            // 如果没有捕获第一个帧尾
            if(one_where->date_p < 19)
            {
                // 判断有没有超出一帧的范围 没有超出
                one_where->date[one_where->date_p] = date;
                one_where->date_p ++;
            }
            else
            {
                // 如果超出了
                one_where->date_p = 0;
                memset(one_where->date,0,20);
                one_where->step = 0;
            }
        }
        else
        {
            // 如果捕获到了第一个帧尾
            one_where->step ++;
        }
    }break;

    case 5:
    {
        if(date == REMOTE_WEI2)
        {
            // 如果再次捕获到帧尾2的话就确定该帧已经结束
            u16 ID = 0x00;
            memcpy(&ID,one_where->date,2);
            //判断是不是 11 位数据长度
            if(one_where->date_p == 10)
            {
                solve_gym_msg(ID,one_where->date+2);
            }
            one_where->step = 0;
            one_where->date_p = 0;
            memset(one_where->date,0,20);
        }
        else
        {
            // 如果第二次不是帧尾2的话就认为刚才那一帧是数据  补充上一个数据并记录本次数据并把步骤向前推
            one_where->date[one_where->date_p ++] = REMOTE_WEI1;
            one_where->date[one_where->date_p ++] = date;
            one_where->step --;
        }
    }break;

    default:
    {
        one_where->step = 0;
    }break;
    }
}






Remote_angle_t* get_angle_p(void)
{
    return &remote_angle;
}


Remote_speed_t* get_speed_set_p(void)
{
    return &Remote_speed;
}
