/******************************************
    最新更新时间：2023/6/24                              
    作者：韩昂轩

    1.0  添加了spi1初始化函数，以及spi的接收发送函数。      23.6.24

    spi板级支持包   GPIOB                 
*******************************************
*/

#include "BSP_spi.h"




void SPI1_Init(void)
{
    GPIO_InitTypeDef gpio_define;
    SPI_InitTypeDef spi_define;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

    gpio_define.GPIO_Mode = GPIO_Mode_OUT;                           //复用模式
    gpio_define.GPIO_OType = GPIO_OType_PP;                         //推挽输出
    gpio_define.GPIO_PuPd = GPIO_PuPd_UP;                           //上拉
    gpio_define.GPIO_Speed = GPIO_Medium_Speed;                          //高速
    gpio_define.GPIO_Pin = GPIO_Pin_0;                                  //引脚号
    GPIO_Init(GPIOB,&gpio_define);
    gpio_define.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOA,&gpio_define);

    gpio_define.GPIO_Mode = GPIO_Mode_AF;                           //复用模式
    gpio_define.GPIO_OType = GPIO_OType_PP;                         //推挽输出
    gpio_define.GPIO_PuPd = GPIO_PuPd_UP;                           //上拉
    gpio_define.GPIO_Speed = GPIO_High_Speed;                          //高速
    gpio_define.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;                   //引脚号
    GPIO_Init(GPIOB,&gpio_define);

    gpio_define.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOA,&gpio_define);

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI1);           //引脚复用
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);

    spi_define.SPI_Direction = SPI_Direction_2Lines_FullDuplex;     //设置工作模式为全双工
    spi_define.SPI_Mode = SPI_Mode_Master;                          //设置为主机
    spi_define.SPI_DataSize = SPI_DataSize_8b;                      //数据长度为8位
    spi_define.SPI_CPOL = SPI_CPOL_High;                            //设置高电平为空闲电平
    spi_define.SPI_CPHA = SPI_CPHA_2Edge;                           //设置电平为第二个电平采集
    spi_define.SPI_NSS = SPI_NSS_Soft;                              //软件片选
    spi_define.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;   //分频系数
    spi_define.SPI_FirstBit = SPI_FirstBit_MSB;                     //数据从MSB开始
    spi_define.SPI_CRCPolynomial = 10;                               //CRC校验
    SPI_Init(SPI1,&spi_define);
    SPI_CalculateCRC(SPI1,DISABLE);
/*
    DMA_InitTypeDef SPI_DMA_t;

    SPI_DMA_t.DMA_Channel = DMA_Channel_3;                          //通道三
    SPI_DMA_t.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 //传输方向
    SPI_DMA_t.DMA_BufferSize = 20;                                  //缓冲大小
    SPI_DMA_t.DMA_PeripheralBaseAddr = (u32)(&SPI1.DR);             //外设地址
    SPI_DMA_t.DMA_Memory0BaseAddr = 0;                              //存储器地址
    SPI_DMA_t.DMA_PeripheralDataSize = 8;                           //外设数据位宽
    SPI_DMA_t.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        //外设地址增量
    SPI_DMA_t.DMA_MemoryInc = DMA_MemoryInc_Disable;                //存储器地址增量
    SPI_DMA_t.DMA_MemoryDataSize = 8;                               //存储器位宽
    SPI_DMA_t.DMA_Mode = DMA_Mode_Normal;                           //dma模式
    SPI_DMA_t.DMA_Priority = DMA_Priority_Medium;                   //dma优先级
    SPI_DMA_t.DMA_FIFOMode = DMA_FIFOMode_Disable;                  //dmafifo
    SPI_DMA_t.DMA_MemoryBurst =DMA_MemoryBurst_Single;              
    SPI_DMA_t.DMA_PeripheralBurst = DMA_MemoryBurst_Single;

    DMA_Init(DMA2_Stream2,&SPI_DMA_t);                              //dma初始化
    DMA_Cmd(DMA2_Stream2,ENABLE);
*/
    SPI_Cmd(SPI1,ENABLE);                                           //使能Spi
}



/// @brief SPI接收发送一个字节
/// @param SPIx SPI号
/// @param data 发送的数据
/// @return 接收到的数据
uint8_t SPI_ReadWriteByte(SPI_TypeDef* SPIx,uint8_t data)
{
    while(SPI_I2S_GetFlagStatus(SPIx,SPI_I2S_FLAG_TXE) == RESET);       //等待发送区为空
    SPI_I2S_SendData(SPIx,data);                                        //发送数据
    while(SPI_I2S_GetFlagStatus(SPIx,SPI_I2S_FLAG_RXNE) == RESET);      //等待接收完一个字节
    return SPI_I2S_ReceiveData(SPIx);                                   //返回接收到的数据
}





/// @brief SPI发送多个数据
/// @param SPIx SPI号
/// @param data 待发送数据数组
/// @param len 发送数据长度
void SPI_WriteData(SPI_TypeDef* SPIx,uint8_t* data,uint16_t len)
{
    for(int x = 0;x < len;x++)
    {
        SPI_ReadWriteByte(SPIx,data[x]);
    }
}
