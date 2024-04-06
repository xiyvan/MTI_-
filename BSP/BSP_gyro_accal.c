/******************************************
    2023/9/7                              
    作者：RM
    移植：韩昂轩
    1.0  成功移植并试验 能正确显示芯片信息加速度角速度等数据    23.9.7

    bmi088芯片驱动包                  
*******************************************
*/



#include "BSP_gyro_accal.h"
#include "BSP_spi.h"
#include "stm32f4xx.h"
#include "BSP_Delay.h"
#include "BMI088driver.h"
#include "BMI088reg.h"

static u8 BMI088_read_write_byte(u8 reg);
static void BMI088_write_single_reg(u8 reg,u8 data);
volatile static void BMI088_ACCEL_NS_L(void);
volatile static void BMI088_ACCEL_NS_H(void);
volatile static void BMI088_GYRO_NS_L(void);
volatile static void BMI088_GYRO_NS_H(void);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);

#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }




static void BMI088_accel_read_muli_reg(u8 reg,u8 *buf,u8 len);
static void BMI088_gyro_read_muli_reg(u8 reg,u8 *buf,u8 len);
static u8 bmi088_accel_init(void);
u8 bmi088_gyro_init(void);



static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};
u8 buf[8] = {0};


void BMI088_read(float* gyro,float* accel,float* temperate)
{
    
    s16 bmi088_raw_temp;

    SPI_ReadWriteByte(SPI1,(BMI088_ACCEL_XOUT_L | 0x80));
    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L,buf,6);                 /*读取加速度计数据*/
    bmi088_raw_temp = (s16)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_3G_SEN;
    bmi088_raw_temp = (s16)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_3G_SEN;
    bmi088_raw_temp = (s16)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_3G_SEN;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID,buf,8);
    if(buf[0] == 0x0f)
    {
        bmi088_raw_temp = (s16)((buf[3]) << 8) | buf[2];
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_2000_SEN;
        bmi088_raw_temp = (s16)((buf[5]) << 8) | buf[4];
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_2000_SEN;
        bmi088_raw_temp = (s16)((buf[7]) << 8) | buf[6];
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_2000_SEN;
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }
    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}





/// @brief 总陀螺仪初始化函数
/// @return 
u8 BMI088_init(void)
{
    u8 error = 0;
    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();
    return error;
}



/// @brief 加速度计初始化
/// @return 
static u8 bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}


u8 bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}



/// @brief 读取加速度计的数据
/// @param reg 地址
/// @param buf 缓存数组
/// @param len 长度
static void BMI088_accel_read_muli_reg(u8 reg,u8 *buf,u8 len)
{
    BMI088_ACCEL_NS_L();
    SPI_ReadWriteByte(SPI1,(reg | 0x80));
    SPI_ReadWriteByte(SPI1,(reg | 0x80));
    for(int i = 0;i < 6;i++)
    {
        *buf = SPI_ReadWriteByte(SPI1,0x55);
        buf ++;
    }
    BMI088_ACCEL_NS_H();
}


/// @brief 读取陀螺仪的数据
/// @param reg 地址
/// @param buf 缓存数组
/// @param len 长度
static void BMI088_gyro_read_muli_reg(u8 reg,u8 *buf,u8 len)
{
    BMI088_GYRO_NS_L();
    SPI_ReadWriteByte(SPI1,(reg | 0x80));
    for(int i = 0;i < 8;i++)
    {
        *buf = SPI_ReadWriteByte(SPI1,0x55);
        buf ++;
    }
    BMI088_GYRO_NS_H();
}




volatile static void BMI088_ACCEL_NS_L(void)
{
    GPIO_ResetBits(GPIOA,GPIO_Pin_4);
}
volatile static void BMI088_ACCEL_NS_H(void)
{
    GPIO_SetBits(GPIOA,GPIO_Pin_4);
}


volatile static void BMI088_GYRO_NS_L(void)
{
    GPIO_ResetBits(GPIOB,GPIO_Pin_0);
}
volatile static void BMI088_GYRO_NS_H(void)
{
    GPIO_SetBits(GPIOB,GPIO_Pin_0);
}


static u8 BMI088_read_write_byte(u8 reg)
{
    return SPI_ReadWriteByte(SPI1,reg);
}


static void BMI088_write_single_reg(u8 reg,u8 data)
{
    SPI_ReadWriteByte(SPI1,reg);
    SPI_ReadWriteByte(SPI1,data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    BMI088_read_write_byte(reg | 0x80);
    *return_data = BMI088_read_write_byte(0x55);
}
