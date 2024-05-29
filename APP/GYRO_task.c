/******************************************
    2023/10/19                              
    作者：韩昂轩
    
    1.0  添加了陀螺仪信息接收并解码的功能   23.10.19
    1.0.1 删除了接收信息的功能添加了板载IMU的接收函数并添加温控  23.10.25
    1.0.2 移植了官方的ahrs姿态算法          24.4.4

    陀螺仪信息解算任务                  
*******************************************
*/


#include "GYRO_task.h"
#include "BSP_gyro_accal.h"
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BSP_gyro_accal.h"
#include "BSP_pwm.h"
#include "Z_math.h"
#include "AHRS.h"
#include "typedef_main.h"
#include "FreeRTOS.h"
#include "task.h"

///*********************************************************  函数声明  ******************************************************///

static void gyro_init(void);
static void imu_cali_slove(float gyro[3], float accel[3], float mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

///*********************************************************  变量定义   *****************************************************///

float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

float INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad
float last_INS_angle[3] = {0.0f, 0.0f, 0.0f};   // 记录上一次的角度
s32 ins_qvan[3] = {0};                          // 记录转过的圈数

PID_type_def gyro_temp_PID;
bmi088_real_data_t bmi088_real_data;
ist8310_real_data_t ist8310_real_data;


float gyro_offset[3];
float accel_offset[3];
float mag_offset[3];
static float accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};


static float INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static float INS_accel[3] = {0.0f, 0.0f, 0.0f};
static float INS_mag[3] = {0.0f, 0.0f, 0.0f};
static float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};

static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \

#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \

volatile u8 gyro_update_flag = 0;
volatile u8 accel_update_flag = 0;
volatile u8 accel_temp_update_flag = 0;
volatile u8 mag_update_flag = 0;
volatile u8 imu_start_dma_flag = 0;

float mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
float gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
float accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};

static const float timing_time = 0.001f;   //任务运行的时间 单位 s

///********************************************************* endl **********************************************************///


void GYRO_task(void *pvParameters)
{
    while (BMI088_init())
    {
        vTaskDelay(100);
    }
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);
    AHRS_init(INS_quat, INS_accel, INS_mag);

    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];

    gyro_init();
    while (1)
    {
        BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
        imu_cali_slove(INS_gyro,INS_accel,INS_mag,&bmi088_real_data,&ist8310_real_data);

///===================================================== 低通滤波  ================================================================///
        accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];
///===================================================== endl  ==========================================================================///
        AHRS_update(INS_quat,timing_time,INS_gyro,accel_fliter_3,INS_mag);
        get_angle(INS_quat,INS_angle,INS_angle+1,INS_angle+2);
        INS_angle[0] += 3.1415926f;INS_angle[1] += 3.1415926f;INS_angle[2] += 3.1415926f;      // 把角度都拉到正数
///******************************************************* 记录圈数  *******************************************************************///
        if((INS_angle[0] - last_INS_angle[0]) < -6.28f)
        {
            ins_qvan[0] ++;
        }
        else if((INS_angle[0] - last_INS_angle[0]) > 6.28f)
        {
            ins_qvan[0] --;
        }

        if((INS_angle[1] - last_INS_angle[1]) < -6.28f)
        {
            ins_qvan[1] --;
        }
        else if((INS_angle[1] - last_INS_angle[1]) > 6.28f)
        {
            ins_qvan[1] ++;
        }

        if((INS_angle[2] - last_INS_angle[2]) < -6.28f)
        {
            ins_qvan[2] --;
        }
        else if((INS_angle[2] - last_INS_angle[2]) > 6.28f)
        {
            ins_qvan[2] ++;
        }

///***************************************************** endl  **********************************************************************///
        last_INS_angle[0] = INS_angle[0];last_INS_angle[1] = INS_angle[1];last_INS_angle[2] = INS_angle[2];
        TIM_SetCompare1(TIM10,FZ_math_limt(PID_cale(&gyro_temp_PID,40,0),1900.0f,0.0f));           // 温控
        vTaskDelay(1);
    }
}





/// @brief 初始化陀螺仪
static void gyro_init(void)
{
    float pid_core[3] = {GYRO_TEMP_PID_KP,GYRO_TEMP_PID_KI,GYRO_TEMP_PID_KD};
    PID_Init(&gyro_temp_PID,pid_core,GYRO_TEMP_PID_MAXOUT,GYRO_TEMP_PID_MAXIOUT);
}



/**
  * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[out]     mag: 加上零漂和旋转
  * @param[in]      bmi088: 陀螺仪和加速度计数据
  * @param[in]      ist8310: 磁力计数据
  * @retval         none
  */
static void imu_cali_slove(float gyro[3], float accel[3], float mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
        accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
        mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
    }
}







