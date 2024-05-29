/*********************************************
 * @author 韩昂轩（Han Angxvan）
 * 创建时间：24.4.30
 * 
 *      掉线检测任务
**********************************************/



#include "CK_Timeout_task.h"
#include "FreeRTOS.h"
#include "task.h"


//******************************************* 函数声明  **********************************************///

static void ck_timeout_init(CK_Timeout_struct_t* time);
static void ck_time_check(CK_Timeout_struct_t* time);
static void ck_time_new(CK_Timeout_struct_t* time);


///***************************************** 变量定义  **************************************************///


CK_Timeout_struct_t CK_Timeout_main;

/// 用来保存各个外设掉线判定时间（一定要按顺序写，而且不能超过外设数量）
#if (CAR_TYPE == CAR_TYPE_Caster_wheel)
u16 timeout_time[TIMEOUT_ALL] = {5,10,10,10,10,10,10,10,10};
#else
u16 timeout_time[TIMEOUT_ALL] = {5,10,10,10,10};
#endif
///**************************************** endl *******************************************************///


void CK_Timeout_task(void *pvParameters)
{
    ck_timeout_init(&CK_Timeout_main);
    vTaskDelay(100);                    // 等待其他外设初始化完毕 防止误检
    while(1)
    {
        ck_time_new(&CK_Timeout_main);
        ck_time_check(&CK_Timeout_main);
        vTaskDelay(1);
    }
}





/// @brief 初始化把所有时间置零
/// @param time 掉线检测任务结构体指针
static void ck_timeout_init(CK_Timeout_struct_t* time)
{
    for(u8 i = 0;i < TIMEOUT_ALL;i++)
    {
        time->driver_time[i] = 0;
    }
    time->all_time = 0;
}




/// @brief 总体时间更新
/// @param time 掉线检测任务结构体指针
static void ck_time_new(CK_Timeout_struct_t* time)
{
    time->all_time += 1;

    if(time->all_time >= 4294967200)
    {
        time->all_time = 0;
    }
}




/// @brief 检测掉线情况并记录掉线编号，最后保存最大的编号
/// @param time 掉线检测任务结构体
static void ck_time_check(CK_Timeout_struct_t* time)
{
    u8 state = 0;
    for(u8 i = 0;i < TIMEOUT_ALL;i++)
    { 
        if(time->all_time - time->driver_time[i] >= timeout_time[i])
        {
            time->TimeOut_num = i;  // 记录掉线外设编号（会记录最大的一个）
            state = 1;              // 标记有外设掉线
        }
    }

    if(state == 0)
    {
        time->TimeOut_num = 0xff;   // num为0xff的时候为没有掉线的
    }
}




/// @brief 外设时间更新
/// @param num 外设编号
void CkTime_DriverTimeNew(u8 num)
{
    CK_Timeout_main.driver_time[num] = CK_Timeout_main.all_time;
}



/// @brief 获取掉线设备编号
/// @return 掉线设备编号  255为没有掉线设备 否则就为掉线设备编号
u8 Get_TimeOutNum(void)
{
    return CK_Timeout_main.TimeOut_num;
}
