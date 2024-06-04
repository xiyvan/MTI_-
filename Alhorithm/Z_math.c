/******************************************
    2023/9/10                              
    作者：韩昂轩（Hang Angxvan）
    
    1.0  添加了浮点型数据的绝对值计算函数    23.9.10
    1.0.1 添加了浮点型数据的限制函数        23.10.19
    1.0.2 添加了双向补偿函数                23.12.15
    1.0.3 添加了阶跃函数转斜坡函数          24.4.23
    1.0.4 添加了卡尔曼滤波                  24.6.3

    数学运算库
*******************************************
*/

#include "Z_math.h"


/// @brief float类型的绝对值计算函数
/// @param x float类型的数据
/// @return 返回的该数据绝对值
float FZ_math_absolute(float x)
{
    if(x == 0)
    {
        return 0;
    }
    if(x > 0)
    {
        return x;
    }
    else if (x < 0)
    {
        return -x;
    }
}


/// @brief float类型的限制函数
/// @param x float类型数据
/// @param max 最大值
/// @param min 最小值
/// @return 限制后的值
float FZ_math_limt(float x,float max,float min)
{
    if(x > max)
    {
        x = max;
    }
    else if(x < min)
    {
        x = min;
    }
    return x;
}



/// @brief 双向补偿函数
/// @param o_con 正向补偿
/// @param n_con 反向补偿
/// @param input 输入值
/// @return 补偿后的数值
float FZ_math_bidire_compen(float o_con,float n_con,float input)
{
    if(input > 0)
    {
        input = input + o_con;
    }
    else if(input < 0)
    {
        input = input + n_con;
    }
    return input;
}


///**********************************************************  低通滤波  **************************************************************///

/// @brief 低通滤波初始化
/// @param first_order_filter_type 低通滤波结构体变量指针
/// @param frame_period 滤波参数
/// @param num 滤波参数
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}


/// @brief 低通滤波计算
/// @param first_order_filter_type 低通滤波结构体变量
/// @param input 输入值
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
    first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}




///********************************************************** 阶跃转斜坡信号  *****************************************************************///




/// @brief 步进变斜坡
/// @param date 结构体指针
/// @param set 设定子
/// @param step 步长，每步变化值
/// @return 输出值
float FZ_math_StepToSlope_cale(step_slope_msg_t* date,float set,float step)
{
    date->set = set;
    date->step = step;
    if(set > 0)
    {
        date->out = date->out + date->step;
    }
    else
    {
        date->out = date->out - date->step;
    }
    if((set > 0) && (date->out >= date->set))
    {
        date->out = date->set;
    }
    else if((set < 0) && (date->out <= date->set))
    {
        date->out = date->set;
    }
    return date->out;
}


///******************************************************************************* 死区限制  *****************************************************///

/// @brief 死区限制
/// @param dead 死区范围
/// @param input 输入
/// @param mid 中值
/// @return 限制后的数值
float FZ_math_deadzone_limt(float dead,float input,float mid)
{
    if((input >= mid - dead) && (input < mid + dead))
    {
        return mid;
    }
    else
    {
        return input;
    }
}



///***************************************************************************** 卡尔曼滤波  ********************************************************///


// 初始化卡尔曼滤波器
void KalmanFilter_Init(KalmanFilter *kf, float Q, float R, float initial_estimate, float initial_error_covariance) {
    kf->Q = Q;
    kf->R = R;
    kf->x = initial_estimate;
    kf->P = initial_error_covariance;
}



// 卡尔曼滤波更新
void KalmanFilter_Update(KalmanFilter *kf, float measurement) {
    // 预测更新
    kf->P = kf->P + kf->Q;

    // 计算卡尔曼增益
    kf->K = kf->P / (kf->P + kf->R);

    // 更新状态估计
    kf->x = kf->x + kf->K * (measurement - kf->x);

    // 更新估计协方差
    kf->P = (1 - kf->K) * kf->P;
}



///****************************************************** 均值滤波  ********************************************************///

/// @brief 均值滤波
/// @param date 均值滤波数据结构体
unsigned short SZ_math_MeanFilt(MeanFilt_msg_t* date)
{
    date->temp[2] = date->temp[1];
    date->temp[1] = date->temp[0];
    date->temp[0] = date->input;

    date->out = (date->temp[2] + date->temp[1] + date->temp[0]) / 3;
    return date->out;
}


///****************************************************  endl  ************************************************************///

