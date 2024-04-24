#ifndef _Z_MATH_H_
#define _Z_MATH_H_



typedef __packed struct
{
    float input;                //输入数据
    float out;                  //输出数据
    float num[1];               //滤波参数
    float frame_period;         //滤波时间间隔   /s 单位秒
} first_order_filter_type_t;



typedef struct
{
    float set;                      // 设定值
    float out;                      // 输出
    float step;                     // 步长
}step_slope_msg_t;







// 浮点数字取绝对值 1.输入数据
float FZ_math_absolute(float x);

// 浮点限制函数 1.输入数据 2.最大值 3.最小值
float FZ_math_limt(float x,float max,float min);

//双向补偿函数 1.正补偿系数  2.负补偿系数
float FZ_math_bidire_compen(float o_con,float n_con,float input);

// 低通滤波
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);

// 阶跃变斜坡
float FZ_math_StepToSlope_cale(step_slope_msg_t* date,float set,float step);


#endif
