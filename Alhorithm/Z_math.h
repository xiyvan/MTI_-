#ifndef _Z_MATH_H_
#define _Z_MATH_H_


//******************************************** 低通滤波  ******************************///

typedef __packed struct
{
    float input;                //输入数据
    float out;                  //输出数据
    float num[1];               //滤波参数
    float frame_period;         //滤波时间间隔   /s 单位秒
} first_order_filter_type_t;

// 低通滤波
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);

///******************************************  阶跃转斜坡  ****************************///

typedef struct
{
    float set;                      // 设定值
    float out;                      // 输出
    float step;                     // 步长
}step_slope_msg_t;

// 阶跃变斜坡
float FZ_math_StepToSlope_cale(step_slope_msg_t* date,float set,float step);



///***************************************** 卡尔曼滤波  ***********************************///

// 定义卡尔曼滤波器结构体
typedef struct 
{
    float x;   // 状态估计
    float P;   // 估计协方差
    float Q;   // 过程噪声协方差
    float R;   // 测量噪声协方差
    float K;   // 卡尔曼增益
} KalmanFilter;

// 卡尔曼更新
void KalmanFilter_Update(KalmanFilter *kf, float measurement);
// 卡尔曼初始化
void KalmanFilter_Init(KalmanFilter *kf, float Q, float R, float initial_estimate, float initial_error_covariance);


//********************************************  均值滤波  **************************************///


typedef struct 
{
    unsigned short input;       // 输入
    unsigned short temp[3];     // 中间保存三次
    unsigned short out;         // 输出

}MeanFilt_msg_t;
unsigned short SZ_math_MeanFilt(MeanFilt_msg_t* date);


///******************************************* 其他算法  ***********************************///



// 浮点数字取绝对值 1.输入数据
float FZ_math_absolute(float x);

// 浮点限制函数 1.输入数据 2.最大值 3.最小值
float FZ_math_limt(float x,float max,float min);

//双向补偿函数 1.正补偿系数  2.负补偿系数
float FZ_math_bidire_compen(float o_con,float n_con,float input);

// 死区限制
float FZ_math_deadzone_limt(float dead,float input,float mid);

#endif
