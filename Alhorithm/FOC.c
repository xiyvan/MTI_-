/******************************************
    2023/6/16                              
    作者：韩昂轩                       
    
    1.0  添加了有感FOC算法（未添加sin cos 函数库）  23.6.16

    FOC算法包             
*******************************************
*/


#include "FOC.h"


static float ecd_change_hd(int i);
static float limt_F(int rqt);




/// @brief 初始化FOC结构体
/// @param foc_init_t foc变量指针
/// @param p 极对数
/// @param the_ta 点击角度指针
void FOC_Init(FOC_type_def* foc_init_t,int p,int* the_ta)
{
    if(foc_init_t == 0)
    {
        return;
    }

    foc_init_t->p = p;
    foc_init_t->the_ta = the_ta;
}







/// @brief FOC计算
/// @param foc_calc foc结构体变量指针 
/// @param set_iq 设定力矩
void FOC_calc(FOC_type_def* foc_calc,int set_iq)
{
    if(foc_calc == 0)
    {
        return;
    }

    foc_calc->iq = set_iq;                                              //设置力矩
    foc_calc->the_ta_t = limt_F((*foc_calc->the_ta) * foc_calc->p);        //计算电角度  机械角度 * 极对数  

    foc_calc->i_alph = -foc_calc->iq * sin(foc_calc->the_ta_t);  //计算 i阿尔法
    foc_calc->i_bei_ta = foc_calc->iq * cos(foc_calc->the_ta_t);

    foc_calc->ia = foc_calc->i_alph;                                      //计算a相电流
    foc_calc->ib = (foc_calc->i_bei_ta * 1.73205 - foc_calc->i_alph)/2;   //计算B相电流
    foc_calc->ic = (-foc_calc->i_alph - 1.73205 * foc_calc->i_bei_ta)/2;  //计算C相电流
}







float sin(float x)
{

    return x;
}

float cos(float x)
{
    return x;
}




/// @brief 角度限制函数
/// @param rqt 输入电机编码器值
/// @param mode 模式选择 0为角度模式 1为弧度模式 
/// @return 限制后的数值
static float limt_F(int rqt)
{
    float x = 0.0;
    int y = 0;
    (float)rqt = ecd_change_hd(rqt);
    rqt *= 100000;
    y = (int)rqt;
    x = y % (360 * 100000);
    x /= 100000.0;
    return x;
}





/// @brief 电机编码器值转换为角度值
/// @param i 电机编码器值
/// @return 转换后的角度值
static float ecd_change_hd(int i)
{
    float x = 0.000;
    x = i/encoder_max * 360;
    return x;
}
