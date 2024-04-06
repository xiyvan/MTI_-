
#ifndef _CHASSIS_SOLVE_H_
#define _CHASSIS_SOLVE_H_




/*  lqr 修正参数 脚*/
#define CHASSIS_SOLVE_LQR_AMEND_MATRIX_THETA    0.5f
#define CHASSIS_SOLVE_LQR_AMEND_MATRIX_THETA1   0.5f
#define CHASSIS_SOLVE_LQR_AMEND_MATRIX_X        -0.0f
#define CHASSIS_SOLVE_LQR_AMEND_MATRIX_X1       -0.2f
#define CHASSIS_SOLVE_LQR_AMEND_MATRIX_PHI      0.5f
#define CHASSIS_SOLVE_LQR_AMEND_MATRIX_PHI1     1.0f




/*  1.0f  2.2f  腿*/
#define CHASSIS_SOLVE_LQR_AMEND_MATRIX_THETA_L      1.0f
#define CHASSIS_SOLVE_LQR_AMEND_MATRIX_THETA1_L     1.0f
#define CHASSIS_SOLVE_LQR_AMEND_MATRIX_X_L          -0.0f
#define CHASSIS_SOLVE_LQR_AMEND_MATRIX_X1_L         -0.5f
#define CHASSIS_SOLVE_LQR_AMEND_MATRIX_PHI_L        0.1f
#define CHASSIS_SOLVE_LQR_AMEND_MATRIX_PHI1_L       0.3f



typedef struct 
{
    float theta;
    float theta1;
    float x;
    float x1;
    float phi;
    float phi1;

    float lqr_k[2][6];          //保存lqr的参数
    float lqr_amend[2][6];      // lqr修正参数
    float T;                    //保存lqr解算出来的轮子电机的力矩
    float Tp;                   //保存lqr结算出来的沿着腿切向的力
}CHASSIS_leg_lqr;


#endif

