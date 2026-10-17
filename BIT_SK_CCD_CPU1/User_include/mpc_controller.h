/*
 * mpc_controller.h
 *
 *  Created on: 2025年12月13日
 *      Author: 54113
 */

#ifndef USER_INCLUDE_MPC_CONTROLLER_H_
#define USER_INCLUDE_MPC_CONTROLLER_H_


#include <math.h>
#include <string.h>

extern float ud_pre;
extern float uq_pre;

// 控制器参数结构体
typedef struct {
//-----input-----//
    float wref;
    float wfdb;
    float id;
    float iq;
    float TL;

    float alpha_i;      // 电流环参数
    float alpha_w;      // 速度环参数
    float Ts;          // 采样时间
    int Np;            // 预测时域
    int n_s;           // 状态维度
    int n_c;           // 控制维度
    int nF;            // 观测窗口长度

    // 权重矩阵参数
    float Q_speed;     // 速度误差权重
    float Q_id;        // d轴电流误差权重
    float Q_iq;        // q轴电流误差权重
    float R_weight;    // 控制输出权重

    // 限幅
    float umax;        // 电压限幅

    // PI参数（用于iq_ref计算）
    float speed_gain;
    float e_inte_gain;
    float e_gain;
    float TL_gain;

    // 积分饱和限幅
    float e_integral_max;

    void (*calc)();
} MPC_CONTROLLER_PM;

typedef MPC_CONTROLLER_PM *MPC_CONTROLLER_PM_handle;

#define MPC_CONTROLLER_PM_DEFAULTS {                   \
                           0,                          \
                           0,                          \
                           0,                          \
                           0,                          \
                           0,                          \
                           63.7,                        \
                           471.4,                        \
                           1e-3,                       \
                           5,                         \
                           3,                          \
                           2,                          \
                           10,                         \
                           10,                        \
                           0.01,                        \
                           0.015,                        \
                           0.005,                       \
                           10,                        \
                           0.4,                      \
                           0.01,                        \
                           120,                       \
                           0,                        \
                           5,                         \
                           (void (*)(unsigned int))MPC_DirectSpeedController  \
                          }

// 控制器状态结构体
typedef struct {
    // 历史数据缓冲区
    float id_hist[6];//长度为 MPC_CONTROLLER_PM.nF+1
    float iq_hist[6];
    float w_hist[6];
    float ud_hist[6];
    float uq_hist[6];
    int hist_size;// MPC_CONTROLLER_PM.Np+1
    int hist_idx;

    // 积分项
    float e_integral;

    // 输出
    float ud;
    float uq;
    float iq_ref;
} MPC_STATE_PM;

typedef MPC_STATE_PM *M1_MPC_STATE_PM_handle;

#define MPC_STATE_PM_DEFAULTS {                   \
                           {0},                        \
                           {0},                        \
                           {0},                        \
                           {0},                        \
                           {0},                        \
                           6,                         \
                           0,                          \
                           0,                          \
                           0,                          \
                           0,                          \
                           0,                          \
                          }

// 函数声明
void MPC_DirectSpeedController(MPC_STATE_PM *state, MPC_CONTROLLER_PM *params);
void MPC_Cleanup(MPC_STATE_PM *state);

// 辅助函数
static void update_history(MPC_STATE_PM *state, float id, float iq, float speed, float ud, float uq);
static float estimate_F(const float *y_hist, const float *x_hist, float alpha, float Ts, int nF, int hist_size, int hist_idx);
static void linearized_matrix(float A[3][3], float B[3][2], float C[3],
                             float alpha_i, float alpha_w,
                             float F_d, float F_q, float F_w, float Ts);
static void prediction_matrices(float psi[15][3], float theta[15][10], float Gamma[15],
                               float A[3][3], float B[3][2], float C[3],
                               int Np, int n_s, int n_c);


#endif /* USER_INCLUDE_MPC_CONTROLLER_H_ */
