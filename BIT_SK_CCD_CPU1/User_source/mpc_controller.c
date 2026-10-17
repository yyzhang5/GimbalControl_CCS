/*
 * mpc_controller.c
 *
 *  Created on: 2025年12月13日
 *      Author: 54113
 */

#include "mpc_controller.h"
#include "AGlobalVariables.h"
#include <stdlib.h>

float ud_pre = 0;
float uq_pre = 0;
// 清理资源
//void MPC_Cleanup(MPC_State *state) {
//    if (state->id_hist) free(state->id_hist);
//    if (state->iq_hist) free(state->iq_hist);
//    if (state->w_hist) free(state->w_hist);
//    if (state->ud_hist) free(state->ud_hist);
//    if (state->uq_hist) free(state->uq_hist);
//}

// 更新历史数据
static void update_history(MPC_STATE_PM *state, float id, float iq, float speed, float ud, float uq) {
    // 循环缓冲区
    state->id_hist[state->hist_idx] = id;
    state->iq_hist[state->hist_idx] = iq;
    state->w_hist[state->hist_idx] = speed;
    state->ud_hist[state->hist_idx] = ud;
    state->uq_hist[state->hist_idx] = uq;

    state->hist_idx = (state->hist_idx + 1) % state->hist_size;
}

// 扰动估计函数
static float estimate_F(const float *y_hist, const float *x_hist,
                       float alpha, float Ts, int nF,
                       int hist_size, int hist_idx) {
    float sum_val = 0.0f;
    float gain = -3.0f / (nF * nF * nF * Ts);

    // 计算扰动估计
    int m;
    for (m = 2; m <= nF + 1; m++) {
        // 获取索引（考虑循环缓冲区）
        int idx_prev = (hist_idx - (nF + 2 - m) + hist_size) % hist_size;
        int idx_curr = (hist_idx - (nF + 1 - m) + hist_size) % hist_size;

//        float y_prev = y_hist[idx_prev];
//        float x_prev = x_hist[idx_prev];
//        float y_curr = y_hist[idx_curr];
//        float x_curr = x_hist[idx_curr];

        // 第一项: [n_F - 2(m-1)] i_d(m-1)
        float term1 = (nF - 2*(m-1)) * y_hist[idx_prev];
        // 第二项: α_i (m-1) T_s [n_F - (m-1)] u_d(m-1)
        float term2 = alpha * (m-1) * Ts * (nF - (m-1)) * x_hist[idx_prev];
        // 第三项: (n_F - 2m) i_d(m)
        float term3 = (nF - 2*m) * y_hist[idx_curr];
        // 第四项: α_i m T_s (n_F - m) u_d(m)
        float term4 = alpha * m * Ts * (nF - m) * x_hist[idx_curr];

        sum_val += term1 + term2 + term3 + term4;
    }

    return gain*sum_val;
}

// 构建预测矩阵
static void prediction_matrices(float psi[15][3], float theta[15][10], float Gamma[15],
                               float A[3][3], float B[3][2], float C[3],
                               int Np, int n_s, int n_c) {
    int i,j,k;
    // 初始化矩阵为0
    for (i = 0; i < n_s * Np; i++) {
        for (j = 0; j < n_s; j++) {
            psi[i][j] = 0.0f;
        }
        for (j = 0; j < n_c * Np; j++) {
            theta[i][j] = 0.0f;
        }
        Gamma[i] = 0.0f;
    }

    // 计算A的幂次矩阵
    float A_power[6][3][3];  // 假设Np最大为5(5+1=6代表A的0次方到10次方)

    // A^0 = I
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 3; j++) {
            A_power[0][i][j] = (i == j) ? 1.0f : 0.0f;//A的0次方——单位阵
        }
    }
    // 计算A^1到A^(Np-1)
    for (k = 1; k < Np+1; k++) {
        // A^k = A^(k-1) * A
        A_power[k][0][0] = 1;
        A_power[k][1][1] = 1;
        A_power[k][2][2] = 1;
        A_power[k][2][1] = k * A[2][1];
    }

    // 构建psi矩阵
    int row, col;
    for (i = 0; i < Np; i++) {
        // psi(i) = A^(i+1)
        for (row = 0; row < n_s; row++) {
            for (col = 0; col < n_s; col++) {
                psi[i*n_s + row][col] = A_power[i+1][row][col];
            }
        }
    }

    // 构建theta矩阵
    for (i = 0; i < Np; i++) {
        for (j = 0; j <= i; j++) {
            // theta(i,j) = A^(i-j) * B
            int power = i - j;
            theta[i*n_s + 0][j*n_c + 0] = B[0][0];
            theta[i*n_s + 1][j*n_c + 1] = B[0][0];
            theta[i*n_s + 2][j*n_c + 1] = power*A[2][1]*B[0][0];
        }
    }

    // 构建Gamma矩阵
    Gamma[0] = C[0];
    Gamma[1] = C[1];
    Gamma[2] = C[2];
    for (i = 1; i < Np; i++) {
        // Σ A^j * C，C=[Ts*Fd, Ts*Fq, Ts*Fw]
        Gamma[i*3 + 0] = Gamma[(i-1)*3 + 0] + C[0];
        Gamma[i*3 + 1] = Gamma[(i-1)*3 + 1] + C[1];
        Gamma[i*3 + 2] = Gamma[(i-1)*3 + 2] + i*A[2][1]*C[1] + C[2];
    }
}

void calculate_H(float theta[15][10], float Q[15], float H[10][10], float R) {
    // 1. 计算中间矩阵 M = Q * theta（15×10），利用U的下三角特性化简
    float M[15][10] = {0};  // 15×10中间矩阵
    int i, j, k;
    for (i = 0; i < 15; i++) {  // M的行（对应Q的行）
        for (k = 0; k < 10; k++) {  // M的列（对应U的列）
            M[i][k] = Q[i] * theta[k][i];
        }
    }
    // 2.计算 H = U^T * M（10×15 × 15×10 = 10×10），继续化简
    for (i = 0; i < 10; i++) {  // H的行
        for (j = 0; j < 10; j++) {  // H的列
            // U^T的第i行 = U的第i列，仅行0~i有值（对应U的行0~i）
            for (k = 0; k <= i; k++) {  // 仅遍历U^T的非零行（k≤i）
                H[i][j] += 2*theta[k][i] * M[k][j];
            }
            // 以上在计算theta'*Q*theta，下面计算H，即前者加上R
            if (i==j) H[i][j] += 2*R;
        }
    }
}

void calculate_f(float E[15], float Q[15], float theta[15][10], float f[10]) {
    // 1. 计算中间向量 M = Q^T*E（15×1），利用Q对角特性化简
    float M[15] = {0};
    int i, k;
    for (i = 0; i < 15; i++) {
        M[i] = Q[i] * E[i]; // 仅对角线乘法，无循环
    }
    // 2. 计算 f = 2*theta^T * M（10×1），利用theta下三角化简
    for (i = 0; i < 10; i++) { // f的第i行（theta^T的第i列）
        // theta是下三角：第k行仅列0~k有值 → theta^T第i列仅行0~i有值
        for (k = 0; k <= i; k++) {     // 仅遍历非零行k≤i
            f[i] += theta[k][i] * M[k];    // theta^T[i][k] = theta[k][i]
        }
        f[i] *= 2.0f; // 乘以2，完成最终计算
    }
}
float study_gain = 0.001;
static float U_history[10] = {0};
void Hf_solve(float U_history[10], float H[10][10], float f[10], MPC_STATE_PM *state) {
    float grad[10] = {0};  // 梯度缓存
    int i, j;

    // 2. 投影梯度迭代求解
    //    for (iter = 0; iter < 5; iter++) {
    // 计算梯度：grad = H*U + f
    for (i = 0; i < 10; i++) {
        grad[i] = f[i];
        for (j = 0; j < 10; j++) {
            grad[i] += H[i][j] * U_history[j];
        }
        U_history[i] -= study_gain * grad[i];
    }

    state -> ud = U_history[0];
    state -> uq = U_history[1];
}

// 主控制函数
float gaint = 1.0;
void MPC_DirectSpeedController(MPC_STATE_PM *state, MPC_CONTROLLER_PM *params) {
    // 更新历史数据（使用上一次的控制输出）
    update_history(state, params->id, params->iq, params->wfdb, state->ud, state->uq);

    // 估计扰动
    float Fd_est;
    Fd_est = gaint*estimate_F(state->id_hist, state->ud_hist,
                             params->alpha_i, params->Ts, params->nF,
                             state->hist_size, state->hist_idx);
    float Fq_est;
    Fq_est = gaint*estimate_F(state->iq_hist, state->uq_hist,
                             params->alpha_i, params->Ts, params->nF,
                             state->hist_size, state->hist_idx);
    float Fw_est;
    Fw_est = gaint*estimate_F(state->w_hist, state->iq_hist,
                             params->alpha_w, params->Ts, params->nF,
                             state->hist_size, state->hist_idx);

    float e;
    e = params->wref - params->wfdb;
    // 计算iq_ref（速度环PI + 前馈）
    // 原MATLAB: iq_ref = 0.0003*speed + 1.7*TL + 0.68*e + 120*e_integral
    state->iq_ref = params->speed_gain * params->wfdb + params->TL_gain * params->TL + params->e_gain * e;

    if (abs(e)<0.05){
        // 积分项
        state->e_integral += params->Ts * e;
        // 积分限幅
        if (state->e_integral > params->e_integral_max) {
            state->e_integral = params->e_integral_max;
        } else if (state->e_integral < -params->e_integral_max) {
            state->e_integral = -params->e_integral_max;
        }

        // 添加积分项
        state->iq_ref += params->e_inte_gain * state->e_integral;
    }
    // 构建线性化模型，直接硬编码
    float A[3][3];
    A[0][0] = 1.0f; A[1][1] = 1.0f; A[2][2] = 1.0f;
    A[2][1] = params->Ts * params->alpha_w;

    float B[3][2];
    B[0][0] = params->Ts * params->alpha_i;
    B[1][1] = B[0][0];

    float C[3];
    C[0] = params->Ts * Fd_est;
    C[1] = params->Ts * Fq_est,
    C[2] = params->Ts * Fw_est;

    // 构建预测矩阵（假设Np=10, n_s=3, n_c=2）
    float psi[15][3],theta[15][10],Gamma[15];     // 30x1
    prediction_matrices(psi, theta, Gamma, A, B, C,//参数固定可固定
                       params->Np, params->n_s, params->n_c);

    // 构建状态向量
    float xk[3];
    xk[0] = params->id;
    xk[1] = params->iq;
    xk[2] = params->wfdb;

    // 构建参考轨迹
    float Rf[15];
    int i, j;
    for (i = 0; i < params->Np; i++) {
        Rf[3*i] = 0.0f;               // id_ref = 0 (MTPA)
        Rf[3*i + 1] = state->iq_ref;   // iq_ref
        Rf[3*i + 2] = params->wref;       // speed_ref
    }

    // 计算预测误差 E = psi*xk - Rf + Gamma
    float E[15];
    for (i = 0; i < 15; i++) {
        E[i] = -Rf[i] + Gamma[i];
        for (j = 0; j < 3; j++) {
            E[i] += psi[i][j] * xk[j];
        }
    }

// ---------- 原MATLAB: Uk=inv(H)*(theta'*Q*(-E)- R*inv(theta)*E); ---------- //
    float weight[15] = {0};
    float H[10][10] = {{0}};
    float f[10] = {0};

    for (i = 0; i < 15; i++) {
        weight[i] = (i % 3 == 0) ? params->Q_id :
                    (i % 3 == 1) ? params->Q_iq : params->Q_speed;
    }
    // 5. 计算 H = theta' * Q * theta + R
    calculate_H(theta, weight, H, params->R_weight);
    // 6. 计算 f = (2*E'*Q*theta)'
    calculate_f(E, weight, theta, f);

    // 7. 求解ud和uq
    Hf_solve(U_history, H, f, state);

    // 限幅
    if (state->ud > params->umax) state->ud = params->umax;
    if (state->ud < -params->umax) state->ud = -params->umax;
    if (state->uq > params->umax) state->uq = params->umax;
    if (state->uq < -params->umax) state->uq = -params->umax;
    // 已经在函数开头更新了控制输出历史（用于下一次扰动估计）

}
