#include "IDDK_PM_Servo_F2837x-Settings.h"  // hinf.h/hinf_mix.h在这里面被调用
#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "sci_base.h"
#include "emif.h"
#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#include "F2837xD_sysctrl.h"
#include "F28x_Project.h"
#include "AGlobalVariables.h"
#include "F2837x_QEP_module.h"
#include "f2837xbmsk.h"
#include "F2837xD_adc.h"
#include "IQmathLib.h"
#include "F2837xD_eqep.h"

// 目标丢失后搜索用（case 1） 无目的搜索
// M1 x轴用
#define SCAN_MAX_M1 200.0f      // 最大角度
#define SCAN_MIN_M1 120.0f      // 最小角度
#define SCAN_STEP_ANGLE1_M1 4.5f // 扫描步进角度_快
#define SCAN_STEP_ANGLE2_M1 2.0f
#define SCAN_DIR_CCW 1       // counterclockwise 逆时针
#define SCAN_DIR_CW -1     // clockwise 顺时针
int scan_dir_M1 = SCAN_DIR_CCW;
float scan_target_pos_M1 = 160.0f;
// M2 y轴用
#define SCAN_MAX_M2 295.0f      // 最大角度
#define SCAN_MIN_M2 215.0f      // 最小角度
#define SCAN_STEP_ANGLE1_M2 4.5f // 扫描步进角度_快
#define SCAN_STEP_ANGLE2_M2 2.0f
#define SCAN_DIR_UP -1       //
#define SCAN_DIR_DOWN 1    //
int scan_dir_M2 = SCAN_DIR_UP;
float scan_target_pos_M2 = 255.0f;

float M1gainB_kp = 1.7;
float M1gainB_ki = 1.3;
float M1gainS_kp = 1.5;
float M1gainS_ki = 1.15;

//float M1gainR_kp = 1.5;
//float M1gainR_ki = 1.15;
//float M1gainL_kp = 1.5;
//float M1gainL_ki = 1.2;

float M2gainB_kp = 1.45;
float M2gainB_ki = 1.2;
float M2gainS_kp = 1.32;
float M2gainS_ki = 1.1;
// 目标丢失后搜索用（case 2） 螺旋搜索
// 螺旋核心参数
#define STEP_ANGLE      0.0001f     // 螺旋角速度
#define STEP_R          0.0001f    // 半径变化速度
#define MAX_RADIUS      40.0f    // 最大螺旋半径
#define x_pos_center 160.0f
#define y_pos_center 255.0f
// X 轴独立状态（Pan）
int x_flag = 0;
float target_x = 0.0;
static float radius_x = 0.0;
static float angle_x = 0.0f;
static char dir_x = 1;        // 1：向外扩  -1：向内收
// Y 轴独立状态（Tilt）
int y_flag = 0;
float target_y = 0.0;
static float radius_y = 0.0;
static float angle_y = 0.0f;
static char dir_y = 1;


void M1_Loop_Current(float IdRef, float IqRef)
{
    M1_PI_Iq.Ref = IqRef;
    M1_PI_Id.Ref = IdRef;

    M1_PI_Iq.Kp = M1Param.PID_Cur_Iq.Kp*0.1;
    M1_PI_Iq.Ki = M1Param.PID_Cur_Iq.Ki*0.1;
    M1_PI_Id.Kp = M1Param.PID_Cur_Id.Kp*0.1;
    M1_PI_Id.Ki = M1Param.PID_Cur_Id.Ki*0.1;

    M1_PI_Id.Fbk = park1.Ds;
    M1_PI_Iq.Fbk = park1.Qs;
    ipark1.Ds = M1_PI_Id.Out;    // 闭环运行，D Q轴的值来自PI运算
    ipark1.Qs = M1_PI_Iq.Out;
    //M1Ref.Vq_Ref = 0.0;

    PI_MACRO(M1_PI_Iq);
    PI_MACRO(M1_PI_Id);
}

void M1_Loop_Speed(float SpdRef)
{
    M1_PID_Spd.Ref = SpdRef;
    M1_PID_Spd.Fdb = -M1Data.Speed;                                 // Mac_speed_Filter

//      if ( Data1_C2TC1_R[0] > 70){
//        M1_PID_Spd.Kp  = M1Param.PID_Spd.Kp*0.01 * M1gainR_kp;//SciA_kp_speed,;
//        M1_PID_Spd.Ki  = M1Param.PID_Spd.Ki*0.0001 * M1gainR_ki;//SciA_ki_speed,;
//        M1_PID_Spd.Kd  = M1Param.PID_Spd.Kd;//SciA_kd_speed,;
//    }
//    else if ( Data1_C2TC1_R[0] < 58){
//        M1_PID_Spd.Kp  = M1Param.PID_Spd.Kp*0.01 * M1gainL_kp;//SciA_kp_speed,;
//        M1_PID_Spd.Ki  = M1Param.PID_Spd.Ki*0.0001 * M1gainL_ki;//SciA_ki_speed,;
//        M1_PID_Spd.Kd  = M1Param.PID_Spd.Kd;//SciA_kd_speed,;
//    }


    if (Data1_C2TC1_R[0] > 74  || Data1_C2TC1_R[0] < 54){
        M1_PID_Spd.Kp  = M1Param.PID_Spd.Kp*0.01 * M1gainB_kp;//SciA_kp_speed,;
        M1_PID_Spd.Ki  = M1Param.PID_Spd.Ki*0.0001 * M1gainB_ki;//SciA_ki_speed,;
        M1_PID_Spd.Kd  = M1Param.PID_Spd.Kd;//SciA_kd_speed,;
    }
    else if (54 <= Data1_C2TC1_R[0] <58  || 70 < Data1_C2TC1_R[0] <= 74) {
        M1_PID_Spd.Kp  = M1Param.PID_Spd.Kp*0.01 * M1gainS_kp;//SciA_kp_speed,;
        M1_PID_Spd.Ki  = M1Param.PID_Spd.Ki*0.0001 * M1gainS_ki;//SciA_ki_speed,;
        M1_PID_Spd.Kd  = M1Param.PID_Spd.Kd;//SciA_kd_speed,;
    }
    else
    {
        M1_PID_Spd.Kp  = M1Param.PID_Spd.Kp*0.01;//SciA_kp_speed,;
        M1_PID_Spd.Ki  = M1Param.PID_Spd.Ki*0.0001;//SciA_ki_speed,;
        M1_PID_Spd.Kd  = M1Param.PID_Spd.Kd;//SciA_kd_speed,;
    }

    // 不分段
//    M1_PID_Spd.Kp  = M1Param.PID_Spd.Kp*0.01;//SciA_kp_speed,;
//    M1_PID_Spd.Ki  = M1Param.PID_Spd.Ki*0.0001;//SciA_ki_speed,;
//    M1_PID_Spd.Kd  = M1Param.PID_Spd.Kd;//SciA_kd_speed,;
    M1_PID_Spd.calc(&M1_PID_Spd);

    ipark1.Qs = M1_PID_Spd.Out;
//    ipark1.Qs = 0.2;
    ipark1.Ds = 0;
//    M1_Loop_Current(M1Data.Id_Ref, M1_PID_Spd.Out);

//    M1_RMP_SPEED_Ctr.TargetValue = M1_PID_Spd.Out;
//    RC_SPEED_MACRO(M1_RMP_SPEED_Ctr)
//
//    M1_Loop_Current(M1Data.Id_Ref, M1_RMP_SPEED_Ctr.SetpointValue);
}

void M1_Loop_MPC(float SpdRef)
{
    M1_MPC_Spd_Crr.wref = SpdRef;
    M1_MPC_Spd_Crr.wfdb = M1Data.Speed;   // Mac_speed_Filter
    M1_MPC_Spd_Crr.id = park1.Ds;
    M1_MPC_Spd_Crr.iq = park1.Qs;
    M1_MPC_Spd_Crr.TL = 0;//之后这里可以替换成ESO观测扰动

    M1_MPC_Spd_Crr.alpha_i = M1Param.MPC_Spd_Crr.alpha_i;
    M1_MPC_Spd_Crr.alpha_w = M1Param.MPC_Spd_Crr.alpha_w;
    M1_MPC_Spd_Crr.Ts = M1Param.MPC_Spd_Crr.Ts;
    M1_MPC_Spd_Crr.Np = M1Param.MPC_Spd_Crr.Np;
    M1_MPC_Spd_Crr.n_s = M1Param.MPC_Spd_Crr.n_s;
    M1_MPC_Spd_Crr.n_c = M1Param.MPC_Spd_Crr.n_c;
    M1_MPC_Spd_Crr.nF = M1Param.MPC_Spd_Crr.nF;
    M1_MPC_Spd_Crr.Q_speed = M1Param.MPC_Spd_Crr.Q_speed;
    M1_MPC_Spd_Crr.Q_id = M1Param.MPC_Spd_Crr.Q_id;
    M1_MPC_Spd_Crr.Q_iq = M1Param.MPC_Spd_Crr.Q_iq;
    M1_MPC_Spd_Crr.R_weight = M1Param.MPC_Spd_Crr.R_weight;
    M1_MPC_Spd_Crr.umax = M1Param.MPC_Spd_Crr.umax;
    M1_MPC_Spd_Crr.speed_gain = M1Param.MPC_Spd_Crr.speed_gain;
    M1_MPC_Spd_Crr.e_inte_gain = M1Param.MPC_Spd_Crr.e_inte_gain;
    M1_MPC_Spd_Crr.e_gain = M1Param.MPC_Spd_Crr.e_gain;
    M1_MPC_Spd_Crr.TL_gain = M1Param.MPC_Spd_Crr.TL_gain;
    M1_MPC_Spd_Crr.e_integral_max = M1Param.MPC_Spd_Crr.e_integral_max;

    MPC_DirectSpeedController(&M1_MPC_State, &M1_MPC_Spd_Crr);

    ipark1.Ds = M1_MPC_State.ud;    // 闭环运行，D Q轴的值来自PI运算
    ipark1.Qs = M1_MPC_State.uq;
}
void M2_Loop_MPC(float SpdRef)
{
    M2_MPC_Spd_Crr.wref = SpdRef;
    M2_MPC_Spd_Crr.wfdb = M2Data.Speed;   // Mac_speed_Filter
    M2_MPC_Spd_Crr.id = park2.Ds;
    M2_MPC_Spd_Crr.iq = park2.Qs;
    M2_MPC_Spd_Crr.TL = 0;//之后这里可以替换成ESO观测扰动

    M2_MPC_Spd_Crr.alpha_i = M2Param.MPC_Spd_Crr.alpha_i;
    M1_MPC_Spd_Crr.alpha_w = M2Param.MPC_Spd_Crr.alpha_w;
    M2_MPC_Spd_Crr.Q_speed = M2Param.MPC_Spd_Crr.Q_speed;
    M2_MPC_Spd_Crr.Q_id = M2Param.MPC_Spd_Crr.Q_id;
    M2_MPC_Spd_Crr.Q_iq = M2Param.MPC_Spd_Crr.Q_iq;
    M2_MPC_Spd_Crr.R_weight = M2Param.MPC_Spd_Crr.R_weight;
    M2_MPC_Spd_Crr.speed_gain = M2Param.MPC_Spd_Crr.speed_gain;
    M2_MPC_Spd_Crr.e_inte_gain = M2Param.MPC_Spd_Crr.e_inte_gain;
    M2_MPC_Spd_Crr.e_gain = M2Param.MPC_Spd_Crr.e_gain;
    M2_MPC_Spd_Crr.TL_gain = M2Param.MPC_Spd_Crr.TL_gain;

    MPC_DirectSpeedController(&M2_MPC_State, &M2_MPC_Spd_Crr);

    ipark2.Ds = M2_MPC_State.ud;//0;    // 闭环运行，D Q轴的值来自PI运算
    ipark2.Qs = M2_MPC_State.uq;
//    ipark2.Ds = 0;    // 闭环运行，D Q轴的值来自PI运算
//    ipark2.Qs = 0;//.4;
}

void M1_Loop_Position(float PosRef, float M1_Pos_Fdb, int Include_speed_loop)
{
    M1Param.PID_Pos.Ki = 0.0;       // 位置环不用积分
    M1_PID_Pos.Ref = PosRef;        //位置给定参考值
    M1_PID_Pos.Kp = M1Param.PID_Pos.Kp*0.1;
    M1_PID_Pos.Ki = M1Param.PID_Pos.Ki;
    M1_PID_Pos.Kd = M1Param.PID_Pos.Kd;

    M1_PID_Pos.Fdb = M1_Pos_Fdb;  //实际位置反馈值 posEncMechTheta[POSITION_ENCODER]
    M1_PID_Pos.calc(&M1_PID_Pos);          //位置闭环运算

    if(Include_speed_loop){//位置环
        M1Ref.Speed_Refdms = 0.0;
        M1_Loop_Speed(M1_PID_Pos.Out);}
    else{////位置+电流环
        ipark1.Qs = M1_PID_Pos.Out;
        ipark1.Ds = 0;
    }
}

float M1_Loop_Theta(int num1)
{
    double fx;
    double fx_CCD = 2501.965;
    fx =  fx_CCD;

    if((Data1_C2TC1_R[0] < 3)||(Data1_C2TC1_R[0] > 125)) {
        switch (num1){
        case 1:
            // 丢失目标后搜索
            if (M1_PID_Pos.Fdb >= SCAN_MAX_M1)
            {
                scan_dir_M1 = SCAN_DIR_CW; // 到上限，反向扫描
                // DELAY_US(80);
            }
            else if (M1_PID_Pos.Fdb <= SCAN_MIN_M1)
            {
                scan_dir_M1 = SCAN_DIR_CCW;  // 到下限，正向扫描
                // DELAY_US(80);
            }
            // 2. 更新扫描目标位置（平滑步进）
            if ((M1_PID_Pos.Fdb<=SCAN_MAX_M1-10.0)&&(M1_PID_Pos.Fdb>=SCAN_MIN_M1+10.0)){
                scan_target_pos_M1 = M1_PID_Pos.Fdb + scan_dir_M1 * SCAN_STEP_ANGLE1_M1;
            }
            else{
                scan_target_pos_M1 = M1_PID_Pos.Fdb + scan_dir_M1 * SCAN_STEP_ANGLE2_M1;
            }

            M1_PID_Pos.Ref = scan_target_pos_M1;
            break;
        case 2:
            target_x = M1_PID_Pos.Fdb; // 实时坐标，角度
            if (x_flag == 0){
                radius_x = fabs(target_x - x_pos_center);
                x_flag = 1;
            }
            // ============= 螺旋半径动态变化 =============
                radius_x += dir_x*STEP_R;
                if(radius_x >= MAX_RADIUS)
                    dir_x = -1;  // 触达最大半径 → 开始回收
                else if(radius_x <= 0.0)
                    dir_x = 1;
            // ============= 计算螺旋坐标 =============
            angle_x += STEP_ANGLE;
            if (angle_x >= 360.0) angle_x -= 360.0;
            target_x = x_pos_center + radius_x * cos(angle_x);
            // ============= 边界保护 =============
            if(target_x > 200) target_x = 200;
            if(target_x < 120) target_x = 120;
            M1_PID_Pos.Ref = target_x;
            break;
        default:
            // 丢失目标后停止不动
            M1_PID_Pos.Ref = M1_PID_Pos.Fdb;
        }
    }
    else {
        x_flag = 0;
        Err_x = 64 - Data1_C2TC1_R[0];//偏差计算 符号
        M1_PID_Pos.Ref =  M1_PID_Pos.Fdb + 57.2958 * atan(Err_x / fx);
        //Data1_C2TC1_R_x = Data1_C2TC1_R[2];
    }
    return M1_PID_Pos.Ref;
}

float M2_Loop_Theta(int num2)
{
    double fy;
    double fy_CCD = 2485.811;
    fy =  fy_CCD;

    if((Data1_C2TC1_R[1] < 3)||(Data1_C2TC1_R[1] > 125)) {
        switch (num2){
        case 1:
            // 丢失目标后搜索 无规则搜索
            if (M2_PID_Pos.Fdb >= SCAN_MAX_M2)
            {
                scan_dir_M2 = SCAN_DIR_UP; // 到“下”限（最大值），向上扫描
                // DELAY_US(80);
            }
            else if (M2_PID_Pos.Fdb <= SCAN_MIN_M2)
            {
                scan_dir_M2 = SCAN_DIR_DOWN;  // 到“上”限（最小值），向下扫描
                // DELAY_US(80);
            }
            // 2. 更新扫描目标位置（平滑步进）
            if ((M2_PID_Pos.Fdb<=SCAN_MAX_M2-10.0)&&(M2_PID_Pos.Fdb>=SCAN_MIN_M2+10.0)){
                scan_target_pos_M2 = M2_PID_Pos.Fdb + scan_dir_M2 * SCAN_STEP_ANGLE1_M2;
            }
            else{
                scan_target_pos_M2 = M2_PID_Pos.Fdb + scan_dir_M2 * SCAN_STEP_ANGLE2_M2;
            }

            M2_PID_Pos.Ref = scan_target_pos_M2;
            break;
        case 2:
            target_y = M2_PID_Pos.Fdb; // 实时坐标
            if (y_flag == 0){
                radius_y = fabs(target_y - y_pos_center);
                y_flag = 1;
            }
            // ============= 螺旋半径动态变化 =============
            radius_y += dir_y*STEP_R;
            if(radius_y >= MAX_RADIUS)
                dir_y = -1;  // 触达最大半径 → 开始回收
            else if(radius_y <= 0.0)
                dir_y = 1;
            // ============= 计算螺旋坐标 =============
            angle_y += STEP_ANGLE;
            if (angle_y >= 360.0) angle_y -= 360.0;
            target_y = y_pos_center + radius_y * sin(angle_y);
            // ============= 边界保护 =============
            if(target_y > 295) target_y = 295;
            if(target_y < 215) target_y = 215;
            M2_PID_Pos.Ref = target_y;
            break;
        default:
            // 丢失目标后停止不动
            M2_PID_Pos.Ref = M2_PID_Pos.Fdb;
        }
    }
    else {
        y_flag = 0;
        Err_y = Data1_C2TC1_R[1] - 64;//偏差计算 符号
        M2_PID_Pos.Ref =  M2_PID_Pos.Fdb + 57.2958 * atan(Err_y / fy);
        //Data1_C2TC1_R_x = Data1_C2TC1_R[2];
    }
    return M2_PID_Pos.Ref;
}

void M1_Loop_Picture()
{
    M1_PID_Pic.Ref = 0;
    M1_PID_Pic.Kp = M1Param.PID_Pic.Kp*0.1;
    M1_PID_Pic.Ki = M1Param.PID_Pic.Ki*0.00001;
    M1_PID_Pic.Kd = M1Param.PID_Pic.Kd;

    M1_PID_Pic.calc(&M1_PID_Pic);          //位置闭环运算

    M1_Loop_Speed(M1_PID_Pic.Out);    // 第二个值1表示速度环+电流环，0表示仅速度环，无电流环
}


float M1_Angle_Update(float angle)
{
//    模拟的电角度
    angle += 0.042f*2.0f*PI/360.0f;   // 每秒转 20°

    if (angle >= 2.0f*PI)
    {
        angle = 0.0f;  // 超过360°则清零
    }
    return angle;
}

///--------------------- M2 三环定义 -------------------------
void M2_Loop_Current(float IdRef_M2, float IqRef_M2)
{
    M2_PI_Iq.Ref = IqRef_M2;
    M2_PI_Id.Ref = IdRef_M2;

    M2_PI_Iq.Kp = M2Param.PID_Cur_Iq.Kp;
    M2_PI_Iq.Ki = M2Param.PID_Cur_Iq.Ki;
    M2_PI_Id.Kp = M2Param.PID_Cur_Id.Kp;
    M2_PI_Id.Ki = M2Param.PID_Cur_Id.Ki;

    M2_PI_Id.Fbk = park2.Ds;
    M2_PI_Iq.Fbk = park2.Qs;
    ipark2.Ds = M2_PI_Id.Out;//0;//0;    // 闭环运行，D Q轴的值来自PI运算
    ipark2.Qs = M2_PI_Iq.Out;
//    M2Ref.Vq_Ref = 0.0;

    PI_MACRO(M2_PI_Iq);
    PI_MACRO(M2_PI_Id);
}

void M2_Loop_Speed(float SpdRef_M2)
{
    M2_PID_Spd.Ref = -SpdRef_M2;
    M2_PID_Spd.Fdb = -M2Data.Speed;  // Mac_speed_Filter

//    if (Data1_C2TC1_R[1] > 70){
//        M2_PID_Spd.Kp  = M2Param.PID_Spd.Kp*0.01 * M2gainR_kp;//SciA_kp_speed,;
//        M2_PID_Spd.Ki  = M2Param.PID_Spd.Ki*0.0001 * M2gainR_ki;//SciA_ki_speed,;
//        M2_PID_Spd.Kd  = M2Param.PID_Spd.Kd;//SciA_kd_speed,;
//    }
//    else if (Data1_C2TC1_R[1] < 58){
//        M2_PID_Spd.Kp  = M2Param.PID_Spd.Kp*0.01 * M2gainL_kp;//SciA_kp_speed,;
//        M2_PID_Spd.Ki  = M2Param.PID_Spd.Ki*0.0001 * M2gainL_ki;//SciA_ki_speed,;
//        M2_PID_Spd.Kd  = M2Param.PID_Spd.Kd;//SciA_kd_speed,;
//    }



    if (Data1_C2TC1_R[1] > 74  || Data1_C2TC1_R[1] < 54){
        M2_PID_Spd.Kp  = M2Param.PID_Spd.Kp*0.01 * M2gainB_kp;//SciA_kp_speed,;
        M2_PID_Spd.Ki  = M2Param.PID_Spd.Ki*0.0001 * M2gainB_ki;//SciA_ki_speed,;
        M2_PID_Spd.Kd  = M2Param.PID_Spd.Kd;//SciA_kd_speed,;
    }
    else if (54 <= Data1_C2TC1_R[1] <58  || 70 < Data1_C2TC1_R[1] <= 74){
        M2_PID_Spd.Kp  = M2Param.PID_Spd.Kp*0.01 * M2gainS_kp;//SciA_kp_speed,;
        M2_PID_Spd.Ki  = M2Param.PID_Spd.Ki*0.0001 * M2gainS_ki;//SciA_ki_speed,;
        M2_PID_Spd.Kd  = M2Param.PID_Spd.Kd;//SciA_kd_speed,;
    }

    else
    {
        M2_PID_Spd.Kp  = M2Param.PID_Spd.Kp*0.01;//SciA_kp_speed,;
        M2_PID_Spd.Ki  = M2Param.PID_Spd.Ki*0.0001;//SciA_ki_speed,;
        M2_PID_Spd.Kd  = M2Param.PID_Spd.Kd;//SciA_kd_speed,;
    }

//    M2_PID_Spd.Kp  = M2Param.PID_Spd.Kp*0.01;//SciA_kp_speed;
//    M2_PID_Spd.Ki  = M2Param.PID_Spd.Ki*0.0001;//SciA_ki_speed;
//    M2_PID_Spd.Kd  = M2Param.PID_Spd.Kd;//SciA_ki_speed;
    M2_PID_Spd.calc(&M2_PID_Spd);

//    M2_PID_Spd.Out=0.1;
    ipark2.Qs = M2_PID_Spd.Out;
//    ipark1.Qs = 0.2;
    ipark2.Ds = 0;
//    if (Data1_C2TC1_R[1] == 0) ipark2.Qs = 0.0;

//    ipark2.Qs = M2_PID_Spd.Out;
}

void M2_Loop_Position(float PosRef, float M2_Pos_Fdb, int Include_speed_loop)
{
    M2Param.PID_Pos.Ki = 0.0;           // 位置环不用积分，因为积分需要相对较长时间，而位置环频率低，积分项会限制其性能
    M2_PID_Pos.Ref = PosRef;         //位置给定参考值
    M2_PID_Pos.Kp = M2Param.PID_Pos.Kp*0.1;
    M2_PID_Pos.Ki = M2Param.PID_Pos.Ki;
    M2_PID_Pos.Kd = M2Param.PID_Pos.Kd;

    M2_PID_Pos.Fdb = M2_Pos_Fdb;  //实际位置反馈值 posEncMechTheta[POSITION_ENCODER]
    M2_PID_Pos.calc(&M2_PID_Pos);          //位置闭环运算

    if(Include_speed_loop){//位置环+电流环
            M2Ref.Speed_Refdms = 0.0;
            M2_Loop_Speed(M2_PID_Pos.Out);}
        else{////位置
            ipark2.Qs = -M2_PID_Pos.Out;
            ipark2.Ds = 0;
        }
}


void M2_Loop_Picture()
{
    M2_PID_Pic.Ref = 0;
    M2_PID_Pic.Kp = M2Param.PID_Pic.Kp*0.1;
    M2_PID_Pic.Ki = M2Param.PID_Pic.Ki*0.00001;
    M2_PID_Pic.Kd = M2Param.PID_Pic.Kd;

    M2_PID_Pic.calc(&M2_PID_Pic);          //闭环运算

    M2_Loop_Speed(M2_PID_Pic.Out);    // 第二个值1表示速度环+电流环，0表示仅速度环，无电流环
}

///------------ 速度获取 M1 ---------------
float M1_Temp_spd_dms = 0.0;
float M1_Temp_spd = 0.0;
float M1_Pos_MechTheta_pre = 0.0;
float M1_Temp_Spd_out_dms = 0.0;
//float M1_BIT_Get_Speed(float M1_Pos_MechTheta, float M1_Time_interval)
//{
//    if ((M1_Pos_MechTheta-M1_Pos_MechTheta_pre)<330)
//    {
//        M1_Pos_MechTheta_pre = M1_Pos_MechTheta_pre-360;
//    }
//    if ((M1_Pos_MechTheta-M1_Pos_MechTheta_pre)>330)
//    {
//        M1_Pos_MechTheta_pre = M1_Pos_MechTheta_pre+360;
//    }
//
//    M1_Temp_spd_dms = (M1_Pos_MechTheta-M1_Pos_MechTheta_pre) / M1_Time_interval; // 速度只有在“当前-前一时刻”则速度和位置环才能好用
//    M1_Temp_Spd_out_dms = M1_Speed_Filter(M1_Temp_spd_dms);
//
//    M1_Pos_MechTheta_pre = M1_Pos_MechTheta;
//
//    return M1_Temp_Spd_out_dms;
//}

float M1_BIT_Get_Speed(float M1_Pos_MechTheta, float M1_Time_interval)
{
    if ((M1_Pos_MechTheta-M1_Pos_MechTheta_pre)< 330)
    {
        M1_Pos_MechTheta_pre = M1_Pos_MechTheta_pre-360;
    }
    if ((M1_Pos_MechTheta-M1_Pos_MechTheta_pre)> 330)
    {
        M1_Pos_MechTheta_pre = M1_Pos_MechTheta_pre+360;
    }

    M1_Temp_spd = M1_Pos_MechTheta-M1_Pos_MechTheta_pre;
    M1_Temp_spd_dms = M1_Temp_spd / M1_Time_interval; // 速度只有在“当前-前一时刻”则速度和位置环才能好用
    M1_Temp_Spd_out_dms = M1_Speed_Filter(M1_Temp_spd_dms);

    M1_Pos_MechTheta_pre = M1_Pos_MechTheta;

    return M1_Temp_Spd_out_dms;
}
///------------ 速度获取 M2 ---------------
float M2_Temp_spd_dms = 0.0;
float M2_Pos_MechTheta_pre = 0.0;
float M2_Temp_Spd_out_dms = 0.0;
float M2_BIT_Get_Speed(float M2_Pos_MechTheta, float M2_Time_interval)
{
    if ((M2_Pos_MechTheta-M2_Pos_MechTheta_pre)<330)
    {
        M2_Pos_MechTheta_pre = M2_Pos_MechTheta_pre-360;
    }
    if ((M2_Pos_MechTheta-M2_Pos_MechTheta_pre)>330)
    {
        M2_Pos_MechTheta_pre = M2_Pos_MechTheta_pre+360;
    }

    M2_Temp_spd_dms = (M2_Pos_MechTheta-M2_Pos_MechTheta_pre) / M2_Time_interval;
    M2_Temp_Spd_out_dms = M2_Speed_Filter(M2_Temp_spd_dms);

    M2_Pos_MechTheta_pre = M2_Pos_MechTheta;

    return M2_Temp_Spd_out_dms;
}

//------------------------ M1速度获取后滤波处理 -------------------------------
float M1_Spd_Flt_out[3]={0};
float M1_Spd_Flt_in[3]={0};
float Para_M1[3] = {0};      // 0是增益，1是输出前一个值，2是输出前2个值
float M1_Speed_Filter(float M1_Temp_spd_dms)
{
//    Para_M1[1] = 1.8226949, Para_M1[2] = -0.83718165, Para_M1+[0] = 0.00362168;     // 20Hz
//    Para_M1[1] = 1.7786318, Para_M1[2] = -0.8008026,  Para_M1[0] = 0.00554271;     // 25Hz
    Para_M1[1] = 1.5610181, Para_M1[2] = -0.64135154, Para_M1[0] = 0.02008337;     // 50Hz
//      Para_M1[1] = 1.4754804, Para_M1[2] = -0.58691950, Para_M1[0] = 0.05571953;     // 60Hz
//    Para_M1[1] = 1.1429805, Para_M1[2] = -0.41280160, Para_M1[0] = 0.06745527;     // 100Hz

    M1_Spd_Flt_in[0]  = M1_Temp_spd_dms;
    M1_Spd_Flt_out[0] = Para_M1[1] * M1_Spd_Flt_out[1] + Para_M1[2]*M1_Spd_Flt_out[2]+Para_M1[0]*M1_Spd_Flt_in[2]+2*Para_M1[0]*M1_Spd_Flt_in[1]+Para_M1[0]*M1_Spd_Flt_in[0];//20HZ
    M1_Spd_Flt_out[2] = M1_Spd_Flt_out[1];
    M1_Spd_Flt_out[1] = M1_Spd_Flt_out[0];
    M1_Spd_Flt_in[2]  = M1_Spd_Flt_in[1];
    M1_Spd_Flt_in[1]  = M1_Spd_Flt_in[0];

    return  M1_Spd_Flt_out[0];
}

//------------------------ M2速度获取后滤波处理 -------------------------------
float M2_Spd_Flt_out[3]={0};
float M2_Spd_Flt_in[3]={0};
float Para_M2[3] = {0};      // 0是增益，1是输出前一个值，2是输出前2个值
float M2_Speed_Filter(float M2_Temp_spd_dms)
{
//    Para_M2[1] = 1.8226949, Para_M2[2] = -0.83718165, Para_M2[0] = 0.00362168;     // 20Hz
//    Para_M2[1] = 1.7786318, Para_M2[2] = -0.8008026,  Para_M2[0] = 0.00554271;     // 25Hz
//    Para_M2[1] = 1.5610181, Para_M2[2] = -0.64135154, Para_M2[0] = 0.02008337;     // 50Hz
//      Para_M2[1] = 1.4754804, Para_M2[2] = -0.58691950, Para_M2[0] = 0.05571953;     // 60Hz
    Para_M2[1] = 1.1429805, Para_M2[2] = -0.41280160, Para_M2[0] = 0.06745527;     // 100Hz

    M2_Spd_Flt_in[0]  = M2_Temp_spd_dms;
    M2_Spd_Flt_out[0] = Para_M2[1] * M2_Spd_Flt_out[1] + Para_M2[2]*M2_Spd_Flt_out[2]+Para_M2[0]*M2_Spd_Flt_in[2]+2*Para_M2[0]*M2_Spd_Flt_in[1]+Para_M2[0]*M2_Spd_Flt_in[0];//20HZ
    M2_Spd_Flt_out[2] = M2_Spd_Flt_out[1];
    M2_Spd_Flt_out[1] = M2_Spd_Flt_out[0];
    M2_Spd_Flt_in[2]  = M2_Spd_Flt_in[1];
    M2_Spd_Flt_in[1]  = M2_Spd_Flt_in[0];

    return  M2_Spd_Flt_out[0];
}


// ****************************************************************************
//TODO Motor Control ISR
// ****************************************************************************
void closeBridge(void)
{
EALLOW;
    EPwm1Regs.AQCSFRC.bit.CSFA=1;
    EPwm1Regs.AQCSFRC.bit.CSFB=1;
    EPwm2Regs.AQCSFRC.bit.CSFA=1;
    EPwm2Regs.AQCSFRC.bit.CSFB=1;
    EPwm3Regs.AQCSFRC.bit.CSFA=1;
    EPwm3Regs.AQCSFRC.bit.CSFB=1;
EDIS;
}

//#################################################
//电机停止时，三个上管关闭，三个下管开启
//-----------------------------------------------
void openLowBridge(void)
{
EALLOW;
    EPwm1Regs.AQCSFRC.bit.CSFA=1;
    EPwm1Regs.AQCSFRC.bit.CSFB=2;
    EPwm2Regs.AQCSFRC.bit.CSFA=1;
    EPwm2Regs.AQCSFRC.bit.CSFB=2;
    EPwm3Regs.AQCSFRC.bit.CSFA=1;
    EPwm3Regs.AQCSFRC.bit.CSFB=2;
EDIS;
}


//#################################################
//CSFA  CSFB 寄存器设置0， 强制输出失效
//-----------------------------------------------
void setPWMFree(void)
{
    EALLOW;
        EPwm1Regs.AQCSFRC.bit.CSFA=0;
        EPwm1Regs.AQCSFRC.bit.CSFB=0;
        EPwm2Regs.AQCSFRC.bit.CSFA=0;
        EPwm2Regs.AQCSFRC.bit.CSFB=0;
        EPwm3Regs.AQCSFRC.bit.CSFA=0;
        EPwm3Regs.AQCSFRC.bit.CSFB=0;
    EDIS;
}

// ****************************************************************************
// ****************************************************************************
//TODO  DMC Protection Against Over Current Protection
// ****************************************************************************
// ****************************************************************************



void cmpssConfig(volatile struct CMPSS_REGS *v, int16 Hi, int16 Lo)
{
	// Set up COMPCTL register
	v->COMPCTL.bit.COMPDACE    = 1;             // Enable CMPSS
	v->COMPCTL.bit.COMPLSOURCE = NEGIN_DAC;     // NEG signal from DAC for COMP-L
	v->COMPCTL.bit.COMPHSOURCE = NEGIN_DAC;     // NEG signal from DAC for COMP-H
	v->COMPCTL.bit.COMPHINV    = 0;             // COMP-H output is NOT inverted
	v->COMPCTL.bit.COMPLINV    = 1;             // COMP-L output is inverted
	v->COMPCTL.bit.ASYNCHEN    = 0;             // Disable aynch COMP-H ouput
	v->COMPCTL.bit.ASYNCLEN    = 0;             // Disable aynch COMP-L ouput
	v->COMPCTL.bit.CTRIPHSEL    = CTRIP_FILTER; // Dig filter output ==> CTRIPH
	v->COMPCTL.bit.CTRIPOUTHSEL = CTRIP_FILTER; // Dig filter output ==> CTRIPOUTH
	v->COMPCTL.bit.CTRIPLSEL    = CTRIP_FILTER; // Dig filter output ==> CTRIPL
	v->COMPCTL.bit.CTRIPOUTLSEL = CTRIP_FILTER; // Dig filter output ==> CTRIPOUTL

	// Set up COMPHYSCTL register
	v->COMPHYSCTL.bit.COMPHYS   = 2; // COMP hysteresis set to 2x typical value

	// set up COMPDACCTL register
	v->COMPDACCTL.bit.SELREF    = 0; // VDDA is REF for CMPSS DACs
	v->COMPDACCTL.bit.SWLOADSEL = 0; // DAC updated on sysclock
	v->COMPDACCTL.bit.DACSOURCE = 0; // Ramp bypassed

	// Load DACs - High and Low
	v->DACHVALS.bit.DACVAL = Hi;     // Set DAC-H to allowed MAX +ve current
	v->DACLVALS.bit.DACVAL = Lo;     // Set DAC-L to allowed MAX -ve current

	// digital filter settings - HIGH side
	v->CTRIPHFILCLKCTL.bit.CLKPRESCALE = clkPrescale; // set time between samples, max : 1023
	v->CTRIPHFILCTL.bit.SAMPWIN        = sampwin;     // # of samples in window, max : 31
	v->CTRIPHFILCTL.bit.THRESH         = thresh;      // recommended : thresh > sampwin/2
	v->CTRIPHFILCTL.bit.FILINIT        = 1;           // Init samples to filter input value

	// digital filter settings - LOW side
	v->CTRIPLFILCLKCTL.bit.CLKPRESCALE = clkPrescale; // set time between samples, max : 1023
	v->CTRIPLFILCTL.bit.SAMPWIN        = sampwin;     // # of samples in window, max : 31
	v->CTRIPLFILCTL.bit.THRESH         = thresh;      // recommended : thresh > sampwin/2
	v->CTRIPLFILCTL.bit.FILINIT        = 1;           // Init samples to filter input value

	// Clear the status register for latched comparator events
	v->COMPSTSCLR.bit.HLATCHCLR = 1;
	v->COMPSTSCLR.bit.LLATCHCLR = 1;

	return;
}


void HVDMC_Protection(void)
{
	EALLOW;

	// Configure GPIO used for Trip Mechanism

	//GPIO input for reading the status of the LEM-overcurrent macro block (active low), GPIO40
	//could trip PWM based on this, if desired
	// Configure as Input
//	GpioCtrlRegs.GPBPUD.bit.GPIO40  = 1; // disable pull ups
//	GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0; // choose GPIO for mux option
//	GpioCtrlRegs.GPBDIR.bit.GPIO40  = 0; // set as input
//	GpioCtrlRegs.GPBINV.bit.GPIO40  = 1; //invert the input such that '0' is good and '1' is bad after inversion
//
//	InputXbarRegs.INPUT2SELECT = 40;     //Select GPIO40 as INPUTXBAR2

	//GPIO99/TZ1  第一套电机
	GpioCtrlRegs.GPDPUD.bit.GPIO99  = 1; // disable pull ups
	GpioCtrlRegs.GPDMUX1.bit.GPIO99 = 0; // choose GPIO for mux option
	GpioCtrlRegs.GPDDIR.bit.GPIO99  = 0; // set as input
	GpioCtrlRegs.GPDINV.bit.GPIO99  = 0; //NO invert the input ,such that '1' is good and '0' is bad after inversion

	InputXbarRegs.INPUT1SELECT = 99;     //Select GPIO99 as INPUTXBAR2

//	//GPIO133/TZ1  第二套电机
//	GpioCtrlRegs.GPEPUD.bit.GPIO133  = 1; // disable pull ups
//	GpioCtrlRegs.GPEMUX1.bit.GPIO133 = 0; // choose GPIO for mux option
//	GpioCtrlRegs.GPEDIR.bit.GPIO133  = 0; // set as input
//	GpioCtrlRegs.GPEINV.bit.GPIO133  = 0; //NO invert the input ,such that '1' is good and '0' is bad after inversion
//
//	InputXbarRegs.INPUT2SELECT = 133;     //Select GPIO133 as INPUTXBAR2

	//Clearing the Fault(active low), GPIO41,
	// Configure as Output
//	GpioCtrlRegs.GPBPUD.bit.GPIO41  = 1; // disable pull ups
//	GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0; // choose GPIO for mux option
//	GpioCtrlRegs.GPBDIR.bit.GPIO41  = 1; // set as output
//	GpioDataRegs.GPBSET.bit.GPIO41  = 1;

	//Forcing IPM Shutdown (Trip) using GPIO58 (Active high)
	// Configure as Output
//	GpioCtrlRegs.GPBPUD.bit.GPIO58   = 1; // disable pull ups
//	GpioCtrlRegs.GPBMUX2.bit.GPIO58  = 0; // choose GPIO for mux option
//	GpioCtrlRegs.GPBDIR.bit.GPIO58   = 1; // set as output
//	GpioDataRegs.GPBCLEAR.bit.GPIO58 = 1;

	// LEM Current phase V(ADC A2, COMP1) and W(ADC B2, COMP3), High Low Compare event trips
//	LEM_curHi = 2048 + LEM(curLimit);
//	LEM_curLo = 2048 - LEM(curLimit);
//	cmpssConfig(&Cmpss1Regs, LEM_curHi, LEM_curLo);  //Enable CMPSS1 - LEM V
//	cmpssConfig(&Cmpss3Regs, LEM_curHi, LEM_curLo);  //Enable CMPSS3 - LEM W

#if (CNGD == HOT)
	// Shunt Current phase V(ADC A4, COMP2) and W(ADC C2, COMP6), High Low Compare event trips
	SHUNT_curHi = 2048 + SHUNT(curLimit);
	SHUNT_curLo = 2048 - SHUNT(curLimit);
	cmpssConfig(&Cmpss2Regs, SHUNT_curHi, SHUNT_curLo);  //Enable CMPSS2 - Shunt V
	cmpssConfig(&Cmpss6Regs, SHUNT_curHi, SHUNT_curLo);  //Enable CMPSS6 - Shunt U
#endif
/*
	// Configure TRIP 4 to OR the High and Low trips from both comparator 1 & 3
	// Clear everything first
	EPwmXbarRegs.TRIP4MUX0TO15CFG.all  = 0x0000;
	EPwmXbarRegs.TRIP4MUX16TO31CFG.all = 0x0000;
	// Enable Muxes for ored input of CMPSS1H and 1L, i.e. .1 mux for Mux0
	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX0  = 1;  //cmpss1 - tripH or tripL
	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX4  = 1;  //cmpss3 - tripH or tripL
	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX2  = 1;  //cmpss2 - tripH or tripL
	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX10 = 1;  //cmpss6 - tripH or tripL
	EPwmXbarRegs.TRIP4MUX0TO15CFG.bit.MUX3  = 1;  //inputxbar2 trip

	// Disable all the muxes first
	EPwmXbarRegs.TRIP4MUXENABLE.all = 0x0000;
	// Enable Mux 0  OR Mux 4 to generate TRIP4
	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX0  = 1;
	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX4  = 1;
	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX2  = 1;
	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX10 = 1;
	EPwmXbarRegs.TRIP4MUXENABLE.bit.MUX1  = 1;

	EPwm1Regs.DCTRIPSEL.bit.DCAHCOMPSEL = 3; //Trip 4 is the input to the DCAHCOMPSEL
	EPwm1Regs.TZDCSEL.bit.DCAEVT1       = TZ_DCAH_HI;
	EPwm1Regs.DCACTL.bit.EVT1SRCSEL     = DC_EVT1;
	EPwm1Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;
	EPwm1Regs.TZSEL.bit.DCAEVT1         = 1;           // 1/0 - Enable/Disable One Shot Mode

	EPwm2Regs.DCTRIPSEL.bit.DCAHCOMPSEL = 3; //Trip 4 is the input to the DCAHCOMPSEL
	EPwm2Regs.TZDCSEL.bit.DCAEVT1       = TZ_DCAH_HI;
	EPwm2Regs.DCACTL.bit.EVT1SRCSEL     = DC_EVT1;
	EPwm2Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;
	EPwm2Regs.TZSEL.bit.DCAEVT1         = 1;

	EPwm3Regs.DCTRIPSEL.bit.DCAHCOMPSEL = 3; //Trip 4 is the input to the DCAHCOMPSEL
	EPwm3Regs.TZDCSEL.bit.DCAEVT1       = TZ_DCAH_HI;
	EPwm3Regs.DCACTL.bit.EVT1SRCSEL     = DC_EVT1;
	EPwm3Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC;
	EPwm3Regs.TZSEL.bit.DCAEVT1         = 1;
*/

	EPwm1Regs.TZSEL.bit.CBC6 = 0x1; // Emulator Stop
	EPwm2Regs.TZSEL.bit.CBC6 = 0x1; // Emulator Stop
	EPwm3Regs.TZSEL.bit.CBC6 = 0x1; // Emulator Stop

    EPwm1Regs.TZSEL.bit.OSHT1   = 1;  //enable TZ1 for OSHT
    EPwm2Regs.TZSEL.bit.OSHT1   = 1;  //enable TZ1 for OSHT
    EPwm3Regs.TZSEL.bit.OSHT1   = 1;  //enable TZ1 for OSHT

	// What do we want the OST/CBC events to do?
	// TZA events can force EPWMxA
	// TZB events can force EPWMxB

	EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low
	EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low

	EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low
	EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low

	EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low
	EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low

	// Clear any spurious OV trip
//	EPwm1Regs.TZCLR.bit.DCAEVT1 = 1;
//	EPwm2Regs.TZCLR.bit.DCAEVT1 = 1;
//	EPwm3Regs.TZCLR.bit.DCAEVT1 = 1;

	EPwm1Regs.TZCLR.bit.OST = 1;
	EPwm2Regs.TZCLR.bit.OST = 1;
	EPwm3Regs.TZCLR.bit.OST = 1;

	EDIS;

//************************** End of Prot. Conf. ***************************//
}

// ****************************************************************************
// ****************************************************************************
//TODO PWM Configuration
// ****************************************************************************
// ****************************************************************************
void PWM_1ch_UpDwnCnt_CNF(int16 n, Uint16 period, int16 db) {
	EALLOW;
	// Time Base SubModule Registers
	(*ePWM[n]).TBCTL.bit.PRDLD = TB_IMMEDIATE; // set Immediate load
	(*ePWM[n]).TBPRD = period / 2; // 10KHz, PWM frequency = 1 / period
	(*ePWM[n]).TBPHS.bit.TBPHS = 0;
	(*ePWM[n]).TBCTR = 0;
	(*ePWM[n]).TBCTL.bit.CTRMODE   = TB_COUNT_UPDOWN;
	(*ePWM[n]).TBCTL.bit.HSPCLKDIV = TB_DIV1;
	(*ePWM[n]).TBCTL.bit.CLKDIV    = TB_DIV1;

	(*ePWM[n]).TBCTL.bit.PHSEN    = TB_DISABLE;
	(*ePWM[n]).TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync "down-stream"

	// Counter Compare Submodule Registers
	(*ePWM[n]).CMPA.bit.CMPA = 0; // set duty 0% initially
	(*ePWM[n]).CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	(*ePWM[n]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

	// Action Qualifier SubModule Registers
	(*ePWM[n]).AQCTLA.bit.CAU = AQ_CLEAR;
	(*ePWM[n]).AQCTLA.bit.CAD = AQ_SET;

	// Active high complementary PWMs - Set up the deadband
	(*ePWM[n]).DBCTL.bit.IN_MODE  = DBA_ALL;
	(*ePWM[n]).DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	(*ePWM[n]).DBCTL.bit.POLSEL   = DB_ACTV_HIC;
	(*ePWM[n]).DBRED = db;
	(*ePWM[n]).DBFED = db;
	EDIS;
}

void PWM_1ch_UpCnt_CNF(int16 n, Uint16 period) {
	EALLOW;
	// Time Base SubModule Registers
	(*ePWM[n]).TBCTL.bit.PRDLD = TB_IMMEDIATE; // set Immediate load
	(*ePWM[n]).TBPRD = period-1; // PWM frequency = 1 / period
	(*ePWM[n]).TBPHS.bit.TBPHS = 0;
	(*ePWM[n]).TBCTR = 0;
	(*ePWM[n]).TBCTL.bit.CTRMODE   = TB_COUNT_UP;
	(*ePWM[n]).TBCTL.bit.HSPCLKDIV = TB_DIV1;
	(*ePWM[n]).TBCTL.bit.CLKDIV    = TB_DIV1;

	(*ePWM[n]).TBCTL.bit.PHSEN    = TB_DISABLE;
	(*ePWM[n]).TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // sync "down-stream"

	// Counter Compare Submodule Registers
	(*ePWM[n]).CMPA.bit.CMPA        = 0; // set duty 0% initially
	(*ePWM[n]).CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	(*ePWM[n]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;

	// Action Qualifier SubModule Registers
	(*ePWM[n]).AQCTLA.bit.CAU = AQ_CLEAR;
	(*ePWM[n]).AQCTLA.bit.ZRO = AQ_SET;

	// Active high complementary PWMs - Set up the deadband
	(*ePWM[n]).DBCTL.bit.IN_MODE  = DBA_ALL;
	(*ePWM[n]).DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
	(*ePWM[n]).DBCTL.bit.POLSEL   = DB_ACTV_HIC;
	(*ePWM[n]).DBRED = 0;
	(*ePWM[n]).DBFED = 0;
	EDIS;
}

// ****************************************************************************
// ****************************************************************************
//TODO ADC Configuration
// ****************************************************************************
// ****************************************************************************
void ConfigureADC(void)
{
	//Write ADC configurations and power up the ADC for both ADC A
	Uint16 i;

	EALLOW;

// write configurations for ADC-A， External REFERENCE must be provided
	AdcaRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
	AdcaRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;		// AD采样是12位分辨率
	AdcaRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;		// 单端信号只能12bit，差分只能是16bit

	//Set pulse positions to late
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

	//power up the ADC
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	//write configurations for ADC-B
	// External REFERENCE must be provided
	AdcbRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
	AdcbRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
	AdcbRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;

	//Set pulse positions to late
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

	//power up the ADC
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

/// write configurations for ADC-C
//  External REFERENCE must be provided
	AdccRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
	AdccRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
	AdccRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;

	//Set pulse positions to late
	AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;

	//power up the ADC
	AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
//
//	//write configurations for ADC-D
	// External REFERENCE must be provided
	AdcdRegs.ADCCTL2.bit.PRESCALE   = 6; //set ADCCLK divider to /4
	AdcdRegs.ADCCTL2.bit.RESOLUTION = RESOLUTION_12BIT;
	AdcdRegs.ADCCTL2.bit.SIGNALMODE = SIGNAL_SINGLE;

	//Set pulse positions to late
	AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;

	//power up the ADC
	AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	//delay for > 1ms to allow ADC time to power up
	for(i = 0; i < 1000; i++){
		asm("   RPT#255 || NOP");
	}

	EDIS;
}

// ****************************************************************************
// ****************************************************************************
// POSITION LOOP UTILITY FUNCTIONS
// ****************************************************************************
// ****************************************************************************

// slew programmable ramper
_iq ramper(_iq in, _iq out, _iq rampDelta) {
	_iq err;

	err = in - out;
	if (err > rampDelta)
		return(out + rampDelta);
  	else if (err < -rampDelta)
  		return(out - rampDelta);
    else
    	return(in);
}

// Ramp Controller for speed reference (Not currently used)
_iq ramper_speed(_iq in, _iq out, _iq rampDelta) {
	_iq err;

	err = in - out;
	if (err > rampDelta)
	{
		if((out+rampDelta)>1.0)
			return(1.0);
		else
			return (out+rampDelta);
	}
  	else if (err < -rampDelta)
  	{
  		if(out-rampDelta<=0.0)
  			return(0.0);
  		else
  			return(out - rampDelta);
  	}
    else
    	return(in);
}



//#if BUILDLEVEL != LEVEL1
//inline void currentSensorSuite() //--------------------- 这个是C++的内联函数，不需要提前声明 --------------------------------
int Temp_curA = 0;
void currentSensorSuite()
{
	volatile int16 temp;  // temp variable used to avoid warning msgs

///////// TODO
	Temp_curA = AdcdResultRegs.ADCPPB1RESULT.all;
	current_sensor[LEM_CURRENT_SENSE-1].As   = (float)IFB_LEMV_PPB* ADC_PU_PPB_SCALE_FACTOR * LEM_TO_SHUNT;
	current_sensor[LEM_CURRENT_SENSE-1].Bs   = (float)IFB_LEMW_PPB* ADC_PU_PPB_SCALE_FACTOR * LEM_TO_SHUNT;
	current_sensor[LEM_CURRENT_SENSE-1].Cs   = -current_sensor[LEM_CURRENT_SENSE-1].As
			                                   -current_sensor[LEM_CURRENT_SENSE-1].Bs;


	current_sensor[SD_CURRENT_SENSE-1].As    = ((temp=SDFM1_READ_FILTER1_DATA_16BIT)*SD_PU_SCALE_FACTOR -
			                                    offset_SDFM1) * SDFM_TO_SHUNT;
	current_sensor[SD_CURRENT_SENSE-1].Bs    = ((temp=SDFM1_READ_FILTER2_DATA_16BIT)*SD_PU_SCALE_FACTOR -
			                                    offset_SDFM2) * SDFM_TO_SHUNT;
	current_sensor[SD_CURRENT_SENSE-1].Cs    = -current_sensor[SD_CURRENT_SENSE-1].As
			                                   -current_sensor[SD_CURRENT_SENSE-1].Bs;
	return;
}

void Init_PM(void)
{

	GpioDataRegs.GPASET.bit.GPIO17 = 1;  // LED401 OFF  这两个是双电机和三电机的使能位
	GpioDataRegs.GPCSET.bit.GPIO91 = 1;  // LED402 OFF

//--------------M新动力------------
//	InitKEY();
	MotorDir = 0;
	speedSorce =0;	// 设定速度来源  0--可调电阻；  1--S1,S2,S3按键设定,
//	GPIO_SetupPinMux(86, GPIO_MUX_CPU1, 0);
//	GPIO_SetupPinOptions(86, GPIO_OUTPUT, GPIO_ASYNC);
//    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);             // 控制SCID的RS485
//    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_ASYNC);
//    GPIO_SetupPinMux(33, GPIO_MUX_CPU2, 0);             // 控制SCIA的RS485，由于有一个反向片子，所以需要正负反向
//    GPIO_SetupPinOptions(33, GPIO_OUTPUT, GPIO_ASYNC);
//    GPIO_SetupPinMux(59, GPIO_MUX_CPU1, 0);             // 控制SCIC的RS485
//    GPIO_SetupPinOptions(59, GPIO_OUTPUT, GPIO_ASYNC);
//    GPIO_SetupPinMux(87, GPIO_MUX_CPU1, 0);             // 控制SCIB的RS485，由于有一个反向片子，所以需要正负反向
//    GPIO_SetupPinOptions(87, GPIO_OUTPUT, GPIO_ASYNC);
//    GPIO_SetupPinMux(13, GPIO_MUX_CPU1, 0);             // 控制SCIC的RS485
//    GPIO_SetupPinOptions(13, GPIO_OUTPUT, GPIO_ASYNC);
//    GPIO_SetupPinMux(12, GPIO_MUX_CPU1, 0);             // 控制SCIB的RS485，由于有一个反向片子，所以需要正负反向
//    GPIO_SetupPinOptions(12, GPIO_OUTPUT, GPIO_ASYNC);

    Run_Delay = 100;

//  Init FLAGS
    RunMotor = 1;
    LedCnt1  = 0;
    LedCnt2  = 0;
    EnableResolverISR = 1;

}

void Init_EPwm(void)
{

// ****************************************************************************
// PWM Configuration
// ****************************************************************************
// Initialize PWM module
	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;	// TB模块的时钟同步

// *****************************************
// Inverter PWM configuration
// By default on soprano the PWM clock is divided by 2  *ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV=1 * Deadband needs to be 2.0us => 10ns*200=2us

	PWM_1ch_UpDwnCnt_CNF(1,INV_PWM_TICKS,200);	//10KHz, 死区时间200ns
	PWM_1ch_UpDwnCnt_CNF(2,INV_PWM_TICKS,200);
	PWM_1ch_UpDwnCnt_CNF(3,INV_PWM_TICKS,200);

	PWM_1ch_UpDwnCnt_CNF(4,INV_PWM_TICKS,200);	//10KHz, 死区时间200ns
	PWM_1ch_UpDwnCnt_CNF(5,INV_PWM_TICKS,200);
	PWM_1ch_UpDwnCnt_CNF(6,INV_PWM_TICKS,200);
//// **********************************************
//// Sigma Delta clock set up - pwm5
//// *********************************************
///* Configure PWM5A for SD Clock i.e. 20Mhz
//	 * 20 Mhz => 50ns => 50ns/10
//*/
//	PWM_1ch_UpCnt_CNF(5,SDFM_TICKS);
//	EPwm5Regs.CMPA.bit.CMPA=EPwm5Regs.TBPRD>>1;

	// **********************************************
	// Carrier clock for resolver - pwm6
	// *********************************************
//	// Resolver PWM6 time base for injection
//	PWM_1ch_UpDwnCnt_CNF(6,RESOLVER_PWM_TICKS,160);



	// configure 2 and 3 as slaves
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm2Regs.TBCTL.bit.PHSEN    = TB_ENABLE;
	EPwm2Regs.TBPHS.bit.TBPHS    = 2;
	EPwm2Regs.TBCTL.bit.PHSDIR   = TB_UP;

	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm3Regs.TBCTL.bit.PHSEN    = TB_ENABLE;
	EPwm3Regs.TBPHS.bit.TBPHS    = 2;
	EPwm3Regs.TBCTL.bit.PHSDIR   = TB_UP;

	EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm4Regs.TBCTL.bit.PHSEN    = TB_ENABLE;
	EPwm4Regs.TBPHS.bit.TBPHS    = 2;
	EPwm4Regs.TBCTL.bit.PHSDIR   = TB_UP;

	EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm5Regs.TBCTL.bit.PHSEN    = TB_ENABLE;
	EPwm5Regs.TBPHS.bit.TBPHS    = 2;
	EPwm5Regs.TBCTL.bit.PHSDIR   = TB_UP;

	EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm6Regs.TBCTL.bit.PHSEN    = TB_ENABLE;
	EPwm6Regs.TBPHS.bit.TBPHS    = 2;
	EPwm6Regs.TBCTL.bit.PHSDIR   = TB_UP;
//	SyncSocRegs.SYNCSELECT.bit.EPWM4SYNCIN = 0; 	 //EPwm1SyncOut   3n+1
	SyncSocRegs.SYNCSELECT.bit.EPWM10SYNCIN = 0;  //EPwm1Sync Out
	EPwm10Regs.TBCTL.bit.SYNCOSEL        = TB_SYNC_IN;

// ********************************************************************
// PWM 11 for 用来进行主伺服控制
// ********************************************************************
	PWM_1ch_UpCnt_CNF(11,1000);	// 这个决定了主伺服程序的周期
	EPwm11Regs.TBCTL.bit.PHSEN  = TB_ENABLE;
	EPwm11Regs.TBPHS.bit.TBPHS  = 2;
	EPwm11Regs.TBCTL.bit.PHSDIR = TB_UP;

	EPwm11Regs.CMPC = EPwm11Regs.TBPRD - SDFM_TICKS*(OSR_RATE+1)*3/4;
	EPwm11Regs.CMPA.bit.CMPA = (SDFM_TICKS*(OSR_RATE+1)*3/3) + 50; // 500 is arbitrary
	EPwm11Regs.CMPD = 0;

// ***********************************
// Set up GPIOs for PWM functions
// **************************************
	InitEPwm1Gpio();
	InitEPwm2Gpio();
	InitEPwm3Gpio();
	InitEPwm4Gpio();
	InitEPwm5Gpio();
	InitEPwm6Gpio();
//	InitEPwmGpio();
	EDIS;
}


void Init_ADC(void)
{
	EALLOW;

	// Analog signals that are sampled
	// Ifb-SV  ADC A4/C+
	// Ifb-SW  ADC B4/C+
	// LEM V   ADC A2/C+ -----更改为D1
	// LEM W   ADC B2/C+- ----更改为D4
	// R_SIN   ADC D1
	// R_COS   ADC C1
	// Vfb-U   ADC C3
	// Vfb-V   ADC A3
	// Vfb-W   ADC B3
	// Vfb-Bus ADC B0

	// Analog Signals brought in but not sampled
	// Ifb-SU  ADC C2/C+ (& A5 not used)
	// SC-A2   ADC C0/C+
	// SC-B2   ADC D0/C+
	// SC-R    ADC D2/C+
	// Bus Volt ADC B0

	// On piccolo 133ns for ACQPS ： AD的配置转换时间
	// hencce ACQPS on soprano is 133/5~30

	// Configure the SOC0 on ADC a-d
#if (CGND == HOT)
	// Shunt Motor Currents (SV) @ A4
	// ********************************
	AdcaRegs.ADCSOC0CTL.bit.CHSEL     = 4;    // SOC0 will convert pin A4
	AdcaRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcaRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure the post processing block (PPB) to eliminate subtraction related calculation
	AdcaRegs.ADCPPB1CONFIG.bit.CONFIG = 0;    // PPB is associated with SOC0
	AdcaRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

	// Shunt Motor Currents (SW) @ B4
	// ********************************
	AdcbRegs.ADCSOC0CTL.bit.CHSEL     = 4;    // SOC0 will convert pin B4
	AdcbRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcbRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdcbRegs.ADCPPB1CONFIG.bit.CONFIG = 0;    // PPB is associated with SOC0
	AdcbRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run
#endif

//***************************ADC_a配置****************************************//
// 每个通道ABCD都有soc0-16,仅有PPB1-4(后处理模块用于处理偏差的)，每次仅能对应一组
	// LEM motor current LEM-V @ at A4  第二套电机的V相电压
	// ********************************
	AdcaRegs.ADCSOC0CTL.bit.CHSEL     = 4;    // SOC0 will convert pin A4
	AdcaRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcaRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure the post processing block (PPB) to eliminate subtraction related calculation
	AdcaRegs.ADCPPB1CONFIG.bit.CONFIG = 0;    // PPB is associated with SOC0
	AdcaRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

	// LEM motor current LEM-V @ at A2		第二套电机的母线电压
	// ********************************
	AdcaRegs.ADCSOC1CTL.bit.CHSEL     = 2;    // SOC1 will convert pin A2
	AdcaRegs.ADCSOC1CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcaRegs.ADCSOC1CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdcaRegs.ADCPPB2CONFIG.bit.CONFIG = 1;    // PPB is associated with SOC1
	AdcaRegs.ADCPPB2OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

	// Phase Voltage Vfb-V @ A3   第二套电机的U相电压
	// ***************************
	AdcaRegs.ADCSOC2CTL.bit.CHSEL     = 3;    // SOC2 will convert pin A3,
	AdcaRegs.ADCSOC2CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcaRegs.ADCSOC2CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdcaRegs.ADCPPB3CONFIG.bit.CONFIG = 2;    // PPB is associated with SOC2
	AdcaRegs.ADCPPB3OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

//***************************ADC_a配置结束************************************//

//***************************ADC_b配置****************************************//

	// LEM motor current LEM-W @ at B2   第一套电机的U相电压
	// ********************************
	AdcbRegs.ADCSOC1CTL.bit.CHSEL     = 2;    // SOC0 will convert pin B2
	AdcbRegs.ADCSOC1CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcbRegs.ADCSOC1CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdcbRegs.ADCPPB2CONFIG.bit.CONFIG = 1;    // PPB is associated with SOC1
	AdcbRegs.ADCPPB2OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

	// Phase Voltage Vfb-W @ B3     第一套电机的V相电压
	// ***************************
	AdcbRegs.ADCSOC2CTL.bit.CHSEL     = 3;    // SOC2 will convert pin B3
	AdcbRegs.ADCSOC2CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcbRegs.ADCSOC2CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdcbRegs.ADCPPB3CONFIG.bit.CONFIG = 2;    // PPB is associated with SOC2
	AdcbRegs.ADCPPB3OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

//***************************ADC_b配置结束************************************//

//***************************ADC_c配置************************************//

	// Phase Voltage Vfb-U @ C2     第二套电机的U相电流
	// ****************************
	AdccRegs.ADCSOC0CTL.bit.CHSEL     = 2;    // SOC2 will convert pin C2
	AdccRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdccRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdccRegs.ADCPPB1CONFIG.bit.CONFIG = 1;    // PPB is associated with SOC0
	AdccRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

	// Phase Voltage Vfb-U @ C3     第二套电机的V相电流
	// ****************************
	AdccRegs.ADCSOC1CTL.bit.CHSEL     = 3;    // SOC2 will convert pin C3
	AdccRegs.ADCSOC1CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdccRegs.ADCSOC1CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdccRegs.ADCPPB2CONFIG.bit.CONFIG = 1;    // PPB is associated with SOC1
	AdccRegs.ADCPPB2OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

	// Phase Voltage Vfb-W @ C4  第二套电机的W相电流
	// ****************************
	AdccRegs.ADCSOC2CTL.bit.CHSEL     = 4;    // SOC2 will convert pin C4
    AdccRegs.ADCSOC2CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
    // Configure PPB to eliminate subtraction related calculation
    AdccRegs.ADCPPB3CONFIG.bit.CONFIG = 1;    // PPB is associated with SOC2
    AdccRegs.ADCPPB3OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

    // Resolver Fbk - Cosine @ C15
    // ********************************
//  AdccRegs.ADCSOC0CTL.bit.CHSEL     = 15;   // SOC0 will convert pin C15
//  AdccRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles。用的30个时钟周期的采样时隙
//  AdccRegs.ADCSOC0CTL.bit.TRIGSEL   = 15;   // trigger on ePWM6 SOCA/C
//  // Configure PPB to eliminate subtraction related calculation
//  AdccRegs.ADCPPB1CONFIG.bit.CONFIG = 0;    // PPB is associated with SOC0
//  AdccRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

//***************************ADC_c配置结束************************************//

//***************************ADC_d配置************************************//

    // Phase Voltage Ifb-U @ D3     第一套电机的U相电流
    // ****************************
    AdcdRegs.ADCSOC0CTL.bit.CHSEL     = 3;    // SOC0 will convert pin D3
    AdcdRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
    // Configure PPB to eliminate subtraction related calculation
    AdcdRegs.ADCPPB1CONFIG.bit.CONFIG = 0;    // PPB is associated with SOC0
    AdcdRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

    // Phase Voltage Ifb-V @ D1     第一套电机的V相电流
    // ****************************
	AdcdRegs.ADCSOC1CTL.bit.CHSEL     = 1;    // SOC1 will convert pin D1
	AdcdRegs.ADCSOC1CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
	AdcdRegs.ADCSOC1CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
	// Configure PPB to eliminate subtraction related calculation
	AdcdRegs.ADCPPB2CONFIG.bit.CONFIG = 1;    // PPB is associated with SOC1
	AdcdRegs.ADCPPB2OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

	// Phase Voltage Ifb-W @ D4     第一套电机的W相电流
	// ****************************
    AdcdRegs.ADCSOC2CTL.bit.CHSEL     = 4;    // SOC2 will convert pin D4
    AdcdRegs.ADCSOC2CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
    AdcdRegs.ADCSOC2CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
    // Configure PPB to eliminate subtraction related calculation
    AdcdRegs.ADCPPB3CONFIG.bit.CONFIG = 2;    // PPB is associated with SOC2
    AdcdRegs.ADCPPB3OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

	//RG D15,滑动变阻器，可调变阻器的采样值
    // ****************************
	AdcdRegs.ADCSOC3CTL.bit.CHSEL     = 15;
	AdcdRegs.ADCSOC3CTL.bit.ACQPS     = 30;
	AdcdRegs.ADCSOC3CTL.bit.TRIGSEL   = 5;
	// Configure PPB to eliminate subtraction related calculation
	AdcdRegs.ADCPPB4CONFIG.bit.CONFIG = 0;
	AdcdRegs.ADCPPB4OFFCAL.bit.OFFCAL = 0;

	//VfbBus @ D0     第一套电机的母线电压
	// ****************************
//	AdcdRegs.ADCSOC3CTL.bit.CHSEL     = 0;    // SOC3 will convert pin D0
//	AdcdRegs.ADCSOC3CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
//	AdcdRegs.ADCSOC3CTL.bit.TRIGSEL   = 5;    // trigger on ePWM1 SOCA/C
//	// Configure PPB to eliminate subtraction related calculation
//	AdcdRegs.ADCPPB4CONFIG.bit.CONFIG = 1;     // PPB is associated with SOC3
//	AdcdRegs.ADCPPB4OFFCAL.bit.OFFCAL = 0;     // Write zero to this for now till offset ISR is run

    // Resolver Fbk - sine @ D1 注意结果存在results0中
    // ********************************
//  AdcdRegs.ADCSOC0CTL.bit.CHSEL     = 1;    // SOC0 will convert pin D1
//  AdcdRegs.ADCSOC0CTL.bit.ACQPS     = 30;   // sample window in SYSCLK cycles
//  AdcdRegs.ADCSOC0CTL.bit.TRIGSEL   = 15;   // trigger on ePWM6 SOCA/C
//  // Configure PPB to eliminate subtraction related calculation
//  AdcdRegs.ADCPPB1CONFIG.bit.CONFIG = 0;    // PPB is associated with SOC0
//  AdcdRegs.ADCPPB1OFFCAL.bit.OFFCAL = 0;    // Write zero to this for now till offset ISR is run

//***************************ADC_d配置结束************************************//

	// ******************************************************
	// static analog trim for all ADCs (A, B, C and D)
	// *******************************************************
	adc_cal=(int*)0x0000743F;
	*adc_cal=0x7000;
	adc_cal=(int*)0x000074BF;
	*adc_cal=0x7000;
	adc_cal=(int*)0x0000753F;
	*adc_cal=0x7000;
	adc_cal=(int*)0x000075BF;
	*adc_cal=0x7000;

	// Setting up link from EPWM to ADC
	EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD; // Select SOC from counter at ctr = 0
	EPwm1Regs.ETPS.bit.SOCAPRD  = ET_1ST;     // Generate pulse on 1st even
	EPwm1Regs.ETSEL.bit.SOCAEN  = 1;          // Enable SOC on A group

	EPwm6Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD; // Select SOC from counter at ctr = 0
	EPwm6Regs.ETPS.bit.SOCAPRD  = ET_1ST;     // Generate pulse on 1st even
	EPwm6Regs.ETSEL.bit.SOCAEN  = 1;          // Enable SOC on A group

	EPwm11Regs.ETSEL.bit.INTSEL = ET_CTRU_CMPA;   // INT on PRD event
	EPwm11Regs.ETSEL.bit.INTEN  = 1;              // Enable INT
	EPwm11Regs.ETPS.bit.INTPRD  = ET_1ST;         // Generate INT on every event

	EDIS;
}


void Init_DAC(void)
{
	EALLOW;

	// SETUP DACS
	DacaRegs.DACCTL.bit.DACREFSEL = REFERENCE_VREF;
	DacaRegs.DACCTL.bit.LOADMODE  = 1;      // enable value change only on sync signal
	//Enable DAC output
	DacaRegs.DACOUTEN.bit.DACOUTEN = 1;
	DacaRegs.DACCTL.bit.SYNCSEL    = 5;     // sync sel 5 meanse sync from pwm 6
	DacaRegs.DACVALS.bit.DACVALS   = 1024;

    // SETUP DACS
    DacbRegs.DACCTL.bit.DACREFSEL = REFERENCE_VREF;
    DacbRegs.DACCTL.bit.LOADMODE  = 1;      // enable value change only on sync signal
    //Enable DAC output
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;
    DacbRegs.DACCTL.bit.SYNCSEL    = 5;     // sync sel 5 meanse sync from pwm 6
    DacbRegs.DACVALS.bit.DACVALS   = 1024;

//	DacbRegs.DACCTL.bit.DACREFSEL  = REFERENCE_VREF;
//	//Enable DAC output
//	DacbRegs.DACOUTEN.bit.DACOUTEN = 1;
//	DacbRegs.DACVALS.bit.DACVALS   = 1024;

	DaccRegs.DACCTL.bit.DACREFSEL  = REFERENCE_VREF;
	//Enable DAC output
	DaccRegs.DACOUTEN.bit.DACOUTEN = 1;
	DaccRegs.DACVALS.bit.DACVALS   = 1024;
	EDIS;
}

void Init_DataLog()
{

	// ****************************************************
	// Initialize DATALOG module，就是采集数据到RAM的一片区域
	// ****************************************************
	DLOG_4CH_F_init(&dlog_4ch1);
	dlog_4ch1.input_ptr1 = &DlogCh1;	//data value
	dlog_4ch1.input_ptr2 = &DlogCh2;
	dlog_4ch1.input_ptr3 = &DlogCh3;
	dlog_4ch1.input_ptr4 = &DlogCh4;
	dlog_4ch1.output_ptr1 = &DBUFF_4CH1[0];
	dlog_4ch1.output_ptr2 = &DBUFF_4CH2[0];
	dlog_4ch1.output_ptr3 = &DBUFF_4CH3[0];
	dlog_4ch1.output_ptr4 = &DBUFF_4CH4[0];
	dlog_4ch1.size = 200;
	dlog_4ch1.pre_scalar = 5;
	dlog_4ch1.trig_value = 0.01;
	dlog_4ch1.status = 2;

}

void Init_Random()
{
    int i,j;
    for (i = 0; i < Random.order; i++) {
        Random.state[i] = Random.initial_state[i];
    }
    for (i = 0; i < Random.N; i++) {
        // 计算反馈值
        Random.feedback = (Random.state[Random.tap_positions[0]] + Random.state[Random.tap_positions[1]]) % 2; // 反馈计算
        Random.noise[i] = Random.state[Random.order - 1]; // 输出最后一位
        // 更新状态
        for (j = Random.order - 1; j > 0; j--) {
            Random.state[j] = Random.state[j - 1]; // 移位
        }
        Random.state[0] = Random.feedback; // 将反馈值放入状态的开头
    }
}

void Init_SD()
{

	// ****************************************************************************
	// ****************************************************************************
	//TODO 	Sigma Delta Initialization
	// ****************************************************************************
	// ****************************************************************************
	// Setup GPIO for SD current measurement
	GPIO_SetupPinOptions(48, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(48,0,7);

	GPIO_SetupPinOptions(49, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(49,0,7);

	GPIO_SetupPinOptions(50, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(50,0,7);

	GPIO_SetupPinOptions(51, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(51,0,7);

	// Setup GPIO for SD voltage measurement
	GPIO_SetupPinOptions(52, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(52,0,7);

	GPIO_SetupPinOptions(53, GPIO_INPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(53,0,7);

	/*******************************************************/
	/* Input Control Module */
	/*******************************************************/
	//Configure Input Control Mode: Modulator Clock rate = Modulator data rate
	Sdfm_configureInputCtrl(1,FILTER1,MODE_0);
	Sdfm_configureInputCtrl(1,FILTER2,MODE_0);
	Sdfm_configureInputCtrl(1,FILTER3,MODE_0);

	/*******************************************************/
	/* Comparator Module */
	/*******************************************************/
	//Comparator HLT and LLT
	//Configure Comparator module's comparator filter type and comparator's OSR value,
	// high level threshold, low level threshold
	Sdfm_configureComparator(1, FILTER1, SINC3, OSR_32, HLT, LLT);
	Sdfm_configureComparator(1, FILTER2, SINC3, OSR_32, HLT, LLT);
	Sdfm_configureComparator(1, FILTER3, SINC3, OSR_32, HLT, LLT);

	/*******************************************************/
	/* Sinc filter Module */
	/*******************************************************/
	//Configure Data filter modules filter type, OSR value and enable / disable data filter
	// 16 bit data representation is chosen for OSR 128 using Sinc3, from the table in the TRM
	// the max value represented for OSR 128 using sinc 3 is +/-2097152 i.e. 2^21
	// to represent this in 16 bit format where the first bit is sign shift by 6 bits
	Sdfm_configureData_filter(1, FILTER1, FILTER_ENABLE, SINC3, OSR_RATE, DATA_16_BIT, SHIFT_6_BITS);
	Sdfm_configureData_filter(1, FILTER2, FILTER_ENABLE, SINC3, OSR_RATE, DATA_16_BIT, SHIFT_6_BITS);
	Sdfm_configureData_filter(1, FILTER3, FILTER_ENABLE, SINC3, OSR_RATE, DATA_16_BIT, SHIFT_6_BITS);

	// PWM11.CMPC, PWM11.CMPD, PWM12.CMPC and PWM12.CMPD signals cannot synchronize the filters. This option is not being used in this example. 这个例程这边没用
	Sdfm_configureExternalreset(1,FILTER_1_EXT_RESET_ENABLE, FILTER_2_EXT_RESET_ENABLE, FILTER_3_EXT_RESET_ENABLE, FILTER_4_EXT_RESET_ENABLE);

	// Enable master filter bit of the SDFM module 1
	Sdfm_enableMFE(1);

}

void Init_QEP(void)
{

	// ****************************************************************************
	// Initialize QEP module
	// ****************************************************************************
	// Setup GPIO for QEP operation
	GPIO_SetupPinOptions(20, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(20,0,1);

	GPIO_SetupPinOptions(21, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(21,0,1);

	GPIO_SetupPinOptions(23, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(23,0,1);

	// Setup GPIO for QEP operation
	GPIO_SetupPinOptions(24, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(24,0,1);

	GPIO_SetupPinOptions(25, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(25,0,1);

	GPIO_SetupPinOptions(26, GPIO_INPUT, GPIO_SYNC);
	GPIO_SetupPinMux(26,0,1);

	//	GPIO22/EN IPM  第一套电机
	GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(22,0,0);
	//	GPIO27/EN IPM  第二套电机
	GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(27,0,0);
}

void Init_LED(void)
{

	// ****************************************************************************
	// To profile use GPIO 42 and GPIO43 (gen purpose)
	// ****************************************************************************
	//configure LED
	GPIO_SetupPinOptions(16, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(16,0,0);

	GPIO_SetupPinOptions(17, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(17,0,0);

	GPIO_SetupPinOptions(91, GPIO_OUTPUT, GPIO_ASYNC);
	GPIO_SetupPinMux(91,0,0);
}

void OffSet_Calibration()
{
	// TODO
	// ****************************************************************************
	// Feedbacks OFFSET Calibration Routine
	// ****************************************************************************
	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;

	offset_Rsin  = 0;
	offset_Rcos  = 0;
#if (CNGD == HOT)
	offset_shntV = 0;
	offset_shntW = 0;
	offset_shntU = 0;
#endif
	offset_lemW  = 0;
	offset_lemV  = 0;
	offset_SDFM1 = 0;
	offset_SDFM2 = 0;

	for (OffsetCalCounter=0; OffsetCalCounter<20000; )
	{
		if(EPwm11Regs.ETFLG.bit.INT==1)
		{
			if(OffsetCalCounter>1000)
			{
				offset_SDFM1 = K1*offset_SDFM1 + K2*(temp=SDFM1_READ_FILTER1_DATA_16BIT)*SD_PU_SCALE_FACTOR;
				offset_SDFM2 = K1*offset_SDFM2 + K2*(temp=SDFM1_READ_FILTER2_DATA_16BIT)*SD_PU_SCALE_FACTOR;
#if (CNGD == HOT)
				offset_shntV = K1*offset_shntV + K2*(IFB_SV)*ADC_PU_SCALE_FACTOR; 			//Phase A offset
				offset_shntW = K1*offset_shntW + K2*(IFB_SW)*ADC_PU_SCALE_FACTOR; 			//Phase B offset
#endif
				//				offset_lemV  = K1*offset_lemV + K2*(IFB_LEMV)*ADC_PU_SCALE_FACTOR;
				//				offset_lemW  = K1*offset_lemW + K2*(IFB_LEMW)*ADC_PU_SCALE_FACTOR;

				offset_lemV= K1*offset_lemV + K2*(AdcdResultRegs.ADCRESULT0)*0.00024414; 			//Phase A offset
				offset_lemW= K1*offset_lemW + K2*(AdcdResultRegs.ADCRESULT1)*0.00024414; 			//Phase B offset


				offset_Rsin  = K1*offset_Rsin + K2*R_SIN*ADC_PU_SCALE_FACTOR;
				offset_Rcos  = K1*offset_Rcos + K2*R_COS*ADC_PU_SCALE_FACTOR;
			}
			EPwm11Regs.ETCLR.bit.INT=1;
			OffsetCalCounter++;
		}
	}

	// ********************************************
	// Init OFFSET regs with identified values
	// ********************************************
	EALLOW;
#if (CNGD == HOT)
	AdcaRegs.ADCPPB1OFFREF = (offset_shntV*4096.0);     // setting shunt Iu offset
	AdcbRegs.ADCPPB1OFFREF = (offset_shntW*4096.0);     // setting shunt Iv offset
#endif
	AdccRegs.ADCPPB1OFFREF = (offset_Rcos*4096.0);      // setting resolver cos in offset
	AdcdRegs.ADCPPB1OFFREF = (offset_Rsin*4096.0);      // setting resolver sin in offset

	//	AdcaRegs.ADCPPB2OFFREF = (offset_lemV*4096.0);      // setting LEM Iv offset
	//	AdcbRegs.ADCPPB2OFFREF = (offset_lemW*4096.0);      // setting LEM Iw offset

	//D3 D1
	//	AdcdRegs.ADCPPB1OFFREF = (offset_lemV*4096.0);      // setting LEM Iv offset
	//	AdcdRegs.ADCPPB2OFFREF = (offset_lemW*4096.0);      // setting LEM Iw offset

	EDIS;
}

void DAC_Out(char ch_sel, unsigned int data)
{	// 三个DA输出均是12位，0-4095
	switch(ch_sel)
	{
		case 'A':
		    DacaRegs.DACVALS.all = data;
		    break;
		case 'B':
		    DacbRegs.DACVALS.all = data;
		    break;
		case 'C':
		    DaccRegs.DACVALS.all = data;
		    break;
		default:
		    DacaRegs.DACVALS.all = 0;
		    DacbRegs.DACVALS.all = 0;
		    DaccRegs.DACVALS.all = 0;
		    break;
	}

}

float Temp_pos_geno = 0.0;
int Flag_find_zero = 0;
float step_pos_temp = 0.001;
float Pos_Gen_Temp(float step_pos)
{
    if (Flag_find_zero == 0)
    {
        Temp_pos_geno = Temp_pos_geno + step_pos;
        if (Temp_pos_geno>360)
        {
            Temp_pos_geno = 0;
        }
    }
    if (Flag_find_zero == 1)
    {
        Temp_pos_geno = Temp_pos_geno + step_pos_temp;
        if (Temp_pos_geno>360)
        {
            Temp_pos_geno = 0;
            step_pos_temp = 0;
        }
    }


    return Temp_pos_geno;
}


void Reset_Motors_M1(void)
{
    if(abs(Motors_Start.Motor1)>1)
    {
        Motors_Start.Motor1=0;
    }
    if(abs(Motors_SetInput.Motor1)>2)
    {
        Motors_Start.Motor1=0;
    }
    if((Motors_Start.Motor1 == 0)&&(M1_M2_START == 0))
    {
        M1Ref.Pos_Refdu = 0;
        M1Ref.Speed_Refdms = 0;
        M1Ref.Iq_Ref = 0;
        M1Ref.Vq_Ref = 0;
        M1Ref.Pic_Ref = 0;
    }
    switch (Motors_Loop.Motor1){
    case 0:
        M1Ref.Pos_Refdu = 0;
        M1Ref.Speed_Refdms = 0;
        M1Ref.Iq_Ref = 0;
        M1Ref.Pic_Ref = 0;
        break;
    case 1:
        M1Ref.Vq_Ref = 0;
        M1Ref.Speed_Refdms = 0;
        M1Ref.Iq_Ref = 0;
        M1Ref.Pos_Refdu = 0;
        break;
    case 2:
        M1Ref.Pos_Refdu = 0;
        M1Ref.Vq_Ref = 0;
        M1Ref.Iq_Ref = 0;
        M1Ref.Pic_Ref = 0;
        break;
    case 3:
        M1Ref.Pos_Refdu = 0;
        M1Ref.Vq_Ref = 0;
        M1Ref.Iq_Ref = 0;
        M1Ref.Pic_Ref = 0;
        break;
    case 4:
        M1Ref.Vq_Ref = 0;
        M1Ref.Speed_Refdms = 0;
        M1Ref.Iq_Ref = 0;
        M1Ref.Pic_Ref = 0;
        break;
    case 5:
        M1Ref.Speed_Refdms = 0;
        M1Ref.Iq_Ref = 0;
        M1Ref.Vq_Ref = 0;
        M1Ref.Pic_Ref = 0;
        break;
    case 6:
        M1Ref.Vq_Ref = 0;
        M1Ref.Pos_Refdu = 0;
        M1Ref.Iq_Ref = 0;
        M1Ref.Pic_Ref = 0;
        break;
    case 7:
        M1Ref.Vq_Ref = 0;
        M1Ref.Pos_Refdu = 0;
        M1Ref.Iq_Ref = 0;
        M1Ref.Pic_Ref = 0;
        break;
    default:
        M1Ref.Pos_Refdu = 0;
        M1Ref.Speed_Refdms = 0;
        M1Ref.Iq_Ref = 0;
        M1Ref.Vq_Ref = 0;
        M1Ref.Pic_Ref = 0;
        break;
    }
}

void Reset_Motors_M2(void)
{
    if(abs(Motors_Start.Motor2)>1)
    {
        Motors_Start.Motor2 = 0;
    }
    if((Motors_Start.Motor2 == 0)&&(M1_M2_START == 0))
    {
        M2Ref.Pos_Refdu = 0;
        M2Ref.Speed_Refdms = 0;
        M2Ref.Iq_Ref = 0;
        M2Ref.Vq_Ref = 0;
        M2Ref.Pic_Ref = 0;
    }
    switch (Motors_Loop.Motor2){
    case 0:
        M2Ref.Pos_Refdu = 270;
        M2Ref.Speed_Refdms = 0;
        M2Ref.Iq_Ref = 0;
        M2Ref.Pic_Ref = 0;
        break;
    case 1:
        M2Ref.Vq_Ref = 0;
        M2Ref.Speed_Refdms = 0;
        M2Ref.Iq_Ref = 0;
        M2Ref.Pos_Refdu = 0;
        break;
    case 2:
        M2Ref.Pos_Refdu = 0;
        M2Ref.Vq_Ref = 0;
        M2Ref.Iq_Ref = 0;
        M2Ref.Pic_Ref = 0;
        break;
    case 3:
        M2Ref.Pos_Refdu = 0;
        M2Ref.Vq_Ref = 0;
        M2Ref.Iq_Ref = 0;
        M2Ref.Pic_Ref = 0;
        break;
    case 4:
        M2Ref.Vq_Ref = 0;
        M2Ref.Speed_Refdms = 0;
        M2Ref.Iq_Ref = 0;
        M2Ref.Pic_Ref = 0;
        break;
    case 5:
        M2Ref.Speed_Refdms = 0;
        M2Ref.Iq_Ref = 0;
        M2Ref.Vq_Ref = 0;
        M2Ref.Pic_Ref = 0;
        break;
    case 6:
        M2Ref.Vq_Ref = 0;
        M2Ref.Pos_Refdu = 0;
        M2Ref.Iq_Ref = 0;
        M2Ref.Pic_Ref = 0;
        break;
    case 7:
        M2Ref.Vq_Ref = 0;
        M2Ref.Pos_Refdu = 0;
        M2Ref.Iq_Ref = 0;
        M2Ref.Pic_Ref = 0;
        break;
    default:
        M2Ref.Pos_Refdu = 0;
        M2Ref.Speed_Refdms = 0;
        M2Ref.Iq_Ref = 0;
        M2Ref.Vq_Ref = 0;
        M2Ref.Pic_Ref = 0;
        break;
    }
}







