#include "IDDK_PM_Servo_F2837x-Settings.h"
#include "F2837xD_sysctrl.h"
#include "F2837x_QEP_module.h"
#include "f2837xbmsk.h"
#include "F2837xD_adc.h"
#include "IQmathLib.h"
#include "F2837xD_eqep.h"
#include "sci_base.h"
#include "emif.h"
#include "mpc_controller.h"

extern float gaint;
extern float study_gain;
//--------------------------------------------------------------
#define _usSamples 30
#define POWER_UP_LEV    0.3
#define POWER_DROW_LEV  0.25

#define  LOOP_2 2               // 双闭环
#define  LOOP_3 3               // 三闭环
#define  LOOP_TYPE   LOOP_2     // 选择双闭环 或者 三闭环
//#define POLES     8               // Number of poles  买的成品米格电机是8，步科电机是10，做的电机是16
//#define POLES     10               // Number of poles  买的成品米格电机是8，步科电机是10，做的电机是16
//#define POLES   30              // Number of poles  买的成品米格电机是8，步科电机是10，做的电机是15对，这个poles是个数不是对数
#define POLES   42              // Number of poles  买的成品米格电机是8，步科电机是10，做的电机是15对，这个poles是个数不是对数

extern int Data1_C2TC1_R[20];   // mapped to GS0 of shared RAM owned by CPU02
extern int Data1_C1TC2_W[20]; // mapped to GS1 of shared RAM owned by CPU01

extern int Err_x;
extern int Err_y;
extern uint16_t isrfuncLoadStart;
extern uint16_t isrfuncLoadEnd;
extern uint16_t isrfuncRunStart;
extern uint16_t isrfuncLoadSize;

// M1分段PID参数
extern float M1gainB_kp;
extern float M1gainB_ki;
extern float M1gainS_kp;
extern float M1gainS_ki;
//// M1分段PID参数
//extern float M1gainR_kp;
//extern float M1gainR_ki;
//extern float M1gainL_kp;
//extern float M1gainL_ki;
// M2分段PID参数
//extern float M2gainR_kp;
//extern float M2gainR_ki;
//extern float M2gainL_kp;
//extern float M2gainL_ki;
extern float M2gainB_kp;
extern float M2gainB_ki;
extern float M2gainS_kp;
extern float M2gainS_ki;
extern int search;
//定义母线电压检测上电、掉电值
//若IPM驱动板用户使用 低压电机做有感实验时，将以下分别赋值0.08 和 0.0； IPM驱动板的分压电阻针对高压配置的，所以使用低压电机是无法做无感实验，除非修改电阻值，建议与技术联系再修改
//对于IPM驱动板用户 使用高压电机做实验， 或者 IR2136驱动板用户做低压实验，则 分别赋值0.30 和 0.25


//S2--S6 在TM1650扫描下的返回键码值
#define S2_CODE 0x44
#define S3_CODE 0x4C
#define S4_CODE 0x54
#define S5_CODE 0x5C
#define S6_CODE 0x64

#define S1_CODE 0x00//S1为独立按键，按下为 0x00

//独立按键S1
#define S1GPIO GPIO61
#define S1MUX GpioCtrlRegs.GPBMUX2.bit.S1GPIO
#define S1DIR GpioCtrlRegs.GPBDIR.bit.S1GPIO
#define S1PUD GpioCtrlRegs.GPBPUD.bit.S1GPIO
#define S1QSEL GpioCtrlRegs.GPBQSEL2.bit.S1GPIO
#define rS1DAT()  GpioDataRegs.GPBDAT.bit.S1GPIO

#define   STOP 0  // 电机停止
#define   CCW 1   // 电机顺时针
#define   CW 2    // 电机逆时针

//设置位置闭环结构体定义
#define T_SCALE 1200            //10K 中断，1个单位为 0.0001秒
#define POS_NUM  7              //目标位置

#define CALIBRATEANGLE             65439//16288


//definitions for selecting DACH reference
#define REFERENCE_VDDA     0

//definitions for COMPH input selection
#define NEGIN_DAC          0
#define NEGIN_PIN          1

//definitions for CTRIPH/CTRIPOUTH output selection
#define CTRIP_ASYNCH       0
#define CTRIP_SYNCH        1
#define CTRIP_FILTER       2
#define CTRIP_LATCH        3

//---------------------【3个define,只能开启1个，调试步骤说明】---------------------------------------------------------------------------
//#define OPEN_LOOP_TEST  //打开该句，开环运行模式，只能在有感模式下运行

//------------------ 函数自定义 ------------------------
interrupt void MainServe_Timer0_M1_ISR(void);
interrupt void MainServe_Timer1_M2_ISR(void);


extern void ConfigureADC(void);

//------- Flash ---------
extern void DeviceInit();
extern void MemCopy();
extern void InitFlash();

//------- 两个主要中断服务 ---------
extern interrupt void Communicate_ISR(void);
extern interrupt void ResolverISR(void);


// **********************************************************


extern _iq ramper(_iq in, _iq out, _iq rampDelta);
extern void HVDMC_Protection(void);
extern void PWM_1ch_UpDwnCnt_CNF(int16 n, Uint16 period, int16 db);
extern void PWM_1ch_UpCnt_CNF(int16 n, Uint16 period);
extern void ConfigureADC(void);

extern void closeBridge(void);
extern void openLowBridge(void);
extern void setPWMFree(void);


extern void Init_LED(void);
//extern void InitKEY(void);
extern void scanKey(void);
extern void Commun_To_Desk(void);
extern void Init_PM(void);
extern void baseParamsInit(void);
extern void Init_EPwm(void);
extern void Init_ADC(void);
extern void Init_DAC(void);
extern void DAC_Out(char ch_sel, unsigned int data);
extern float Pos_Gen_Temp(float step_pos);
extern void Init_DataLog(void);
extern void Init_SD(void);
extern void Init_Random(void);
extern void Init_QEP(void);
extern void OffSet_Calibration(void);
//extern Uint16 READ_Position(int position_offset);
extern void derivParamsCal(void);

extern interrupt void FOCIdleISR(void);
extern interrupt void MotorControlISR(void);

// State Machine function prototypes
//------------------------------------
// Alpha states
extern void A0(void);   //state A0
extern void B0(void);   //state B0
extern void C0(void);   //state C0
extern void  CMPSS_DIG_FILTER(volatile struct CMPSS_REGS *v, Uint16 curHi, Uint16 curLo);
// A branch states
//extern void A1(void); //state A1
//
//// B branch states
//extern void B1(void); //state B1
//
//// C branch states
//extern void C1(void); //state C1

// Variable declarations
extern void (*Alpha_State_Ptr)(void);   // Base States pointer
extern void (*A_Task_Ptr)(void);        // State pointer A branch
extern void (*B_Task_Ptr)(void);        // State pointer B branch
extern void (*C_Task_Ptr)(void);        // State pointer C branch


//extern inline void currentSensorSuite();
extern void currentSensorSuite();

extern interrupt void UartA_Rx_isr(void);
extern interrupt void UartB_Rx_isr(void);
extern interrupt void UartB_Tx_isr(void);
extern interrupt void UartC_Rx_isr(void);
extern interrupt void UartD_Rx_isr(void);
interrupt void SPIATXFIFO(void);
interrupt void SPIARXFIFO(void);
interrupt void SPIBTXFIFO(void);
interrupt void SPIBRXFIFO(void);

// ----------------------------- 结构体全局定义 -------------------------------
//****************** Main函数用变量 ***********************
extern volatile struct SDFM_REGS *SDFM[];
//extern volatile struct SYNC_SOC_REGS SyncSocRegs;


//****************** 主伺服函数用变量 ***********************
typedef struct{
    int count;
    int N;
    int order;
    int feedback;

    int noise[2047];
    int tap_positions[2];
    int state[11];
    int initial_state[11];
} Random_Reg;
extern Random_Reg Random;

typedef struct {
    int Motor1;
    int Motor2;
} Motors_Reg;       // 电机启动和三环控制标志
extern Motors_Reg Motors_Start;
extern Motors_Reg Motors_Loop;
extern Motors_Reg Motors_SetInput;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
} PID_Reg;      // PID控制算法的参数

typedef struct {
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
} MPC_Reg;      // PID控制算法的参数

typedef struct {
    PID_Reg PID_Pos;
    PID_Reg PID_Spd;
    PID_Reg PID_Pic;
    PID_Reg PID_Cur_Id;
    PID_Reg PID_Cur_Iq;
    MPC_Reg MPC_Spd_Crr;
} MotorParam_Reg;
extern MotorParam_Reg M1Param,M2Param;


typedef struct {
    float Pos_Ref;
    float Pos_MechTheta;
    float Pos_MechTheta_offset;
    float Pos_ElecTheta;
    float Pic_Ref;
    float Speed_Ref;
    float Speed;
    float Id_Ref;
    float Iq_Ref;
    float Vd_Ref;
    float Vq_Ref;
    int   Poles_D;    //极对数，位标器电机是15对，购买电机是8对极
} Motros_Data_Reg;      // PID控制算法的参数
extern Motros_Data_Reg M1Data,M2Data;

typedef struct {
    float Pos_Refdu;
    float Speed_Refdms;
    float Pic_Ref;
    float Iq_Ref;
    float Vq_Ref;
} Motros_Ref_du;      // PID控制算法的参数
extern Motros_Ref_du M1Ref, M2Ref;


extern RESOLVER_OUTPUT rslvrOut;
extern volatile struct EQEP_REGS *eQEP[];
extern QEP qep1 ;
extern volatile RESOLVER resolver1 ; // 旋变用
extern volatile ABS_ENCODER endat1 ;
extern volatile ABS_ENCODER biss1 ;
extern volatile struct EPWM_REGS *ePWM[];

// Instance a ramp controller to smoothly ramp the frequency
extern RMPCNTL rc1 ;
extern RMPCNTL rc2 ; // for speed

//  Instance a ramp generator to simulate an Anglele
extern RAMPGEN rg1 ;

//  Instance a phase voltage calculation
extern PHASEVOLTAGE volt1 ;

// Instance a Space Vector PWM modulator. This modulator generates a, b and c
// phases based on the d and q stationery reference frame inputs

typedef struct {    // 结构体本体的定义是放到.h文件中，不是放在.c文件
    float As;      // phase A
    float Bs;      // phase B
    float Cs;      // phase C
} CURRENT_SENSOR;
extern CURRENT_SENSOR current_sensor[3];

//设置位置闭环结构体定义

typedef struct pos_Type{
    int16  cntr1;              //当前位置停留时间计数  （0.001秒为单位）
    int16  ptr1;               //目标位置切换计数
    int16  TimerCnt[POS_NUM];  //运行到目标位置后的停顿时间
    float Array[POS_NUM];      //目标位置设定
    float Speed[POS_NUM];     //改变 posArray[]数组里的位置变化，所需的速度设定，单位 RPM（转每分钟）
}
pos_info;

extern pos_info posMotor;

extern SVGEN svgen1 ;
extern SVGEN svgen2 ;
extern CLARKE clarke1;
extern CLARKE clarke2;
extern PARK   park1   ;
extern PARK   park2   ;
extern IPARK  ipark1  ;
extern IPARK  ipark2  ;

extern DLOG_4CH_F dlog_4ch1;
extern DLOG_4CH_F dlog_4ch2;

// Instance PI(D) regulators to regulate the d and q  axis currents, speed and position
extern PIDREG          M1_PID_Pos ;
extern PIDREG          M1_PID_Spd ;
extern PIDREG          M1_PID_Pic ;
extern PI_CONTROLLER   M1_PI_Id   ;
extern PI_CONTROLLER   M1_PI_Iq   ;
extern PIDREG          M2_PID_Pos;
extern PIDREG          M2_PID_Spd ;
extern PIDREG          M2_PID_Pic ;
extern PI_CONTROLLER   M2_PI_Id   ;
extern PI_CONTROLLER   M2_PI_Iq   ;

extern PIDREG3         pid_pos_2  ;          // (optional - for eval)
extern PI_CONTROLLER   pi_pos_2   ;
extern PID_CONTROLLER   pid_spd_2 ;
extern PI_CONTROLLER   pi_id_2    ;
extern PI_CONTROLLER   pi_iq_2    ;

extern MPC_CONTROLLER_PM M1_MPC_Spd_Crr;
extern MPC_CONTROLLER_PM M2_MPC_Spd_Crr;
extern MPC_STATE_PM M1_MPC_State;
extern MPC_STATE_PM M2_MPC_State;


//------------------------------ 全局变量自定义 -----------------------------------
//****************** Main函数用变量 ***********************
extern Uint16 Error_Main_Cnt;
extern volatile Uint16 EnableFlag;



//****************** 主伺服函数用变量 ***********************
extern volatile int16 temp;

extern int16 OffsetCalCounter;

//SD Trip Level - scope for additonal work
extern Uint16  HLT,LLT;
extern volatile float offset_SDFM1;  // offset in SD current V fbk channel @ 0A
extern volatile float offset_SDFM2;  // offset in SD current W fbk channel @ 0A
extern _iq K1,  K2;       // Offset filter coefficient K2: T/(T+0.05);

extern float  Pos_gen_step;
extern float  M1_Pos_Fdb;
extern float  M2_Pos_Fdb;
extern Uint16 M_Servo_Cnt[3];
extern Uint32 Loop_Flag_cnt[2];               //lsw状态切换计数
extern int    EnableResolverISR;  // not used
extern int    Run_Delay;          // not used
extern float  ADC_Pos;          //ADC原始值换做设定速度值
extern int16  KEY_speed;          //按键设定速度值
extern int16  Set_speed;               //设定速度值
extern float  curLimit ;
extern int    LedCnt1;
extern int    LedCnt2;
extern Uint32 Loop_Flag_def[2];   //无感运行的D轴抱轴时间
extern int16  VTimer0[4];           // Virtual Timers slaved off CPU Timer 0 (A events)
extern int16  VTimer1[4];       // Virtual Timers slaved off CPU Timer 1 (B events)
extern int16  VTimer2[4];       // Virtual Timers slaved off CPU Timer 2 (C events)

extern float32 T;    // Samping period (sec), see parameter.h

extern Uint16  clkPrescale,
        sampwin     ,
        thresh      ,
        LEM_curHi   ,
        LEM_curLo   ,
        SHUNT_curHi,
        SHUNT_curLo ,
        TripFlagDMC ,               //PWM trip status
        clearTripFlagDMC ,
        RunMotor ;
extern Uint16 startRES ;
extern Uint16  sensorOnHighSpeed ;
extern Uint16  sensorOnLowSpeed  ;
extern int16    SerialCommsTimer;
extern Uint16  PosLoopPrescaler_M1,      // Speed loop pre scalar
        PosLoopCount_M1,
        SpeedLoopPrescaler_M1,      // Speed loop pre scalar
        SpeedLoopCount_M1,           // Speed loop counter
        RandomLoopPrescaler_M1,
        RandomLoopCount_M1,
        RandomLoopPrescaler_M2,
        RandomLoopCount_M2;
extern Uint16  PosLoopPrescaler_M2,      // Speed loop pre scalar
        PosLoopCount_M2,
        SpeedLoopPrescaler_M2,      // Speed loop pre scalar
        SpeedLoopCount_M2;

extern char keyVal;

//*************** 角位置 ******************
extern _iq posEncElecTheta[6], posEncMechTheta[6];
extern Uint16 lswWaitStart; //设置开环运行减速标志
extern int32 OldRawTheta;   //上一次位置
extern Uint16 RawTheraCnt;  //位置不动之后的计数，用来判断速度为0
extern long temppos;
extern long lastPos;
extern _iq posEncElecTheta[6], posEncMechTheta[6];

extern _iq  cntr, alignCnt;
extern _iq  IdRef_start,  IdRef_run ;
extern char speedSorce ;             //设定速度来源  0--可调电阻；  1--S1,S2,S3按键设定,

extern int SpeedSetdms,PosSetdu;

// Variables for Datalog module
extern float DBUFF_4CH1[200],
      DBUFF_4CH2[200],
      DBUFF_4CH3[200],
      DBUFF_4CH4[200],
      DlogCh1,
      DlogCh2,
      DlogCh3,
      DlogCh4;

//***************** PID中电流和速度初始给定 *****************

// ****************************************************************************
// Variables for current measurement
// ****************************************************************************
// Offset calibration routine is run to calibrate for any offsets on the opamps
extern _iq offset_Rsin,         // offset in resolver sine fbk channel @ no excitation
    offset_Rcos,         // offset in resolver cos  fbk channel @ no excitation
#if (CNGD == HOT)
    offset_shntV,        // offset in shunt current V fbk channel @ 0A
    offset_shntW,        // offset in shunt current W fbk channel @ 0A
    offset_shntU,        // offset in shunt current U fbk channel @ 0A
#endif
    offset_lemV,         // offset in LEM current V fbk channel @ 0A
    offset_lemW;         // offset in LEM current W fbk channel @ 0A

// 位置信息生成
// ============================================
extern Uint32 posClaCnt;
extern float softStart,ftemp;

extern char MotorDir;     //电机运行状态（顺时针转、逆时针转（仅有感支持）、停止）
extern float Zpos;

extern short buff[16];

extern Uint16 keyDelay ;

//////// -------------- 三环 ----------
extern float M1_Loop_Theta(int num1);
extern float M2_Loop_Theta(int num2);
extern void M1_Loop_Current(float IdRef, float IqRef);
extern void M1_Loop_Speed(float SpdRef);
extern void M1_Loop_Position(float PosRef, float M1_Pos_Fdb, int Include_speed_loop);
extern void M1_Loop_Picture(void);
extern void M2_Loop_Picture(void);
extern float M1_Angle_Update(float angle);
extern void M1_Loop_MPC(float SpdRef); //MPC1
extern void M2_Loop_MPC(float SpdRef); //MPC2
extern void M2_Loop_Current(float IdRef_M2, float IqRef_M2);
extern void M2_Loop_Speed(float SpdRef_M2);
extern void M2_Loop_Position(float PosRef, float M2_Pos_Fdb, int Include_speed_loop);
extern float M1_BIT_Get_Speed(float M1_Pos_MechTheta, float M1_Time_interval);
extern float M2_BIT_Get_Speed(float M2_Pos_MechTheta, float M2_Time_interval);
extern float M1_Speed_Filter(float M1_Temp_spd_dms);
extern float M2_Speed_Filter(float M2_Temp_spd_dms);

//*************** 通讯 *********************
extern void Rec_From_DeskRs485(void);
extern unsigned int CRCcode;
extern char CmdT[16];
extern int SciTX_cnt;   // 最大是6
extern char chx;
extern Uint32 Sci_Wait_cnt[3];
extern Uint32 Sci_send_cnt[3];
extern unsigned int temp_draw[8] ;
extern unsigned int RxA_Buff_data[16];
extern unsigned int RxB_Buff_data[11];
extern int Rx_Num_All;
extern unsigned int RxC_Buff_data[11];

extern Uint16 Parameters[6];
extern Uint16 Contol_PM[2];
extern unsigned int RxB_Double_temp[22];
extern unsigned int RxC_Double_temp[22];

extern void Reset_Motors_M1(void);
extern void Reset_Motors_M2(void);
extern void Read_SSIA_Encoder(void);
extern void Read_SSIB_Encoder(void);
extern void Send_To_Encoder_M1(int CF_M1);  // 读取编码器时候发的指令
extern void Send_To_Encoder_M2(int CF_M2);  // 读取编码器时候发的指令
extern float Get_Pos_Encoder_M1(float Temp_Pos_Encoder_Err);
extern float Get_Pos_Encoder_M2(float Temp_Pos_Encoder_Err2);
extern double Deal_loopA(float Temp_Pos_Encoder_Err);
extern double Deal_loopB(float Temp_Pos_Encoder_Err2);
// Default ADC initialization
extern int ChSel[16]   ;
extern int  TrigSel[16] ;
extern int ACQPS[16]  ;

extern int *adc_cal;

extern int ASSIST;
extern int ASSIST_C;

extern uint16_t isrfuncLoadStart;
extern uint16_t isrfuncLoadEnd;
extern uint16_t isrfuncRunStart;
extern uint16_t isrfuncLoadSize;

extern uint8_t Pos_1[2];
extern uint8_t Pos_2[2];
extern uint8_t Pos_3[2];
extern uint8_t Pos_4[2];
extern float Pos_3_temp;
extern uint8_t Pos_temp1[4];
extern uint8_t Pos_temp2[4];
extern uint8_t Pos_temp3[4];
extern uint8_t Pos_temp4[4];

extern float K1_M1;
extern float K1_M2;
extern float K2_M1;
extern float K2_M2;

extern int M1_M2_START;
extern int M1_M2_LOOP;

extern float PRBS_temp;
