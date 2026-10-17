#include "AGlobalVariables.h"

// ----------------------------- 结构体全局定义 -------------------------------
//****************** Main函数用变量 ***********************
volatile struct SDFM_REGS *SDFM[] = {0, &Sdfm1Regs, &Sdfm2Regs};
//volatile struct SYNC_SOC_REGS SyncSocRegs;		// 在F2837xD_GlobalVariableDefs.c中定义了，这里不再体现


//****************** 主伺服函数用变量 ***********************
// Used to indirectly access eQEP module
volatile struct EQEP_REGS *eQEP[] =
 				  { &EQep1Regs,
 				  	&EQep1Regs,
					&EQep2Regs,
				  };
QEP qep1 = QEP_DEFAULTS;

volatile RESOLVER resolver1 = RESOLVER_DEFAULTS; // 旋变用
volatile ABS_ENCODER endat1 = ABSENC_DEFAULTS;
volatile ABS_ENCODER biss1  = ABSENC_DEFAULTS;
void (*Alpha_State_Ptr)(void);	// Base States pointer
void (*A_Task_Ptr)(void);		// State pointer A branch
void (*B_Task_Ptr)(void);		// State pointer B branch
void (*C_Task_Ptr)(void);		// State pointer C branch

volatile struct EPWM_REGS *ePWM[] = {		// Used to indirectly access all EPWM modules
		&EPwm1Regs,			//intentional: (ePWM[0] not used)
		&EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs, &EPwm5Regs, &EPwm6Regs,
		&EPwm7Regs, &EPwm8Regs, &EPwm9Regs, &EPwm10Regs, &EPwm11Regs, &EPwm12Regs};

// Instance a ramp controller to smoothly ramp the frequency
RMPCNTL rc1 = RMPCNTL_DEFAULTS;
RMPCNTL rc2 = RMPCNTL_DEFAULTS; // for speed

//	Instance a ramp generator to simulate an Anglele
RAMPGEN rg1 = RAMPGEN_DEFAULTS;

//	Instance a phase voltage calculation
PHASEVOLTAGE volt1 = PHASEVOLTAGE_DEFAULTS;


// Instance a Space Vector PWM modulator. This modulator generates a, b and c
// phases based on the d and q stationery reference frame inputs


CURRENT_SENSOR current_sensor[3];

//位置的给定
pos_info posMotor={
        0,
        0,
        {2*T_SCALE,T_SCALE,T_SCALE,T_SCALE,T_SCALE,T_SCALE,4*T_SCALE},//目标到达后电机停止的时间
        {0,        10,      -10,      30,      -10,     0,    -1},//位置，单位圈，正负代表正反转
        //{60,      100,    300,    300,   300,   300,  300} //仅200W以下电机用； 速度，单位RPM（转每分钟）；注意速度和圈数的匹配;
        {60,      100,    1000,    1000,   1000,   1000,  300}};

//volatile struct ADC_REGS AdcbRegs;
SVGEN svgen1 = SVGEN_DEFAULTS;
SVGEN svgen2 = SVGEN_DEFAULTS;
CLARKE clarke1 = CLARKE_DEFAULTS;
CLARKE clarke2 = CLARKE_DEFAULTS;
PARK   park1   = PARK_DEFAULTS;
PARK   park2   = PARK_DEFAULTS;
IPARK  ipark1  = IPARK_DEFAULTS;
IPARK  ipark2  = IPARK_DEFAULTS;

DLOG_4CH_F dlog_4ch1;
DLOG_4CH_F dlog_4ch2;

// Instance PI(D) regulators to regulate the d and q  axis currents, speed and position
//PID_CONTROLLER	M1_PID_Spd = {PID_TERM_DEFAULTS, PID_PARAM_DEFAULTS, PID_DATA_DEFAULTS};
PIDREG          M1_PID_Pos = PIDREG_DEFAULTS_pos_M1;
PIDREG          M1_PID_Spd = PIDREG_DEFAULTS_speed_M1;
PIDREG          M1_PID_Pic = PIDREG_DEFAULTS_picture_M1;
PI_CONTROLLER   M1_PI_Id   = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER   M1_PI_Iq  = PI_CONTROLLER_DEFAULTS;
PIDREG          M2_PID_Pos = PIDREG_DEFAULTS_pos_M2;
PIDREG          M2_PID_Spd = PIDREG_DEFAULTS_speed_M2;
PIDREG          M2_PID_Pic = PIDREG_DEFAULTS_picture_M2;
PI_CONTROLLER   M2_PI_Id   = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER   M2_PI_Iq  = PI_CONTROLLER_DEFAULTS;

//实例化MPC控制器
MPC_CONTROLLER_PM M1_MPC_Spd_Crr = MPC_CONTROLLER_PM_DEFAULTS;
MPC_CONTROLLER_PM M2_MPC_Spd_Crr = MPC_CONTROLLER_PM_DEFAULTS;
MPC_STATE_PM M1_MPC_State = MPC_STATE_PM_DEFAULTS;
MPC_STATE_PM M2_MPC_State = MPC_STATE_PM_DEFAULTS;


PIDREG3         pid_pos_2 = PIDREG3_DEFAULTS;          // (optional - for eval)
PI_CONTROLLER   pi_pos_2  = PI_CONTROLLER_DEFAULTS;
PID_CONTROLLER	pid_spd_2 = {PID_TERM_DEFAULTS, PID_PARAM_DEFAULTS, PID_DATA_DEFAULTS};
PI_CONTROLLER   pi_id_2   = PI_CONTROLLER_DEFAULTS;
PI_CONTROLLER   pi_iq_2   = PI_CONTROLLER_DEFAULTS;

Motors_Reg Motors_Start = {0,0};
Motors_Reg Motors_Loop  = {0,0};
Motors_Reg Motors_SetInput = {0,0};
//PID_Reg PID_Cur_Id= {0.1,0.01,0}, PID_Cur_Iq= {0.1,0.01,0}, PID_Spd= {0.1,0.001,0} , PID_Pos= {0,0,0};
//MotorParam_Reg M1Param = {15,0,8,  0.0001,0.000001,0,  0.01,0.001,0,  0.01,0.001,0,  0,0,0};
//MotorParam_Reg M1Param = {12,0,8,  1.1,0.001,0,  0.1,0.01,0,  0.1,0.01,0,  0,0,0};
MotorParam_Reg M1Param = {9.2,0,0,  160,0.3,0,  22,0.0,0.0, 0.02,0.005,0,  1.5,0.01,0,// 各种PID参数  位置单环 0.006    非图像转速环0.03,0.000005,0
//                         {12.0,0,0,  50,0.05,0,  7,0.0,0.0, 0.02,0.005,0,  1.5,0.01,0, // 各种PID参数  位置单环 0.006
                          //{
                          //case 14.0,0,0,  80,0.1,0,  14,0.0,0.0, 0.02,0.005,0,  1.5,0.01,0,7 图像环及转速环参数：0.098,0,0,  0.78,0.00008,0,
                          //case 2 转速环参数：0.64,0.00007,0,
                          //case 1 转速环及图像环参数：0.7,0.00006,0, 0.5,0.0000001,0,
                          400, 220, 1e-3, 10, 3, 2, 10, 200, 200, 100, 0.05, 1, 0.0003, 120, 0.68, 1.7, 10}; // MPC参数
MotorParam_Reg M2Param = {5.5,0,0,  160,0.36,0,  42,0.0,0.0, 0.02,0.005,0, 1.5,0.01,0, // 各种PID参数  位置单环 0.006    非图像转速环0.03,0.000005,0
//                         {6.4,0,0,  220,0.08,0,  6,0.0,0.0, 0.02,0.005,0, 1.5,0.01,0,
                          400, 220, 1e-3, 10, 3, 2, 10, 200, 200, 100, 0.05, 1, 0.0003, 120, 0.68, 1.7, 10}; // MPC参数
//MPC第一代:                0.003, 0.3, 1e-3, 10, 3, 2, 10, 0.02, 0.1, 0.8, 0.05, 10, 0.3, 200, 30, 0, 10
//MPC第二代(有重物)12.22:     0.003, 1.2, 1e-3, 10, 3, 2, 10, 20, 0.1, 0.3, 0.005, 10, 0.3, 300, 138, 0, 10
// 64 470// 0.32, 2.35, 1e-3, 5, 3, 2, 5, 5, 2, 15, 50, 10, 5, 100, 35, 0.0, 5
Motros_Data_Reg M1Data={0,0,35,0,0,0,0,0,0,0,0,21}, M2Data={0,0,95,0,0,0,0,0,0,0,0,21}; //offset_sk=9,步科=1
Motros_Ref_du M1Ref={0}, M2Ref ={0};//{220,0,0,0,0};

Random_Reg Random = {0,2047,11,0,{0},{10,8},{0},{1,1,1,1,1,1,1,1,1,1,1}};
//------------------------------ 全局变量自定义 -----------------------------------
//****************** Main函数用变量 ***********************
Uint16 Error_Main_Cnt = 0;
volatile Uint16 EnableFlag = FALSE;



//****************** 主伺服函数用变量 ***********************

volatile int16 temp;

int16 OffsetCalCounter;
Uint16  HLT = 0x7FFF,
        LLT = 0x0;
volatile float offset_SDFM1;  // offset in SD current V fbk channel @ 0A
volatile float offset_SDFM2;  // offset in SD current W fbk channel @ 0A

_iq K1 = _IQ(0.998),		  // Offset filter coefficient K1: 0.05/(T+0.05);
    K2 = _IQ(0.001999);	      // Offset filter coefficient K2: T/(T+0.05);


//uint16_t Position;
float Pos_gen_step = 0.003;
float M1_Pos_Fdb = 0.0;
float M2_Pos_Fdb = 0.0;
Uint16 M_Servo_Cnt[3] = {0};
Uint32 Loop_Flag_cnt[2]={0,0};          //lsw状态切换计数
int    EnableResolverISR;  // not used
int    Run_Delay;          // not used
float ADC_Pos = 0.0;          //ADC原始值换做设定速度值
int16 KEY_speed = 600;          //按键设定速度值
int16 Set_speed=0;               //设定速度值
float curLimit = 8.0;
int    LedCnt1=0;
int	   LedCnt2=0;
Uint32 Loop_Flag_def[2]={30,3000};	//无感运行的D轴抱轴时间
int16	VTimer0[4];				// Virtual Timers slaved off CPU Timer 0 (A events)
int16	VTimer1[4]; 			// Virtual Timers slaved off CPU Timer 1 (B events)
int16	VTimer2[4]; 			// Virtual Timers slaved off CPU Timer 2 (C events)

float32 T = 0.001/ISR_FREQUENCY;    // Samping period (sec), see parameter.h

Uint16  clkPrescale = 20,
		sampwin     = 30,
		thresh      = 18,
		LEM_curHi   = LEM(8.0),
		LEM_curLo   = LEM(8.0),
		SHUNT_curHi = SHUNT(8.0),
		SHUNT_curLo = SHUNT(8.0),
		TripFlagDMC = 0,			// PWM trip status
		clearTripFlagDMC = 0,
		RunMotor = 0;
Uint16 startRES = 0;
Uint16  sensorOnHighSpeed  =  4000;
Uint16  sensorOnLowSpeed  =   400;
int16	SerialCommsTimer;
Uint16  PosLoopPrescaler_M1 = 2,      // Speed loop pre scalar
        PosLoopCount_M1 = 1,
        SpeedLoopPrescaler_M1 = 10,      // Speed loop pre scalar
        SpeedLoopCount_M1 = 1,           // Speed loop counter
        RandomLoopPrescaler_M1 = 1,
        RandomLoopCount_M1 = 1,
        RandomLoopPrescaler_M2 = 1,
        RandomLoopCount_M2 = 1;
Uint16  PosLoopPrescaler_M2 = 2,      // Speed loop pre scalar
        PosLoopCount_M2 = 1,
        SpeedLoopPrescaler_M2 = 10,      // Speed loop pre scalar
        SpeedLoopCount_M2 = 1;

char keyVal;

//*************** 角位置 ******************
Uint16 lswWaitStart = 0; //设置开环运行减速标志
int32 OldRawTheta = 0;   //上一次位置
Uint16 RawTheraCnt = 0;  //位置不动之后的计数，用来判断速度为0
long temppos;
long lastPos=0;

_iq posEncElecTheta[6],
    posEncMechTheta[6];

_iq  cntr=0,
	 alignCnt = 20000;
_iq  IdRef_start = _IQ(0.15),  IdRef_run   = _IQ(0.0);
char speedSorce =0;             //设定速度来源  0--可调电阻；  1--S1,S2,S3按键设定,



// Variables for Datalog module
float DBUFF_4CH1[200],
      DBUFF_4CH2[200],
      DBUFF_4CH3[200],
      DBUFF_4CH4[200],
      DlogCh1,
      DlogCh2,
      DlogCh3,
      DlogCh4;

//***************** PID中电流和速度初始给定 *****************


//PID_CONTROLLER	pid_pos = {PID_TERM_DEFAULTS, PID_PARAM_DEFAULTS, PID_DATA_DEFAULTS};
//PI_CONTROLLER pi_spd = PI_CONTROLLER_DEFAULTS;

// Instance a PWM driver instance
//PWMGEN pwm1 = PWMGEN_DEFAULTS;
// ****************************************************************************
// Variables for current measurement
// ****************************************************************************
// Offset calibration routine is run to calibrate for any offsets on the opamps
_iq offset_Rsin,         // offset in resolver sine fbk channel @ no excitation
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
Uint32 posClaCnt=0;
float softStart=0.0,ftemp;

//自定义变量和函数

char MotorDir = 0;     //电机运行状态（顺时针转、逆时针转（仅有感支持）、停止）

float Zpos=0.0;

short buff[16];

Uint16 keyDelay = 300;

//*************** 通讯 *********************
unsigned int CRCcode;
char CmdT[16];
int SciTX_cnt = 8;		// 最大是6
char chx=0;
Uint32 Sci_Wait_cnt[3] = {0};
Uint32 Sci_send_cnt[3] = {0};
unsigned int temp_draw[8]  = {0};
unsigned int RxA_Buff_data[16] = {0};
unsigned int RxB_Buff_data[11] = {0};
int Rx_Num_All = 11;
unsigned int RxC_Buff_data[11] = {0};


Uint16 Parameters[6] = {0};
Uint16 Contol_PM[2] = {0};
unsigned int RxB_Double_temp[22] = {0};
unsigned int RxC_Double_temp[22] = {0};

// Default ADC initialization
int ChSel[16]   = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int	TrigSel[16] = {5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5};
int ACQPS[16]   = {8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};

int *adc_cal;

int ASSIST = 0;
int ASSIST_C = 256;

uint16_t isrfuncLoadStart;
uint16_t isrfuncLoadEnd;
uint16_t isrfuncRunStart;
uint16_t isrfuncLoadSize;

uint8_t Pos_1[2] = {0,0};
uint8_t Pos_2[2] = {0,0};
uint8_t Pos_3[2] = {0,0};
uint8_t Pos_4[2] = {0,0};
float Pos_3_temp = 0.0;
uint8_t Pos_temp1[4] = {0,0,0,0};
uint8_t Pos_temp2[4] = {0,0,0,0};
uint8_t Pos_temp3[4] = {0,0,0,0};
uint8_t Pos_temp4[4] = {0,0,0,0};

float K1_M1 = 200.0;
float K1_M2 = 350.0;
float K2_M1 = 40.0;//30.0;
float K2_M2 = 20.0;

int M1_M2_START = 0;
int M1_M2_LOOP = 0;

float PRBS_temp = 1.0;
