
#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "F2837xD_device.h"
#include "F2837xD_sysctrl.h"

// Configure the period for each timer
#define EPWM1_TIMER_TBPRD  4096  // Period register
#define EPWM1_MAX_CMPA     4050
#define EPWM1_MIN_CMPA       50
#define EPWM1_MAX_CMPB     4050
#define EPWM1_MIN_CMPB       50
#define EPWM_CMP_UP   1
#define EPWM_CMP_DOWN 0

interrupt void cpu_timer2_isr(void);

typedef struct {
    int state;
    int state_pre;
    int speed;
    int speed_pre;
    int speed_next;
} Kinco_Data;      // PID控制算法的参数
extern Kinco_Data Kinco;

extern uint16_t isrfuncLoadStart;
extern uint16_t isrfuncLoadEnd;
extern uint16_t isrfuncRunStart;
extern uint16_t isrfuncLoadSize;

extern unsigned int Data2_C2TC1_W[20];   // mapped to GS0 of shared RAM owned by CPU02
extern unsigned int Data2_C1TC2_R[20];     // mapped to GS1 of shared RAM owned by CPU01

extern interrupt void UartA_Rx_isr(void);
extern interrupt void UartB_Rx_isr(void);
extern interrupt void UartC_Rx_isr(void);
extern interrupt void UartD_Rx_isr(void);
extern void Shared_Ram_dataWrite_c2(void);// function to write data from shared RAM owned by c2
extern void SCI_Init(void);
extern void SCI_InitA(void);
extern void SCI_InitB(void);
extern void SCI_InitC(void);
extern void SCI_InitD(void);
extern void SCI_GPIO(void);
extern void Init_Kinco(void);
extern void ConfigureADC(void);
extern void SetupADCContinuous(Uint16 channel);

extern void Send_To_MATLAB();
extern void Send_To_Targetmotor();
extern void Send_To_Kinco();
extern void HexA_Printf(int a);
extern void Scia_xmit(int a);
extern void HexB_Printf(int b);
extern void Scib_xmit(int b);
extern void HexC_Printf(int a);
extern void Scic_xmit(int a);


extern int Flag_cpu2_cnt ;
extern int Switch_cpu2 ;

extern int Flag_CPU2 ;
extern int temp_draw[8];
extern float Delay_Time_SciA ;
extern float Delay_Time_SciB ;
extern float Delay_Time_SciC ;
extern float Delay_Time_SciD ;
extern int Rxa_flag;
extern int Rxb_flag;
extern int Rxc_flag;
extern int Rxd_flag;
extern int RxA_cnt;
extern int RxB_cnt;
extern int RxC_cnt;
extern int RxD_cnt;
extern int RxA_Buff_data[12];
extern int RxB_Buff_data[12];
extern int RxC_Buff_data[12];
extern int RxD_Buff_data[10];
extern int Flag_cpu2_timer2;
extern float Rec_DELAY ;
extern int Rec_Flag ;
extern int Send_Data ;
extern float Delay_Sendtime ;
extern int Flag_Buf_ia ;
extern int Flag_Buf_ib ;
extern int Flag_Buf_ic ;
extern int Flag_Buf_id ;
extern unsigned int RxA_Double_temp[12];
extern unsigned int RxB_Double_temp[12];
extern unsigned int RxC_Double_temp[12];
extern unsigned int RxD_Double_temp[12];
extern int Flag_Buf_Rec_A ;

extern int ParA_RS422_Flag ;
extern int ParB_RS422_Flag ;
extern int ParD_RS422_Flag ;
extern int ParC_RS422_Flag ;
extern Uint32 ParA_RS422[10] ;
extern Uint32 ParB_RS422[10] ;
extern Uint32 ParC_RS422[10] ;
extern Uint32 ParD_RS422[10];
extern int Start_M1 ;
extern int Start_M2 ;
extern int Loop_M1 ;
extern int Loop_M2 ;

extern int TargetMotor_Start;
extern int TargetMotor_Speed;
extern int TargetMotor_Stop;//目标电机控制标志

extern int data3Bytes_flag;

extern uint32_t data1Bytes;
extern uint32_t data2Bytes;
extern uint32_t data3Bytes;
extern float SCIA_Data1;
extern float SCIA_Data2;
extern float SCIB_Data1;
extern float SCIB_Data2;
extern float SCID_Data1;
extern float SCID_Data2;
extern float Pos_inc;
extern unsigned int Pulse_flag;
