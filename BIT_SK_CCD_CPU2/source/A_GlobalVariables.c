
#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "AGlobalVariables.h"




int Flag_cpu2_cnt = 0;
int Switch_cpu2 = 1;

Kinco_Data Kinco = {0, 0, 0.0, 0.0};

int Flag_CPU2 = 0;

int temp_draw[8]={0};
float Delay_Time_SciA = 150.0;
float Delay_Time_SciB = 150.0;
float Delay_Time_SciC = 150.0;
float Delay_Time_SciD = 150.0;
int Rxa_flag=0;
int Rxb_flag=0;
int Rxc_flag=0;
int Rxd_flag=0;
int RxA_cnt=0;
int RxB_cnt=0;
int RxC_cnt=0;
int RxD_cnt=0;
unsigned int RxA_Double_temp[12] = {0};
unsigned int RxB_Double_temp[12] = {0};
unsigned int RxC_Double_temp[12] = {0};
unsigned int RxD_Double_temp[12] = {0};
int RxA_Buff_data[12]={0};
int RxB_Buff_data[12]={0};
int RxC_Buff_data[12]={0};
int RxD_Buff_data[10]={0};
int Flag_cpu2_timer2 = 0;
float Rec_DELAY = 0;
int Rec_Flag = 0;
int Send_Data = 0xAA;
float Delay_Sendtime = 0.0;
int Flag_Buf_ia = 0;
int Flag_Buf_ib = 0;
int Flag_Buf_ic = 0;
int Flag_Buf_id = 0;
int Flag_Buf_Rec_A = 0;

int ParA_RS422_Flag = 0;
int ParB_RS422_Flag = 0;
int ParC_RS422_Flag = 0;
int ParD_RS422_Flag = 0;
Uint32 ParA_RS422[10] = {0};
Uint32 ParB_RS422[10] = {0};
Uint32 ParC_RS422[10] = {0};
Uint32 ParD_RS422[10] = {0};
int Start_M1 = 0;
int Start_M2 = 0;
int Loop_M1 = 0;
int Loop_M2 = 0;

int TargetMotor_Start = 0;
int TargetMotor_Speed = 0;
int TargetMotor_Stop = 0;

int data3Bytes_flag = 0;

uint32_t data1Bytes = 0;
uint32_t data2Bytes = 0;
uint32_t data3Bytes = 0;
float SCIA_Data1 = 0;
float SCIA_Data2 = 0;
float SCIB_Data1 = 0;
float SCIB_Data2 = 0;
float SCID_Data1 = 0;
float SCID_Data2 = 0;
float Pos_inc = 0;
unsigned int Pulse_flag = 0x0000;
