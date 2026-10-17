#include "IDDK_PM_Servo_F2837x-Settings.h"
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
#include "pid_reg.h"
#include "time.h"
#include "mpc_controller.h"

//---为了MPC上位机通讯就把这部分代码从main1移植到Interrupt了---//
int Data1_C2TC1_R[20] = {0};   // mapped to GS0 of shared RAM owned by CPU02
int Data1_C1TC2_W[20] = {0}; // mapped to GS1 of shared RAM owned by CPU01
#pragma DATA_SECTION(Data1_C2TC1_R,  "SHARERAMGS0");        // CPU2
#pragma DATA_SECTION(Data1_C1TC2_W,  "SHARERAMGS1");        // CPU1
//unsigned int polt_position;
void Shared_Ram_DataWrite_C1TC2(void);
//--------------------------------------------------------------------------------
// 更换电机时，需要注意将AGlobalVariables.h的极对数POLES更改，将endat1.RawTheta角位置更改，将Ta，Tb和Tc的顺序更改
// 旋变的函数在emif_AD7606.c文件夹中，本文件中已删除旋变用函数
//--------------------------------------------------------------------------------
//周期设置在 Init_EPwm()函数中，设置ISR_FREQUENCY 10中的10这个数字，表示KHz
int Flag_Com_cnt = 0;
unsigned int Temp_reg_speed[100] = {0};
unsigned int Temp_cnt = 0;

int search = 0;
//------ Memory数据存储 ------//
/*int flag = 0;
#pragma DATA_SECTION(draw_data1,".position_section")
//#pragma DATA_SECTION(draw_data2,".position")
unsigned int draw_data1[8000] = {0};
//unsigned int draw_data2[8000] = {0};
unsigned int num;
*/

interrupt void Communicate_ISR(void)    // 通讯中断,1KHz，在  Init_EPwm()中可以更改周期
{
    M_Servo_Cnt[0]++;
    EINT;                       // 开中断这个指令在非定时器中断中必不可少！

    Flag_Com_cnt++;
    if(Flag_Com_cnt==5)
    {
        GpioDataRegs.GPBSET.bit.GPIO33 = 1;             // 发送
//        Commun_To_Desk();           // 通讯上位机发送
//        Rec_From_DeskRs485();       // 通讯接收
//        scanKey();
        DELAY_US(5);

        Flag_Com_cnt = 0;
    }

    EPwm11Regs.ETCLR.bit.INT = 1;               // 采用epwm触发主伺服中断，应该是10KHz
    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP3;
}

// TODO 主伺服函数控制M1内框电机
interrupt void MainServe_Timer0_M1_ISR(void)  // M1表示内框架，M2表示外框架
{
	M_Servo_Cnt[1]++;                   // 数组中0是通讯/1是M1/2是M2计数
	Reset_Motors_M1();                  // 用于不同选择下其他给定复位置零
    Read_SSIA_Encoder();
    M1Data.Pos_MechTheta = Deal_loopA(M1Data.Pos_MechTheta_offset);
    currentSensorSuite();				// 电流用AD采集，Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
    //Shared_Ram_DataWrite_C1TC2();//上位机用
    if ((Motors_Start.Motor1 == 1)||(M1_M2_START == 1)) // M1表示内框架，M2表示外内框架
	{

//// *****************************************************************************
//// 各个环节带宽的调整
		M1Data.Pos_ElecTheta = ((M1Data.Pos_MechTheta*M1Data.Poles_D/360)-(unsigned int)(M1Data.Pos_MechTheta*M1Data.Poles_D/360))*(360);    //伺服控制基频 10KHz

		if (++SpeedLoopCount_M1 >= SpeedLoopPrescaler_M1)     // 外环  1KHz
		{
		    Temp_reg_speed[Temp_cnt] = M1Data.Speed;
		    Temp_cnt++;
		    if(Temp_cnt>100) Temp_cnt=0;

			M1Data.Speed_Ref = M1Ref.Speed_Refdms;
			M1Data.Speed = M1_BIT_Get_Speed(M1Data.Pos_MechTheta,0.001)/57.295779513;

            if(ASSIST < ASSIST_C) ASSIST++;
            else ASSIST = 1;

            if (++RandomLoopCount_M1 >= RandomLoopPrescaler_M1)     // 扫频  1000Hz
            {
                if(++Random.count > 2046) Random.count = 0;
                RandomLoopCount_M1=0;
            }

			if (++PosLoopCount_M1 >= PosLoopPrescaler_M1)     // 内环 500Hz
	        {
	            if (Motors_SetInput.Motor1 == 1)
	            {
	               M1Ref.Pos_Refdu = ADC_Pos;
	            }
	            M1_Pos_Fdb = M1Data.Pos_MechTheta;
	            M1Data.Pos_Ref = M1Ref.Pos_Refdu;
	            M1Data.Pic_Ref = M1Ref.Pic_Ref;
	            PosLoopCount_M1 = 0;
	        }
			SpeedLoopCount_M1=0;
		}
//// *****************************************************************************

//// *****************************************************************************
//// 控制换选择
		Shared_Ram_DataWrite_C1TC2();//上位机用//jaccobian方法
		switch (Motors_Loop.Motor1) { //单独控制M1
//		switch (M1_M2_LOOP) { //控制同M2
		// PID-全开环
	    case 0:
	        M1Data.Vq_Ref = M1Ref.Vq_Ref;
	        ipark1.Ds = M1Data.Vd_Ref;
	        ipark1.Qs = M1Data.Vq_Ref;
	        break;
// PID-无模型图像环
        case 1:
            if((Data1_C2TC1_R[0]>125)||(Data1_C2TC1_R[0]<3)){
                M1_PID_Pic.Fdb = 0.0;
                M1_PID_Pic.Ui = 0.0;
                M1_PID_Pic.Ui_1 = 0.0;}
            else{
                M1_PID_Pic.Fdb = (64.0 - Data1_C2TC1_R[0])/K1_M1;}
            M1_Loop_Picture();
            break;
        // PID-雅可比图像环
        case 2:
            M1Data.Speed_Ref = Data1_C2TC1_R[0];
            M1Data.Speed_Ref = (M1Data.Speed_Ref/100.0 - 50.0)/K2_M1;
            M1_Loop_Speed(M1Data.Speed_Ref);
            break;
        // PID-速度环
        case 3:
            M1_Loop_Speed(M1Data.Speed_Ref);
            break;
        // PID-位置环
        case 4:
            M1_Loop_Position(M1Data.Pos_Ref, M1_Pos_Fdb,0);
            break;
//         PID-速度+位置环
        case 5:
            M1_Loop_Position(M1Data.Pos_Ref, M1_Pos_Fdb,1);
            break;
        // 正弦波扫频
//        case 5:
//            ipark1.Ds = 0;
//            ipark1.Qs = 0.2*sin(2*M_PI*(ASSIST-1)/ASSIST_C);

        // MPC_test////////////////////////////////////////////这里配置！！
        case 6:
            M1_Loop_MPC(M1Data.Speed_Ref);
            break;
        case 7: // 像素坐标投影+位置环+速度环
            M1Data.Pos_Ref = M1_Loop_Theta(search); //1丢失目标后搜索
            M1_Loop_Position(M1Data.Pos_Ref, M1_Pos_Fdb,1);
            break;
        default:
            M1_Loop_Current(0, 0);
            M1_Loop_Speed(0);
            M1_Loop_Position(0, 0, 0);
		}
//// *****************************************************************************

//// *****************************************************************************
//// 电机驱动
        clarke1.As=((AdcdResultRegs.ADCRESULT1)*0.00024414 -offset_lemV)*2;
        clarke1.Bs=((AdcdResultRegs.ADCRESULT2)*0.00024414 -offset_lemW)*2;

		clarke1.calc(&clarke1);
		park1.Angle = M1Data.Pos_ElecTheta*2*PI/360;
		park1.Alpha = clarke1.Alpha;
		park1.Beta = clarke1.Beta;
		park1.calc(&park1);

        ipark1.Angle = park1.Angle;
//		模拟电角度
//		ipark1.Angle = M1_Angle_Update(ipark1.Angle);
		ipark1.calc(&ipark1);
		svgen1.Ualpha = ipark1.Alpha;
		svgen1.Ubeta  = ipark1.Beta;
		svgen1.calc(&svgen1);

		// 内框abc ok
		EPwm1Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Ta)+INV_PWM_HALF_TBPRD;//a
		EPwm2Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tb)+INV_PWM_HALF_TBPRD;//b
		EPwm3Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen1.Tc)+INV_PWM_HALF_TBPRD;
//// *****************************************************************************
	}

//    DAC_Out('A',M1Data.Pos_ElecTheta);
//    DAC_Out('B',-M1Data.Pos_ElecTheta);
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

/////////// ------------------------ 主伺服函数控制M2外框电机 ------------------------------------
// TODO 主伺服函数控制M2外框电机
interrupt void MainServe_Timer1_M2_ISR(void)
{
    M_Servo_Cnt[2]++;
    Reset_Motors_M2();
    Read_SSIB_Encoder();
    M2Data.Pos_MechTheta = Deal_loopB(M2Data.Pos_MechTheta_offset);
//    polt_position = M2Data.Pos_MechTheta/360.0*2*3.1415926535897932384626433*10420;
    if ((Motors_Start.Motor2 == 1)||(M1_M2_START == 1))       // M1表示外框架，M2表示内框架
    {
        M2Data.Pos_ElecTheta = ((M2Data.Pos_MechTheta*M2Data.Poles_D/360)-(unsigned int)(M2Data.Pos_MechTheta*M2Data.Poles_D/360))*(360);

        //------Memory存储数据------//
        /*if(flag&&(num<8000)){
            draw_data1[num] = M2Data.Speed*5000;
            num++;
        }*/

        if (++SpeedLoopCount_M2 >= SpeedLoopPrescaler_M2)  // 外环 200Hz
        {
            M2Data.Speed_Ref = M2Ref.Speed_Refdms;
            M2Data.Speed = M2_BIT_Get_Speed(M2Data.Pos_MechTheta,0.001)/57.295779513;

            if (++RandomLoopCount_M2 >= RandomLoopPrescaler_M2)     // 扫频  1000Hz
            {
                if(++Random.count > 2046) Random.count = 0;
                RandomLoopCount_M2=0;
            }

            if (++PosLoopCount_M2 >= PosLoopPrescaler_M2)  // 内环 100Hz
            {
                M2Data.Pos_Ref = M2Ref.Pos_Refdu;
                M2_Pos_Fdb = M2Data.Pos_MechTheta;
                M2Data.Pic_Ref = M2Ref.Pic_Ref;
                PosLoopCount_M2 = 0;
            }
            SpeedLoopCount_M2=0;
        }

        switch (Motors_Loop.Motor2) { //单独控制M2
//        switch (M1_M2_LOOP) { //控制同M1
        // 全开环
        case 0:
            M2Data.Vq_Ref = M2Ref.Vq_Ref;
            ipark2.Ds = M2Data.Vd_Ref;
            ipark2.Qs = M2Data.Vq_Ref;
            break;
        // PID-无模型图像环
        case 1:
            if((Data1_C2TC1_R[1]>125)||(Data1_C2TC1_R[1]<3)){
                M2_PID_Pic.Fdb = 0.0;
                M2_PID_Pic.Ui = 0.0;
                M2_PID_Pic.Ui_1 = 0.0;}
            else{
                M2_PID_Pic.Fdb = -(Data1_C2TC1_R[1] - 64.0)/K1_M2;}
            M2_Loop_Picture();
            break;
        // PID-雅可比图像环
        case 2:
            M2Data.Speed_Ref = Data1_C2TC1_R[1];
            M2Data.Speed_Ref = (M2Data.Speed_Ref/100.0 - 50.0)/K2_M2;
            M2_Loop_Speed(-M2Data.Speed_Ref);
            break;
        // 速度环
        case 3:
            M2_Loop_Speed(M2Data.Speed_Ref);
            break;
        // PID-位置环
        case 4:
            M2_Loop_Position(M2Data.Pos_Ref, M2_Pos_Fdb,0);
            break;
        case 5:
            M2Data.Pos_Ref = M2Ref.Pos_Refdu;
            M2_Loop_Position(M2Data.Pos_Ref, M2_Pos_Fdb,1);
            break;
        case 6:
            M2_Loop_MPC(M2Data.Speed_Ref);
            break;
        case 7: // 像素坐标投影+位置环+速度环
            M2Data.Pos_Ref = M2_Loop_Theta(search); //1丢失目标后搜索
            M2_Loop_Position(M2Data.Pos_Ref, M2_Pos_Fdb,1);
            break;
        // 默认状态
        default:
                M2_Loop_Current(0, 0);
                M2_Loop_Speed(0);
                M2_Loop_Position(0, 0, 0);
        }

//// *****************************************************************************
//////  对应P203口的ADC 和 初始化SOC，是接驱动板的接口处
        clarke2.As=((AdccResultRegs.ADCRESULT1)*0.00024414 -offset_lemV)*2; // Phase A curr.  需要且已经归一化
        clarke2.Bs=((AdccResultRegs.ADCRESULT2)*0.00024414 -offset_lemW)*2; // Phase B curr.  需要且已经归一化
        clarke2.calc(&clarke2);

        park2.Angle = M2Data.Pos_ElecTheta*2*PI/360;        //编码器获取的实时角度 posEncElecTheta[POSITION_ENCODER]
        park2.Alpha = clarke2.Alpha;
        park2.Beta = clarke2.Beta;
        park2.calc(&park2);

        ipark2.Angle = park2.Angle;                         //编码器获取的实时角度
        ipark2.calc(&ipark2);
        svgen2.Ualpha = ipark2.Alpha;
        svgen2.Ubeta  = ipark2.Beta;
        svgen2.calc(&svgen2);
//////  步科编码器电机a b c 位标器内框c b a 位标器外框a c b
        EPwm4Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen2.Ta)+INV_PWM_HALF_TBPRD;
        EPwm5Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen2.Tb)+INV_PWM_HALF_TBPRD;
        EPwm6Regs.CMPA.bit.CMPA = (INV_PWM_HALF_TBPRD*svgen2.Tc)+INV_PWM_HALF_TBPRD;
    }

    DAC_Out('A',M2Data.Pos_ElecTheta);
//   DAC_Out('B',M2Data.Speed*3000);
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}



//------------------------------------ MainISR Ends Here ---------------------------------------------
/*********************************************************************************************************/

//串口上传数据，进行图形曲线的绘制
//波特率921600bps
//上传通道数 设定到 CmdT[0]
// 如果需要上传更密集的曲线以便观测图形的连续性，则请自行修改为观测1条曲线。

//int Delay_Time_Sci = 0.3;     // 已经在CPU2中实现了通讯
//
//void Commun_To_Desk()
//{
//	int ii;
//	//M1Data.Pos_MechTheta,M2Data.Pos_MechTheta;
//	temp_draw[0] = 0xAA55;
//	temp_draw[1] = M1Data.Speed*10+30000;
//	temp_draw[2] = M2Data.Speed*10+30000;;
//	temp_draw[3] = M1Data.Pos_MechTheta;
//	temp_draw[4] = M2Data.Pos_MechTheta;
//	temp_draw[5] = clarke1.As;
//	temp_draw[6] = clarke1.Bs;
//	temp_draw[7] = (temp_draw[0]^temp_draw[1]^temp_draw[2]^temp_draw[3]^temp_draw[4]^temp_draw[5]^temp_draw[6])&0XFFFF;//&0XFFFF; // 校验位
//
////	GpioDataRegs.GPBSET.bit.GPIO33 = 1;             // 发送
//
//	for(ii=0;ii<SciTX_cnt;ii++)
//	{
//		HexA_Printf(temp_draw[ii]);
//		DELAY_US(Delay_Time_Sci);
//	}
////	GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1;
////	DELAY_US(2);
//}


Uint16 CRC_pars[6] = {0};
void Rec_From_DeskRs485( void)
{
    int i=0;
    for(i=0;i<17;i++)
    {
        if (RxB_Double_temp[i]==0xB6 )
        {   // 跟上位机顺序一样，字头是B6，为传更多参数和控制指令

            CRC_pars[0] = (RxB_Double_temp[i]*256+RxB_Double_temp[i+1])^((RxB_Double_temp[i+3]*256)+(RxB_Double_temp[i+2]));
            CRC_pars[1] = (CRC_pars[0])^(RxB_Double_temp[i+5]*256+RxB_Double_temp[i+4]);
            CRC_pars[2] = (CRC_pars[1])^(RxB_Double_temp[i+7]*256+RxB_Double_temp[i+6]);
            CRC_pars[3] = (CRC_pars[2])^(RxB_Double_temp[i+9]*256+RxB_Double_temp[i+8]);
            CRC_pars[4] = (CRC_pars[3])^(RxB_Double_temp[i+11]*256+RxB_Double_temp[i+10]);
            CRC_pars[5] = (CRC_pars[4])^(RxB_Double_temp[i+13]*256+RxB_Double_temp[i+12]);

            if ((CRC_pars[5]) == ((RxB_Double_temp[i+15]*256+RxB_Double_temp[i+14])&0XFFFF))
            {
                Contol_PM[0]  = (RxB_Double_temp[i+1])&0X00FF;
                Parameters[0] = (RxB_Double_temp[i+3]*256+RxB_Double_temp[i+2])&0XFFFF;
                Parameters[1] = (RxB_Double_temp[i+5]*256+RxB_Double_temp[i+4])&0XFFFF;
                Parameters[2] = (RxB_Double_temp[i+7]*256+RxB_Double_temp[i+6])&0XFFFF;
                Parameters[3] = (RxB_Double_temp[i+9]*256+RxB_Double_temp[i+8])&0XFFFF;
                Parameters[4] = (RxB_Double_temp[i+11]*256+RxB_Double_temp[i+10])&0XFFFF;
                Parameters[5] = (RxB_Double_temp[i+13]*256+RxB_Double_temp[i+12])&0XFFFF;
            }
        }
    }
}


// Setup OCP limits and digital filter parameters of CMPSS
void  CMPSS_DIG_FILTER(volatile struct CMPSS_REGS *v, Uint16 curHi, Uint16 curLo)
{
	// comparator references
	v->DACHVALS.bit.DACVAL = curHi;   // positive max current limit
	v->DACLVALS.bit.DACVAL = curLo;   // negative max current limit

	// digital filter settings - HIGH side
	v->CTRIPHFILCLKCTL.bit.CLKPRESCALE = clkPrescale;    // set time between samples, max : 1023
	v->CTRIPHFILCTL.bit.SAMPWIN        = sampwin;        // # of samples in window, max : 31
	v->CTRIPHFILCTL.bit.THRESH         = thresh;         // recommended : thresh > sampwin/2

	// digital filter settings - LOW side
	v->CTRIPLFILCLKCTL.bit.CLKPRESCALE = clkPrescale;    // Max count of 1023 */
	v->CTRIPLFILCTL.bit.SAMPWIN        = sampwin;        // # of samples in window, max : 31
	v->CTRIPLFILCTL.bit.THRESH         = thresh;         // recommended : thresh > sampwin/2

	return;
}

//#################################################
//初始化独立按键 S1
//连接到GPIO61
//-----------------------------------------------
//void InitKEY()
//{
//    EALLOW;
//    S1MUX = 0;//GPIO功能
//    S1DIR = 0;//输入
//    S1PUD = 1;//由于开发板外置了上拉电阻，禁用DSP内部上拉
//    EDIS;
//}

/************************************************************************/
//扫描读取按键操作函数
//独立IO按键 S1     GPIO27
//扫描按键S2-S6 采用TM1650编码
//用户可以在这里处理按键的内容
/************************************************************************/

//有感可以2个方向运行，无感只能一个方向
/*按键  数码管内容  含义
 * S1   B100        设定速度1，600RPM
 * S2   B200        设定速度2，1000RPM
 * S2   B300        设定速度3，1500RPM
 * S4   A100        驱动板可调电阻设定速度，顺时针
 * S5   A200        驱动板可调电阻设定速度，逆时针转（仅有感支持2个方向运行）
 * S6   F000        停机（或者母线断电）
 * */
//
int ADC_Pos_temp=0;
void scanKey(void)
{
    //驱动板可调电阻RG的输出，作为速度RPM的给定
    ADC_Pos_temp = AdcdResultRegs.ADCRESULT3;

    //有感速度RPM限幅
    if(ADC_Pos_temp > sensorOnHighSpeed)
        ADC_Pos_temp = sensorOnHighSpeed;
    else if(ADC_Pos_temp < sensorOnLowSpeed )
        ADC_Pos_temp = sensorOnLowSpeed;

    ADC_Pos = (ADC_Pos_temp-sensorOnLowSpeed)/10;
}

//---上位机通讯函数---//
void Shared_Ram_DataWrite_C1TC2(void)
{
//    memcpy(Pos_1, &M1Data.Pos_MechTheta, sizeof(M1Data.Pos_MechTheta));
//    memcpy(Pos_2, &M2Data.Pos_MechTheta, sizeof(M2Data.Pos_MechTheta));
//    memcpy(Pos_3, &M1Data.Speed, sizeof(M1Data.Speed));
//    memcpy(Pos_4, &M2Data.Speed, sizeof(M2Data.Speed));
////    Pos_3_temp = (float)Data1_C2TC1_R[2];
////    memcpy(Pos_3, &Pos_3_temp, sizeof(Pos_3_temp));

//位置，M1内框，M2外框
//    Pos_temp1[0] = Pos_1[0] & 0xFF;
//    Pos_temp1[1] = Pos_1[0] >> 8 & 0xFF;
//    Pos_temp1[2] = Pos_1[1] & 0xFF;
//    Pos_temp1[3] = Pos_1[1] >> 8 & 0xFF;
//    Pos_temp2[0] = Pos_2[0] & 0xFF;
//    Pos_temp2[1] = Pos_2[0] >> 8 & 0xFF;
//    Pos_temp2[2] = Pos_2[1] & 0xFF;
//    Pos_temp2[3] = Pos_2[1] >> 8 & 0xFF;
//电机速度
//    Pos_temp3[0] = Pos_3[0] & 0xFF;
//    Pos_temp3[1] = Pos_3[0] >> 8 & 0xFF;
//    Pos_temp3[2] = Pos_3[1] & 0xFF;
//    Pos_temp3[3] = Pos_3[1] >> 8 & 0xFF;
//    Pos_temp4[0] = Pos_4[0] & 0xFF;
//    Pos_temp4[1] = Pos_4[0] >> 8 & 0xFF;
//    Pos_temp4[2] = Pos_4[1] & 0xFF;
//    Pos_temp4[3] = Pos_4[1] >> 8 & 0xFF;

//    Data1_C1TC2_W[0] = 0xAA55;
//    Data1_C1TC2_W[1] = Pos_temp1[0]*256 + Pos_temp1[1];
//    Data1_C1TC2_W[2] = Pos_temp1[2]*256 + Pos_temp1[3];
//    Data1_C1TC2_W[3] = Pos_temp2[0]*256 + Pos_temp2[1];
//    Data1_C1TC2_W[4] = Pos_temp2[2]*256 + Pos_temp2[3];
//    Data1_C1TC2_W[5] = Pos_temp3[2]*256 + Pos_temp3[3];
//    Data1_C1TC2_W[6] = Pos_temp3[2]*256 + Pos_temp3[3];
//    Data1_C1TC2_W[7] = Pos_temp4[2]*256 + Pos_temp4[3];
//    Data1_C1TC2_W[8] = Pos_temp4[2]*256 + Pos_temp4[3];
//    Data1_C1TC2_W[9] = 0x0D00;//校验位
//    Data1_C1TC2_W[10] = M1_M2_LOOP;

//    Data1_C1TC2_W[4] = 0xAA55;
//    Data1_C1TC2_W[5] = 0x11*256 + Pos_temp1[0];
//    Data1_C1TC2_W[6] = Pos_temp1[1]*256 + Pos_temp1[2];
//    Data1_C1TC2_W[7] = Pos_temp1[3]*256 + 0x0D;

//  指向跟踪控制
    memcpy(Pos_1, &M1Data.Pos_MechTheta, sizeof(M1Data.Pos_MechTheta));
    memcpy(Pos_2, &M2Data.Pos_MechTheta, sizeof(M2Data.Pos_MechTheta));

    Pos_temp1[0] = Pos_1[0] & 0xFF;
    Pos_temp1[1] = Pos_1[0] >> 8 & 0xFF;
    Pos_temp1[2] = Pos_1[1] & 0xFF;
    Pos_temp1[3] = Pos_1[1] >> 8 & 0xFF;
    Data1_C1TC2_W[0] = 0xAA55;
    Data1_C1TC2_W[1] = 0x11*256 + Pos_temp1[0];
    Data1_C1TC2_W[2] = Pos_temp1[1]*256 + Pos_temp1[2];
    Data1_C1TC2_W[3] = Pos_temp1[3]*256 + 0x0D;

    Pos_temp2[0] = Pos_2[0] & 0xFF;
    Pos_temp2[1] = Pos_2[0] >> 8 & 0xFF;
    Pos_temp2[2] = Pos_2[1] & 0xFF;
    Pos_temp2[3] = Pos_2[1] >> 8 & 0xFF;
    Data1_C1TC2_W[4] = 0xAA55;
    Data1_C1TC2_W[5] = 0x11*256 + Pos_temp2[0];
    Data1_C1TC2_W[6] = Pos_temp2[1]*256 + Pos_temp2[2];
    Data1_C1TC2_W[7] = Pos_temp2[3]*256 + 0x0D;

//    Pos_temp3[0] = Pos_3[0] & 0xFF;
//    Pos_temp3[1] = Pos_3[0] >> 8 & 0xFF;
//    Pos_temp3[2] = Pos_3[1] & 0xFF;
//    Pos_temp3[3] = Pos_3[1] >> 8 & 0xFF;
//    Data1_C1TC2_W[8] = 0xAA55;
//    Data1_C1TC2_W[9] = 0x13*256 + Pos_temp3[0];
//    Data1_C1TC2_W[10] = Pos_temp3[1]*256 + Pos_temp3[2];
//    Data1_C1TC2_W[11] = Pos_temp3[3]*256 + 0x0D;

//  上位机绘图
//    Data1_C1TC2_W[0] = 0xAA55;
//    Data1_C1TC2_W[1] = M1Data.Pos_MechTheta*10-1000;
//    Data1_C1TC2_W[2] = M1Data.Pos_MechTheta*10;
//    Data1_C1TC2_W[3] = M2Data.Pos_MechTheta*10;
//    Data1_C1TC2_W[4] = M2Data.Speed_Ref*10000+10000;
////    Data1_C1TC2_W[5] = polt_position+30;
//    Data1_C1TC2_W[6] = 0x0000;
//    Data1_C1TC2_W[7] = (Data1_C1TC2_W[0]^Data1_C1TC2_W[1]^Data1_C1TC2_W[2]^Data1_C1TC2_W[3]^Data1_C1TC2_W[4]^Data1_C1TC2_W[5]^Data1_C1TC2_W[6])&0XFFFF;//&0XFFFF; // 校验位
}


//    //---------------TM1650获取扫描按键S101 S102---------------------
//    //TM1650_Read(CMD_KEY,&keyVal);
//
//    if(keyVal == S2_CODE)
//    {
//        KEY_speed = 1000;          //按键设定速度值
//        speedSorce =1;
//        MotorDir = 1;
//
//        //B200
////        TM1650_Send(DIG4,SEG7Table[0x0B]);
////        TM1650_Send(DIG3,SEG7Table[2]);
////        TM1650_Send(DIG2,SEG7Table[0]);
////        TM1650_Send(DIG1,SEG7Table[0]);
//    }
//    //S3接到SEGB和DIG1上
//    if(keyVal == S3_CODE)
//    {
//        KEY_speed = 1500;          //按键设定速度值
//        speedSorce =1;
//        MotorDir = 1;
//
//        //B300
////        TM1650_Send(DIG4,SEG7Table[0x0B]);
////        TM1650_Send(DIG3,SEG7Table[3]);
////        TM1650_Send(DIG2,SEG7Table[0]);
////        TM1650_Send(DIG1,SEG7Table[0]);
//    }
//    //S4接到SEGC和DIG1上
//    if(keyVal == S4_CODE)
//    {
//        // A100
////        TM1650_Send(DIG4,SEG7Table[0x0A]);
////        TM1650_Send(DIG3,SEG7Table[1]);
////        TM1650_Send(DIG2,SEG7Table[0]);
////        TM1650_Send(DIG1,SEG7Table[0]);
//
//        speedSorce =0;
//        MotorDir = 1;
//    }
//    //S5接到SEGD和DIG1上
//    if(keyVal == S5_CODE)
//    {
//        // A200
////        TM1650_Send(DIG4,SEG7Table[0x0A]);
////        TM1650_Send(DIG3,SEG7Table[2]);
////        TM1650_Send(DIG2,SEG7Table[0]);
////        TM1650_Send(DIG1,SEG7Table[0]);
//
//        speedSorce =0;
//        MotorDir = 2;
//    }
//    //S6接到SEGE和DIG1上
//    if(keyVal == S6_CODE)
//    {
//        // F000
////        TM1650_Send(DIG4,SEG7Table[0x0F]);
////        TM1650_Send(DIG3,SEG7Table[0]);
////        TM1650_Send(DIG2,SEG7Table[0]);
////        TM1650_Send(DIG1,SEG7Table[0]);
//
//        MotorDir = 0;
//        speedSorce =0;
//    }
//}
//
//
// Reference Position Generator for position loop
//_iq refPosGen(_iq out)
//{
//	_iq in = posArray[ptr1] ;
//
//	out = ramper(in, out, posSlewRate);
//
//	if (in == out)
//	if (++cntr1 > 1000)
//	{
//		cntr1 = 0;
//		if (++ptr1 >= ptrMax)
//			ptr1 = 0;
//	}
//	return (out);
//}

/****************************************************************************
 * End of Code *
 * ***************************************************************************
 */
