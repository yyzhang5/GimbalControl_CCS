/* ============================================================================
 *
 * 从这版开始，去掉旋变代码，速度等由位置差分得到了
 *2024.7.9程序已经更改为M1用定时器0控制10KHz，M2用定时器1控制10KHz，PWM11中断用来进行通讯1KHz，并将其他无用程序删掉了
 *2024.7.24 程序已经完成双核通讯，CPU2主要是进行RS422通讯，可以快速发送，但是无法同时快速接收，接收慢点是可以的完成的！
*
// =============================== LEVEL 5 ======================================
// 删除TI的level了，直接主伺服开始，本版本是从2024.3.26开始编写，改成了习惯的方式，中断和常用函数在Ainterrupts.c中，配置和固定的函数在functions中
 *
 *
 * M新动力   DSP28377D 电机例程（基于TI官方例程改进，在开发板上可直接运行）
 * 硬件组成：DSP28377D开发板 +AD2S1210旋变模块 + 电机驱动板 + 4极对伺服电机
 * 注意：DSP28377D开发板需要单独提供5V电源供电（从黑色电源座），而不从IR2136低压电机驱动板JP2取电；高压驱动板可以从JP2取电
 *
 * 本例程设置    //AD2S1210模块 + 旋变电机4极对
                endat1.PolePairs    = POLES/2;  //POLES 为 8，即为4极对

    电机核心代码说明： 一个低脉冲先触发AD2S1210，再读取角度、读取速度。利用该角度与速度，为FOC算法提供参数参与计算。
                      在mainIST中断服务函数里，提供了6步FOC步骤的注释说明

###############################################################################
配套我们提供电机的旋变信号定义： 黑REF+ 黑白REF-  绿SIN+  绿白SIN- 红COS+ 红白COS-
###############################################################################


若自备旋变电机的，需要设置以下几个基础信息以及实验步骤（必须按步骤顺序）：
     1，电机极对数，在  HVPM_Sensorless-Settings.H   里，设置define,比如 4极对电机，  设置  #define POLES   8  // Number of poles
     2，暂不接电机UVW，通过手拧电机轴，看数码管显示电机是否从0-360度表示一圈，注意要连续变化才对。
     3，在当前的main文件里开头，选择设置程序开环模式。OPEN_LOOP_TEST
     4，RAM模式下载运行，按键S1-S5运行，S6停止。 切记，如果堵转、抖动，请立即断电驱动电源P601接口的（比如关闭空开）
        在不转情况下，再设置 下面第二句打开参与编译，屏蔽第一句

       qep1.ElecTheta = endat1.ElecTheta;      //获取的实时角度
       //qep1.ElecTheta = 1.0 - endat1.ElecTheta;//此处2句，只能打开一句

      5，若第4步都试过了，注意是2种角度的给定。  如果都不转的话，还是屏蔽 1.0减去ElecTheta的这句； 然后更换UVW，看哪UVW线序可以正反转的。

      以上开环可以转之后，需要进行电气零度与旋变传感器零度的偏差查找：
      6，设置电机抱轴, 下载test_IDDK_PM_Servo_F2837x_ID_LOCK

      ，手拧看动不动，确保不动很紧，再断电，断开UVW，
      7，回到这个程序，不接UVW，下载程序，读取旋变的位置反馈值（Position观测这个变量），即为偏差值
      8，将偏差值在  HVPM_Sensorless-Settings.H   里做更正，#define CALIBRATEANGLE             65439//16288

      以上完成了偏差值的查找
      9，将程序设置闭环模式的define，接上UVW，上电运行试试效果

      以下关于旋变的经验分享：
      10，旋变的接线，REF+ -之间是 30欧姆电阻，SIN 和 COS之间是60欧姆左右；依据这个判断检查一下 。通常，REF的电阻是 sin cos的一半。
      11，旋变接线如果正负接反等，会导致角度对称非正确的角度，需要调换正负线序，或者UVW需要更新线序之类的、

 * 支持有感的 开环、双闭环、三闭环模式
 * 当下载程序到FLASH时，建议使用C2prog工具通过仿真器下载hex文件，比在CCS环境下快得多。

 *
 *1，上电顺序
 * 注意：本例程未对母线电压判断，上电顺序建议如下
 *   如果DSP的5V不是通过JP2连接到驱动板：  驱动板24V--->经过隔离的220V---> DSP的5V
 *   如果DSP的通过JP2连接到驱动板取电5V：  驱动板24V--->复位DSP，2秒内通电----> 经过隔离的220V
 *
 *2，确认双闭环 还是 三闭环模式
 *#define LOOP_TYPE LOOP_2    //选择双闭环 或者 三闭环

,------------------------//双闭环程序测试方法------------------------------------------------------
   C1函数，判断按键S1， 正转，反转，停止三个状态
   可调电阻调速
   A0函数调用的串口发送函数，注意使用串口图形软件配合使用，具体看DrawLine函数，串口图形软件的配置通道数

-----------------------//三闭环程序测试方法-------------------------


 * P203接电机驱动板，硬件分配：
 *    电流采样IfbU  D3
 *            IfbV  D1
*    可调电阻IR_AD  AIN15
*
*
//以下为官方注释，有所修改，仅参考看，不做程序依据
System Name:  	IDDK PMSM Servo Control

File Name:	  	IDDK_PM_Servo_F2837x.C

Target:			Rev 1.3 of the F2837x control card

Mother board :  Industrial Drive Development Kit - IDDK - R2.2.1

Author:			C2000 Systems Lab, 06th Novemeber 2015

Description:	Project uses the following ISRs
				1. Resolver ISR triggered by ADC INT, this is defined as the
				     high priority interrupt and runs at 160Khz
				2. Inverter ISR rate is 10Khz and is triggered by EPM11,
				     reason to choose EPWM11 is because of the SD reading
				     EPWM11 provides periodic sync resets to the SDFM module,
				     after each reset the SDFM data is ready after 3 OSRs and
				     needs to be read immediately.
				3. SPI ISR --> for EnDAT and BiSS position encoder interface

===========================================================================  */
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments �2004-2015
//----------------------------------------------------------------------------------
//  Revision History:
//  v1.0 - Initial Release
//  v2.0 - Addition of EnDAT absolute encoder interface library and code
//       - Addition of BiSS-C encoder interfce library and code
//       - Bug fix in over current protection algorithm
//         - new variable 'curLimit' added, which the user can set up at actuals
//           set to 10.0 if protection needed at 10A
//       - Default code set for COLD control GND configuration
//           (SHUNT currents and voltages can be sensed in HOT config only)
//       - device support library changed from v150 to v170
//         - TrigRegs changed to SyncSocRegs
//
//----------------------------------------------------------------------------------
//  Date	  | Description / Status
//----------------------------------------------------------------------------------
// 4 Nov 2015  - Example project for PM BiSS-C Library Usage
//----------------------------------------------------------------------------------
/**********************************************************************************
 * Peripheral functions:
   EPWMs
		- EPWM1, EPWM2, EPWM3 ---> Inverter PWMs for phases A, B, C
		- EPWM5  ---> clk for Sigma Delta
		- EPWM6  ---> Resolver feedback sampling @ 160KHz
		- EPWM11 ---> 现在是用来进行通讯控制      。      sync SD filter windows with motor control PWMs
		- EPWM4  ---> Not available for users if EnDAT / BiSS interface is active

	SPIs
		- SPIB  ---> Not available for users if EnDAT / BiSS interface is active

	Analog to Digital Conversion channels
	  ADC A4/C+  --->  Ifb-SV
	  ADC B4/C+  ---> Ifb-SW
	  ADC A2/C+  ---> LEM V
	  ADC B2/C+  ---> LEM W
	  ADC D1     ---> R_SIN
	  ADC C1     ---> R_COS
	  ADC C3     ---> Vfb-U
	  ADC A3     ---> Vfb-V
	  ADC B3     ---> Vfb-W
	  ADC B0     ---> Vfb-Bus

Analog Signals brought in but not sampled
	  ADC C2/C+  ---> Ifb-SU
	  ADC A5     --->
	  ADC C0/C+  ---> SC-A2
	  ADC D0/C+  ---> SC-B2
	  ADC D2/C+  ---> SC-R

	  DAC-A  ---> Resolver carrier excitation
	  DAC-B  ---> General purpose display
	  DAC-C  ---> General purpose display

Note that the vectorial sum of d-q PI outputs should be less than 1.0 which refers to maximum duty cycle for SVGEN. Another duty cycle limiting factor
is current sense through shunt resistors which depends on hardware/software implementation. Depending on the application requirements 3,2 or a single
shunt resistor can be used for current waveform reconstruction. The higher number of shunt resistors allow the higher duty cycle operation and better
dc bus utilization. The users should adjust the PI saturation levels carefully during open loop tests (i.e pi_id.Umax, pi_iq.Umax and Umins) as
in project manuals. Violation of this procedure yields distorted  current waveforms and unstable closed loop operations which may damage the inverter.
 ********************************************************************************
 */
// Include header files used in the main function，define float maths and then include IQmath library

#include "IDDK_PM_Servo_F2837x-Settings.h"
#include "sci_base.h"
#include "emif.h"
#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#include "F2837xD_sysctrl.h"
#include "F28x_Project.h"
#include "AGlobalVariables.h"
#include "F2837xD_epwm.h"
#include "pid_reg.h"
#include "F2837xD_Ipc_drivers.h"

#ifdef _FLASH
#pragma CODE_SECTION(Communicate_ISR,"ramfuncs");
#pragma CODE_SECTION(ResolverISR,"ramfuncs");
#endif

#pragma INTERRUPT (ResolverISR, HPI)
#pragma INTERRUPT (Communicate_ISR, LPI)

//----上位机----//
//unsigned int Data1_C2TC1_R[20] = {0};   // mapped to GS0 of shared RAM owned by CPU02
//unsigned int Data1_C1TC2_W[20] = {0}; // mapped to GS1 of shared RAM owned by CPU01
//#pragma DATA_SECTION(Data1_C2TC1_R,  "SHARERAMGS0");        // CPU2
//#pragma DATA_SECTION(Data1_C1TC2_W,  "SHARERAMGS1");        // CPU1
int Err_x = 0;
int Err_y = 0;
//void Shared_Ram_DataWrite_C1TC2(void);

//interrupt void cpu_timer0_isr(void);
//#pragma CODE_SECTION(cpu_timer0_isr,"isrfunc");

//*****************************************************************************
// 调整了CMD文件配置以便适合用RAM仿真调试，可能导致点2次运行才从main函数开始执行
void InitIPC(void);
// TODO main
void main(void)

{
//#ifdef _FLASHh
//// Copy time critical code and Flash setup code to RAM，RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart symbols are created by the linker. Refer to the linker files.
//    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);
//#endif
	InitSysCtrl();	//  Initialize System Control: PLL, WatchDog, enable Peripheral Clocks，function is found in the F28M3Xx_SysCtrl.c file.
//	IPCBootCPU2(0x0000000B);
	//#ifdef _FLASH	// Only used if running from FLASH, Note that the variable FLASH is defined by the compiler
//	InitFlash();	// Call the flash wrapper init function
//#endif //(FLASH)
	DINT;	// Disable CPU interrupts,DINT是对全部的中断使能EINT或禁用DINT的作用
	InitPieCtrl(); // Initialize the PIE control registers to their default state.The default state is all PIE interrupts disabled and flags are cleared.
	IER = 0x0000;	// CPU的中断使能寄存器 interrupt enable register
	IFR = 0x0000;	// CPU的中断标志寄存器 interrupt flag enable register
	InitPieVectTable();

//----- TODO 各种初始化 -------
	Init_PM();
	Init_EPwm();            // 原程序中的主伺服的周期在这里设置，ISR_FREQUENCY 10KHz，改周期的话需要更改这个设置，现已改为1KHz，作为通讯用
	Init_DataLog();
	ConfigureADC();         //Configure the ADC and power it up
    Init_Random();
	Init_ADC();
	Init_DAC();
//	Init_SD();
	Init_QEP();             // Init QEP parameters，增量编码器需要和电机实际配置一致,使能IPM
	Init_LED();
//	HVDMC_Protection();     // Call HVDMC Protection function 驱动板R31电阻一定要焊接，否则过流保护无效！！！
	OffSet_Calibration();   // 这个主要是电流模块的偏置校正，未启动状态下读取初始值作为偏差值
	SCI_Init();             // 注意看该函数的实际配置
//	EMIF_Init();            // 外部总线EMIF初始化，74HC139后的CS2控制PCS,EMIF中会用到GPIO85引脚，因此取消注释后SCIARX用不了
//	AD2S1210_Init();
	InitCpuTimers();   // For this example, only initialize the Cpu Timers
//	IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);

	EALLOW;
	DevCfgRegs.CPUSEL5.bit.SCI_A = 1;   // 1表示连接到CPU2，0表示连接到CPU1
    DevCfgRegs.CPUSEL5.bit.SCI_B = 1;   // 1表示连接到CPU2，0表示连接到CPU1
    DevCfgRegs.CPUSEL5.bit.SCI_C = 1;   // 1表示连接到CPU2，0表示连接到CPU1
	DevCfgRegs.CPUSEL5.bit.SCI_D = 1;   // 1表示连接到CPU2，0表示连接到CPU1
//	ClkCfgRegs.CLKSEM.all = 0xA5A50001;
//	ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0; // Set LSPCLK equal to SYSCLK/1 = 200MHz

	EDIS;
	while( !(MemCfgRegs.GSxMSEL.bit.MSEL_GS0 &
	             MemCfgRegs.GSxMSEL.bit.MSEL_GS14))
	    {
	        EALLOW; // 默认应该GSx都是CPU1控制的，这里将0和14的权限给CPU2
	        MemCfgRegs.GSxMSEL.bit.MSEL_GS0 = 1;//0:CPU1 is master for this memory
	                                            //1:CPU2 is master for this memory
	        MemCfgRegs.GSxMSEL.bit.MSEL_GS14 = 1;
	        EDIS;
	    }
  memcpy(&isrfuncRunStart, &isrfuncLoadStart, (uint32_t)&isrfuncLoadSize);

//TODO ISR Mapping
	EALLOW;
// ADC C EOC of SOC0 is used to trigger Resolver Interrupt
	AdccRegs.ADCINTSEL1N2.bit.INT1SEL  = 0;
	AdccRegs.ADCINTSEL1N2.bit.INT1CONT = 1;
	AdccRegs.ADCINTSEL1N2.bit.INT1E    = 1;

// ePWM11 INT is used to trigger Motor Control ISR
	EPwm11Regs.ETSEL.bit.INTSEL = ET_CTRU_CMPA;   	// INT on PRD event
	EPwm11Regs.ETSEL.bit.INTEN  = 1;              	// Enable INT
	EPwm11Regs.ETPS.bit.INTPRD  = ET_1ST;         	// Generate INT on every event

	PieVectTable.TIMER0_INT  = &MainServe_Timer0_M1_ISR;
	PieVectTable.TIMER1_INT  = &MainServe_Timer1_M2_ISR;
//	PieVectTable.ADCC1_INT   = &ResolverISR;
	PieVectTable.EPWM11_INT  = &Communicate_ISR;	// 通讯中断程序
/*
    PieVectTable.SCIB_RX_INT = &UartB_Rx_isr;    	// 中断配置步骤-----上位机通讯
    PieVectTable.SCIB_TX_INT = &UartB_Tx_isr;    	// 这个发送中断没用上
*/

//    PieVectTable.SCIC_RX_INT = &UartC_Rx_isr;       //中断配置步骤-----编码器通讯
    PieVectTable.SPIA_RX_INT = &SPIARXFIFO;
    PieVectTable.SPIB_RX_INT = &SPIBRXFIFO;

    ConfigCpuTimer(&CpuTimer0, 200, 100);    // 配置CPUTimer0， 每最后值个us 中断一次
    ConfigCpuTimer(&CpuTimer1, 200, 100);
    CpuTimer0Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0,启动定时器0
    CpuTimer1Regs.TCR.all = 0x4000; // Use write-only instruction to set TSS bit = 0,启动定时器1

//    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER1.bit.INTx7  = 1;              // Timer0
    PieCtrlRegs.PIEIER3.bit.INTx11 = 1;  			 // Enable PWM11INT in PIE group 3
    PieCtrlRegs.PIEIER9.bit.INTx3  = 1;         	 // 中断配置步骤-----3  RXB
    PieCtrlRegs.PIEIER9.bit.INTx4  = 1;         	 // 中断配置步骤-----3  TXB
//    PieCtrlRegs.PIEIER8.bit.INTx5  = 1;              // 接收中断配置步骤---- RXC
//    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;              // Enable the PIE block
    PieCtrlRegs.PIEIER6.bit.INTx1 = 1;              // Enable PIE Group 6, INT 1  接收中断
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;              // Enable PIE Group 6, INT 3  接收中断

//#if POSITION_ENCODER == RESOLVER_POS_ENCODER
//	PieCtrlRegs.PIEIER1.bit.INTx3  = 1;  			// Enable ADCC1INT in PIE group 1
//#endif

	EPwm11Regs.ETCLR.bit.INT=1;
	IER |= M_INT1;      // 定时器0
	IER |= M_INT3; 		// Enable group 3 interrupts
//	IER |= M_INT8;      // SCICD的接收和发送中断使能
//    IER |= M_INT9; 		// SCIAB的接收和发送中断使能
    IER |= M_INT13;     // 定时器1
    IER |= 0x20;                            // Enable CPU INT6
	EINT;          		// Enable Global interrupt INTM
	ERTM;          		// Enable Global realtime interrupt DBGM
	EDIS;

//TODO 主循环
//    Shared_Ram_DataWrite_C1TC2();
    IPCLtoRFlagSet(IPC_FLAG10);
	for(;;)
	{

//	    Motors_Start.Motor1 = M1_M2_START;
//	    Motors_Start.Motor2 = M1_M2_START;
//	    Motors_Loop.Motor1 = M1_M2_LOOP;
//        Motors_Loop.Motor2 = M1_M2_LOOP;

        if(IPCLtoRFlagBusy(IPC_FLAG10) == 0)
        {
// Write an array to a memory location owned by CPU01
//            Shared_Ram_DataWrite_C1TC2();
// Set a flag to notify CPU02 that data is available
            IPCLtoRFlagSet(IPC_FLAG10);
        }
	}
}

// Write a pattern to an array in shared RAM
//void Shared_Ram_DataWrite_C1TC2(void)
//{
//    memcpy(Pos_1, &M1Data.Pos_MechTheta, sizeof(M1Data.Pos_MechTheta));
//    memcpy(Pos_2, &M2Data.Pos_MechTheta, sizeof(M2Data.Pos_MechTheta));
//    memcpy(Pos_3, &M1Data.Speed, sizeof(M1Data.Speed));
//    memcpy(Pos_4, &M2Data.Speed, sizeof(M2Data.Speed));
//    Pos_3_temp = (float)Data1_C2TC1_R[2];
//    memcpy(Pos_3, &Pos_3_temp, sizeof(Pos_3_temp));

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
/*    memcpy(Pos_1, &M1Data.Pos_MechTheta, sizeof(M1Data.Pos_MechTheta));
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
    Data1_C1TC2_W[5] = 0x12*256 + Pos_temp2[0];
    Data1_C1TC2_W[6] = Pos_temp2[1]*256 + Pos_temp2[2];
    Data1_C1TC2_W[7] = Pos_temp2[3]*256 + 0x0D;*/

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
//    Data1_C1TC2_W[1] = 360.0;
//    Data1_C1TC2_W[2] = M1Data.Pos_MechTheta*10;
//    Data1_C1TC2_W[3] = M2Data.Pos_MechTheta*10;
//    Data1_C1TC2_W[4] = M2Data.Speed_Ref*10000;
//    Data1_C1TC2_W[5] = M2Data.Speed*10000;
//    Data1_C1TC2_W[6] = 0x0000;
//    Data1_C1TC2_W[7] = (Data1_C1TC2_W[0]^Data1_C1TC2_W[1]^Data1_C1TC2_W[2]^Data1_C1TC2_W[3]^Data1_C1TC2_W[4]^Data1_C1TC2_W[5]^Data1_C1TC2_W[6])&0XFFFF;//&0XFFFF; // 校验位
//}


//void InitIPC(void)
//{
//    IPCInitialize(&g_sIpcController1, IPC_INT0, IPC_INT0);
//    IPCInitialize(&g_sIpcController2, IPC_INT1, IPC_INT1);
//    Uint16 counter;
//    pulMsgRam = (void *)CPU01TOCPU02_PASSMSG;
//    pulMsgRam2 = (void *)CPU02TOCPU01_PASSMSG;
//    pusCPU01BufferPt = (void *)GS0SARAM_START;
//    pusCPU01BufferPt_B = (void *)(GS0SARAM_START + 16);
//    pulMsgRam[0] = (Uint32)&usCPU01Buffer[0];
//    pulMsgRam[2] = (Uint32)&usCPU01Buffer_B[0];
//    for(counter = 0; counter <8; counter++)
//    {
//        usCPU01Buffer[counter] = 0.0;
//        usCPU01Buffer_B[counter] = 0.0;
//    }
//    usCPU01Buffer[0]=3.14;
//}

