//###########################################################################
//
//  这个版本是RS422版本，可以不用时间约束，全双工，很方便
//
// FILE:   adc_epwm_cpu02.c
// TITLE:  Example to demonstrate ADCA and EPWM1 on CPU2
//
// This example demonstrates how to make use of the ADC and EPWM peripherals
// from CPU2.  Device clocking (PLL) and GPIO setup are done using CPU1,
// while all other configuration of the peripherals is done using CPU2.
//
// CPU2 configures EPWM1 in up count mode in a similar fashion to what is
// done in the epwm_up_aq example.  The ADC is configured in continuous
// conversion mode similar to the adc_soc_continuous example.  GPIO0 can be
// connected to ADCINA0 and the results buffer AdcaResults graphed in CCS to
// view the duty cycle of the generated waveform.
//
//###########################################################################
// $TI Release: F2837xD Support Library v190 $
// $Release Date: Mon Feb  1 16:51:57 CST 2016 $
// $Copyright: Copyright (C) 2013-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "F2837xD_Ipc_drivers.h"
#include "AGlobalVariables.h"

unsigned int Data2_C2TC1_W[20]={0};   // mapped to GS0 of shared RAM owned by CPU02
unsigned int Data2_C1TC2_R[20]={0};     // mapped to GS1 of shared RAM owned by CPU01
#pragma DATA_SECTION(Data2_C1TC2_R,"SHARERAMGS1");         // CPU1控制
#pragma DATA_SECTION(Data2_C2TC1_W,"SHARERAMGS0");         // CPU2控制


void main(void)
{
    while(!( MemCfgRegs.GSxMSEL.bit.MSEL_GS0 &
             MemCfgRegs.GSxMSEL.bit.MSEL_GS14 ) )
    {    }
    InitSysCtrl();
    DINT;
    InitPieCtrl();
    memcpy(&isrfuncRunStart, &isrfuncLoadStart, (uint32_t)&isrfuncLoadSize);

    IER = 0x0000;
    IFR = 0x0000;
    InitCpuTimers();   // For this example, only initialize the Cpu Timers

    InitPieVectTable();

    EALLOW;
    PieVectTable.TIMER2_INT  = &cpu_timer2_isr;
	PieVectTable.SCIA_RX_INT = &UartA_Rx_isr;
    PieVectTable.SCIB_RX_INT = &UartB_Rx_isr;
	PieVectTable.SCIC_RX_INT = &UartC_Rx_isr;
	PieVectTable.SCID_RX_INT = &UartD_Rx_isr;
	EDIS;

//Power up the PWM and ADC
	EALLOW;
	CpuSysRegs.PCLKCR7.bit.SCI_B=1;
//    CpuSysRegs.PCLKCR7.bit.SCI_C=1;
//	CpuSysRegs.PCLKCR7.bit.SCI_D=1;

    SCI_Init();

//   Init_Kinco();

	ConfigCpuTimer(&CpuTimer2, 200, 2500);     // 每最后值个us 中断一次
	CpuTimer2Regs.TCR.all = 0x4000;

//	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
	PieCtrlRegs.PIEIER9.bit.INTx1  = 1;              // SCIA接收中断----3   RXA
	PieCtrlRegs.PIEIER8.bit.INTx5  = 1;              // 接收中断配置步骤---- RXC
    PieCtrlRegs.PIEIER8.bit.INTx7  = 1;              // SCID接收中断----3   RXD
    PieCtrlRegs.PIEIER9.bit.INTx3  = 1;              // 中断配置步骤-----3  RXB
//    PieCtrlRegs.PIEIER9.bit.INTx4  = 1;              // 中断配置步骤-----3  TXB



       // 这个发送中断没用上

    IER |= M_INT9;      // SCIAB的接收和发送中断使能
    IER |= M_INT8;      // SCICD的接收和发送中断使能
    IER |= M_INT14;     // 定时器2
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM
    EDIS;
    while(1)
    {
        Flag_cpu2_timer2++;
        //目标电机控制
//        if(TargetMotor_Start == 1)
//        {
//            switch (TargetMotor_Speed){
//            int i;
//            int length;
//            unsigned char command = [];
//            case 1://一级转速
//
//                command = {0x01, 0x23, 0xFF, 0x60, 0x00, 0x55, 0x35, 0x00, 0x00, 0xF3};
//                length = sizeof(command) / sizeof(command[0]);
//                for (i=0; i < length; i++)
//                {
//                    Scid_xmit(command[i]);
//                    DELAY_US(75.0);
//                }
//                break;
//            case 2://二级转速
//
//                command = {0x01, 0x23, 0xFF, 0x60, 0x00, 0xAA, 0x6A, 0x00, 0x00, 0x69};
//                length = sizeof(command) / sizeof(command[0]);
//                for (i=0; i < length; i++)
//                {
//                    Scid_xmit(command[i]);
//                    DELAY_US(75.0);
//                }
//                break;
//            case 3://三级转速
//
//                command = {0x01, 0x23, 0xFF, 0x60, 0x00, 0x00, 0xA0, 0x00, 0x00, 0xDD};
//                length = sizeof(command) / sizeof(command[0]);
//                for (i=0; i < length; i++)
//                {
//                    Scid_xmit(command[i]);
//                    DELAY_US(75.0);
//                }
//                break;
//            default:
//                break;
//            }
//
//        }
//        else if(TargetMotor_Stop == 1)
//        {
//            int i;
//            unsigned char command[] = {0x01, 0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00, 0x2E};
//            int length = sizeof(command) / sizeof(command[0]);
//            for (i=0; i < length; i++)
//            {
//                Scid_xmit(command[i]);
//                DELAY_US(75.0);
//            }
//        }
    }
}

/////TODO




