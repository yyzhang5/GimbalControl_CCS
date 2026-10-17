//-----------------------------------------------
//串口初始化
// 波特率 baud = LSPCLK/8/((BRR+1)
//            baud @LSPCLK = 37.5MHz (150 MHz SYSCLK)/4，
//           关于LSPCLK的初始化，在main函数系统初始化函数 InitSysCtrl  里的 InitPeripheralClocks 实现 LSPCLK 4分频 SYSLCK
// 115200   8N1
// P13 框图介绍SCI
// P22 1.2.10.1 SCI FIFO Description，介绍FIFO使用
//-----------------------------------------------

#include "sci_base.h"
#include "AGlobalVariables.h"
#include "F2837xD_Pie_defines.h"
#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "debug.h"

interrupt void UartB_Rx_isr(void);
interrupt void UartB_Tx_isr(void);
interrupt void UartC_Rx_isr(void);
interrupt void SPIATXFIFO(void);
interrupt void SPIARXFIFO(void);
interrupt void SPIBTXFIFO(void);
interrupt void SPIBRXFIFO(void);

void SCI_Init()
{
    SCI_GPIO();
//    SCI_InitB();
//    SCI_InitC();
    spiA_init();
    spiB_init();
}

void SCI_GPIO()
{
//	GPIO_SetupPinMux(85, GPIO_MUX_CPU2, 5);				// SCIA，后面的数字5是需要根据手册进行更改的
//	GPIO_SetupPinOptions(85, GPIO_INPUT, GPIO_PUSHPULL);
//	GPIO_SetupPinMux(84, GPIO_MUX_CPU2, 5);
//	GPIO_SetupPinOptions(84, GPIO_OUTPUT, GPIO_ASYNC);
//
//	GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 6);				// SciB Rx
//	GPIO_SetupPinOptions(55, GPIO_INPUT, GPIO_PUSHPULL);
//	GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 6);				// SciB Tx
//	GPIO_SetupPinOptions(54, GPIO_OUTPUT, GPIO_ASYNC);
//
//	GPIO_SetupPinMux(57, GPIO_MUX_CPU1, 6);				// SCIC，这个复用是第几个需要查表 RM手册P930，56 57的SCIC是6
//	GPIO_SetupPinOptions(57, GPIO_INPUT, GPIO_PUSHPULL);
//	GPIO_SetupPinMux(56, GPIO_MUX_CPU1, 6);
//	GPIO_SetupPinOptions(56, GPIO_OUTPUT, GPIO_ASYNC);
//
//    GPIO_SetupPinMux(94, GPIO_MUX_CPU2, 6);             // SCID，这个复用是第几个需要查表 RM手册P930，56 57的SCIC是6
//    GPIO_SetupPinOptions(94, GPIO_INPUT, GPIO_PUSHPULL);
//    GPIO_SetupPinMux(93, GPIO_MUX_CPU2, 6);
//    GPIO_SetupPinOptions(93, GPIO_OUTPUT, GPIO_ASYNC);

//    GPIO_SetupPinOptions(54, GPIO_OUTPUT,  GPIO_ASYNC);  // SPISOMIA

    // 对应板子上的B通道
    GPIO_SetupPinOptions(55, GPIO_INPUT,  GPIO_ASYNC);  // SPISOMIA
    GPIO_SetupPinOptions(56, GPIO_OUTPUT, GPIO_ASYNC);  // SPICLKA
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 1);  // 映射到SPI功能
    GPIO_SetupPinMux(56, GPIO_MUX_CPU1, 1);

    // 对应板子上的C通道
    GPIO_SetupPinOptions(61, GPIO_INPUT,  GPIO_ASYNC);  // SPISOMIB
    GPIO_SetupPinOptions(58, GPIO_OUTPUT, GPIO_ASYNC);  // SPICLKB
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 6);  // 映射到SPI功能
    GPIO_SetupPinMux(58, GPIO_MUX_CPU1, 6);

/*
    GPIO_SetupPinMux(12, GPIO_MUX_CPU2, 6);             // SCIC，后面的数字5是需要根据手册进行更改的
    GPIO_SetupPinOptions(12, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(13, GPIO_MUX_CPU2, 6);
    GPIO_SetupPinOptions(13, GPIO_OUTPUT, GPIO_ASYNC);*/

    // 上位机
    GPIO_SetupPinMux(86, GPIO_MUX_CPU2, 5);             // SCID，后面的数字5是需要根据手册进行更改的
    GPIO_SetupPinOptions(86, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(87, GPIO_MUX_CPU2, 5);
    GPIO_SetupPinOptions(87, GPIO_OUTPUT, GPIO_ASYNC);
}

void SCI_InitB()		//可以通讯，但是在每次发送时候要加一点时间延迟，要不然传输会有错误
{
//	LSPCLK 时钟配置  RM手册P345,LSPCLK = 200MHz
	EALLOW;
	ClkCfgRegs.LOSPCP.bit.LSPCLKDIV= 0;  // SYSCLK/1
	EDIS;

    ScibRegs.SCIHBAUD.all = 0;
    ScibRegs.SCILBAUD.all = 9;
    ScibRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback, No parity,8 char bits,

    ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
    ScibRegs.SCICCR.bit.STOPBITS = 0; // 1个停止位
//  ScibRegs.SCICCR.bit.PARITY = 0;   // 0-奇校验，1-偶校验
    ScibRegs.SCICCR.bit.PARITYENA = 0; // 奇偶检验禁止
    ScibRegs.SCICCR.bit.LOOPBKENA = 0; // 自测试模式禁止
    ScibRegs.SCICCR.bit.ADDRIDLE_MODE = 0; // 0-空闲位模式协议，1-地址位模式协议
    ScibRegs.SCICCR.bit.SCICHAR = 0x07;    // 8bits
    ScibRegs.SCICTL1.bit.RXERRINTENA = 0;  // 接收中断禁止
    ScibRegs.SCICTL1.bit.SWRESET = 0; // Reset
    ScibRegs.SCICTL1.bit.TXENA = 1;
    ScibRegs.SCICTL1.bit.RXENA = 1;

    ScibRegs.SCICTL2.bit.RXBKINTENA = 0;    // 禁止RXRDY/BRKDT中断
    ScibRegs.SCIFFTX.bit.TXFIFORESET = 0;  // 复位FIFO    TXFIFOXRESET
    ScibRegs.SCIFFTX.bit.TXFIFORESET = 1;  // 重新使能FIFO
    ScibRegs.SCIFFTX.bit.SCIFFENA = 1;      // FIFO使能
    ScibRegs.SCIFFTX.bit.TXFFIENA = 1;      //使能TXFIFO中断
//  ScibRegs.SCIFFTX.bit.TXFFIL   = 0x06;   //这两句是触发发送中断的，下面的数值小于相等上面的值时触发，有多少位数据发送
//  ScibRegs.SCIFFTX.bit.TXFFST   = 0x06;
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 0;   //复位接收FIFO
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;   //重新使能接收FIFO
    ScibRegs.SCIFFRX.bit.RXFFIENA = 1;      //使能RXFIFO中断

//  ScibRegs.SCIFFRX.bit.RXFFST = 10;       //ST>=IL才会触发中断
    ScibRegs.SCIFFRX.bit.RXFFIL = 11;        // 接收11个字节一中断

    ScibRegs.SCIFFCT.bit.FFTXDLY = 0x00;    // 两字节之间无延迟
    ScibRegs.SCICTL1.bit.SWRESET = 1;       // 重新使能SCI
}

void SCI_InitC()
{
// LSPCLK 时钟配置  RM手册P345，LSPCLK = 200MHz
	EALLOW;
	ClkCfgRegs.LOSPCP.bit.LSPCLKDIV= 0;  // SYSCLK/1,2的0次方是1，所以就是200M/1=200M
	EDIS;

    ScicRegs.SCIHBAUD.all = 0;
    ScicRegs.SCILBAUD.all = 49;
    ScicRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback, No parity,8 char bits,

    ScicRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
    ScicRegs.SCICCR.bit.STOPBITS = 0; // 1个停止位
//    ScicRegs.SCICCR.bit.PARITY = 0;   // 0-奇校验，1-偶校验
    ScicRegs.SCICCR.bit.PARITYENA = 0; // 奇偶检验禁止
    ScicRegs.SCICCR.bit.LOOPBKENA = 0; // 自测试模式禁止
    ScicRegs.SCICCR.bit.ADDRIDLE_MODE = 0; // 0-空闲位模式协议，1-地址位模式协议
    ScicRegs.SCICCR.bit.SCICHAR = 0x07;    // 8bits
    ScicRegs.SCICTL1.bit.RXERRINTENA = 0;  // 接收中断禁止
    ScicRegs.SCICTL1.bit.SWRESET = 0; // Reset
    ScicRegs.SCICTL1.bit.TXENA = 1;
    ScicRegs.SCICTL1.bit.RXENA = 1;

    ScicRegs.SCICTL2.bit.RXBKINTENA = 0;    // 禁止RXRDY/BRKDT中断
    ScicRegs.SCIFFTX.bit.TXFIFORESET = 0;  // 复位FIFO    TXFIFOXRESET
    ScicRegs.SCIFFTX.bit.TXFIFORESET = 1;  // 重新使能FIFO
    ScicRegs.SCIFFTX.bit.SCIFFENA = 1;      // FIFO使能
    ScicRegs.SCIFFTX.bit.TXFFIENA = 1;      //使能TXFIFO中断
    //  ScicRegs.SCIFFTX.bit.TXFFIL   = 0x06;   //这两句是触发发送中断的，下面的数值小于相等上面的值时触发，有多少位数据发送
    //  ScicRegs.SCIFFTX.bit.TXFFST   = 0x06;
    ScicRegs.SCIFFRX.bit.RXFIFORESET = 0;   //复位接收FIFO
    ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;   //重新使能接收FIFO
    ScicRegs.SCIFFRX.bit.RXFFIENA = 1;      //使能RXFIFO中断

//  ScicRegs.SCIFFRX.bit.RXFFST = 10;       //ST>=IL才会触发中断
    ScicRegs.SCIFFRX.bit.RXFFIL = 16;        // 接收11个字节一中断

    ScicRegs.SCIFFCT.bit.FFTXDLY = 0x00;    // 两字节之间无延迟
    ScicRegs.SCICTL1.bit.SWRESET = 1;       // 重新使能SCI

}

void Scib_xmit(int b)
{
    	Sci_Wait_cnt[1]++;
    	ScibRegs.SCITXBUF.bit.TXDT = b;	// 单次只能传8位
///////////////////////////-------------------------------------------/////////////////////////////////////////
//////////////////// ********************  以下是M新动力工程师给的例程，正确没错误 *************************
///////while (SciaRegs.SCIFFTX.bit.TXFFST != 0)
//    while(ScibRegs.SCICTL2.bit.TXEMPTY != 1)
//    {
//        if(Sci_Wait_cnt[1]++ > TIMEROUTSCI)break;
//    }
//    if(Sci_Wait_cnt[1]++ <= TIMEROUTSCI)
//        ScibRegs.SCITXBUF.all = b;

//      ScibRegs.SCIFFTX.bit.TXFFINTCLR=1; //如果在发送中断里清发送中断标志，则会一直不停地发送
//      ScibRegs.SCIFFRX.bit.RXFFINTCLR=1; //如果在发送中断里清发送中断标志，则会一直不停地发送
//    	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9; // Issue PIE ACK
}

void Scic_xmit(int a)
{
//	while (ScicRegs.SCIFFTX.bit.TXFFST != 0)
	while(ScicRegs.SCICTL2.bit.TXEMPTY != 1)
    {
    	Sci_Wait_cnt[2]++;
        if(Sci_Wait_cnt[2] > TIMEROUTSCI)
        	break;
    }
    if(Sci_Wait_cnt[2] <= TIMEROUTSCI)
    {
    	Sci_send_cnt[2]++;
    	ScicRegs.SCITXBUF.all = a;
    }
}

int Delay_Sendtime = 0.4;

void HexB_Printf(int d)
{
	Scib_xmit(d>>8);
	Scib_xmit(d);
}

void HexC_Printf(int d)
{
//  if(d<=0xff)
//  {
//      Scia_xmit(0);
//      Scia_xmit(d);
//  }
//  else
//  {
	Scic_xmit(d>>8);
	Scic_xmit(d);
//  }
}

//#################################################
//-----------------------------------------------
//发送字符串
//Enter1 字符串前的回车换行次数   Enter2 字符串后的回车换行次数
//-----------------------------------------------
void sciPutString(char Enter1,char *s, char Enter2)
{
    char i;

    for(i=0;i<Enter1;i++)
    {
        Scic_xmit(0x0D);
        Scic_xmit(0x0A);
    }

    while(*s!=0)
    {
        Scic_xmit(*(s++));
    }

    for(i=0;i<Enter2;i++)
    {
        Scic_xmit(0x0D);
        Scic_xmit(0x0A);
    }
}

/*int Rxb_flag = 0;
int RxB_cnt = 0;
int flag_Buf_ib = 0;
int flag_Buf_Rec_B = 0;
int UartB_Rx_cnt = 0;
interrupt void UartB_Rx_isr(void)
{// 外框电机角位置编码器
    UartB_Rx_cnt++;

    if (UartB_Rx_cnt>10000) UartB_Rx_cnt=0;

	while(ScibRegs.SCIFFRX.bit.RXFFST)
	{
		Rxb_flag++ ;
		for(RxB_cnt=0;RxB_cnt<Rx_Num_All;RxB_cnt++)
		{
			RxB_Buff_data[RxB_cnt]= (ScibRegs.SCIRXBUF.all) & 0x00FF;
		}
	}

	for( flag_Buf_ib=0;flag_Buf_ib<Rx_Num_All;flag_Buf_ib++)
	{
		RxB_Double_temp[flag_Buf_Rec_B]=RxB_Buff_data[flag_Buf_ib];	// 接收数组又进入另一个缓冲数组中
		flag_Buf_Rec_B++;
		if(flag_Buf_Rec_B == Rx_Num_All*2)
		{
		    flag_Buf_Rec_B=0;
		}
	}

	ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;
//	ScibRegs.SCIFFTX.bit.TXFFINTCLR=1;                      // 很重要 若不清fifo发送中断标志则，不进入发送中断
    PieCtrlRegs.PIEACK.all|=PIEACK_GROUP9;
}*/

//发送中断
/*int IMURevDataNum  = 0;
Uint16 uart_i = 0;
interrupt void UartB_Tx_isr()
{
	uart_i++;
//
//	ScibRegs.SCIFFTX.bit.TXFFINTCLR=1; //如果在发送中断里清发送中断标志，则会一直不停地发送
//	ScibRegs.SCIFFTX.bit.TXFFINTCLR=1; // Clear SCI Interrupt flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9; // Issue PIE ACK

}*/

int Rxc_flag = 0;
int RxC_cnt = 0;
int Flag_Buf_ic=0;
int Flag_Buf_Rec_C;
int UartC_Rx_cnt = 0;
interrupt void UartC_Rx_isr(void)
{
//  接收串口数据
    UartC_Rx_cnt++;
    if (UartC_Rx_cnt>10000) UartC_Rx_cnt=0;

	while(ScicRegs.SCIFFRX.bit.RXFFST)
	{
		Rxc_flag++ ;
		for(RxC_cnt=0;RxC_cnt<Rx_Num_All;RxC_cnt++)
		{
			RxC_Buff_data[RxC_cnt]= (ScicRegs.SCIRXBUF.all) & 0x00FF;
		}
	}

	for( Flag_Buf_ic=0;Flag_Buf_ic<Rx_Num_All;Flag_Buf_ic++)
	{
		RxC_Double_temp[Flag_Buf_Rec_C]=RxC_Buff_data[Flag_Buf_ic];	// 接收数组又进入另一个缓冲数组中
		Flag_Buf_Rec_C++;
		if(Flag_Buf_Rec_C == Rx_Num_All*2)
		{
			Flag_Buf_Rec_C=0;
		}
	}

//    ScicRegs.SCIFFTX.bit.TXFFINTCLR=1;   // 很重要 若不清fifo发送中断标志则，不进入发送中断
    ScicRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=PIEACK_GROUP8;
}

long TxC_time = 1.7;
long TxC_time_cnt = 0;
void Send_To_Encoder_M1(int CF_M1)
{
    TxC_time_cnt++;
//    GpioDataRegs.GPASET.bit.GPIO13 = 1;             // RS485C 内框
//    DELAY_US(0.4);
//    ScicRegs.SCITXBUF.all=CF_M1; // Send data SCIC     内框架
//    DELAY_US(TxC_time);
//    GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;
}
void Send_To_Encoder_M2(int CF_M2)
{
    TxC_time_cnt++;
//    GpioDataRegs.GPASET.bit.GPIO12 = 1;             // RS485B
//    DELAY_US(0.4);
//    ScibRegs.SCITXBUF.all=CF_M2; // Send data SCIB     外框架
//    DELAY_US(TxC_time);
//    GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;
}

int CRC_Enc_Pos[9] = {0};
float Temp_Pos_Encoder = 0.0;
int Flag_textout = 0;
int Flag_textin = 0;
int flag_getpos = 0;
double nTemp = 0;
float M1_BIT_Pos_Encoder=0.0;
float Temp_M1_BIT_Pos_Encoder = 0.0;
float Get_Pos_Encoder_M1(float Temp_Pos_Encoder_Err)
{
//// 以下程序是23位精度的编码器通讯
    if (RxC_Double_temp[flag_getpos] == 0x001A && RxC_Double_temp[flag_getpos+1]== 0x0020)
    {
        Flag_textout++;
        CRC_Enc_Pos[0] = (RxC_Double_temp[flag_getpos]^RxC_Double_temp[flag_getpos+1]);
        CRC_Enc_Pos[1] = (CRC_Enc_Pos[0])^(RxC_Double_temp[flag_getpos+2]);
        CRC_Enc_Pos[2] = (CRC_Enc_Pos[1])^(RxC_Double_temp[flag_getpos+3]);
        CRC_Enc_Pos[3] = (CRC_Enc_Pos[2])^(RxC_Double_temp[flag_getpos+4]);
        CRC_Enc_Pos[4] = (CRC_Enc_Pos[3])^(RxC_Double_temp[flag_getpos+5]);
        CRC_Enc_Pos[5] = (CRC_Enc_Pos[4])^(RxC_Double_temp[flag_getpos+6]);
        CRC_Enc_Pos[6] = (CRC_Enc_Pos[5])^(RxC_Double_temp[flag_getpos+7]);
        CRC_Enc_Pos[7] = (CRC_Enc_Pos[6])^(RxC_Double_temp[flag_getpos+8]);
        CRC_Enc_Pos[8] = (CRC_Enc_Pos[7])^(RxC_Double_temp[flag_getpos+9]);

        if (RxC_Double_temp[flag_getpos+10]==CRC_Enc_Pos[8])
        {
            Flag_textin++;
            nTemp = (RxC_Double_temp[flag_getpos+2]) + RxC_Double_temp[flag_getpos+3] * 256 + (RxC_Double_temp[flag_getpos+4] & 0x007F) * 65536;
            Temp_M1_BIT_Pos_Encoder = nTemp/8388608* 360+ Temp_Pos_Encoder_Err;
        }
    }
    flag_getpos++;
    if (flag_getpos > 11)
    {
        flag_getpos = 0;
    }

    if (Temp_M1_BIT_Pos_Encoder>360)
        M1_BIT_Pos_Encoder = Temp_M1_BIT_Pos_Encoder-360;
    else if (Temp_M1_BIT_Pos_Encoder<0)
        M1_BIT_Pos_Encoder = Temp_M1_BIT_Pos_Encoder+360;
    else
        M1_BIT_Pos_Encoder = Temp_M1_BIT_Pos_Encoder;

	return (360.0-M1_BIT_Pos_Encoder);
}

//--------------------- 外框电机 ----------------------------------
int CRC_Enc_Pos2[9] = {0};
int flag_getpos2 = 0;
double nTemp2 = 0;
float M2_BIT_Pos_Encoder=0.0;
float Temp_M2_BIT_Pos_Encoder = 0.0;
float Get_Pos_Encoder_M2(float Temp_Pos_Encoder_Err2)
{
    if (RxB_Double_temp[flag_getpos2] == 0x001A && RxB_Double_temp[flag_getpos2+1]== 0x0020)
    {
        CRC_Enc_Pos2[0] = (RxB_Double_temp[flag_getpos2]^RxB_Double_temp[flag_getpos2+1]);
        CRC_Enc_Pos2[1] = (CRC_Enc_Pos2[0])^(RxB_Double_temp[flag_getpos2+2]);
        CRC_Enc_Pos2[2] = (CRC_Enc_Pos2[1])^(RxB_Double_temp[flag_getpos2+3]);
        CRC_Enc_Pos2[3] = (CRC_Enc_Pos2[2])^(RxB_Double_temp[flag_getpos2+4]);
        CRC_Enc_Pos2[4] = (CRC_Enc_Pos2[3])^(RxB_Double_temp[flag_getpos2+5]);
        CRC_Enc_Pos2[5] = (CRC_Enc_Pos2[4])^(RxB_Double_temp[flag_getpos2+6]);
        CRC_Enc_Pos2[6] = (CRC_Enc_Pos2[5])^(RxB_Double_temp[flag_getpos2+7]);
        CRC_Enc_Pos2[7] = (CRC_Enc_Pos2[6])^(RxB_Double_temp[flag_getpos2+8]);
        CRC_Enc_Pos2[8] = (CRC_Enc_Pos2[7])^(RxB_Double_temp[flag_getpos2+9]);

        if (RxB_Double_temp[flag_getpos2+10]==CRC_Enc_Pos2[8])
        {
            nTemp2 = (RxB_Double_temp[flag_getpos2+2]) + RxB_Double_temp[flag_getpos2+3] * 256 + (RxB_Double_temp[flag_getpos2+4] & 0x007F) * 65536;
            Temp_M2_BIT_Pos_Encoder = nTemp2/8388608* 360+ Temp_Pos_Encoder_Err2;
        }
    }

    flag_getpos2++;
    if (flag_getpos2 > 11)
    {
        flag_getpos2 = 0;
    }

    if (Temp_M2_BIT_Pos_Encoder>360)
        M2_BIT_Pos_Encoder = Temp_M2_BIT_Pos_Encoder - 360;
    else if (Temp_M2_BIT_Pos_Encoder<0)
        M2_BIT_Pos_Encoder = Temp_M2_BIT_Pos_Encoder + 360;
    else
        M2_BIT_Pos_Encoder = Temp_M2_BIT_Pos_Encoder;

    return (360.0 - M2_BIT_Pos_Encoder);
}

///////////////////////////-------------------------------------------/////////////////////////////////////////
//////////////////////--------------------------------------------------------------------------------------------------
void spiA_init()
{
    EALLOW;
    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV= 2;  // SYSCLK/1  这是2的几次方分之200M
    EDIS;
// 2. 初始化SPI寄存器
    SpiaRegs.SPICCR.bit.SPISWRESET=0; // Reset SPI
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 1;    // 时钟极性(翻转电平)
    SpiaRegs.SPICCR.bit.SPICHAR = 47;       // 理论是32位传输(24+8)，但是这里设置最大是0~F，也就是0~15
    SpiaRegs.SPICCR.bit.SPILBK=0;           // loopback内部自循环的形式

    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;      // 0 时钟在第一个沿采样，第二个沿更新；1时钟在第二个沿采样，第1个沿更新；具体上升和下降还要看时钟空闲
    SpiaRegs.SPICTL.bit.TALK = 1;    // 0禁止发送数据（复位默认），1使能发送数据
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;    // 主机模式
    SpiaRegs.SPICTL.bit.TALK = 1;            // 使能发送
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;       // 标准相位
    SpiaRegs.SPICTL.bit.MASTER_SLAVE=1;
    SpiaRegs.SPICTL.bit.SPIINTENA = 1;       // 使能SPI中断
    SpiaRegs.SPICTL.bit.OVERRUNINTENA = 1;   // 接收溢出中断使能

    SpiaRegs.SPISTS.all=0x0000;

    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = 24;  // 1MHz波特率，这个值是0~2,3~127，分别对应LSPCLK/4，和LSPCLK/（SPI_BIT_RATE+1）

    SpiaRegs.SPIFFTX.bit.TXFFIL = 15;       // 4:0 TXFIFO Interrupt Level
    SpiaRegs.SPIFFTX.bit.TXFFIENA = 1;      // 5 TXFIFO Interrupt Enable
    SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 0;    // 6 TXFIFO Interrupt Clear
    SpiaRegs.SPIFFTX.bit.TXFFINT = 0;       // 7 TXFIFO Interrupt Flag
    SpiaRegs.SPIFFTX.bit.TXFFST = 0;        // 12:8 Transmit FIFO Status
    SpiaRegs.SPIFFTX.bit.TXFIFO = 0;        // 13 TXFIFO Reset
    SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;      // 14 FIFO Enhancements Enable
    SpiaRegs.SPIFFTX.bit.SPIRST = 1;        // 15 SPI Reset

    SpiaRegs.SPIFFRX.bit.RXFFIL = 15;       // 4 = 0 RXFIFO Interrupt Level
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;      // 5 RXFIFO Interrupt Enable
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;    // 6 RXFIFO Interupt Clear
    SpiaRegs.SPIFFRX.bit.RXFFINT = 0;       // 7 RXFIFO Interrupt Flag
    SpiaRegs.SPIFFRX.bit.RXFFST = 0;        // 12 = 8 Receive FIFO Status
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;   // 13 RXFIFO Reset
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 0;    // 14 Receive FIFO Overflow Clear
    SpiaRegs.SPIFFRX.bit.RXFFOVF = 0;       // 15 Receive FIFO Overflow Flag

    SpiaRegs.SPIFFCT.all = 0x00;

    SpiaRegs.SPIPRI.bit.FREE = 1;

    SpiaRegs.SPICCR.bit.SPISWRESET=1;  // Enable SPI

    SpiaRegs.SPIFFTX.bit.TXFIFO=1;
    SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;
}

int SPIA_Tx_cnt = 0;
interrupt void SPIATXFIFO(void)
{
    SPIA_Tx_cnt++;
//    Flag_cnt[1]++;
//    Uint16 i;
//    for(i=0;i<2;i++)
//    {
//       SpiaRegs.SPITXBUF=sdata[i];      // Send data
//    }


    SpiaRegs.SPIFFTX.bit.TXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ACK
}
//int crcReceived=0;
int SPIA_Rx_cnt = 0;
//double ssiRawData=0;
Uint16 rdataA[7] = {0};
interrupt void SPIARXFIFO(void)
{
    SPIA_Rx_cnt++;
    Uint16 i;
    for(i=0;i<3;i++)
    {
        rdataA[i] = SpiaRegs.SPIRXBUF & 0xFFFF;     // Read data
    }
//        switch (crcReceived)
//        {
//        case 0:
//            rdata[0]= SpiaRegs.SPIRXBUF ;
//            break;
//        case 1:
//            rdata[1]=SpiaRegs.SPIRXBUF;
//            break;
//        case 2:
//            rdata[2]=SpiaRegs.SPIRXBUF;
//            break;
//        default:
//            rdata[0]=0;
//            break;
//        }
//
//        crcReceived++;
//        if(crcReceived>1)
//        {
//            crcReceived = 0;
//        }


    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ack
}

void spiA_xmit(Uint16 a)
{
    SpiaRegs.SPITXBUF=a;
}

int SSIA_CNT=0;
void Read_SSIA_Encoder(void)
{
    SSIA_CNT++;
    SpiaRegs.SPITXBUF = 0xFFFF;  // 发送哑数据触发时钟
    SpiaRegs.SPITXBUF = 0xFFFF;  // 发送哑数据触发时钟
    SpiaRegs.SPITXBUF = 0xFFFF;  // 发送哑数据触发时钟
//    SpiaRegs.SPITXBUF = 0xFFFF;  // 发送哑数据触发时钟
    DELAY_US(20);
}

// Step 7. Insert all local Interrupt Service Routines (ISRs) and functions here:
//Uint8 CRC_Rx=0;
double PosA_8 = 0;
double PosA_16 = 0;
double PosA_24 = 0;
//Uint32 CRC1=0;
//Uint32 CRC2=0;
//Uint8 CRC_F = 0;
//#define CRC6_POLY 0x43
//int iii = 29;
//int CRC_Flag = 0;
//int CRC_Flag_cnt[2] = {0};
double Deal_loopA(float Temp_Pos_Encoder_Err)
{
//    CRC_Rx = (rdata[1]&0x001F)<<1 | (rdata[2]&0x8000)>>15;
//
//    CRC1 = ((Uint32)(rdata[0]&0x7FFF))<<11 | ((Uint32)(rdata[1]))>>5;   // 共26位
//
//   CRC2 = CRC1 << 6;   // 数据左移6位，为CRC计算腾出空间
//
//    for (iii=31; iii >= 0; iii--)        // 循环处理26位数据 + 6位CRC空间
//    {
//
//        if (CRC2 & (1UL << (31)))       // 检查当前MSB（最高位）是否为1，// 如果MSB为1，则与多项式进行异或（模2除法）
//        {
//            CRC2 ^= (CRC6_POLY << (iii - 6)); // 对齐多项式进行异或
//        }
//        CRC2 <<= 1;         // 左移一位处理下一位
//    }
//
//    CRC_F = (CRC2>>24) & 0x003F; // // 提取低6位作为CRC校验码右移24位得到CRC值
//
//    if (CRC_F == CRC_Rx)
//    {
//        CRC_Flag = 1;
//        CRC_Flag_cnt[1]++;
//    }
//    else
//    {
//        CRC_Flag = 0;
//        CRC_Flag_cnt[0]++;
//    }
//  if (CRC_Flag == 1)
//  {
        PosA_8 = ((rdataA[1]&0xFF80)>>7)*1.0;     // 低9位
        PosA_16 = (rdataA[0]&0x7FFF)*1.0;         // 高15位
//      Pos_24 = Pos_16*512*0.000000119*360 + Pos_8*0.000000119*360;    // 24位 0.0000000596； 23位0.000000119
//        PosA_24 = PosA_16*512*0.0000000596*360 + PosA_8*0.0000000596*360; // 24位 0.0000000596； 23位0.000000119,22位0.0000002384
        PosA_24 = (PosA_16*512*360 + PosA_8*360)/16777216 + Temp_Pos_Encoder_Err;
        if(PosA_24>360)
            PosA_24 = PosA_24 - 360;
        else if(PosA_24<0)
            PosA_24 = PosA_24 + 360;

//        }
        return (PosA_24);
}

void spiB_init()
{
    EALLOW;
    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV= 2;  // SYSCLK/1  这是2的几次方分之200M
    EDIS;
// 2. 初始化SPI寄存器
    SpibRegs.SPICCR.bit.SPISWRESET=0; // Reset SPI
    SpibRegs.SPICCR.bit.CLKPOLARITY = 1;    // 时钟极性(翻转电平)
    SpibRegs.SPICCR.bit.SPICHAR = 47;       // 理论是32位传输(24+8)，但是这里设置最大是0~F，也就是0~15
    SpibRegs.SPICCR.bit.SPILBK=0;           // loopback内部自循环的形式

    SpibRegs.SPICTL.bit.CLK_PHASE = 0;      // 0 时钟在第一个沿采样，第二个沿更新；1时钟在第二个沿采样，第1个沿更新；具体上升和下降还要看时钟空闲
    SpibRegs.SPICTL.bit.TALK = 1;    // 0禁止发送数据（复位默认），1使能发送数据
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1;    // 主机模式
    SpibRegs.SPICTL.bit.TALK = 1;            // 使能发送
    SpibRegs.SPICTL.bit.CLK_PHASE = 0;       // 标准相位
    SpibRegs.SPICTL.bit.MASTER_SLAVE=1;
    SpibRegs.SPICTL.bit.SPIINTENA = 1;       // 使能SPI中断
    SpibRegs.SPICTL.bit.OVERRUNINTENA = 1;   // 接收溢出中断使能

    SpibRegs.SPISTS.all=0x0000;

    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 24;  // 1MHz波特率，这个值是0~2,3~127，分别对应LSPCLK/4，和LSPCLK/（SPI_BIT_RATE+1）

    SpibRegs.SPIFFTX.bit.TXFFIL = 15;       // 4:0 TXFIFO Interrupt Level
    SpibRegs.SPIFFTX.bit.TXFFIENA = 1;      // 5 TXFIFO Interrupt Enable
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 0;    // 6 TXFIFO Interrupt Clear
    SpibRegs.SPIFFTX.bit.TXFFINT = 0;       // 7 TXFIFO Interrupt Flag
    SpibRegs.SPIFFTX.bit.TXFFST = 0;        // 12:8 Transmit FIFO Status
    SpibRegs.SPIFFTX.bit.TXFIFO = 0;        // 13 TXFIFO Reset
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1;      // 14 FIFO Enhancements Enable
    SpibRegs.SPIFFTX.bit.SPIRST = 1;        // 15 SPI Reset

    SpibRegs.SPIFFRX.bit.RXFFIL = 15;       // 4 = 0 RXFIFO Interrupt Level
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1;      // 5 RXFIFO Interrupt Enable
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1;    // 6 RXFIFO Interupt Clear
    SpibRegs.SPIFFRX.bit.RXFFINT = 0;       // 7 RXFIFO Interrupt Flag
    SpibRegs.SPIFFRX.bit.RXFFST = 0;        // 12 = 8 Receive FIFO Status
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0;   // 13 RXFIFO Reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 0;    // 14 Receive FIFO Overflow Clear
    SpibRegs.SPIFFRX.bit.RXFFOVF = 0;       // 15 Receive FIFO Overflow Flag

    SpibRegs.SPIFFCT.all = 0x00;

    SpibRegs.SPIPRI.bit.FREE = 1;

    SpibRegs.SPICCR.bit.SPISWRESET=1;  // Enable SPI

    SpibRegs.SPIFFTX.bit.TXFIFO=1;
    SpibRegs.SPIFFRX.bit.RXFIFORESET=1;
}

int SPIB_Tx_cnt = 0;
interrupt void SPIBTXFIFO(void)
{
    SPIB_Tx_cnt++;
//    Flag_cnt[1]++;
//    Uint16 i;
//    for(i=0;i<2;i++)
//    {
//       SpiaRegs.SPITXBUF=sdata[i];      // Send data
//    }


    SpibRegs.SPIFFTX.bit.TXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ACK
}
//int crcReceived=0;
int SPIB_Rx_cnt = 0;
//double ssiRawData=0;
Uint16 rdataB[7] = {0};
interrupt void SPIBRXFIFO(void)
{
    SPIB_Rx_cnt++;
    Uint16 i;
    for(i=0;i<3;i++)
    {
        rdataB[i] = SpibRegs.SPIRXBUF & 0xFFFF;     // Read data
    }
//        switch (crcReceived)
//        {
//        case 0:
//            rdata[0]= SpiaRegs.SPIRXBUF ;
//            break;
//        case 1:
//            rdata[1]=SpiaRegs.SPIRXBUF;
//            break;
//        case 2:
//            rdata[2]=SpiaRegs.SPIRXBUF;
//            break;
//        default:
//            rdata[0]=0;
//            break;
//        }
//
//        crcReceived++;
//        if(crcReceived>1)
//        {
//            crcReceived = 0;
//        }


    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ack
}

void spiB_xmit(Uint16 a)
{
    SpibRegs.SPITXBUF=a;
}

int SSIB_CNT=0;
void Read_SSIB_Encoder(void)
{
    SSIB_CNT++;
    SpibRegs.SPITXBUF = 0xFFFF;  // 发送哑数据触发时钟
    SpibRegs.SPITXBUF = 0xFFFF;  // 发送哑数据触发时钟
    SpibRegs.SPITXBUF = 0xFFFF;  // 发送哑数据触发时钟
//    SpiaRegs.SPITXBUF = 0xFFFF;  // 发送哑数据触发时钟
    DELAY_US(20);
}

// Step 7. Insert all local Interrupt Service Routines (ISRs) and functions here:
//Uint8 CRC_Rx=0;
double PosB_8 = 0;
double PosB_16 = 0;
double PosB_24 = 0;
//Uint32 CRC1=0;
//Uint32 CRC2=0;
//Uint8 CRC_F = 0;
//#define CRC6_POLY 0x43
//int iii = 29;
//int CRC_Flag = 0;
//int CRC_Flag_cnt[2] = {0};
double Deal_loopB(float Temp_Pos_Encoder_Err2)
{
//    CRC_Rx = (rdata[1]&0x001F)<<1 | (rdata[2]&0x8000)>>15;
//
//    CRC1 = ((Uint32)(rdata[0]&0x7FFF))<<11 | ((Uint32)(rdata[1]))>>5;   // 共26位
//
//   CRC2 = CRC1 << 6;   // 数据左移6位，为CRC计算腾出空间
//
//    for (iii=31; iii >= 0; iii--)        // 循环处理26位数据 + 6位CRC空间
//    {
//
//        if (CRC2 & (1UL << (31)))       // 检查当前MSB（最高位）是否为1，// 如果MSB为1，则与多项式进行异或（模2除法）
//        {
//            CRC2 ^= (CRC6_POLY << (iii - 6)); // 对齐多项式进行异或
//        }
//        CRC2 <<= 1;         // 左移一位处理下一位
//    }
//
//    CRC_F = (CRC2>>24) & 0x003F; // // 提取低6位作为CRC校验码右移24位得到CRC值
//
//    if (CRC_F == CRC_Rx)
//    {
//        CRC_Flag = 1;
//        CRC_Flag_cnt[1]++;
//    }
//    else
//    {
//        CRC_Flag = 0;
//        CRC_Flag_cnt[0]++;
//    }
//  if (CRC_Flag == 1)
//  {
        PosB_8 = ((rdataB[1]&0xFF80)>>7)*1.0;     // 低9位
        PosB_16 = (rdataB[0]&0x7FFF)*1.0;         // 高15位
//      Pos_24 = Pos_16*512*0.000000119*360 + Pos_8*0.000000119*360;    // 24位 0.0000000596； 23位0.000000119
//        PosA_24 = PosA_16*512*0.0000000596*360 + PosA_8*0.0000000596*360; // 24位 0.0000000596； 23位0.000000119,22位0.0000002384
        PosB_24 = (PosB_16*512*360 + PosB_8*360)/16777216 + Temp_Pos_Encoder_Err2;
        if(PosB_24>360)
            PosB_24 = PosB_24 - 360;
        else if(PosB_24<0)
            PosB_24 = PosB_24 + 360;

//        }
        return (PosB_24);
}
