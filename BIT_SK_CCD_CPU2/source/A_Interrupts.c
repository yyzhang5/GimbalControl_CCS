#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "AGlobalVariables.h"
#include "F2837xD_Ipc_drivers.h"
#include "F2837xD_Pie_defines.h"
#include "math.h"

interrupt void cpu_timer2_isr(void)
{
   // 发送数据 todo
//   Send_To_MATLAB();
//   Send_To_Kinco();
//   Send_To_Targetmotor();

   //   SCIB与MATLAB通信，获取目标像素偏差
   if (RxB_Double_temp[ParB_RS422_Flag] == 0x00AA && RxB_Double_temp[ParB_RS422_Flag+1] == 0x0055)
      {
          ParB_RS422[0] = RxB_Double_temp[ParB_RS422_Flag+2]; //控制字
          ParB_RS422[1] = RxB_Double_temp[ParB_RS422_Flag+3]; //data1
          ParB_RS422[2] = RxB_Double_temp[ParB_RS422_Flag+4]; //data1
          ParB_RS422[3] = RxB_Double_temp[ParB_RS422_Flag+5]; //data1
          ParB_RS422[4] = RxB_Double_temp[ParB_RS422_Flag+6]; //data1
          ParB_RS422[5] = RxB_Double_temp[ParB_RS422_Flag+7]; //data2
          ParB_RS422[6] = RxB_Double_temp[ParB_RS422_Flag+8]; //data2
          ParB_RS422[7] = RxB_Double_temp[ParB_RS422_Flag+9]; //data2
          ParB_RS422[8] = RxB_Double_temp[ParB_RS422_Flag+10]; //data2
          ParB_RS422[9] = RxB_Double_temp[ParB_RS422_Flag+11]; //校验位

          if(ParB_RS422[9] == 0x000D)
          {
              data1Bytes = 0;
              data1Bytes |= (ParB_RS422[1]);
              data1Bytes |= (ParB_RS422[2] << 8);
              data1Bytes |= (ParB_RS422[3] << 16);
              data1Bytes |= ParB_RS422[4] << 24;

              data2Bytes = 0;
              data2Bytes |= (ParB_RS422[5]);
              data2Bytes |= (ParB_RS422[6] << 8);
              data2Bytes |= (ParB_RS422[7] << 16);
              data2Bytes |= (ParB_RS422[8] << 24);

              // 将 uint32_t 数据转回 single 类型
              SCIB_Data1 = *(float*)&data1Bytes;
              SCIB_Data2 = *(float*)&data2Bytes;

              Data2_C2TC1_W[0] = SCIB_Data1;
              Data2_C2TC1_W[1] = SCIB_Data2;
              Data2_C2TC1_W[2] = 1;
          }
      }

    //   SCIB与MATLAB通信

    ParB_RS422_Flag++;
    if(ParB_RS422_Flag>=12) ParB_RS422_Flag = 0;


    // 与CPU1间的通讯标志
    if(IPCRtoLFlagBusy(IPC_FLAG10) == 1)
    {
        IPCRtoLFlagAcknowledge (IPC_FLAG10);
    }
    PieCtrlRegs.PIEACK.all  = PIEACK_GROUP1;

}

/*void Send_To_MATLAB()
{
    int ii;
    for(ii=0;ii<9;ii++)
    {
        HexA_Printf(Data2_C1TC2_R[ii]);
        DELAY_US(Delay_Time_SciA);
    }
    HexA_Printf(Data2_C1TC2_R[10]);//脉冲标志位
    DELAY_US(Delay_Time_SciA);
    HexA_Printf(Data2_C1TC2_R[9]);//校验位
    DELAY_US(Delay_Time_SciA);
}*/
int flag_send_matlab = 0;
int send_data = 2;
void Send_To_MATLAB()
{
    int ii;
    flag_send_matlab++;
    for(ii=0;ii<4;ii++) //发送0x AA 55(2bit) x轴角位置数据(控制字1bit+数据为4bit) 校验位0D(1bit) =8bit=2*4
    {
//        ScibRegs.SCITXBUF.bit.TXDT = send_data;
        HexB_Printf(Data2_C1TC2_R[ii]);
        DELAY_US(Delay_Time_SciB);
//        DELAY_US(80);

    }
//    HexC_Printf(Data2_C1TC2_R[10]);//脉冲标志位
//    DELAY_US(Delay_Time_SciC);
//    HexC_Printf(Data2_C1TC2_R[9]);//校验位
//    DELAY_US(Delay_Time_SciC);
}


//void Send_To_Targetmotor()
//{
//    int i;
//    unsigned char command[] = {0x01, 0x40, 0x63, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC};
//    int length = sizeof(command) / sizeof(command[0]);
//    for (i=0; i < length; i++)
//    {
//        Scid_xmit(command[i]);
//        DELAY_US(75.0);
//    }
//}//向带动目标旋转的电机发送读取位置的指令

//void Send_To_Kinco()
//{
//    if(Kinco.state != Kinco.state_pre){
//        switch (Kinco.state) {
//        // 电机关闭
//        case 0:
//            HexD_Printf(0x012B);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x4060);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x0006);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x0000);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x002E);
//            DELAY_US(Delay_Time_SciD);
//            break;
//        // 电机打开
//        case 1:
//            HexD_Printf(0x012B);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x4060);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x000F);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x0000);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x0025);
//            DELAY_US(Delay_Time_SciD);
//            break;
//        default:
//            HexD_Printf(0x012B);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x4060);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x0006);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x0000);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x002E);
//            DELAY_US(Delay_Time_SciD);
//        }
//        Kinco.state_pre = Kinco.state;
//    }
//    if(Kinco.speed != Kinco.speed_pre){
//        // 速度值 = [(RPM * 512 * 10000)/1875]
//        switch (Kinco.speed) {
//        // 电机关闭
//        case 0:
//            HexD_Printf(0x0123);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0xFF60);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x0000);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x0000);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x007D);
//            DELAY_US(Delay_Time_SciD);
//            break;
//        case 5:
//            HexD_Printf(0x0123);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0xFF60);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x0055);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x3500);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x00F3);
//            DELAY_US(Delay_Time_SciD);
//            break;
//        case 10:
//            HexD_Printf(0x0123);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0xFF60);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x00AA);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x6A00);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x0069);
//            DELAY_US(Delay_Time_SciD);
//            break;
//        case 15:
//            HexD_Printf(0x0123);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0xFF60);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x0000);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0xA000);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x00DD);
//            DELAY_US(Delay_Time_SciD);
//            break;
//        default:
//            HexD_Printf(0x0123);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0xFF60);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x0000);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x0000);
//            DELAY_US(Delay_Time_SciD);
//            HexD_Printf(0x007D);
//            DELAY_US(Delay_Time_SciD);
//        }
//        Kinco.speed_pre = Kinco.speed;
//    }
//
//    ScidRegs.SCIFFRX.bit.RXFIFORESET = 0;   //复位接收FIFO
//    ScidRegs.SCIFFRX.bit.RXFIFORESET = 1;   //重新使能接收FIFO
//
//    HexD_Printf(0x0140);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0x6360);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0x0000);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0x0000);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0x00FC);
//    DELAY_US(Delay_Time_SciD);
//}

interrupt void UartA_Rx_isr(void)
{
    Rxa_flag++ ;
//  接收串口数据
    while(SciaRegs.SCIFFRX.bit.RXFFST)
    {
        for(RxA_cnt=0;RxA_cnt<12;RxA_cnt++)
        {
            RxA_Buff_data[RxA_cnt]= (SciaRegs.SCIRXBUF.all) & 0x00FF;
        }
        DELAY_US(Rec_DELAY);
    }

    for( Flag_Buf_ia=0;Flag_Buf_ia<12;Flag_Buf_ia++)
    {
        RxA_Double_temp[Flag_Buf_ia]=RxA_Buff_data[Flag_Buf_ia]; // 接收数组又进入另一个缓冲数组中
//        Flag_Buf_Rec_A++;
//        if(Flag_Buf_Rec_A == 8*2)
//        {
//            Flag_Buf_Rec_A=0;
//        }
    }

    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;      // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=PIEACK_GROUP9;
}

interrupt void UartB_Rx_isr(void)
{
    Rxb_flag++ ;
//  接收串口数据
    while(ScibRegs.SCIFFRX.bit.RXFFST)
    {
        for(RxB_cnt=0;RxB_cnt<12;RxB_cnt++)
        {
            RxB_Buff_data[RxB_cnt]= (ScibRegs.SCIRXBUF.all) & 0x00FF;
        }
        DELAY_US(Rec_DELAY);
    }

    for( Flag_Buf_ib=0;Flag_Buf_ib<12;Flag_Buf_ib++)
    {
        RxB_Double_temp[Flag_Buf_ib]=RxB_Buff_data[Flag_Buf_ib]; // 接收数组又进入另一个缓冲数组中
    }

    ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;      // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=PIEACK_GROUP9;
}

interrupt void UartC_Rx_isr(void)
{
    Rxc_flag++ ;
//  接收串口数据
    while(ScicRegs.SCIFFRX.bit.RXFFST)
    {
        for(RxC_cnt=0;RxC_cnt<12;RxC_cnt++)
        {
            RxC_Buff_data[RxC_cnt]= (ScicRegs.SCIRXBUF.all) & 0x00FF;
        }
        DELAY_US(Rec_DELAY);
    }

    for( Flag_Buf_ic=0;Flag_Buf_ic<12;Flag_Buf_ic++)
    {
        RxC_Double_temp[Flag_Buf_ic]=RxC_Buff_data[Flag_Buf_ic]; // 接收数组又进入另一个缓冲数组中
//        Flag_Buf_Rec_A++;
//        if(Flag_Buf_Rec_A == 8*2)
//        {
//            Flag_Buf_Rec_A=0;
//        }
    }

    ScicRegs.SCIFFRX.bit.RXFFINTCLR=1;      // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=PIEACK_GROUP8;
}

interrupt void UartD_Rx_isr(void)
{
    Rxd_flag++ ;
//  接收串口数据
    while(ScidRegs.SCIFFRX.bit.RXFFST)
    {
        for(RxD_cnt=0;RxD_cnt<12;RxD_cnt++)
        {
            RxD_Buff_data[RxD_cnt]= (ScidRegs.SCIRXBUF.all) & 0x00FF;
        }
        DELAY_US(Rec_DELAY);
    }

    for( Flag_Buf_id=0;Flag_Buf_id<10;Flag_Buf_id++)
    {
        RxD_Double_temp[Flag_Buf_id]=RxD_Buff_data[Flag_Buf_id]; // 接收数组又进入另一个缓冲数组中
//        Flag_Buf_Rec_A++;
//        if(Flag_Buf_Rec_A == 8*2)
//        {
//            Flag_Buf_Rec_A=0;
//        }
    }

    ScidRegs.SCIFFRX.bit.RXFFINTCLR=1;      // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all|=PIEACK_GROUP8;
}


void HexA_Printf(int a)
{
//    Delay_Sendtime = 20*(SciaRegs.SCIHBAUD.all *256 +SciaRegs.SCILBAUD.all+1)*8/200+3;  // 根据波特率计算每次发送数据用时，需要进行延迟
//  采用RS422全双工发送，无需进行Delay_Sendtime延时设计
    Scia_xmit(a>>8);
    Scia_xmit(a);
}

void Scia_xmit(int a)
{
        SciaRegs.SCITXBUF.bit.TXDT = a;
}

void HexB_Printf(int b)
{
//    Delay_Sendtime = 20*(SciaRegs.SCIHBAUD.all *256 +SciaRegs.SCILBAUD.all+1)*8/200+3;  // 根据波特率计算每次发送数据用时，需要进行延迟
//  采用RS422全双工发送，无需进行Delay_Sendtime延时设计
    Scib_xmit(b>>8);
    Scib_xmit(b);
}

void Scib_xmit(int b)
{
        ScibRegs.SCITXBUF.bit.TXDT = b;
}

void HexC_Printf(int a)
{
//    Delay_Sendtime = 20*(SciaRegs.SCIHBAUD.all *256 +SciaRegs.SCILBAUD.all+1)*8/200+3;  // 根据波特率计算每次发送数据用时，需要进行延迟
//  采用RS422全双工发送，无需进行Delay_Sendtime延时设计
    Scic_xmit(a>>8);
    Scic_xmit(a);
}

void Scic_xmit(int a)
{
        ScicRegs.SCITXBUF.bit.TXDT = a;
}

void SCI_Init()
{
    SCI_GPIO();
//    SCI_InitA();
    SCI_InitB();
//    SCI_InitD();//用于接收带动目标旋转的电机位置
}

void SCI_GPIO()
{
//    GPIO_SetupPinMux(85, GPIO_MUX_CPU2, 5);             // SCIA，后面的数字5是需要根据手册进行更改的
//    GPIO_SetupPinOptions(85, GPIO_INPUT, GPIO_PUSHPULL);
//    GPIO_SetupPinMux(84, GPIO_MUX_CPU2, 5);
//    GPIO_SetupPinOptions(84, GPIO_OUTPUT, GPIO_ASYNC);
//
//    GPIO_SetupPinMux(94, GPIO_MUX_CPU2, 6);             // SCID，这个复用是第几个需要查表 RM手册P930，56 57的SCIC是6
//    GPIO_SetupPinOptions(94, GPIO_INPUT, GPIO_PUSHPULL);
//    GPIO_SetupPinMux(93, GPIO_MUX_CPU2, 6);
//    GPIO_SetupPinOptions(93, GPIO_OUTPUT, GPIO_ASYNC);

}

void SCI_InitA()
{
// LSPCLK 时钟配置  RM手册P345，LSPCLK = 200MHz
    EALLOW;
    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV= 0;  // SYSCLK/1
    EDIS;

// RM手册 P2110 波特率计算，baud = LSPCLK/8/((BRR+1)，当 LSPCLK = 200MHz
// 当 SCIHBAUD = 0       波特率bps              当 SCIHBAUD = 10
// SCILBAUD =   216--> 115200 ;                   SCILBAUD = 43--> 9600 ;
//          =   97--> 256000
//          =   26  -->921600
//          =   9   -->2.5M
//          =   49  -->0.5M
//          =   99  -->0.25M
    SciaRegs.SCIHBAUD.all = 0;
    SciaRegs.SCILBAUD.all = 216;      //
    SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback, No parity,8 char bits,

    SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
    SciaRegs.SCICCR.bit.STOPBITS = 0; // 1个停止位
    SciaRegs.SCICCR.bit.PARITY = 1;   // 0-奇校验，1-偶校验
    SciaRegs.SCICCR.bit.PARITYENA = 1; // 奇偶检验使能(0-禁止，1-使能)
    SciaRegs.SCICCR.bit.LOOPBKENA = 0; // 自测试模式禁止
    SciaRegs.SCICCR.bit.ADDRIDLE_MODE = 0; // 0-空闲位模式协议，1-地址位模式协议
    SciaRegs.SCICCR.bit.SCICHAR = 0x07;    // 8bits
    SciaRegs.SCICTL1.bit.RXERRINTENA = 0;  // 接收中断禁止
    SciaRegs.SCICTL1.bit.SWRESET = 0; // Reset
    SciaRegs.SCICTL1.bit.TXENA = 1;
    SciaRegs.SCICTL1.bit.RXENA = 1;

    SciaRegs.SCICTL2.bit.RXBKINTENA = 0;    // 禁止RXRDY/BRKDT中断
    SciaRegs.SCIFFTX.bit.TXFIFORESET = 0;  // 复位FIFO    TXFIFOXRESET
    SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;  // 重新使能FIFO
    SciaRegs.SCIFFTX.bit.SCIFFENA = 1;      // FIFO使能
    SciaRegs.SCIFFTX.bit.TXFFIENA = 1;      //使能TXFIFO中断
    //  SciaRegs.SCIFFTX.bit.TXFFIL   = 0x06;   //这两句是触发发送中断的，下面的数值小于相等上面的值时触发，有多少位数据发送
    //  SciaRegs.SCIFFTX.bit.TXFFST   = 0x06;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 0;   //复位接收FIFO
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;   //重新使能接收FIFO
    SciaRegs.SCIFFRX.bit.RXFFIENA = 1;      //使能RXFIFO中断

    //  SciaRegs.SCIFFRX.bit.RXFFST = 10;       //ST>=IL才会触发中断
    SciaRegs.SCIFFRX.bit.RXFFIL = 12;        // 接收12个字节一中断

    SciaRegs.SCIFFCT.bit.FFTXDLY = 0x00;    // 两字节之间无延迟
    SciaRegs.SCICTL1.bit.SWRESET = 1;       // 重新使能SCI
}

void SCI_InitB()
{
// LSPCLK 时钟配置  RM手册P345，LSPCLK = 200MHz
//    EALLOW;
//    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV= 0;  // SYSCLK/1,2的0次方是1，所以就是200M/4=50M
//    EDIS;
    Flag_CPU2++;

    ScibRegs.SCIHBAUD.all = 0;
    ScibRegs.SCILBAUD.all = 53;   // baudrate = 115200
    ScibRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback, No parity,8 char bits,

    ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
    ScibRegs.SCICCR.bit.STOPBITS = 0; // 1个停止位
    ScibRegs.SCICCR.bit.PARITY = 1;   // 0-奇校验，1-偶校验； 上位机画图无校验位
    ScibRegs.SCICCR.bit.PARITYENA = 1; // 奇偶检验使能位 1使能0禁止-指向控制
    //ScicRegs.SCICCR.bit.PARITYENA = 0; // 奇偶检验使能位 1使能0禁止-上位机画图
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
    //  ScidRegs.SCIFFTX.bit.TXFFIL   = 0x06;   //这两句是触发发送中断的，下面的数值小于相等上面的值时触发，有多少位数据发送
    //  ScidRegs.SCIFFTX.bit.TXFFST   = 0x06;
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 0;   //复位接收FIFO
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;   //重新使能接收FIFO
    ScibRegs.SCIFFRX.bit.RXFFIENA = 1;      //使能RXFIFO中断

//  ScidRegs.SCIFFRX.bit.RXFFST = 10;       //ST>=IL才会触发中断
    ScibRegs.SCIFFRX.bit.RXFFIL = 12;        // 视觉伺服接受像素误差12个字节一中断，上位机画图16字节

    ScibRegs.SCIFFCT.bit.FFTXDLY = 0x00;    // 两字节之间无延迟
    ScibRegs.SCICTL1.bit.SWRESET = 1;       // 重新使能SCI
}

/*void SCI_InitD()
{
// LSPCLK 时钟配置  RM手册P345，LSPCLK = 200MHz
    EALLOW;
    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV= 0;  // SYSCLK/1,2的0次方是1，所以就是200M/1=200M
    EDIS;

    ScidRegs.SCIHBAUD.all = 0;
    ScidRegs.SCILBAUD.all = 216;   // baudrate = 115200
    ScidRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback, No parity,8 char bits,

    ScidRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
    ScidRegs.SCICCR.bit.STOPBITS = 0; // 1个停止位
//    ScidRegs.SCICCR.bit.PARITY = 0;   // 0-奇校验，1-偶校验
    ScidRegs.SCICCR.bit.PARITYENA = 0; // 奇偶检验禁止
    ScidRegs.SCICCR.bit.LOOPBKENA = 0; // 自测试模式禁止
    ScidRegs.SCICCR.bit.ADDRIDLE_MODE = 0; // 0-空闲位模式协议，1-地址位模式协议
    ScidRegs.SCICCR.bit.SCICHAR = 0x07;    // 8bits
    ScidRegs.SCICTL1.bit.RXERRINTENA = 0;  // 接收中断禁止
    ScidRegs.SCICTL1.bit.SWRESET = 0; // Reset
    ScidRegs.SCICTL1.bit.TXENA = 1;
    ScidRegs.SCICTL1.bit.RXENA = 1;

    ScidRegs.SCICTL2.bit.RXBKINTENA = 0;    // 禁止RXRDY/BRKDT中断
    ScidRegs.SCIFFTX.bit.TXFIFORESET = 0;  // 复位FIFO    TXFIFOXRESET
    ScidRegs.SCIFFTX.bit.TXFIFORESET = 1;  // 重新使能FIFO
    ScidRegs.SCIFFTX.bit.SCIFFENA = 1;      // FIFO使能
    ScidRegs.SCIFFTX.bit.TXFFIENA = 1;      //使能TXFIFO中断
    //  ScidRegs.SCIFFTX.bit.TXFFIL   = 0x06;   //这两句是触发发送中断的，下面的数值小于相等上面的值时触发，有多少位数据发送
    //  ScidRegs.SCIFFTX.bit.TXFFST   = 0x06;
    ScidRegs.SCIFFRX.bit.RXFIFORESET = 0;   //复位接收FIFO
    ScidRegs.SCIFFRX.bit.RXFIFORESET = 1;   //重新使能接收FIFO
    ScidRegs.SCIFFRX.bit.RXFFIENA = 1;      //使能RXFIFO中断

//  ScidRegs.SCIFFRX.bit.RXFFST = 10;       //ST>=IL才会触发中断
    ScidRegs.SCIFFRX.bit.RXFFIL = 10;        // 接收10个字节一中断

    ScidRegs.SCIFFCT.bit.FFTXDLY = 0x00;    // 两字节之间无延迟
    ScidRegs.SCICTL1.bit.SWRESET = 1;       // 重新使能SCI
}*/

//void Init_Kinco()
//{
//    // 速度环KP
//    HexD_Printf(0x012B);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0xF960);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0x013F);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0x0000);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0x003B);
//    DELAY_US(Delay_Time_SciD);
//
//    // 速度环KI
//    HexD_Printf(0x012B);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0xF960);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0x0201);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0x0000);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0x0078);
//    DELAY_US(Delay_Time_SciD);
//
//    // 位置环KP
//    HexD_Printf(0x012B);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0xFB60);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0x0101);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0x0000);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0x0077);
//    DELAY_US(Delay_Time_SciD);
//
//    // 集成电机速度控制模式
//    HexD_Printf(0x012F);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0x6060);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0x0003);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0x0000);
//    DELAY_US(Delay_Time_SciD);
//    HexD_Printf(0x000D);
//    DELAY_US(Delay_Time_SciD);
//}

void Shared_Ram_dataWrite_c2()
{
}
