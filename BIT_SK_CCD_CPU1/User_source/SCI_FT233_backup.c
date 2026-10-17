#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#include <math.h>
#include <string.h>
#include <stddef.h>
#include <SCI_USER_FT232.h>
#include "IDDK_PM_Servo_F2837x-Settings.h"
#include "sci_base.h"
#include "emif.h"
#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#include "F2837xD_sysctrl.h"
#include "F28x_Project.h"
#include "AGlobalVariables.h"


float Last_End_Value = 3.510936E+6F;  // 帧尾
float New_Start_Value = 3.704969E+6F; // 帧头
Uint16 iterator = 0U;
Uint16 sci_Counter_uplimit = 100; // SCI发送一包数据的容量，100表示每100组数据加一个帧头帧尾打成一包
int xmit_data[6];               // float数据类型，用来一次性传4个数据，数组剩下的两个位置用来存放偶尔会出现的帧头帧尾
float xmit_data_two[4];           // float数据类型，用来一次性传2个数据，数组剩下的两个位置用来存放偶尔会出现的帧头帧尾
float xmit_data_five[7];
float xmit_data_six[8];
float SCI_Receiver[4];
Uint16 errFlg = 0; // 接收错误标志位
Uint16 ReceiveData = 0;
Uint16 ReceiveData_A1 = 0;
int ReceiveData_A2 = 0;
int cc = 0, qq = 0, hh = 0;
int CMGS[5] = {0};

//#ifdef CPU1
////void scib_CMG_gpio_init()
////{
////    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 2);
////    GPIO_SetupPinOptions(15, GPIO_INPUT, GPIO_PUSHPULL);
////    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 2);
////    GPIO_SetupPinOptions(14, GPIO_OUTPUT, GPIO_ASYNC);
////}
//#endif

void scib_init()
{
//#ifdef CPU1
//    scib_CMG_gpio_init();
//#endif
    scib_echoback_init();
    scib_fifo_init();
}
void scib_echoback_init()
{
// Note: Clocks were turned on to the SCIA peripheral in the InitSysCtrl() function

//	EALLOW;
//	ClkCfgRegs.LOSPCP.bit.LSPCLKDIV= 0;  // SYSCLK/1
//	EDIS;
//
//    ScibRegs.SCICCR.all = 0x0007;  // 1 stop bit,  No loopback
//                                   // No parity,8 char bits,
//                                   // async mode, idle-line protocol
//    ScibRegs.SCICTL1.all = 0x0003; // enable TX, RX, internal SCICLK,
//                                   // Disable RX ERR, SLEEP, TXWAKE
//    ScibRegs.SCICTL2.all = 0x0003;
//    ScibRegs.SCICTL2.bit.TXINTENA = 1;
//    ScibRegs.SCICTL2.bit.RXBKINTENA = 1;
//
//    // SCIA at 9600 baud
//    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
//    // @LSPCLK = 30 MHz (120 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x86.
//    //
//    //    ScibRegs.SCIHBAUD.all = 0x0002;
//    //    ScibRegs.SCILBAUD.all = 0x008B;
////    ScibRegs.SCIHBAUD.all = 0x0000;
//
//
//    ScibRegs.SCIHBAUD.all = 0;
//    ScibRegs.SCILBAUD.all = 216;		// 26:921600, 216:115200
//
//
////    #if trans_fre_1000
////        ScibRegs.SCILBAUD.all = 0x007A;
////    #endif
////    #if trans_fre_2000
////        ScibRegs.SCILBAUD.all = 0x0018;
////    #endif
////
////    #if trans_fre_5000
////        ScibRegs.SCILBAUD.all = 0x001D;
////    #endif
////
////    #if trans_fre_10000
////        ScibRegs.SCILBAUD.all = 0x001D;
////    #endif
//
//    ScibRegs.SCICTL1.all = 0x0023; // Relinquish SCI from Reset
//////    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL5_SCI, 2, SYSCTL_CPUSEL_CPU2);
}
void scib_xmit_FT232(int a)
{
    //    while (ScibRegs.SCIFFTX.bit.TXFFST != 0) {}
    while (ScibRegs.SCICTL2.bit.TXRDY == 0)
    {
    }
    ScibRegs.SCITXBUF.all = a;
}
void scib_msg(char *msg)
{
    int i;
    i = 0;
    while (msg[i] != '\0')
    {
        scib_xmit_FT232(msg[i]);
        i++;
    }
}

//
// scia_fifo_init - Initialize the SCI FIFO
//
void scib_fifo_init()
{
//    ScibRegs.SCIFFTX.all = 0xE040;
//    ScibRegs.SCIFFRX.all = 0x2044;
//    ScibRegs.SCIFFCT.all = 0x0;
}

// 与上位机通信

/* Transmit character(s) from the SCIb*/
void scib_xmit_FT232_user(char *pmsg, int msglen, int typeLen) // msglen代表数组长度
{
    int i, j;
    while (ScibRegs.SCIFFTX.bit.TXFFST > 16 - msglen * typeLen)  ;
    if (typeLen == 1)
    {
        for (i = 0; i < msglen; i++)
        {

            //              while (ScibRegs.SCICTL2.bit.TXRDY == 0)
            //            {
            //            } /* The buffer is full;*/

            ScibRegs.SCITXBUF.all = pmsg[i];
        }

        // while(SciaRegs.SCIFFTX.bit.TXFFST != 0){}
    }
    else
    {
        for (i = 0; i < (msglen / 2); i++)
        {
            for (j = 0; j <= 1; j++)
            {
                // while (ScibRegs.SCIFFTX.bit.TXFFST == 16)
                //                   while (ScibRegs.SCICTL2.bit.TXRDY == 0)
                //                {
                //                } /* The buffer is full;*/

                ScibRegs.SCITXBUF.all = pmsg[i] >> (8 * j);
            }
        }

        // while(SciaRegs.SCIFFTX.bit.TXFFST != 0){}
    }
}
//void dsp_to_simulink_sci(float Data1, float Data2, float Data3, float Data4)
Uint16 strLength=0;   // 数据个数
void dsp_to_simulink_sci(int Data1, int Data2, int Data3, int Data4)
{
    // SCI发送函数
//    Uint16 strLength;   // 数据个数
    if (iterator == 0U) // 周期起始
    {
        xmit_data[0] = Last_End_Value;  // 上一包数据的帧尾，值为3.510936E+6F，对应到上位机上是
        xmit_data[1] = New_Start_Value; // 这一包数据的帧头，值为3.704969E+6F，对应到上位机上是
        xmit_data[2] = Data1;
        xmit_data[3] = Data2;
        xmit_data[4] = Data3;
        xmit_data[5] = Data4;
        strLength = 6;//sizeof(xmit_data) / sizeof(xmit_data[0]);
    }
    else
    {
        xmit_data[0] = Data1;
        xmit_data[1] = Data2;
        xmit_data[2] = Data3;
        xmit_data[3] = Data4;
        xmit_data[4] = 0.0; // 不发送帧头帧尾时数组后两个数据无关
        xmit_data[5] = 0.0;
        strLength = 4;//sizeof(xmit_data) / sizeof(xmit_data[0]) - 2;
    }
    int i = 0;
    float data;
    do // 执行上述两组数据之一
    {
        data = xmit_data[i];
        {
            scib_xmit_FT232_user((int *)&data, 4, 4);
        }
        i++;
        strLength--;
    } while (strLength != 0); // 发送结束

    iterator = (iterator + 1) & 255U; // 迭代次数不超过0xff
    if (iterator >= sci_Counter_uplimit)
    {
        iterator = 0;
    }
}

void dsp_to_simulink_sci_Two(float Data1, float Data2)
{
    // SCI发送函数
    Uint16 strLength;   // 数据个数
    if (iterator == 0U) // 周期起始
    {

        xmit_data_two[0] = Last_End_Value;  // 上一包数据的帧尾，值为3.510936E+6F，对应到上位机上是
        xmit_data_two[1] = New_Start_Value; // 这一包数据的帧头，值为3.704969E+6F，对应到上位机上是
        xmit_data_two[2] = Data1;
        xmit_data_two[3] = Data2;

        strLength = sizeof(xmit_data_two) / sizeof(xmit_data_two[0]);
    }
    else
    {
        xmit_data_two[0] = Data1;
        xmit_data_two[1] = Data2;
        xmit_data_two[2] = 0.0; // 不发送帧头帧尾时数组后两个数据无关
        xmit_data_two[3] = 0.0;
        strLength = sizeof(xmit_data_two) / sizeof(xmit_data_two[0]) - 2;
    }
    int i = 0;
    float data;
    do // 执行上述两组数据之一
    {
        data = xmit_data_two[i];
        {
            scib_xmit_FT232_user((int *)&data, 4, 4);
        }
        i++;
        strLength--;
    } while (strLength != 0); // 发送结束

    iterator = (iterator + 1) & 255U; // 迭代次数不超过0xff
    if (iterator >= sci_Counter_uplimit)
    {
        iterator = 0;
    }
}

void dsp_to_simulink_sci_five(float Data1, float Data2, float Data3, float Data4, float Data5)
{
    // SCI发送函数
    Uint16 strLength;   // 数据个数
    if (iterator == 0U) // 周期起始
    {

        xmit_data_five[0] = Last_End_Value;  // 上一包数据的帧尾，值为3.510936E+6F，对应到上位机上是
        xmit_data_five[1] = New_Start_Value; // 这一包数据的帧头，值为3.704969E+6F，对应到上位机上是
        xmit_data_five[2] = Data1;
        xmit_data_five[3] = Data2;
        xmit_data_five[4] = Data3;
        xmit_data_five[5] = Data4;
        xmit_data_five[6] = Data5;
        strLength = sizeof(xmit_data_five) / sizeof(xmit_data_five[0]);
    }
    else
    {
        xmit_data_five[0] = Data1;
        xmit_data_five[1] = Data2;
        xmit_data_five[2] = Data3;
        xmit_data_five[3] = Data4;
        xmit_data_five[4] = Data5;
        xmit_data_five[5] = 0.0; // 不发送帧头帧尾时数组后两个数据无关
        xmit_data_five[6] = 0.0;
        strLength = sizeof(xmit_data_five) / sizeof(xmit_data_five[0]) - 2;
    }
    int i = 0;
    float data;
    do // 执行上述两组数据之一
    {
        data = xmit_data_five[i];
        {
            scib_xmit_FT232_user((int *)&data, 4, 4);
        }
        i++;
        strLength--;
    } while (strLength != 0); // 发送结束

    iterator = (iterator + 1) & 255U; // 迭代次数不超过0xff
    if (iterator >= sci_Counter_uplimit)
    {
        iterator = 0;
    }
}

void dsp_to_simulink_sci_six(float Data1, float Data2, float Data3, float Data4, float Data5, float Data6)
{
    // SCI发送函数
    Uint16 strLength;   // 数据个数
    if (iterator == 0U) // 周期起始
    {

        xmit_data_six[0] = Last_End_Value;  // 上一包数据的帧尾，值为3.510936E+6F，对应到上位机上是
        xmit_data_six[1] = New_Start_Value; // 这一包数据的帧头，值为3.704969E+6F，对应到上位机上是
        xmit_data_six[2] = Data1;
        xmit_data_six[3] = Data2;
        xmit_data_six[4] = Data3;
        xmit_data_six[5] = Data4;
        xmit_data_six[6] = Data5;
        xmit_data_six[7] = Data6;
        strLength = sizeof(xmit_data_six) / sizeof(xmit_data_six[0]);
    }
    else
    {
        xmit_data_six[0] = Data1;
        xmit_data_six[1] = Data2;
        xmit_data_six[2] = Data3;
        xmit_data_six[3] = Data4;
        xmit_data_six[4] = Data5;
        xmit_data_six[5] = Data6;
        xmit_data_six[6] = 0.0; // 不发送帧头帧尾时数组后两个数据无关
        xmit_data_six[7] = 0.0;
        strLength = sizeof(xmit_data_six) / sizeof(xmit_data_six[0]) - 2;
    }
    int i = 0;
    float data;
    do // 执行上述两组数据之一
    {
        data = xmit_data_six[i];
        {
            scib_xmit_FT232_user((int *)&data, 4, 4);
        }
        i++;
        strLength--;
    } while (strLength != 0); // 发送结束

    iterator = (iterator + 1) & 255U; // 迭代次数不超过0xff
    if (iterator >= sci_Counter_uplimit)
    {
        iterator = 0;
    }
}

