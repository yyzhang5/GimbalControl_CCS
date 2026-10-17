/*
 * emif.h 外部中断
 *
 *  Created on: 2020-04-23
 *      Author: cotto
 */
#include "F2837xD_device.h"

#ifndef EMIF_H_
#define EMIF_H_

void ExInt_AD_BUSY(void);
interrupt void xint4_isr(void);
void AD760x_Init(void);

typedef struct
{
  uint16_t dataWR;
}
AD760x_TypeDef;

#define DATA_LEN 1  //数组深度
//------------------------------------------------------------------------->
typedef struct AD760xPara
{
	short int  CHn[8][DATA_LEN];//8个通道，每个深度 DATA_LEN
	short int  CH_All[32];//各通道数组
	uint16_t  R_index;   //写入CHn 数组的 索引
	uint16_t  W_index;   //读出CHn 数组的 索引
	char DMASend ;
}AD760xP;

extern AD760xP AD760x;

extern short ReadSRAM_Buf[];

#define TEST_PASS            0xABCDABCD
#define TEST_FAIL            0xDEADDEAD
#define ASRAM_CS2_START_ADDR 0x100000   //CS2地址空间 0x0010 0000 到 0x002F FFFF , 共计
#define ASRAM_CS2_SIZE       0x8000     //400H为1K ， 这里定义了32K的范围，外置SRAM有512K


void EMIF_Init(void);
void setup_emif1_pinmux_async_16bit(Uint16);
void EMIF_Write(Uint32 start_addr,Uint32 mem_size);
void EMIF_Read(Uint32 start_addr,Uint32 mem_size,short *buf);
void EMIF_AD7606_Read(Uint32 start_addr,Uint32 mem_size,short *buf);
//-----------------AD760x定义----------------------------
//定义FSMC访问地址 AD760x_CSx
#define AD760x_Base  ((uint8_t)(0x00300000 ) //0x00300000 -- 0x0037FFFF
#define AD760x_CS0_FSMC   ((AD760x_TypeDef *) 0x00300000))// XA4 XA3 = b'00
#define AD760x_CS1_FSMC   ((AD760x_TypeDef *) 0x00300000 + 16)
#define AD760x_CS2_FSMC   ((AD760x_TypeDef *) 0x00300000 + 32)
//cs3 用 XA5/GPIO45 模拟



//74HC139D 的CS1
//A4 A3 = 01-->0x08
#define XINF_AD2S1210_POS     ((Uint32)( 0x00300008))     //A6 A5 = 00
#define XINF_AD2S1210_SP     ((Uint32)(0x00300008|0x40))  //A6 A5 = 10
#define XINF_AD2S1210_CONF   ((Uint32)(0x00300008|0x60))  //A6 A5 = 11


//SAMPLE   GPIO14
#define SAMPLE_GPIO GPIO14
#define SAMPLE_MUX GpioCtrlRegs.GPAMUX1.bit.SAMPLE_GPIO
#define SAMPLE_DIR GpioCtrlRegs.GPADIR.bit.SAMPLE_GPIO
#define SAMPLE_PUD GpioCtrlRegs.GPAPUD.bit.SAMPLE_GPIO
#define SAMPLE_H() GpioDataRegs.GPASET.bit.SAMPLE_GPIO =1     // 旋变，旋转变压器用
#define SAMPLE_L() GpioDataRegs.GPACLEAR.bit.SAMPLE_GPIO =1	// GPIO A Data Clear Register (GPIO0到31),1是锁存到0,0表示忽略

//RST  GPIO15
#define AD_RST_GPIO GPIO15
#define AD_RST_MUX GpioCtrlRegs.GPAMUX1.bit.AD_RST_GPIO
#define AD_RST_DIR GpioCtrlRegs.GPADIR.bit.AD_RST_GPIO
#define AD_RST_PUD GpioCtrlRegs.GPAPUD.bit.AD_RST_GPIO
#define AD_RST_H() GpioDataRegs.GPASET.bit.AD_RST_GPIO =1
#define AD_RST_L() GpioDataRegs.GPACLEAR.bit.AD_RST_GPIO =1

//CS4  GPIO44 ; AD7606模块最靠上的一个是通道25-32
//#define AD_CS4_GPIO GPIO44
//#define AD_CS4_MUX GpioCtrlRegs.GPBMUX1.bit.AD_CS4_GPIO
//#define AD_CS4_DIR GpioCtrlRegs.GPBDIR.bit.AD_CS4_GPIO
//#define AD_CS4_PUD GpioCtrlRegs.GPBPUD.bit.AD_CS4_GPIO
//#define AD_CS4_H() GpioDataRegs.GPBSET.bit.AD_CS4_GPIO =1
//#define AD_CS4_L() GpioDataRegs.GPBCLEAR.bit.AD_CS4_GPIO =1
//
////BUSY  GPIO92----需定义为外部中断
//#define EXINT_AD_BUSY_NUM 92
//#define AD_BUSY_GPIO GPIO92
//#define AD_BUSY_MUX GpioCtrlRegs.GPCMUX2.bit.AD_BUSY_GPIO
//#define AD_BUSY_DIR GpioCtrlRegs.GPCDIR.bit.AD_BUSY_GPIO
//#define AD_BUSY_PUD GpioCtrlRegs.GPCPUD.bit.AD_BUSY_GPIO
//#define AD_BUSY_QSEL GpioCtrlRegs.GPCQSEL2.bit.AD_BUSY_GPIO
//

//-----------------------------------------28335-----------------------------
#define Uint8 unsigned char
#define uint8_t unsigned char
#define uint16_t Uint16
#define uint32_t Uint32

extern  Uint32 resCode;
extern int16 Velocitys;
extern uint8_t reg[10];
extern uint8_t Fault;
extern uint16_t Dos_temp;

void AD2S1210_WRITE1(uint8_t addr);
uint8_t AD2S1210_READ1(void);

void AD2S1210_WRITE(Uint8 addr,Uint8 data);
Uint8 AD2S1210_READ(Uint8 addr);
extern Uint16 READ_Position(Uint16 position_offset);//读取位置
int READ_Velocity(void);//读取速度
void AD2S1210_Init(void);




#endif /* EMIF_H_ */
