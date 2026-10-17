/*
 * emif.c  外部总线
 *
 *  Created on: 2020-04-24
 *      Author: M新动力电子  https://shop111940119.taobao.com
 *
 */
#include "emif.h"
#include "AGlobalVariables.h"
#include "F28x_Project.h"     // Device Headerfile and Examples Include File

Uint32 resCode;
int16 Velocitys;
uint8_t reg[10];
uint8_t Fault;
uint16_t Dos_temp;


AD760xP AD760x;

Uint16  ErrCount = 0;
Uint32  TEST_STATUS;
int i;
char ErrCount_local;

short ReadSRAM_Buf[32];


void AD2S1210_WRITE(uint8_t addr,uint8_t data)
{

  *((Uint16*)XINF_AD2S1210_CONF) = addr;
  *((Uint16*)XINF_AD2S1210_CONF) = data;
}

void AD2S1210_WRITE1(uint8_t addr)
{
  *((Uint16*)XINF_AD2S1210_CONF) = addr;
}
uint8_t AD2S1210_READ1(void)
{
    uint8_t temp;
    temp = *((Uint16*)XINF_AD2S1210_CONF);
    temp&=0xff;

  return temp;
}


uint8_t AD2S1210_READ(uint8_t addr)
{
    uint8_t temp;

   // *((Uint16*)XINF_AD2S1210_CONF) = addr;
    AD2S1210_WRITE1(addr);

   // __asm(" RPT #100 || NOP");

    temp =  *((Uint16*)XINF_AD2S1210_CONF);
    temp&=0xff;

  return temp;
}
Uint16 position=0;
Uint16 READ_Position(Uint16 position_offset)//读取位置
{

  position=AD2S1210_READ(0x80);
//  AD2S1210_WRITE1(0x80);
//  position= AD2S1210_READ1();

  position<<=8;

  position+=AD2S1210_READ(0x81);
//  AD2S1210_WRITE1(0x80);
//  position+= AD2S1210_READ1();

  // 需要注意角位置和极对数与控制电机型号匹配
  if(position > position_offset)
	  temppos = position - position_offset;
  else
  {
	  temppos = 65535 + position - position_offset;
  }

  return temppos;
}

int READ_Velocity(void)//读取速度
{
  int velocity;
  velocity=AD2S1210_READ(0x82);

//  AD2S1210_WRITE1(0x82);
//  velocity = AD2S1210_READ1();

  velocity<<=8;

  velocity|=AD2S1210_READ(0x83);
//  AD2S1210_WRITE1(0x83);
//  velocity += AD2S1210_READ1();

  return velocity;
}



void AD2S1210_Init(void)
{
    EALLOW;

    AD2S1210_WRITE(0x92,0x7e);//Set 16Bit 0x7e->12Bit
    AD2S1210_WRITE(0x91,0x20);//Set Driver freq，参照datasheet Page22章节0x32
    //0x20-->10K励磁，晶振是10.24MHz

    AD2S1210_WRITE(0x8b,0x7f);//DOS复位最大阀值
    AD2S1210_WRITE(0x8c,0x01);//DOS复位最小阀值
    AD2S1210_WRITE(0x88,0x01);//LOS阀值
    AD2S1210_WRITE(0x89,0x7f);//DOS超量程阀值
    AD2S1210_WRITE(0x8a,0x7f);//DOS失配阀值

    reg[0]=AD2S1210_READ(0x88);//LOS阀值
    reg[1]=AD2S1210_READ(0x89);//DOS超量程阀值
    reg[2]=AD2S1210_READ(0x8a);//DOS失配阀值
    reg[3]=AD2S1210_READ(0x8b);//DOS复位最大阀值
    reg[4]=AD2S1210_READ(0x8c);//DOS复位最小阀值
    reg[5]=AD2S1210_READ(0x8d);//LOT上限
    reg[6]=AD2S1210_READ(0x8e);//LOT下限
    reg[7]=AD2S1210_READ(0x91);//激励频率
    reg[8]=AD2S1210_READ(0x92);//控制寄存器
    reg[9]=AD2S1210_READ(0xff);//Fault

    Fault=AD2S1210_READ(0xff);//Fault


    AD_RST_MUX = 0;//GPIO功能
    AD_RST_DIR = 1;//输出

    SAMPLE_MUX = 0;
    SAMPLE_DIR = 1;
    EDIS;


    SAMPLE_H();
    AD_RST_H();  ///复位AD7606
    DELAY_US(10000);
    AD_RST_L();
    DELAY_US(1000);
}

//---------------------------------------------
//初始化EMIF接口
//CS2 ,  16bit 数据宽度
//---------------------------------------------
void EMIF_Init(void)
{
    //
    //Configure to run EMIF1 on full Rate (EMIF1CLK = CPU1SYSCLK)
    //
      EALLOW;
      ClkCfgRegs.PERCLKDIVSEL.bit.EMIF1CLKDIV = 1;
      EDIS;

      EALLOW;
    //
    // Grab EMIF1 For CPU1
    //
      Emif1ConfigRegs.EMIF1MSEL.all = 0x93A5CE71;
      if(Emif1ConfigRegs.EMIF1MSEL.all != 0x1)
      {
          ErrCount++;
      }

    //
    //Disable Access Protection (CPU_FETCH/CPU_WR/DMA_WR)
    //
      Emif1ConfigRegs.EMIF1ACCPROT0.all = 0x0;
      if(Emif1ConfigRegs.EMIF1ACCPROT0.all != 0x0)
      {
          ErrCount++;
      }

    //
    // Commit the configuration related to protection. Till this bit remains set
    // content of EMIF1ACCPROT0 register can't be changed.
    //
      Emif1ConfigRegs.EMIF1COMMIT.all = 0x1;
      if(Emif1ConfigRegs.EMIF1COMMIT.all != 0x1)
      {
         ErrCount++;
      }

    //
    // Lock the configuration so that EMIF1COMMIT register can't be
    // changed any more.
    //
      Emif1ConfigRegs.EMIF1LOCK.all = 0x1;
      if(Emif1ConfigRegs.EMIF1LOCK.all != 1)
      {
          ErrCount++;
      }

      EDIS;

    //
    //Configure GPIO pins for EMIF1
    //
      setup_emif1_pinmux_async_16bit(0);//修改过底层，仅设置6需要的总线IO；多余的未初始化

    //
    //Configure the access timing for CS2 space
    //
      Emif1Regs.ASYNC_CS3_CR.all =  (EMIF_ASYNC_ASIZE_16    | // 16Bit Memory
                                                              // Interface
                                     EMIF_ASYNC_TA_1        | // Turn Around time
                                                              // of 2 Emif Clock
                                     EMIF_ASYNC_RHOLD_1     | // Read Hold time
                                                              // of 1 Emif Clock
                                     EMIF_ASYNC_RSTROBE_16   | // Read Strobe time
                                                              // of 4 Emif Clock
                                     EMIF_ASYNC_RSETUP_1    | // Read Setup time
                                                              // of 1 Emif Clock
                                     EMIF_ASYNC_WHOLD_1     | // Write Hold time
                                                              // of 1 Emif Clock
                                     EMIF_ASYNC_WSTROBE_16   | // Write Strobe time
                                                              // of 1 Emif Clock
                                     EMIF_ASYNC_WSETUP_1    | // Write Setup time
                                                              // of 1 Emif Clock
                                     EMIF_ASYNC_EW_DISABLE  | // Extended Wait
                                                              // Disable.
                                     EMIF_ASYNC_SS_DISABLE    // Strobe Select Mode
                                                              // Disable.
                                    );
}

//-------------------------------------------
// 写入SRAM测试
//-----------------------------------------------
void EMIF_Write(Uint32 start_addr,Uint32 mem_size)
{
    unsigned long mem_wds;
    short *XMEM_ps;
    unsigned int i;

    //
    //指针赋初值
    //
    XMEM_ps = (short *)start_addr;

    //
    //测试写 0x5555  0xAAAA 交替到SRAM
    //
    mem_wds = 0xAAAA;
    for (i=0; i < mem_size; i++)
    {
        *XMEM_ps++ = mem_wds;
        mem_wds ^= 0xFFFF;
    }
}


//-------------------------------------------
// 读出SRAM测试
//-----------------------------------------------
void EMIF_Read(Uint32 start_addr,Uint32 mem_size,short *buf)
{
    short *XMEM_ps;
    unsigned int i;
    //
    //指针赋初值
    //
    XMEM_ps = (short *)start_addr;

    //
    //读取SRAM数据
    //
    for (i=0; i < mem_size; i++)
    {
        *buf++ = *XMEM_ps++;
    }
}

//-------------------------------------------
// 读出AD7606数据
// mem_size 取值8  16  24
//-----------------------------------------------
void EMIF_AD7606_Read(Uint32 start_addr,Uint32 mem_size,short *buf)
{
    short *XMEM_ps;
    unsigned int i;
    //
    //指针赋初值
    //
    XMEM_ps = (short *)start_addr;

    for (i=0; i < mem_size; i++)  //读取通道
    {
        *buf++ = *XMEM_ps++;
    }

}


//###########################################################################
//AD760x初始化
//  RST 高电平复位AD760x ； 控制板的P1 跳线帽右侧短接，设置RAGNGE=1 ，量程为正负10V
//  CNA CNB 触发由PWM4A 控制
//  BUSY中断由 ExInt_AD_BUSY() 函数实现
//  读取AD数据，在BUSY中断服务函数 interrupt void xint4_isr(void) 里实现
//------------------------------------------------------------------------------------
void AD760x_Init(void)
{
    EALLOW;
    AD_RST_MUX = 0;//GPIO功能
    AD_RST_DIR = 1;//输出
    EDIS;

    AD_RST_H();  ///复位AD7606
    DELAY_US(10000);      // AD760x 要求50ns的高电平复位时间
    AD_RST_L();
}


