#include "../F2837x_includes/F2837xD_device.h"

#ifndef  __PRINTF_UART_H
#define  __PRINTF_UART_H

#include <stdio.h>
#include <file.h>

#define SCIBaudRate    921600L
#define TIMEROUTSCI    (Uint32)10*(200000000/SCIBaudRate) //估算的等待超时时间，请根据实际修改

#define SCI_FIFO_LEN  16 //定义DSP串口FIFO深度，最大16级深度， 1-16

#define UartRxLEN 20  //接收缓存长度
#define UartTxLEN 20  //发送缓存长度

void HexPrintf(int d);

typedef struct Uart_Type{
	union
	  {
	    Uint16 All;
	    struct{
	            Uint16  UartRevFlag           :1;  //接收到数据标志
	            Uint16  rFifoDataflag          :1;  //接收内存非空
	    		}Status_Bits;
	  	}Mark_Para;
	char rxData[UartRxLEN];				 //接收缓存
	Uint16 rxReadIndex;                         //接收FIFO写入索引
	Uint16 rxWriteIndex;                        //接收FIFO读出索引
}
Uart_Msg; 

extern Uart_Msg SCI_Msg;

//--------------------------------------------------------------------
void handleRxFIFO(void);
void SCI_Init(void);
void SCI_GPIO(void);
void SCI_InitA(void);
void SCI_InitB(void);
void SCI_InitC(void);
void SCI_InitD(void);
void spiA_init(void);
void spiB_init(void);
void Scia_xmit(int a);
void Scib_xmit(int b);
void Scic_xmit(int a);
void spiA_xmit(Uint16 a);
void spiB_xmit(Uint16 a);
void HexA_Printf(int d);
void HexB_Printf(int d);
void HexC_Printf(int d);
void sciPutString(char Enter1,char *s, char Enter2);




void open_uart_debug (void);
int printf(const char* str, ...); 


//---------------------------------------------------------------------
int my_open(const char *path, unsigned flags, int fno);
int my_close(int fno);
int my_read(int fno, char *buffer, unsigned count);
int my_write(int fno, const char *buffer, unsigned count);
off_t my_lseek(int fno, off_t offset, int origin);
int my_unlink(const char *path);
int my_rename(const char *old_name, const char *new_name);


#endif//  __PRINTF_UART_H


