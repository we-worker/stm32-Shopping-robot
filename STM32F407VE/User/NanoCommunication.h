#ifndef __Nano_MODULE_H
#define __Nano_MODULE_H
#include "stm32f4xx.h"
#include "stdio.h"
#include "stdarg.h"



//使用USART2，对应PA2/TX，PA3/RX，


//原先设计的是9
#define FRAME_BYTE_LENGTH 9 //串口通讯一帧数据的字节数（含帧头和帧尾），譬如20个字节为一个完整的数据帧，第1个字节帧头，第2个字节代表命令类型，第3~6字节是命令参数，第7个字节为帧尾
#define FRAME_START 0xA5 //帧头
#define FRAME_END 0x5A  //帧尾



void USART1_Init(void);
void USART1_OUT(USART_TypeDef* USARTx, uint8_t *Data,...);
//char *itoa(int value, char *string, int radix);
//int fputc(int ch, FILE *f);
//void USART_GetChar(uint8_t nChar); //串口接收到一个字节
//void USART_Process(void);
void USART1_Process(void);

extern uint8_t car_flag;//小车状态


#endif
