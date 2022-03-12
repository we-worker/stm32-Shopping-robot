#ifndef __Nano_MODULE_H
#define __Nano_MODULE_H
#include "stm32f4xx.h"
#include "stdio.h"
#include "stdarg.h"



//ʹ��USART2����ӦPA2/TX��PA3/RX��


#define FRAME_BYTE_LENGTH 9 //����ͨѶһ֡���ݵ��ֽ�������֡ͷ��֡β����Ʃ��20���ֽ�Ϊһ������������֡����1���ֽ�֡ͷ����2���ֽڴ����������ͣ���3~6�ֽ��������������7���ֽ�Ϊ֡β
#define FRAME_START 0xA5 //֡ͷ
#define FRAME_END 0x5A  //֡β

//���������֡���ձ�־�ͽ��ջ���������ɿɹ�BTModule.c�����Դ����ֱ��ʹ�ã�ֻҪinclude��BTModule.h�ļ���
extern __IO uint8_t USART_Rx2Buff[FRAME_BYTE_LENGTH]; //���ջ�����
extern __IO uint8_t USART_FrameFlag; //������������֡��־��1������0������

void USART1_Init(void);
void USART1_OUT(USART_TypeDef* USARTx, uint8_t *Data,...);
//char *itoa(int value, char *string, int radix);
//int fputc(int ch, FILE *f);
//void USART_GetChar(uint8_t nChar); //���ڽ��յ�һ���ֽ�
//void USART_Process(void);
void USART1_Process(void);

#endif