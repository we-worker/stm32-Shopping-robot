/*
 * @description: This file is part of shoppingbot
 * @author: hamlin
 * @version: v1.0
 * @homepage: http://www.hamlinzheng.com
 * @github: http://github.com/hamlinzheng
 * @LastEditTime: 2019-04-26 13:32:57
 */
#ifndef __AMT1450_UART_H
#define __AMT1450_UART_H
#include "stm32f4xx.h"

//#define FRAME_START 0xaa        // 帧头

typedef enum{
    AMT1450_FRAME_START = 0xaa,         // 帧头
}amt1450_UART_PROTOCOL_e;



typedef struct
{
    uint8_t RxDataPtr;      // 接收数据指针
    uint8_t RxData[9];      // 接收到的数据
    uint8_t ValidData[9];   // 有效的数据
          
}amt1450_UART_Rx_t;






void init_AMT1450_UART(void);
void AMT1450_UART_Cmd(FunctionalState NewState);
void get_AMT1450Data_UART(uint8_t *begin_Color,uint8_t *jump_Count,uint8_t *jump_Location);
void amt1450_Test_UART(void);


#endif
