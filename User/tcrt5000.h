#ifndef __TCRT5000_H
#define	__TCRT5000_H

#include "stm32f4xx.h"

#define TCRT5000_1  GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_5)    //读取循环传感器1  --PE5
#define TCRT5000_2  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)   //读取循环传感器2  --PE15
#define TCRT5000_0  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)   //读取循环传感器0  --PE11

void TCRT5000_config(void);

#endif
