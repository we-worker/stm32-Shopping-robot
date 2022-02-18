#ifndef __DELAY_H
#define __DELAY_H
#include "stm32f4xx.h"
void TimingDelay_Decrement(void);
void Delay_10us(__IO uint32_t nTime);//延时，单位：10微秒
void Delay_ms(__IO uint32_t nTime);//延时，单位：毫秒
extern __IO uint8_t b10msFlag; //每隔1ms由滴答中断置1，调用后请及时清零。
#if defined MPU6050    //如果使用MPU6050，则需要下列代码
int stm32_get_clock_ms(unsigned long *count); //该函数获得系统毫秒数，在MPU6050库中需要用到
#endif
#endif
