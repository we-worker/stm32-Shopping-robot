#ifndef __KEYLED_H
#define __KEYLED_H
#include "stm32f4xx.h"
void Key_Init(uint8_t nKey);  //nKey=1，只使能Key1，nKey=2，使能Key1和Key2
uint8_t Key_Pressed(uint8_t nKey);  //当有按键按下时返回1，否则返回0。nKey为按键索引，1=Key1，2=Key2
uint8_t Key_Released(uint8_t nKey); //当有按键按下并释放时返回1，否则返回0。nKey为按键索引，1=Key1，2=Key2


void Led_Init(uint8_t nLed);  //nLed=1，只使能Led2，nLed=2，使能Led2和Led3
#define LED2_ON(); GPIO_ResetBits(GPIOD,GPIO_Pin_4);
#define LED2_OFF(); GPIO_SetBits(GPIOD,GPIO_Pin_4);
#define LED3_ON(); GPIO_ResetBits(GPIOD,GPIO_Pin_7);
#define LED3_OFF(); GPIO_SetBits(GPIOD,GPIO_Pin_7);
#endif
