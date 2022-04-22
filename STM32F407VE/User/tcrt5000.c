#include "tcrt5000.h"

//
void TCRT5000_config(void)	
{
    GPIO_InitTypeDef GPIO_InitStructure;	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15 | GPIO_Pin_14	;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);  
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);  
}
