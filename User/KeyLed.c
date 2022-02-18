#include "KeyLed.h"
#include "delay.h"
void Key_Init(uint8_t nKey)  //nKey=1，只使能Key1，nKey=2，使能Key1和Key2
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	
	switch(nKey)
	{
		case 2:
			//配置PE3为Key2
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;				 	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;				//推挽输出
//			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		
			GPIO_Init(GPIOE, &GPIO_InitStructure);
		case 1:
			//配置PE2为Key1
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;				     
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			
			GPIO_Init(GPIOE, &GPIO_InitStructure);
		default:
			;
	}		
}
uint8_t Key_Pressed(uint8_t nKey)  //当有按键按下时返回1，否则返回0。nKey为按键索引，1=Key1，2=Key2
{
	switch(nKey)
	{
		case 2:
			//配置PE3为Key2
			if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)==0)
			{
				Delay_10us(5);
				if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)==0)
					return 1;
			}
			break;
		case 1:
			//配置PE2为Key1
			if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)==0)
			{
				Delay_10us(5);
				if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)==0)
					return 1;
			}
			break;
		default:
			;
	}
	return 0;
}
uint8_t Key_Released(uint8_t nKey) //当有按键按下并释放时返回1，否则返回0。nKey为按键索引，1=Key1，2=Key2
{
	switch(nKey)
	{
		case 2:
			//配置PE3为Key2
			if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)==0)
			{
				Delay_10us(5);
				if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)==0)
				{
					while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3)==0); //如果一直按着则一直等着
					return 1;
				}
			}
			break;
		case 1:
			//配置PE2为Key1
			if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)==0)
			{
				Delay_10us(5);
				if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)==0)
				{
					while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)==0); //如果一直按着则一直等着
					return 1;
				}
			}
			break;
		default:
			;
	}
	return 0;	
}

void Led_Init(uint8_t nLed)  //nLed=1，只使能Led2，nLed=2，使能Led2和Led3
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	switch(nLed)
	{
		case 2:
			//配置PD7为LED3

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;				 //PC12	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//推挽输出
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	
			GPIO_Init(GPIOD, &GPIO_InitStructure);
		case 1:
			//配置PD4为LED2
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;				     
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	
			GPIO_Init(GPIOD, &GPIO_InitStructure);
		default:
			;
	}		
}
