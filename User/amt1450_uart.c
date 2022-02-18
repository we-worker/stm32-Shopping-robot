/*
 * @description: This file is part of shoppingbot
 * @author: hamlin
 * @version: v1.0
 * @homepage: http://www.hamlinzheng.com
 * @github: http://github.com/hamlinzheng
 * @LastEditTime: 2019-04-26 16:05:32
 */
#include "amt1450_uart.h"
#include "BTModule.h"
#include "delay.h"

amt1450_UART_Rx_t amt1450_UART_Rx;

/**
 * PC12->TXD5
 * PD2 ->RXD5
 **/
/**
 * @brief: 初始化amt1450串口通信(UART5)
 * @param {type} 
 * @retval: None
 */
void init_AMT1450_UART(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;  //NVIC中断向量结构体
    /* config UART5 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    /* Connect UART5 pins to AF */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

    /* USART5 GPIO config */ 
    /* Configure UART5 Tx (PC.12) as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_UP;	
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    /* Configure UART5 Rx (PD.02) as input floating */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_UP;	
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* Enable the UART5 OverSampling by 8 */
    USART_OverSampling8Cmd(UART5, ENABLE);  	

    /* UART5 mode config */
    USART_InitStructure.USART_BaudRate = 115200;
    //	USART_InitStructure.USART_BaudRate = 19200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART5, &USART_InitStructure);

    /* Enable UART5 Receive and Transmit interrupts */
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); 
    USART_ITConfig(UART5, USART_IT_TXE, ENABLE); 
    // 优先级配置
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	
		
    // USART_Cmd(UART5, ENABLE);
		AMT1450_UART_Cmd(DISABLE);
}
/**
 * @brief: 串口接收配置使能or失能
 * @param {type} 
 * @retval: None
 */
void AMT1450_UART_Cmd(FunctionalState NewState)
{
	assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    USART_Cmd(UART5, ENABLE);
  }
  else
  {
    USART_Cmd(UART5, DISABLE);
  }
}
/**
 * @brief: SUM校验
 * @param {type} 
 * @retval: None
 */
uint8_t check_SUM(uint8_t* pData, uint8_t length){
    uint16_t sum = 0;
    for (uint8_t i=0; i<length - 1;i++){
        sum += pData[i];
    }
    if ((uint8_t)sum == pData[length - 1])
        return 1;
    return 0;
}
/**
 * @brief: 一个字节中断接收
 * @param {type} 
 * @retval: None
 */
void UART_GetChar(uint8_t pData)
{
		// 串口 
		if (amt1450_UART_Rx.RxDataPtr >= 9) {
				amt1450_UART_Rx.RxDataPtr= 0;
			return;
		}
    if (amt1450_UART_Rx.RxDataPtr == 0 && pData == AMT1450_FRAME_START){        // 帧头
        amt1450_UART_Rx.RxData[amt1450_UART_Rx.RxDataPtr++] = pData;
    }
    else if (amt1450_UART_Rx.RxDataPtr == 1)  {
			amt1450_UART_Rx.RxData[amt1450_UART_Rx.RxDataPtr++] = pData;      // num
	}
    else if(amt1450_UART_Rx.RxDataPtr < ((amt1450_UART_Rx.RxData[1]&0x0f)+3)){
        amt1450_UART_Rx.RxData[amt1450_UART_Rx.RxDataPtr++] = pData;
        // 
        if (amt1450_UART_Rx.RxDataPtr == ((amt1450_UART_Rx.RxData[1]&0x0f)+3)){ // 接收完成
            // 校验
            if (check_SUM(amt1450_UART_Rx.RxData, amt1450_UART_Rx.RxDataPtr)){
                for (uint8_t i=0;i<((amt1450_UART_Rx.RxData[1]&0x0f) + 3);i++)
                    amt1450_UART_Rx.ValidData[i] = amt1450_UART_Rx.RxData[i];		// 数据存储
                // printf ("%d\n", amt1450_UART_Rx.ValidData[1]&0x0f);
            }
            amt1450_UART_Rx.RxDataPtr = 0;
        }
    }
    else amt1450_UART_Rx.RxDataPtr = 0;
}
/**
 * @brief: 获取amt1450数据（压缩数据）
 * @param {type} 
* @retval: begin_Color: 1/黑；0/白
 */
void get_AMT1450Data_UART(uint8_t *begin_Color,uint8_t *jump_Count,uint8_t *jump_Location)
{
    *begin_Color = (amt1450_UART_Rx.ValidData[1] & 0x80) >>7;
    *jump_Count = amt1450_UART_Rx.ValidData[1] & 0x0f;
    for (uint8_t i=0;i<*jump_Count;i++){
      jump_Location[i] = amt1450_UART_Rx.ValidData[2+i];
    }
}


uint8_t begin,jump,count[6];			// 最大6个跳变，即3条线
uint8_t position;
void amt1450_Test_UART(void)
{
    
	  while(1){
		    get_AMT1450Data_UART(&begin, &jump, count);
			  if(jump == 2) position = 0.5f * (count[0] + count[1]);		
		    Delay_ms(10);
		}
}


/**
 * @brief: 中断函数
 * @param {type} 
 * @retval: None
 */
void UART5_IRQHandler(void)
{
		if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)	   //判断读寄存器是否为空
		{	
				USART_ClearITPendingBit(UART5,USART_IT_RXNE); // 清中断标志
				/* Read one byte from the receive data register */
				uint8_t getchar = USART_ReceiveData(UART5);   //将读寄存器的数据缓存到接收缓冲区里
			//printf("%c", getchar);
			  UART_GetChar(getchar);
		}
		if(USART_GetITStatus(UART5, USART_IT_TXE) != RESET)     //这段是为了避免STM32 USART第一个字节发送不出去的BUG 
		{ 
				USART_ITConfig(UART5, USART_IT_TXE, DISABLE);           //禁止发送缓冲器空中断
		}	
}


