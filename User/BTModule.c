#include "BTModule.h"
#include "MotorController.h"
__IO uint8_t nRx2Counter=0; //接收字节数
__IO uint8_t USART_Rx2Buff[FRAME_BYTE_LENGTH]; //接收缓冲区
__IO uint8_t USART_FrameFlag = 0; //接收完整数据帧标志，1完整，0不完整
void USART2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;  //NVIC中断向量结构体
	/* config USART2 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	 /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
	
	/* USART2 GPIO config */ 
	/* Configure USART2 Tx (PD.05) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_UP;	
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	/* Configure USART2 Rx (PD.06) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_UP;	
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	

	
  /* Enable the USART OverSampling by 8 */
  USART_OverSampling8Cmd(USART2, ENABLE);  	

	/* USART2 mode config */
	USART_InitStructure.USART_BaudRate = 115200;
//	USART_InitStructure.USART_BaudRate = 19200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	
	/* Enable USART2 Receive and Transmit interrupts */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 

	USART_Cmd(USART2, ENABLE);
	
		//使能USART2中断
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);	
}

/******************************************************
		格式化串口输出函数
        "\r"	回车符	   USART_OUT(USART1, "abcdefg\r")   
		"\n"	换行符	   USART_OUT(USART1, "abcdefg\r\n")
		"%s"	字符串	   USART_OUT(USART1, "字符串是：%s","abcdefg")
		"%d"	十进制	   USART_OUT(USART1, "a=%d",10)
**********************************************************/
void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...){ 
	const char *s;
    int d;
    char buf[16];
    va_list ap;
    va_start(ap, Data);

	while(*Data!=0){				                          //判断是否到达字符串结束符
		if(*Data==0x5c){									  //'\'
			switch (*++Data){
				case 'r':							          //回车符
					USART_SendData(USARTx, 0x0d);	   
					Data++;
					break;
				case 'n':							          //换行符
					USART_SendData(USARTx, 0x0a);	
					Data++;
					break;
				
				default:
					Data++;
				    break;
			}
	
			 
		}
		else if(*Data=='%'){									  //
			switch (*++Data){				
				case 's':										  //字符串
                	s = va_arg(ap, const char *);
                	for ( ; *s; s++) {
                    	USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                	}
					Data++;
                	break;
            	case 'd':										  //十进制
                	d = va_arg(ap, int);
                	itoa(d, buf, 10);
                	for (s = buf; *s; s++) {
                    	USART_SendData(USARTx,*s);
						while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
                	}
					Data++;
                	break;
				default:
					Data++;
				    break;
			}		 
		}
		else USART_SendData(USARTx, *Data++);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);
	}
}
/******************************************************
		整形数据转字符串函数
        char *itoa(int value, char *string, int radix)
		radix=10 标示是10进制	非十进制，转换结果为0;  

	    例：d=-379;
		执行	itoa(d, buf, 10); 后
		
		buf="-379"							   			  
**********************************************************/
char *itoa(int value, char *string, int radix)
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;
    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

} /* NCL_Itoa */

//重定向printf到串口
int fputc(int ch, FILE *f)
{
 	USART_SendData(USART2, (unsigned char) ch);
 	while( USART_GetFlagStatus(USART2,USART_FLAG_TC)!= SET);
	return (ch);
}

void USART_GetChar(uint8_t nChar) //串口接收到一个字节
{
//	if(USART_FrameFlag == 1) return;   //如果上次的数据帧还没处理过，则返回
//	
//	USART_Rx2Buff[nRx2Counter++]=nChar;  //保存到缓冲区
//	if(nChar == CMD_BYTE_END)   //如果是帧结束标志
//	{
//		if(nRx2Counter == CMD_BYTE_LENGTH)  //如果帧结束标志到了，而且接收的字节数正好等于帧要求字节数，则接收成功，置标志为1
//		{
//			USART_FrameFlag = 1;
//		}
//		nRx2Counter=0;
//	}
//	else if(nRx2Counter>=CMD_BYTE_LENGTH)
//	{
//		nRx2Counter = 0;
//	}
	if(USART_FrameFlag == 1) return;   //如果上次的数据帧还没处理过，则返回
	
	if(nRx2Counter==0 && nChar == FRAME_START)
	{
		USART_Rx2Buff[nRx2Counter++]=nChar;  //保存到缓冲区
	}
	else if(nRx2Counter>0) //接收到帧头以后才继续保存
	{
		USART_Rx2Buff[nRx2Counter++]=nChar;  //保存到缓冲区
		if(nRx2Counter>=FRAME_BYTE_LENGTH)  //接收到一帧数据
		{
			nRx2Counter = 0;
			if(USART_Rx2Buff[FRAME_BYTE_LENGTH-1] == FRAME_END) //如果最后一个字节是帧尾，则数据帧完整
			{
				USART_FrameFlag=1;
			}
		}
	}	
}

void USART_Process(void) //处理数据帧
{
//	uint8_t i;
	uint16_t nVal;
	float fKp,fKi,fKd;
	if(USART_FrameFlag == 1)
	{
		//将数据原封不动发送回去
		for(uint8_t i=0;i<FRAME_BYTE_LENGTH;i++)
		{
			USART_SendData(USART2,USART_Rx2Buff[i]);
			while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET);
		}
		
		if(USART_Rx2Buff[1] == 0x11) //如果命令字节等于0x11，则是设置PID参数指令，这些协议可以自己定义
		{
			//Kp值
			nVal = (USART_Rx2Buff[2]<<8) + USART_Rx2Buff[3];
			fKp = (float)nVal/(float)100.0;   //传输时放大100倍，譬如Kp=2.3，传输时为230整数
			//Ki值
			nVal = (USART_Rx2Buff[4]<<8) + USART_Rx2Buff[5];
			fKi = (float)nVal/(float)100.0;
			//Kp值
			nVal = (USART_Rx2Buff[6]<<8) + USART_Rx2Buff[7];
			fKp = (float)nVal/(float)100.0;
			MotorController_SetPIDParam(fKp,fKi,fKd);
		}
		else if(USART_Rx2Buff[1] == 0x12) //如果命令字节等于0x12，则是设置加速度参数指令，这些协议可以自己定义
		{
			//2、3字节是加速度值
			nVal = (USART_Rx2Buff[2]<<8) + USART_Rx2Buff[3];
			MotorController_SetAcceleration(nVal);
		}
		//处理完毕，将标志清0
		USART_FrameFlag = 0; 
	}
}


/**
  * @brief  This function handles USART2 interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)      //串口2中断服务
{

	uint8_t getchar;
  if(USART_GetITStatus(USART2, USART_IT_RXNE) !=RESET)	   //判断读寄存器是否为空
  {	

    getchar = USART_ReceiveData(USART2);   //将读寄存器的数据缓存到接收缓冲区里
	
    USART_GetChar(getchar);
  }


  if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)                   //这段是为了避免STM32 USART第一个字节发送不出去的BUG 
  { 
     USART_ITConfig(USART2, USART_IT_TXE, DISABLE);					     //禁止发送缓冲器空中断
  }	
  
}
