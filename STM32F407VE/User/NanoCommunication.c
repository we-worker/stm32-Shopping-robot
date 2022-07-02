
#include "BTModule.h"
#include "NanoCommunication.h"
#include "ArmSolution.h"

uint8_t car_flag = 0; //С��״̬

void USART1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; // NVIC�ж������ṹ��
	/* config USART1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* Connect USART pins to AF7 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	/* USART1 GPIO config */
	/* Configure USART1 Tx (PD.05) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* Configure USART1 Rx (PD.06) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Enable the USART OverSampling by 8 */
	USART_OverSampling8Cmd(USART1, ENABLE);

	/* USART1 mode config */
	USART_InitStructure.USART_BaudRate = 9600;
	//	USART_InitStructure.USART_BaudRate = 19200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	/* Enable USART1 Receive and Transmit interrupts */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

	USART_Cmd(USART1, ENABLE);

	//ʹ��USART1�ж�
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/******************************************************
		��ʽ�������������
		"\r"	�س���	   USART_OUT(USART1, "abcdefg\r")
		"\n"	���з�	   USART_OUT(USART1, "abcdefg\r\n")
		"%s"	�ַ���	   USART_OUT(USART1, "�ַ����ǣ�%s","abcdefg")
		"%d"	ʮ����	   USART_OUT(USART1, "a=%d",10)
**********************************************************/
void USART1_OUT(USART_TypeDef *USARTx, uint8_t *Data, ...)
{
	const char *s;
	int d;
	char buf[16];
	va_list ap;
	va_start(ap, Data);

	while (*Data != 0)
	{ //�ж��Ƿ񵽴��ַ���������
		if (*Data == 0x5c)
		{ //'\'
			switch (*++Data)
			{
			case 'r': //�س���
				USART_SendData(USARTx, 0x0d);
				Data++;
				break;
			case 'n': //���з�
				USART_SendData(USARTx, 0x0a);
				Data++;
				break;

			default:
				Data++;
				break;
			}
		}
		else if (*Data == '%')
		{ //
			switch (*++Data)
			{
			case 's': //�ַ���
				s = va_arg(ap, const char *);
				for (; *s; s++)
				{
					USART_SendData(USARTx, *s);
					while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
						;
				}
				Data++;
				break;
			case 'd': //ʮ����
				d = va_arg(ap, int);
				itoa(d, buf, 10);
				for (s = buf; *s; s++)
				{
					USART_SendData(USARTx, *s);
					while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
						;
				}
				Data++;
				break;
			default:
				Data++;
				break;
			}
		}
		else
			USART_SendData(USARTx, *Data++);
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
			;
	}
}

/**
 * @description: ��λ���������ݽ��ܲ��ַ�Ϊstart��over������num num
 * @param {volatile uint8_t} *USART_Rx2Buff
 * @return {*}
 */
uint8_t start_pos_flag = 0; //��ʼ��¼Ŀ������ı�־λ
void USART1_Process_target(volatile uint8_t *USART_Rx2Buff)
{
	if(car_flag == Car_Driving){
		return;
	}
	if (USART_Rx2Buff[1] == 'N' && car_flag == Car_Waiting) // Noneû��Ŀ��
	{
		car_flag= Car_Driving;
		return;
	}

	if (USART_Rx2Buff[1] == 's' && car_flag == Car_Waiting && start_pos_flag == 0) //��ʼ��¼Ŀ����Ʒ����
	{
		Object_pos_index = 0;
		start_pos_flag = 1; //����¼����
		printf("start!!!");
		//����Ϊ���
		for (int i = 0; i < 6; i++)
		{
			Object_pos[i][0] = 0;
			Object_pos[i][1] = 0;
		}

		return;
	}
	if (USART_Rx2Buff[1] == 'o' && car_flag == Car_Waiting && start_pos_flag == 1) // over����Ŀ�괫�����
	{
		printf("Over!!!");
		for (int i = 0; i < Object_pos_index; i++)
		{
			printf("Ŀ��λ��i:(%d,%d)\n", Object_pos[i][0], Object_pos[i][1]);
		}

		Object_pos_index = 0;
		//TODO:��Ҫ��һ���жϣ����С���ڲֿ�������Car_Grab_Store
		car_flag = Car_Grab_Normal; //��Ҫ��һ���жϣ����С���ڲֿ�������Car_Grab_Store
		start_pos_flag = 0;			//�ȴ���һ�ο�ʼ��¼����

		return;
	}

	if (car_flag == Car_Grab_Normal || car_flag == Car_Grab_Store || start_pos_flag == 0)
	{ //����ץȡ
		return;
	}

	int dir = 0;
	uint8_t i;
	int arm_height = 0;
	for (i = 1; USART_Rx2Buff[i] != ' '; i++)
	{
		dir = (dir * 10) + (USART_Rx2Buff[i] - '0');
	}
	for (i = i + 1; i < FRAME_BYTE_LENGTH - 1 && USART_Rx2Buff[i] != 0x5A; i++)
	{
		arm_height = (arm_height * 10) + (USART_Rx2Buff[i] - '0');
	}
	// printf("dir:%d height:%d\n",dir,arm_height);

	arm_height = 1.0 * arm_height / 480 * 300-80;//�������ͻ�е�������ת��

	// float arm_dir = 65 - 1.0 * dir / 640 * 62;

	//����Object_pos��
	extern int Object_pos[6][2];
	extern uint8_t Object_pos_index;

	Object_pos[Object_pos_index][0] = dir;
	Object_pos[Object_pos_index][1] = arm_height;
	Object_pos_index++;
}

//���Ժ���������D1231������ĳ��λ�ã���׼����
void USART1_Process_target_test(volatile uint8_t *USART_Rx2Buff)
{
	if (USART_Rx2Buff[1] == 'D')
	{
		uint8_t i;
		int num = 0;
		for (i = 2; i < FRAME_BYTE_LENGTH - 1 && USART_Rx2Buff[i] != 0x5A; i++)
		{
			num = (num * 10) + (USART_Rx2Buff[i] - '0');
		}

		SetServoAngle(6, num); //�ŵ�����
		printf("��ǰ����Ƕ�:%d\n", num);
		return;
	}
	if (USART_Rx2Buff[1] <= '9' || USART_Rx2Buff[1] >= '0')
	{
		uint8_t i;
		int8_t fuhao_flag=1;
		int numx = 0;
		int numy = 0;
		for (i = 1; i < FRAME_BYTE_LENGTH - 1 && USART_Rx2Buff[i] != ' '; i++)
		{
			numx = (numx * 10) + (USART_Rx2Buff[i] - '0');
		}
		if(USART_Rx2Buff[i+1]=='-'){
			i+=1;
			fuhao_flag=-1;
		}
		for (i = i + 1; i < FRAME_BYTE_LENGTH - 1 && USART_Rx2Buff[i] != 0x5A; i++)
		{
			numy = (numy * 10) + (USART_Rx2Buff[i] - '0');
		}
		numy=numy*fuhao_flag;
		printf("��е�۸ߵ�:%d,%d\n", numx, numy);
		ArmSolution(-numx, numy);
		return;
	}
}

void USART1_Process(void) //��������֡
{
	//	uint8_t i;
	if (USART_FrameFlag == 1)
	{

		// printf("get");
		//������ԭ�ⲻ�����ͻ�ȥ
		//for (int i = 0; i < FRAME_BYTE_LENGTH; i++)
		//{
		//	USART_SendData(USART2, USART_Rx2Buff[i]);
		//	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
		//		;
		//}

		USART1_Process_target(USART_Rx2Buff);
		//USART1_Process_target_test(USART_Rx2Buff);
		//������ϣ�����־��0
		USART_FrameFlag = 0;
	}
	return;
}

/**
 * @brief  This function handles USART1 interrupt request.
 * @param  None
 * @retval None
 */
void USART1_IRQHandler(void) //����1�жϷ���
{
	uint8_t getchar;
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //�ж϶��Ĵ����Ƿ�Ϊ��
	{
		/* Read one byte from the receive data register */
		getchar = USART_ReceiveData(USART1); //�����Ĵ��������ݻ��浽���ջ�������

		USART_GetChar(getchar);
		USART1_Process();
	}

	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) //�����Ϊ�˱���STM32 USART��һ���ֽڷ��Ͳ���ȥ��BUG
	{
		USART_ITConfig(USART1, USART_IT_TXE, DISABLE); //��ֹ���ͻ��������ж�
	}
}
