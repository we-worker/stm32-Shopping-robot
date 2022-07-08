#include "stm32f4xx.h"
#include "ArmSolution.h"
#include "delay.h"
#include "math.h"
#include "NanoCommunication.h"

#define PWM_DUTY_LIMIT 10000 // PWMռ�ձȷ�Χ0~10000,����20ms    250-1250 ���� 0-180��

//��е�۲�����ʼ��
#define L1 120
#define L2 40
#define L3 118
#define L4 35
#define L5 163
#define L6 22
#define x1 (0)
#define y1 0
#define pi 3.1415926f

//��е�ۻ����ƶ���Ŀ��pwm
uint16_t Slow_pwm1 = 250;
uint16_t Slow_pwm2 = 250;
uint16_t Slow_pwm3 = 250;
uint16_t Slow_pwm4 = 250;
uint16_t Slow_pwm5 = 750;
uint16_t Slow_pwm6 = 750;
uint16_t Slow_pwm7 = 250;
uint16_t Slow_pwm8 = 250;

//����Ŀ��λ�ü�¼������λ�����͹���������
int Object_pos[6][2] = {0};
uint8_t Object_pos_index = 0; //��ǰץȡĿ��

/*���������ʼ��(��е��)*/
void ArmDriver_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* GPIOE clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* GPIOE Configuration: TIM9 CH2 (PE6)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*	����Gpio����*/
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM13);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM14);

	/* TIM clock enable ��ʱ������ */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

	/* Time base configuration TIM�������� */
	TIM_TimeBaseStructure.TIM_Period = PWM_DUTY_LIMIT - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 336 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 336 / 2 - 1;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

	//����PWM���
	/* PWM Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = PWM_DUTY_LIMIT / 2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	//����Timͨ��
	TIM_OC1Init(TIM9, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM9, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);

	TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);

	TIM_OC1Init(TIM13, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM13, TIM_OCPreload_Enable);
	TIM_OC1Init(TIM14, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);

	//ʹ�ܶ�ʱ��
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	TIM_ARRPreloadConfig(TIM9, ENABLE);
	TIM_ARRPreloadConfig(TIM13, ENABLE);
	TIM_ARRPreloadConfig(TIM14, ENABLE);

	/* TIM9 enable counter ���ü��� */
	TIM_Cmd(TIM9, ENABLE);
	TIM_Cmd(TIM5, ENABLE);
	TIM_Cmd(TIM13, ENABLE);
	TIM_Cmd(TIM14, ENABLE);

	//��ʼ��һ�£�
	TIM9->CCR1 = 500;
	TIM9->CCR2 = 0;
	TIM5->CCR1 = 500;
	TIM5->CCR2 = 750;
	TIM5->CCR3 = 750;
	TIM5->CCR4 = 750;
	TIM13->CCR1 = 500;
	TIM14->CCR1 = 500;

	//���������ö�ʱ��7��ʵ�ֻ�е�ۻ����ƶ�
	RCC_ClocksTypeDef RCC_Clocks;		 // RCCʱ�ӽṹ��
	NVIC_InitTypeDef NVIC_InitStructure; // NVIC�ж������ṹ��

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	RCC_GetClocksFreq(&RCC_Clocks); //��ȡϵͳʱ��
	//Ԥ��Ƶֵ�ļ��㷽����ϵͳʱ��Ƶ��/TIM6����ʱ��Ƶ�� - 1
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(RCC_Clocks.SYSCLK_Frequency / 20000) - 1; //����Ƶ��10KHz
	//ʹTIM6���Ƶ�ʵļ��㷽����TIM6����ʱ��Ƶ��/��ARR+1���������ARR����TIM_Period��ֵ�����9�����TIM6����ʱ��Ƶ��Ϊ10K�����������Ϊ1ms
	TIM_TimeBaseStructure.TIM_Period = (uint16_t)30 * 10 - 1; //Ҳ����50msһ��
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

	/* Enable the TIM6 Update Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE); //������ж�
	TIM_Cmd(TIM7, ENABLE);
}

/**
 * @description: ���ö���Ƕ�
 * @param {int} nServo���Ŷ��
 * @param {float} angle�Ƕ�(0-180)
 * @return {*}
 */
void SetServoAngle(int nServo, float angle)
{
	if (angle < 0)
		return;
	int pwm = angle * 1.0f / 90 / 20 * PWM_DUTY_LIMIT + 250; //�������Ӧ��pwm��

	if (pwm > 1250)
		pwm = 1250;
	if (pwm < 250)
		pwm = 250;

	switch (nServo)
	{
	case 1:
		Slow_pwm1 = pwm;
		break;
	case 2:
		Slow_pwm2 = pwm;
		break;
	case 3:
		Slow_pwm3 = pwm;
		break;
	case 4:
		Slow_pwm4 = pwm;
		break;
	case 5:
		Slow_pwm5 = pwm;
		break;
	case 6:
		Slow_pwm6 = pwm;
		break;
	case 7:
		Slow_pwm7 = pwm;
		break;
	case 8:
		Slow_pwm8 = pwm;
		break;
	default:
		break;
	}
}

/**
 * @description: ArmSolution ��е��λ�ý���
 * @param {double} x������(-200��50)
 * @param {double} y������(-50��250)
 * @return {*}�Զ����ú�1��2�ź�3�Ŷ���ĽǶ�
 */
void ArmSolution(double x, double y)
{

	float A = sqrt((L5 - L4) * (L5 - L4) + L6 * L6);
	float B = sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
	float o4 = acos(1.0f * (B * B + L1 * L1 - A * A) / 2 / B / L1);

	float o5;
	if (x - x1 == 0)
		o5 = -pi / 2;
	else
		o5 = atan(1.0 * (y - y1) / (x - x1));

	float o1 = pi - o4 + o5;
	if (__ARM_isnanf(o1))
	{
		printf("�������\n");
		return;
	}
	//printf("o1=%.2lf %.2lf\n", o1, o1 * 360 / 2 / pi);

	o4 = acos(1.0f * (B * B - L1 * L1 + A * A) / 2 / B / A);
	float o6 = acos(1.0f * ((L5 - L4) * (L5 - L4) + A * A - L6 * L6) / 2 / A / (L5 - L4));

	o5 = pi / 2 + o5;
	float o3 = pi - o4 - o5 - o6;

	//printf("o3=%.2lf %.2lf\n", o3, o3 * 360 / 2 / pi);

	float a = x + L5 * sin(o3) + L6 * cos(o3) - x1;
	float b = y + L5 * cos(o3) - L6 * sin(o3) - y1;
	float t = 1.0f * (a * a + b * b - L3 * L3 + L2 * L2) / 2 / L2 / sqrt(a * a + b * b);
	// printf("%.2lf",2*a*L2*2*a*L2+2*b*L2*2*b*L2);
	float fi = atanf(-1.0 * b / a);
	if (t > 1 || t < -1)
	{
		printf("�������\n");
		return;
	}
	float o2 = asinf(t) - fi;
	if (o2 < 0)
		o2 += pi;

	//printf("o2=%.2lf %.2lf\n", o2, o2 * 360 / 2 / 3.14159f);

	o4 = o3;
	o3 = (3 / 2 * pi - o3);

	//��ʵ�������
	o1 = o1 * 180 / pi;
	o2 = o2 * 180 / pi;
	o4 = o4 * 180 / pi;
	if (o4 < 35)
		o4 =35;

	o1 = o1 - 23.93f - (o1 - 90) / 7;
	o2 = 180 - o2+45;
	// o4=o4+20;

	//printf("��1��%.2f	��2��%.2f	��4��%.2f\n", o1, o2, o4);

	SetServoAngle(1, o1);
	SetServoAngle(2, o2);
	SetServoAngle(3, o4);
}

/**
 * @description: ��е��ץȡ����
 * @param {*}�Զ�����Object_pos[Object_pos_index]Ŀ��λ�ý�������ץȡ
 * @return {*}
 */
void Arm_Grab()
{
	// arm_height=60;
	// SetServoAngle(6,90);
	// SetServoAngle(5, 90);
	extern uint8_t car_flag;
	char target_pos[]="U_L";
	if (car_flag == Car_Grab_Normal)
	{

		//����λ��ǿ�ƹ̶�
		if (Object_pos[Object_pos_index][1] >= 40){
			Object_pos[Object_pos_index][1] = 135;
			target_pos[0]='U';//��
		}
		else{
			Object_pos[Object_pos_index][1] = -32;
			target_pos[0]='D';//��
		}
//ԭ����200,300,40
		if (Object_pos[Object_pos_index][0] < 235	){
			target_pos[2]='R';//��
			Object_pos[Object_pos_index][0] = 19;
		}
		else if (Object_pos[Object_pos_index][0] < 350){
			target_pos[2]='M';//��
			Object_pos[Object_pos_index][0] = 29;
		}
		else{
			target_pos[2]='L';//��
			Object_pos[Object_pos_index][0] = 40; //����
		}
		printf("%s\n",target_pos);

		SetServoAngle(5, 100); //��צ��Сһ��
		SetServoAngle(6, Object_pos[Object_pos_index][0]);
		if (Object_pos[Object_pos_index][1] >= 135)
		{						   //����������
			ArmSolution(-30, 210); //�Ⱦٸ�һ��
			Delay_ms(2000);
			ArmSolution(-210, Object_pos[Object_pos_index][1]);
		}
		//����������
		if (Object_pos[Object_pos_index][1] <= 0)
		{
			ArmSolution(-100, 50); //�Ⱦٸ�һ��
			Delay_ms(2000);
			ArmSolution(-200, Object_pos[Object_pos_index][1]);//-171
		}
		
		

		Delay_ms(1000);

		SetServoAngle(5, 120); //ץ����צ��

		Delay_ms(1000); //���ҶԱ�һ��λ��
		if (Object_pos[Object_pos_index][1] >= 135)
		{						   //����������
			ArmSolution(-70, 140); //����һ�£�����ĵ�
		}
		else
		{
			ArmSolution(-110, 0); //����һ�£�����ĵ�
		}
		Delay_ms(1000);

		ArmSolution(-80, 100); //

		SetServoAngle(6, 145); //�ŵ�����
		Delay_ms(1000);
		ArmSolution(-147, 75); //��ǰ��һ��
		Delay_ms(1000);
		SetServoAngle(5, 75); //����
		Delay_ms(1000);
		//ץ����ʼ��
		//SetServoAngle(6, 85);
		SetServoAngle(6, 29);
		ArmSolution(-120, 20); //λ�ûع�
		Delay_ms(1500);

		
	}
	else//��һ��ץȡ��ʽ
	{
	}
	//ץȡĿ����й�0
	printf("ץȡ��һ��\n");
	Object_pos[Object_pos_index][0] = 0;
	Object_pos[Object_pos_index][0] = 0;
	Object_pos_index++;
	if ((Object_pos[Object_pos_index][1] == 0 && Object_pos[Object_pos_index][0] == 0) || Object_pos_index >= 6)
	{

		Object_pos_index = 0;
		printf("��ץȡ���\n");
		//�������غ���
		
	}
}

/*��������ƶ����������ÿ�����Ҫ����������ı�*/
void Slow_Pwm(uint8_t nServo)
{
	int8_t flag = 1;
	switch (nServo)
	{
	case 1:
		if (TIM9->CCR1 > Slow_pwm1)
			flag = -1;
		if (TIM9->CCR1 == Slow_pwm1)
			flag = 0;
		if (-flag * TIM9->CCR1 + flag * Slow_pwm1 > 10)
			flag = 10 * flag;
		TIM9->CCR1 = TIM9->CCR1 + flag;
		break;
	case 2:
		if (TIM9->CCR2 > Slow_pwm2)
			flag = -1;
		if (TIM9->CCR2 == Slow_pwm2)
			flag = 0;
		if (-flag * TIM9->CCR2 + flag * Slow_pwm2 > 10)
			flag = 10 * flag;
		TIM9->CCR2 = TIM9->CCR2 + flag;
		break;
	case 3:
		if (TIM5->CCR1 > Slow_pwm3)
			flag = -1;
		if (TIM5->CCR1 == Slow_pwm3)
			flag = 0;
		if (-flag * TIM5->CCR1 + flag * Slow_pwm3 > 10)
			flag = 10 * flag;
		TIM5->CCR1 = TIM5->CCR1 + flag;
		break;
	case 4:
		if (TIM5->CCR2 > Slow_pwm4)
			flag = -1;
		if (TIM5->CCR2 == Slow_pwm4)
			flag = 0;
		if (-flag * TIM5->CCR2 + flag * Slow_pwm4 > 10)
			flag = 10 * flag;
		TIM5->CCR2 = TIM5->CCR2 + flag;
		break;
	case 5:
		if (TIM5->CCR3 > Slow_pwm5)
			flag = -1;
		if (TIM5->CCR3 == Slow_pwm5)
			flag = 0;
		if (-flag * TIM5->CCR3 + flag * Slow_pwm5 > 10)
			flag = 10 * flag;
		TIM5->CCR3 = TIM5->CCR3 + flag;
		break;
	case 6:
		if (TIM5->CCR4 > Slow_pwm6)
			flag = -1;
		if (TIM5->CCR4 == Slow_pwm6)
			flag = 0;
		if (-flag * TIM5->CCR4 + flag * Slow_pwm6 > 10)
			flag = 10 * flag;
		TIM5->CCR4 = TIM5->CCR4 + flag;
		break;
	case 7:
		if (TIM13->CCR1 > Slow_pwm7)
			flag = -1;
		if (TIM13->CCR1 == Slow_pwm7)
			flag = 0;
		if (-flag * TIM13->CCR1 + flag * Slow_pwm7 > 10)
			flag = 10 * flag;
		TIM13->CCR1 = TIM13->CCR1 + flag;
		break;
	case 8:
		if (TIM14->CCR1 > Slow_pwm8)
			flag = -1;
		if (TIM14->CCR1 == Slow_pwm8)
			flag = 0;
		if (-flag * TIM14->CCR1 + flag * Slow_pwm8 > 10)
			flag = 10 * flag;
		TIM14->CCR1 = TIM14->CCR1 + flag;
		break;
	default:
		break;
	}
}

/*����Ϊ��е�ۻ����ƶ�ʹ�õ�tim7�ж�*/
void TIM7_IRQHandler(void)
{
	//�Ƿ��и����ж�
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		//����жϱ�־
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		//�����ж�
		Slow_Pwm(1);
		Slow_Pwm(2);
		Slow_Pwm(3);
		Slow_Pwm(5);
		Slow_Pwm(6);
		Slow_Pwm(8);
	}
}
