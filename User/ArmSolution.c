#include "stm32f4xx.h"
#include "ArmSolution.h"

#define PWM_DUTY_LIMIT 10000 // PWM占空比范围0~10000,代表20ms    250-1250 代表 0-180度
#include "math.h"

#define L1 120
#define L2 36
#define	L3 115
#define L4 33
#define L5 125


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

	/*	配置Gpio复用*/
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM13);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM14);

	/* TIM clock enable 定时器启用 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

	/* Time base configuration TIM基础设置 */
	TIM_TimeBaseStructure.TIM_Period = PWM_DUTY_LIMIT - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 336 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

	//配置PWM输出
	/* PWM Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = PWM_DUTY_LIMIT / 2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	//配置Tim通道
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

	//使能定时器
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	TIM_ARRPreloadConfig(TIM9, ENABLE);
	TIM_ARRPreloadConfig(TIM13, ENABLE);
	TIM_ARRPreloadConfig(TIM14, ENABLE);

	/* TIM9 enable counter 启用计数 */
	TIM_Cmd(TIM9, ENABLE);
	TIM_Cmd(TIM5, ENABLE);
	TIM_Cmd(TIM13, ENABLE);
	TIM_Cmd(TIM14, ENABLE);
}



void SetServoAngle(int nServo, double angle)
{
	if (angle < 0)
		return;
	int pwm = angle * 1.0 / 90 / 20 * PWM_DUTY_LIMIT + 250; //解算出对应的pwm波

	switch (nServo)
	{
	case 1:
		TIM9->CCR1 = pwm;
	case 2:
		TIM9->CCR2 = pwm;
	case 3:
		TIM5->CCR1 = pwm;
	case 4:
		TIM5->CCR2 = pwm;
	case 5:
		TIM5->CCR3 = pwm;
	case 6:
		TIM5->CCR4 = pwm;
	case 7:
		TIM13->CCR1 = pwm;
	case 8:
		TIM14->CCR1 = pwm;
	default:
		break;
	}
}
void ArmSolution(double x,double y){

	//求解角3
	double tsin=1.0*(x*x+y*y+L5*L5-L1*L1)/(2*L5*sqrt(x*x+y*y));
	double o3=3.1415926-asin(tsin)-atan(x/y);
	//求解角1
	tsin=1.0*(x*x+y*y+L1*L1-L5*L5)/(2*L1*sqrt(x*x+y*y));
	double o1=asin(tsin)-atan(x/y);
	//求解角2
	double t1=L1*cos(o1)-L4*cos(o3);
	double t2=L1*sin(o1)-L4*sin(o3);
	tsin=1.0*(t1*t1+t2*t2+L2*L2-L3*L3)/(2*L2*sqrt(t1*t1+t2*t2));
	double o2=asin(tsin)-atan(t1/t2);

	double o4=o3-3.14159;
	//printf("角1：%.2f	角2：%.2f	角3：%.2f	角4：%.2f",o1*360/6.28,o2*360/6.28,o3*360/6.28,o4*360/6.28);
}
