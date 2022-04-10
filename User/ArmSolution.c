#include "stm32f4xx.h"
#include "ArmSolution.h"
#include "delay.h"
#include "math.h"


#define PWM_DUTY_LIMIT 10000 // PWM占空比范围0~10000,代表20ms    250-1250 代表 0-180度

#define L1 120
#define L2 45
#define L3 118
#define L4 35
#define L5 170
#define L6 22
#define x1 (0)
#define y1 0
#define pi 3.1415926f

uint16_t Slow_pwm1=250;
uint16_t Slow_pwm2=250;
uint16_t Slow_pwm3=250;
uint16_t Slow_pwm4=250;
uint16_t Slow_pwm5=250;
uint16_t Slow_pwm6=250;
uint16_t Slow_pwm7=250;
uint16_t Slow_pwm8=250;

float arm_angle4=90;
int height=60;

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
	
	TIM_TimeBaseStructure.TIM_Prescaler = 336/2 - 1;
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
	
	//初始化一下：
	TIM9->CCR1=500;
	TIM9->CCR2=0;
	TIM5->CCR1=2500;
	TIM5->CCR2=1500;
	TIM5->CCR3=700;
	TIM5->CCR4=750;
	TIM13->CCR1=500;
	TIM14->CCR1=500;

}



void SetServoAngle(int nServo, float angle)
{
	if (angle < 0)
		return;
	int pwm = angle * 1.0f / 90 / 20 * PWM_DUTY_LIMIT + 250; //解算出对应的pwm波
	
	if(pwm>1250)
		pwm=1250;
	if(pwm<250)
		pwm=250;
	
	switch (nServo)
	{
	case 1:
		Slow_pwm1=pwm;
		break;
	case 2:
		Slow_pwm2=pwm;
		break;
	case 3:
		Slow_pwm3=pwm;
		break;
	case 4:
		Slow_pwm4=pwm;
		break;
	case 5:
		Slow_pwm5=pwm;
		break;
	case 6:
		Slow_pwm6=pwm;
		break;
	case 7:
		Slow_pwm7=pwm;
		break;
	case 8:
		Slow_pwm8=pwm;
		break;
	default:
		break;
	}
}
void ArmSolution(double x,double y){

	  float A = sqrt((L5 - L4) * (L5 - L4) + L6 * L6);
    float B = sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
    float o4 = acos(1.0f * (B * B + L1 * L1 - A * A) / 2 / B / L1);

    float o5;
    if (x - x1 == 0)
        o5 = -pi / 2;
    else
        o5 = atan(1.0 * (y - y1) / (x - x1));

    float o1 = pi - o4 + o5;
    printf("o1=%.2lf %.2lf\n", o1, o1 * 360 / 2 / pi);

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
    if(t>1 || t<-1){
        return ;
    }
    float o2 = asinf(t) - fi;
    if (o2 < 0)
        o2 += pi;

    printf("o2=%.2lf %.2lf\n", o2, o2 * 360 / 2 / 3.14159);
	

	o4=o3;
	o3=(3/2*pi-o3);
		
	//真实舵机解算
	o1=o1*180/pi;
	o2=o2*180/pi;
	o4=o4*180/pi;
	
	
	o1=o1-23.93f-(o1-90)/7;
	o2=180-o2;
	//o4=o4+20;
	
	printf("角1：%.2f	角2：%.2f	角3：%.2f	角4：%.2f\n",o1,o2,o3,o4);
	
		
		SetServoAngle(1, o1);
		SetServoAngle(2, o2);
		SetServoAngle(3, o4);
}


void Slow_Pwm(uint8_t nServo)
{
	int8_t flag=1;
	switch (nServo)
	{
	case 1:
		if(TIM9->CCR1>Slow_pwm1)
			flag=-1;
		if(TIM9->CCR1==Slow_pwm1)
			flag=0;
		if(-flag*TIM9->CCR1+flag*Slow_pwm1>10)
			flag=10*flag;
		TIM9->CCR1 =TIM9->CCR1+flag;
		break;
	case 2:
		if(TIM9->CCR2>Slow_pwm2)
			flag=-1;
		if(TIM9->CCR2==Slow_pwm2)
			flag=0;
		if(-flag*TIM9->CCR2+flag*Slow_pwm2>10)
			flag=10*flag;
		TIM9->CCR2 =TIM9->CCR2+flag;
		break;
	case 3:
		if(TIM5->CCR1>Slow_pwm3)
			flag=-1;
		if(TIM5->CCR1==Slow_pwm3)
			flag=0;
		if(-flag*TIM5->CCR1+flag*Slow_pwm3>10)
			flag=10*flag;
		TIM5->CCR1=TIM5->CCR1+flag;
		break;
	case 4:
		if(TIM5->CCR2>Slow_pwm4)
			flag=-1;
		if(TIM5->CCR2==Slow_pwm4)
			flag=0;
		if(-flag*TIM5->CCR2+flag*Slow_pwm4>10)
			flag=10*flag;
		TIM5->CCR2 =TIM5->CCR2+flag;
		break;
	case 5:
		if(TIM5->CCR3>Slow_pwm5)
			flag=-1;
		if(TIM5->CCR3==Slow_pwm5)
			flag=0;
		if(-flag*TIM5->CCR3+flag*Slow_pwm5>10)
			flag=10*flag;
		TIM5->CCR3 =TIM5->CCR3+flag;
		break;
	case 6:
		if(TIM5->CCR4>Slow_pwm6)
			flag=-1;
		if(TIM5->CCR4==Slow_pwm6)
			flag=0;
		if(-flag*TIM5->CCR4+flag*Slow_pwm6>10)
			flag=10*flag;
		TIM5->CCR4 =TIM5->CCR4+flag;
		break;
	case 7:
		if(TIM13->CCR1>Slow_pwm7)
			flag=-1;
		if(TIM13->CCR1==Slow_pwm7)
			flag=0;
		if(-flag*TIM13->CCR1+flag*Slow_pwm7>10)
			flag=10*flag;
		TIM13->CCR1=TIM13->CCR1+flag;
		break;
	case 8:
		if(TIM14->CCR1>Slow_pwm8)
			flag=-1;
		if(TIM14->CCR1==Slow_pwm8)
			flag=0;
		if(-flag*TIM14->CCR1+flag*Slow_pwm8>10)
			flag=10*flag;
		TIM14->CCR1=TIM14->CCR1+flag;
		break;
	default:
		break;
	}
}

