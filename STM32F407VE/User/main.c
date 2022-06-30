#include "stm32f4xx.h"
#include "misc.h"
#include "delay.h"
#include "KeyLed.h"
#include "MotorDriver.h"
#include "MotorController.h"
#include "amt1450_uart.h"
#include "tcrt5000.h"
#include "PID.h"
#include "BTModule.h"
//#include "AStarRoute.h"
#include "DFS_Map.h"
#include "NanoCommunication.h"
#include "ArmSolution.h"

void NVIC_Configuration(void); //中断配置
void GPIO_Config(void);		   //通用输入输出端口配置
void System_Clock(void);	   //系统clock韩式

void get_runto_Grab();

//全局变量
uint16_t n10msCount;
uint8_t b100msFlag;
uint16_t n100msCount;

//定义预定速度
int32_t nSpeed;

//定义与地图行为有关的变量
extern int car_direction;

extern uint8_t begin, jump, count[6]; //循迹模块的相关参数
extern uint8_t line_position;		  //直线当前所在的位置

extern uint8_t car_flag; //小车运行状态

int main(void)
{
	n100msCount = 0;
	GPIO_Config();		  //端口初始化
	NVIC_Configuration(); //中断初始化
	Led_Init(1);		  //只使用LED2，LED3的引脚可以做其他用途
	Key_Init(2);		  //只使用Key1，Key2的引脚可以做其他用途
	init_AMT1450_UART();  // 初始化amt1450串口通信，串口使用UART5
	USART2_Init();
	USART1_Init();
	TCRT5000_config();
	Delay_ms(100);

	printf("Stm32_Waiting\n");
	
	while (Key_Released(2) == 0)
	{
	} //如果Key1l没有按下，则一直等待

	Delay_ms(10);
	AMT1450_UART_Cmd(ENABLE);

	//电机相关初始化。

	
	MotorDriver_Init(1);
	MotorDriver_Init(2);
	MotorDriver_Start(1, PWM_DUTY_LIMIT / 2);
	MotorDriver_Start(2, PWM_DUTY_LIMIT / 2);
	// MotorDriver_Start(3, PWM_DUTY_LIMIT / 2);
	// MotorDriver_Start(4, PWM_DUTY_LIMIT / 2);
	Encoder_Init(2);

	MotorController_Init(13500, 75, 2); //初始化调速器，参数1：轮子转一圈输出的脉冲个数；参数2：轮子直径，单位mm；参数3：几个电机需要调速
	MotorController_Enable(ENABLE);
	MotorController_SetAcceleration(10000); //设置加速度值，单位：mm/秒*秒
	Delay_ms(100);

	//机械臂相关初始化
	ArmDriver_Init();

	//定义一个循迹pid参数，并初始化。
	PID s_PID;
	s_PIDInit(&s_PID);

	nSpeed = 700;

	printf("Init_Finish\n");

	int t = 1;
	int height = 100;
	int flag = 1;
	// Arm_Grab();
	SetServoAngle(5, 75);
	ArmSolution(-120, 20);
	void Arm_test(int t, int height, int flag);
	car_flag=2;


	while (1)
	{

		Crossing_Detection();

		Map_Action(); //地图行为

		int32_t fpid_out = Follow_PID(&s_PID, line_position);			 //循迹pid
		MotorController_SetSpeed(2, fpid_out+250);				 //电机控制
		MotorController_SetSpeed(1, fpid_out-250);

		
		//Arm_test(t, height, flag);
		get_runto_Grab();
		

		/*
		if (car_flag == Car_Stop)
		{
			MotorController_SetSpeed(1, 0); //电机控制
			MotorController_SetSpeed(2, 0);
			Delay_ms(10000);
		}
		*/
		//后面是系统的滴答计时器
		System_Clock();
	}
}

void Arm_test(int t, int height, int flag)
{
	if (car_flag == Car_Grab_Normal || car_flag == Car_Grab_Store)
	{
		Arm_Grab();
	}

	if (b10msFlag == 1)
	{
		b10msFlag = 0; //把 10ms 标志位清零
		n10msCount++;
		t++;
		if (t % 5 == 0)
		{
			height = height + 1 * flag;
			if (height >= 180)
			{
				flag = -1;
			}
			if (height < 90)
			{
				flag = 1;
			}
			// ArmSolution(-120,100);
			SetServoAngle(5, height);
		}
	}
}
void get_runto_Grab()
{
	if (car_flag == Car_Grab_Normal)
	{
		if(Object_pos_index==0){//初次进入需要左转+直行一格子
			TurnBY_PID(90);
			Straight_go(200);
			MotorController_SetSpeed(1,0);
			MotorController_SetSpeed(2,0);
			Delay_ms(1000);
		}
		Arm_Grab();
		if(Object_pos_index==0){//如果抓取结束后，抓取位置还是0，就是抓取结束，需要退出
			Straight_back_mm(200,280);
			TurnBY_PID(-90);
			car_flag = Car_Driving; //车子状态制成行驶
			car_flag=Car_Waiting;//这里是测试，到时候删掉
			MotorController_SetSpeed(1,0);
			MotorController_SetSpeed(2,0);
			Delay_ms(10000);
		}
	}

	// if (b10msFlag == 1)
	// {
	// 	b10msFlag = 0; //把 10ms 标志位清零
	// 	n10msCount++;
	// 	t++;
	// 	if (t % 5 == 0)
	// 	{
	// 		height = height + 1 * flag;
	// 		if (height >= 180)
	// 		{
	// 			flag = -1;
	// 		}
	// 		if (height < 0)
	// 		{
	// 			flag = 1;
	// 		}
	// 		// ArmSolution(-120,100);
	// 		// SetServoAngle(4, height);
	// 	}
	// }
}

void System_Clock(void)
{
	//系统的滴答计时器
	if (b10msFlag == 1)
	{
		b10msFlag = 0; //把 10ms 标志位清零
		n10msCount++;
		// 10ms标志每10个计数等于100ms
		if (n10msCount % 10 == 0)
			b100msFlag = 1;

		if (n10msCount == 50) // 500ms时点亮LED
		{
			LED2_ON();
		}
		else if (n10msCount >= 100) // 1000ms时关闭LED，同时计数清零，计算一次速度值
		{
			LED2_OFF();
			n10msCount = 0;
		}
	}
}

/*中断配置函数*/
void NVIC_Configuration(void)
{
	/* Configure one bit for preemption priority */
	/* 优先级组 说明了抢占优先级所用的位数，和子优先级所用的位数。这里是2，2*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	//设置嘀嗒时钟中断
	if (SysTick_Config(SystemCoreClock / 100000))
	{
		/* Capture error */

		while (1)
			;
	}
}

/*端口配置函数*/
void GPIO_Config(void)
{
	//使能GPIOA/GPIOC的总线，否则端口不能工作，如果不用这个端口，可以不用使能。
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	  // PB2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //推挽输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
