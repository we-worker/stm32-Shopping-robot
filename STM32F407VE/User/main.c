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

//全局变量
uint16_t n10msCount;
uint8_t b100msFlag;
uint16_t n100msCount;






//定义预定速度
int32_t nSpeed;

//定义与地图行为有关的变量
extern int car_direction;

extern uint8_t begin, jump, count[6];//循迹模块的相关参数
extern uint8_t line_position;//直线当前所在的位置

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

	
	printf("hello\n");
	
	while (Key_Released(2) == 0)
	{
	} //如果Key1没有按下，则一直等待

	Delay_ms(10);
	AMT1450_UART_Cmd(ENABLE);

	//电机相关初始化。
	
	MotorDriver_Init(1);
	MotorDriver_Init(2);
	MotorDriver_Start(1, PWM_DUTY_LIMIT / 2);
	MotorDriver_Start(2, PWM_DUTY_LIMIT / 2);
	MotorDriver_Start(3, PWM_DUTY_LIMIT / 2);
	MotorDriver_Start(4, PWM_DUTY_LIMIT / 2);
	Encoder_Init(2);

	MotorController_Init(13500, 75, 2); //初始化调速器，参数1：轮子转一圈输出的脉冲个数；参数2：轮子直径，单位mm；参数3：几个电机需要调速
	MotorController_Enable(ENABLE);
	MotorController_SetAcceleration(10000); //设置加速度值，单位：mm/秒*秒
	Delay_ms(100);
	
	
	//定义一个循迹pid参数，并初始化。
	PID s_PID;
	s_PIDInit(&s_PID);

	nSpeed = 700;

	ArmDriver_Init();


	printf("hello\n");
	int t=1;
	int height =100;int flag=1;
	//Arm_Grab();
	SetServoAngle(5, 75);
	ArmSolution(-120,20);
	
	
	while(1){

		extern int grab_flag;
		if(grab_flag==1){
			Arm_Grab();
		}

		if (b10msFlag == 1)
		{
			b10msFlag = 0; //把 10ms 标志位清零
			n10msCount++;
			// 10ms标志每10个计数等于100ms
			if (n10msCount % 10 == 0)
				b100msFlag = 1;
			
			t++;
			if(t%5==0){
					height=height+1*flag;
					if(height>=180){
						flag=-1;
					}
					if(height<0){
						flag=1;
					}
				//ArmSolution(-120,100);
				//SetServoAngle(4, height);
			}

			//Slow_Pwm(4);
		}



	}
	

	//Straight_back_mm(200,250);
	while (1)
	{

		//
		//不断循环执行的代码块
		//

		//Crossing_Detection();

		//Map_Action(); //地图行为

		//int32_t sp_out = Straight_PID(map_count, map[map_index][1]); //直走，目标距离控制pid
		//int32_t fpid_out = Follow_PID(&s_PID, line_position);			 //循迹pid
		//MotorController_SetSpeed(1, fpid_out+250);				 //电机控制
		//MotorController_SetSpeed(2, fpid_out-250);
			// MotorController_SetSpeed(1, 0); //电机控制
			// MotorController_SetSpeed(2, 0);
			// Delay_ms(10000);


		//后面是系统的滴答计时器
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
