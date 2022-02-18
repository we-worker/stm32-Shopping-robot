#include "stm32f4xx.h"
#include "misc.h"
#include "delay.h"
#include "KeyLed.h"
#include "MotorDriver.h"
#include "MotorController.h"
#include "amt1450_uart.h"
#include "tcrt5000.h"
#include "PID.h"

void NVIC_Configuration(void); //中断配置
void GPIO_Config(void);		   //通用输入输出端口配置

//全局变量
uint16_t n10msCount;
uint8_t b100msFlag;
uint16_t n100msCount;

int32_t nSpeed;

int map_index=0;
int map_count=0;

void Map_Action(int count){
	int map[]={1,1,2,2,2,1,3,1,1,2,2,1,1,1,1};//1左转 2右转 3 向后转
	int index[]={8,13,19,20,26,28,29,30,31,33,36,42,43,44,45};
	if (count==index[map_index]){
			Delay_ms(75);
		switch(map[map_index]){
			case 1:Turn_I(0,300,90);
				break;
			case 2:Turn_I(0,300,-90);
				break;
			case 3:Turn_I(0,300,180);
				break;
		}
		map_index++;
	}
}



int main(void)
{
	n100msCount = 0;
	GPIO_Config();		  //端口初始化
	NVIC_Configuration(); //中断初始化
	Led_Init(1);		  //只使用LED2，LED3的引脚可以做其他用途
	Key_Init(2);		  //只使用Key1，Key2的引脚可以做其他用途
	init_AMT1450_UART();  // 初始化amt1450串口通信，串口使用UART5
	TCRT5000_config();
	Delay_ms(100);

	while (Key_Released(1) == 0)
	{
	} //如果Key1没有按下，则一直等待

	Delay_ms(10);
	AMT1450_UART_Cmd(ENABLE);
	//	amt1450_Test_UART();        //测试AMT1450，在while中

	MotorDriver_Init(2);
	MotorDriver_Start(1, PWM_DUTY_LIMIT / 2);
	MotorDriver_Start(2, PWM_DUTY_LIMIT / 2);
	MotorDriver_Start(3, PWM_DUTY_LIMIT / 2);
	MotorDriver_Start(4, PWM_DUTY_LIMIT / 2);
	Encoder_Init(2);

	MotorController_Init(330, 82, 2); //初始化调速器，参数1：轮子转一圈输出的脉冲个数；参数2：轮子直径，单位mm；参数3：几个电机需要调速
	MotorController_Enable(ENABLE);
	MotorController_SetAcceleration(8000); //设置加速度值，单位：mm/秒*秒
	Delay_ms(100);

	PID s_PID;
	s_PIDInit(&s_PID);

	nSpeed = 700;
	
	while (1)
	{

		//
		//不断循环执行的代码块
		//

		// if(Key_Released(2)==1)  按键使用

		uint8_t begin, jump, count[6]; // 最大6个跳变，即3条线
		uint8_t position;
		uint8_t lastjump;
		get_AMT1450Data_UART(&begin, &jump, count); //讲数据存储在三个变量中
		if (jump == 2)
			position = 0.5f * (count[0] + count[1]); // position=两次跳变的中间位置
		if (jump !=2&&lastjump==2)
			map_count++;
		
		lastjump=jump;
		//Delay_ms(10);
		
		Map_Action(map_count);
		
		
		int32_t spid_out = Straight_PID(&s_PID, position);
		MotorController_SetSpeed(1, -nSpeed + spid_out);
		MotorController_SetSpeed(2, nSpeed + spid_out);

		
		
		
		
		
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
