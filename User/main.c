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

void NVIC_Configuration(void); //中断配置
void GPIO_Config(void);		   //通用输入输出端口配置

//全局变量
uint16_t n10msCount;
uint8_t b100msFlag;
uint16_t n100msCount;

//定义预定速度
int32_t nSpeed;

//定义与地图行为有关的变量
int map_index = 0;
int map_count = 0;
extern int car_direction;
//地图，前一个为行为，1，2，3，4分别为左转，右转，向后转，向左大转。后一个为第几个路口。
int map[][2] = {{1, 8}, {1, 13}, {2, 19}, {2, 20}, {2, 26}, {1, 28}, {3, 29}, {1, 30}, {1, 31}, {2, 33}, {2, 36}, {1, 42}, {1, 43}, {1, 44}, {1, 45}};

//地图行为，根据目前是在第几个路口，执行相关转向操作
void Map_Action(int *count)
{

	if (*count == map[map_index][1])
	{
		switch (map[map_index][0])
		{
		case 1:
			Straight_go_mm(300, 221 / 2); //走过车身的一半长
			TurnBY_PID(90);				  // Turn_I(0,300,90);
			Car_Direction_change(1);	  //小车方向转变，1为向左。
			break;
		case 2:
			Straight_go_mm(300, 221 / 2); //走过车身的一半长
			TurnBY_PID(-90);
			Car_Direction_change(-1);
			break;
		case 3:
			Straight_go_mm(300, 221 / 2); //走过车身的一半长
			TurnBY_PID(180);
			Car_Direction_change(2);
			break;
		case 4:
			Turn_I(850, 200, 90); //超大转，路口额外+1，因为会错过一个路口
			Car_Position_add(1);
			Car_Direction_change(1);
			*count += 2;
			Car_Position_add(1);
			break;
		default:
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
	USART2_Init();
	// TCRT5000_config();
	Delay_ms(100);

	while (Key_Released(2) == 0)
	{
	} //如果Key1没有按下，则一直等待

	Delay_ms(10);
	AMT1450_UART_Cmd(ENABLE);
	//	amt1450_Test_UART();        //测试AMT1450，在while中

	//电机相关初始化。
	MotorDriver_Init(2);
	MotorDriver_Start(1, PWM_DUTY_LIMIT / 2);
	MotorDriver_Start(2, PWM_DUTY_LIMIT / 2);
	MotorDriver_Start(3, PWM_DUTY_LIMIT / 2);
	MotorDriver_Start(4, PWM_DUTY_LIMIT / 2);
	Encoder_Init(2);

	MotorController_Init(330, 64, 2); //初始化调速器，参数1：轮子转一圈输出的脉冲个数；参数2：轮子直径，单位mm；参数3：几个电机需要调速
	MotorController_Enable(ENABLE);
	MotorController_SetAcceleration(8000); //设置加速度值，单位：mm/秒*秒
	Delay_ms(100);

	//定义一个循迹pid参数，并初始化。
	PID s_PID;
	s_PIDInit(&s_PID);

	nSpeed = 700;

	//这个标志变量用来判断是不是还在路口
	uint8_t crossing_flag = 0;

	while (1)
	{

		//
		//不断循环执行的代码块
		//

		// AMT1450循迹模块的使用，见https://www.luheqiu.com/deane/begin-smart_tracking_car/
		uint8_t begin, jump, count[6]; // 最大6个跳变，即3条线
		uint8_t position;
		get_AMT1450Data_UART(&begin, &jump, count); //讲数据存储在三个变量中
		if (jump == 2)
			position = 0.5f * (count[0] + count[1]); // position=两次跳变的中间位置，即线的位置

		//如果颜色没有跳变，且最左端为白色，且没进入路口，则
		if (jump == 0 && begin == 0 && crossing_flag == 1)
		{
			map_count++;		 //地图计数加1
			Car_Position_add(1); //小车位置加1
			crossing_flag = 0;	 //标志进入路口
		}
		//如果有线出现，那么代表驶出了路口，标志=1
		if (jump == 2)
		{
			crossing_flag = 1;
		}

		Map_Action(&map_count); //地图行为

		int32_t sp_out = Straight_PID(map_count, map[map_index][1]); //直走，目标距离控制pid
		int32_t fpid_out = Follow_PID(&s_PID, position);			 //循迹pid
		MotorController_SetSpeed(1, fpid_out - sp_out);				 //电机控制
		MotorController_SetSpeed(2, fpid_out + sp_out);				 //电机控制

		//后面这些不知道有什么用，也不敢删，下次问问学长。
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
