#include "MotorController.h"
#include "MotorDriver.h"
__IO uint16_t MotorController_EncoderResolution= 390;
__IO uint8_t MotorController_WheelDiameter = 64;//原始64，32为偏差
__IO uint16_t MotorController_Acc=0;
__IO int16_t MotorController_MotorA_SpeedSet,MotorController_MotorB_SpeedSet,MotorController_MotorC_SpeedSet,MotorController_MotorD_SpeedSet;
__IO int16_t MotorController_MotorA_SpeedCur,MotorController_MotorB_SpeedCur,MotorController_MotorC_SpeedCur,MotorController_MotorD_SpeedCur;
__IO uint16_t MotorController_MotorA_SpeedPWM,MotorController_MotorB_SpeedPWM,MotorController_MotorC_SpeedPWM,MotorController_MotorD_SpeedPWM;
__IO int32_t MotorController_MotorA_EncCnt,MotorController_MotorB_EncCnt,MotorController_MotorC_EncCnt,MotorController_MotorD_EncCnt;
__IO float MotorController_MotorA_SpeedErr1,MotorController_MotorB_SpeedErr1,MotorController_MotorC_SpeedErr1,MotorController_MotorD_SpeedErr1;
__IO float MotorController_MotorA_SpeedErr2,MotorController_MotorB_SpeedErr2,MotorController_MotorC_SpeedErr2,MotorController_MotorD_SpeedErr2;
__IO uint8_t MotorController_MotorEnabledCount;  //需要调节的电机数量
__IO float MotorController_KP, MotorController_KI, MotorController_KD;  //PID参数


float  Motor_speed1=0;
float  Motor_speed2=0;

//MotorController_Init() 初始化函数
//nEncoderResolution编码器分辨率，轮子一圈的脉冲数；nWheelDiameter轮子的直径，单位：mm
//nMotorCount电机数量，如果为2，则开启A，B电机；如果为4，则开启A、B、C、D四个电机
void MotorController_Init(uint16_t nEncoderResolution, uint8_t nWheelDiameter,uint8_t nMotorCount) 
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_ClocksTypeDef RCC_Clocks;  //RCC时钟结构体
	NVIC_InitTypeDef NVIC_InitStructure;  //NVIC中断向量结构体
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); 
	
	RCC_GetClocksFreq(&RCC_Clocks);		//获取系统时钟
	//预分频值的计算方法：系统时钟频率/TIM6计数时钟频率 - 1
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (RCC_Clocks.SYSCLK_Frequency / 20000) - 1;  //计数频率10KHz
	//使TIM6溢出频率的计算方法：TIM6计数时钟频率/（ARR+1），这里的ARR就是TIM_Period的值，设成9，如果TIM6计数时钟频率为10K，则溢出周期为1ms
	TIM_TimeBaseStructure.TIM_Period =(uint16_t) MOTOR_CONTROLLER_PERIOD*10 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	
	/* Enable the TIM6 Update Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ClearFlag(TIM6, TIM_FLAG_Update);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE); //打开溢出中断
	
	MotorController_Enable(DISABLE);

	MotorController_EncoderResolution = nEncoderResolution;
	MotorController_WheelDiameter = nWheelDiameter;
	MotorController_Acc = 0;
	
	MotorController_MotorA_SpeedSet = 0;
	MotorController_MotorB_SpeedSet = 0;
	MotorController_MotorC_SpeedSet = 0;
	MotorController_MotorD_SpeedSet = 0;
	MotorController_MotorA_SpeedCur = 0;
	MotorController_MotorB_SpeedCur = 0;
	MotorController_MotorC_SpeedCur = 0;
	MotorController_MotorD_SpeedCur = 0;
	MotorController_MotorA_SpeedPWM = PWM_DUTY_LIMIT/2;
	MotorController_MotorB_SpeedPWM = PWM_DUTY_LIMIT/2;
	MotorController_MotorC_SpeedPWM = PWM_DUTY_LIMIT/2;
	MotorController_MotorD_SpeedPWM = PWM_DUTY_LIMIT/2;
	MotorController_MotorEnabledCount = nMotorCount;
	
	//初始化PID参数
	MotorController_KP = MOTOR_CONTROLLER_KP;
	MotorController_KI = MOTOR_CONTROLLER_KI;
	MotorController_KD = MOTOR_CONTROLLER_KD;
}
void MotorController_SetAcceleration(uint16_t nAcc) //设置轮子的加速度值，单位mm/s/s，设为0相当于最小值1。
{
	MotorController_Acc = nAcc * MOTOR_CONTROLLER_PERIOD / 1000 + 1;
}
void MotorController_SetSpeed(uint8_t nMotor, int16_t nSpeed) //设置轮子转速，nMotor电机编号，nSpeed轮子线速度，单位：mm/s
{
	switch(nMotor)
	{
		case 1:
			MotorController_MotorA_SpeedSet = nSpeed;
			if(MotorDriver_GetMotorState(1) == 1)  //如果电机处于停止状态
			{
				MotorController_MotorA_SpeedPWM = PWM_DUTY_LIMIT/2;
				MotorDriver_Start(1,MotorController_MotorD_SpeedPWM);
			}
			break;
		case 2:
			MotorController_MotorB_SpeedSet = nSpeed;
			if(MotorDriver_GetMotorState(2) == 1)  //如果电机处于停止状态
			{
				MotorController_MotorB_SpeedPWM = PWM_DUTY_LIMIT/2;
				MotorDriver_Start(2,MotorController_MotorD_SpeedPWM);
			}		
			break;
		case 3:
			MotorController_MotorC_SpeedSet = nSpeed;
			if(MotorDriver_GetMotorState(3) == 1)  //如果电机处于停止状态
			{
				MotorController_MotorC_SpeedPWM = PWM_DUTY_LIMIT/2;
				MotorDriver_Start(3,MotorController_MotorD_SpeedPWM);
			}		
			break;
		case 4:
			MotorController_MotorD_SpeedSet = nSpeed;
			if(MotorDriver_GetMotorState(4) == 1)  //如果电机处于停止状态
			{
				MotorController_MotorD_SpeedPWM = PWM_DUTY_LIMIT/2;
				MotorDriver_Start(4,MotorController_MotorD_SpeedPWM);
			}		
			break;
		default:
			;
	}
}

void MotorController_Enable(FunctionalState NewState)
{
  assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	//启用速度调节器前把所有中间变量都清零。
	MotorController_MotorA_EncCnt = Encoder_GetEncCount(1);
	MotorController_MotorA_SpeedErr2 = 0;
	MotorController_MotorA_SpeedErr1 = 0;
	
	MotorController_MotorB_EncCnt = Encoder_GetEncCount(2);
	MotorController_MotorB_SpeedErr2 = 0;
	MotorController_MotorB_SpeedErr1 = 0;

	MotorController_MotorC_EncCnt = Encoder_GetEncCount(3);
	MotorController_MotorC_SpeedErr2 = 0;
	MotorController_MotorC_SpeedErr1 = 0;
	
	MotorController_MotorD_EncCnt = Encoder_GetEncCount(4);
	MotorController_MotorD_SpeedErr2 = 0;
	MotorController_MotorD_SpeedErr1 = 0;
	
	MotorController_MotorA_SpeedSet = 0;
	MotorController_MotorB_SpeedSet = 0;
	MotorController_MotorC_SpeedSet = 0;
	MotorController_MotorD_SpeedSet = 0;
	MotorController_MotorA_SpeedCur = 0;
	MotorController_MotorB_SpeedCur = 0;
	MotorController_MotorC_SpeedCur = 0;
	MotorController_MotorD_SpeedCur = 0;
	MotorController_MotorA_SpeedPWM = PWM_DUTY_LIMIT/2;
	MotorController_MotorB_SpeedPWM = PWM_DUTY_LIMIT/2;
	MotorController_MotorC_SpeedPWM = PWM_DUTY_LIMIT/2;
	MotorController_MotorD_SpeedPWM = PWM_DUTY_LIMIT/2;	
  
  if (NewState != DISABLE)
  {
		TIM_Cmd(TIM6,ENABLE);
	}
	else
	{
		TIM_Cmd(TIM6,DISABLE);
	}
}
void MotorController_SpeedTunner(void)
{
	int16_t nSpeedExpect;
	static int16_t pwmDelta = 0;
	int16_t pwmSet;
	float fError;
	int32_t nCnt;
	float fSpeedCur;
	switch(MotorController_MotorEnabledCount)
	{
		case 4:
			if(MotorController_MotorD_SpeedCur < MotorController_MotorD_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorD_SpeedCur + MotorController_Acc;
				if(nSpeedExpect > MotorController_MotorD_SpeedSet) nSpeedExpect = MotorController_MotorD_SpeedSet;
			}
			else if(MotorController_MotorD_SpeedCur > MotorController_MotorD_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorD_SpeedCur - MotorController_Acc;
				if(nSpeedExpect < MotorController_MotorD_SpeedSet) nSpeedExpect = MotorController_MotorD_SpeedSet;
			}
			else
			{
				nSpeedExpect = MotorController_MotorD_SpeedSet;		
			}
			MotorController_MotorD_SpeedCur = nSpeedExpect;	
			
			nCnt = Encoder_GetEncCount(4);
			fSpeedCur = 3.14*(nCnt - MotorController_MotorD_EncCnt)* MotorController_WheelDiameter * 1000 / (MotorController_EncoderResolution*4*MOTOR_CONTROLLER_PERIOD);
			fError = nSpeedExpect - fSpeedCur;
			
			pwmDelta = MotorController_KP * (fError - MotorController_MotorD_SpeedErr1) 
									+ MotorController_KI * (fError + MotorController_MotorD_SpeedErr1)/2
									+ MotorController_KD * (fError-2*MotorController_MotorD_SpeedErr1+MotorController_MotorD_SpeedErr2);
			pwmSet = MotorController_MotorD_SpeedPWM + pwmDelta;

			if(pwmSet>PWM_DUTY_LIMIT) pwmSet = PWM_DUTY_LIMIT;
			else if(pwmSet<0) pwmSet = 0;
			MotorController_MotorD_SpeedPWM = pwmSet;					
			MotorDriver_SetPWMDuty(4,MotorController_MotorD_SpeedPWM);
			MotorController_MotorD_SpeedErr2 = MotorController_MotorD_SpeedErr1;
			MotorController_MotorD_SpeedErr1 = fError;
			MotorController_MotorD_EncCnt = nCnt;
			
		case 3:
			if(MotorController_MotorC_SpeedCur < MotorController_MotorC_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorC_SpeedCur + MotorController_Acc;
				if(nSpeedExpect > MotorController_MotorC_SpeedSet) nSpeedExpect = MotorController_MotorC_SpeedSet;
			}
			else if(MotorController_MotorC_SpeedCur > MotorController_MotorC_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorC_SpeedCur - MotorController_Acc;
				if(nSpeedExpect < MotorController_MotorC_SpeedSet) nSpeedExpect = MotorController_MotorC_SpeedSet;
			}
			else
			{
				nSpeedExpect = MotorController_MotorC_SpeedSet;		
			}
			MotorController_MotorC_SpeedCur = nSpeedExpect;	
			
			nCnt = Encoder_GetEncCount(3);
			fSpeedCur = 3.14*(nCnt - MotorController_MotorC_EncCnt)* MotorController_WheelDiameter * 1000 / (MotorController_EncoderResolution*4*MOTOR_CONTROLLER_PERIOD);
			fError = nSpeedExpect - fSpeedCur;
			
			pwmDelta = MotorController_KP * (fError - MotorController_MotorC_SpeedErr1) 
									+ MotorController_KI * (fError + MotorController_MotorC_SpeedErr1)/2
									+ MotorController_KD * (fError-2*MotorController_MotorC_SpeedErr1+MotorController_MotorC_SpeedErr2);

			pwmSet = MotorController_MotorC_SpeedPWM + pwmDelta;

			if(pwmSet>PWM_DUTY_LIMIT) pwmSet = PWM_DUTY_LIMIT;
			else if(pwmSet<0) pwmSet = 0;
			MotorController_MotorC_SpeedPWM = pwmSet;					
			MotorDriver_SetPWMDuty(3,MotorController_MotorC_SpeedPWM);
			MotorController_MotorC_SpeedErr2 = MotorController_MotorC_SpeedErr1;
			MotorController_MotorC_SpeedErr1 = fError;
			MotorController_MotorC_EncCnt = nCnt;
		case 2:
			if(MotorController_MotorB_SpeedCur < MotorController_MotorB_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorB_SpeedCur + MotorController_Acc;
				if(nSpeedExpect > MotorController_MotorB_SpeedSet) 
					nSpeedExpect = MotorController_MotorB_SpeedSet;
			}
			else if(MotorController_MotorB_SpeedCur > MotorController_MotorB_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorB_SpeedCur - MotorController_Acc;
				if(nSpeedExpect < MotorController_MotorB_SpeedSet) 
					nSpeedExpect = MotorController_MotorB_SpeedSet;
			}
			else
			{
				nSpeedExpect = MotorController_MotorB_SpeedSet;		
			}
			MotorController_MotorB_SpeedCur = nSpeedExpect;			
			nCnt = Encoder_GetEncCount(2);

			fSpeedCur = 3.14*(nCnt - MotorController_MotorB_EncCnt)* MotorController_WheelDiameter * 1000 / (MotorController_EncoderResolution*4*MOTOR_CONTROLLER_PERIOD);
			Motor_speed2=fSpeedCur;
			fError = nSpeedExpect - fSpeedCur;

			pwmDelta = MotorController_KP * (fError - MotorController_MotorB_SpeedErr1) 
									+ MotorController_KI * (fError + MotorController_MotorB_SpeedErr1)/2
									+ MotorController_KD * (fError-2*MotorController_MotorB_SpeedErr1+MotorController_MotorB_SpeedErr2);
			pwmSet = MotorController_MotorB_SpeedPWM + pwmDelta;

			if(pwmSet>PWM_DUTY_LIMIT) pwmSet = PWM_DUTY_LIMIT;
			else if(pwmSet<0) pwmSet = 0;
			MotorController_MotorB_SpeedPWM = pwmSet;			
			MotorDriver_SetPWMDuty(2,MotorController_MotorB_SpeedPWM);

			MotorController_MotorB_SpeedErr2 = MotorController_MotorB_SpeedErr1;
			MotorController_MotorB_SpeedErr1 = fError;
			MotorController_MotorB_EncCnt = nCnt;
		case 1:
			if(MotorController_MotorA_SpeedCur < MotorController_MotorA_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorA_SpeedCur + MotorController_Acc;
				if(nSpeedExpect > MotorController_MotorA_SpeedSet) nSpeedExpect = MotorController_MotorA_SpeedSet;
			}
			else if(MotorController_MotorA_SpeedCur > MotorController_MotorA_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorA_SpeedCur - MotorController_Acc;
				if(nSpeedExpect < MotorController_MotorA_SpeedSet) nSpeedExpect = MotorController_MotorA_SpeedSet;
			}
			else
			{
				nSpeedExpect = MotorController_MotorA_SpeedSet;		
			}
			MotorController_MotorA_SpeedCur = nSpeedExpect;	
			
			nCnt = Encoder_GetEncCount(1);
			fSpeedCur = 3.14*(nCnt - MotorController_MotorA_EncCnt)* MotorController_WheelDiameter * 1000 / (MotorController_EncoderResolution*4*MOTOR_CONTROLLER_PERIOD);
			Motor_speed1=fSpeedCur;
			fError = nSpeedExpect - fSpeedCur;
			
			pwmDelta = MotorController_KP * (fError - MotorController_MotorA_SpeedErr1) 
									+ MotorController_KI * (fError + MotorController_MotorA_SpeedErr1)/2
									+ MotorController_KD * (fError-2*MotorController_MotorA_SpeedErr1+MotorController_MotorA_SpeedErr2);
			pwmSet = MotorController_MotorA_SpeedPWM + pwmDelta;

			if(pwmSet>PWM_DUTY_LIMIT) pwmSet = PWM_DUTY_LIMIT;
			else if(pwmSet<0) pwmSet = 0;
			MotorController_MotorA_SpeedPWM = pwmSet;			
			MotorDriver_SetPWMDuty(1,MotorController_MotorA_SpeedPWM);
			MotorController_MotorA_SpeedErr2 = MotorController_MotorA_SpeedErr1;
			MotorController_MotorA_SpeedErr1 = fError;
			MotorController_MotorA_EncCnt = nCnt;			
		default:
			;
	}
}
void MotorController_SetPIDParam(float Kp,float Ki,float Kd)
{
	MotorController_KP = Kp;
	MotorController_KI = Ki;
	MotorController_KD = Kd;	
}
/*以下为中断服务程序，注意不要和stm32f10x_it.c文件中的重复*/
void TIM6_DAC_IRQHandler(void)
{
	 //是否有更新中断
	if(TIM_GetITStatus(TIM6,TIM_IT_Update) != RESET)
	{
		 //清除中断标志
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update); 
		//处理中断
		MotorController_SpeedTunner();
	}
	
}
