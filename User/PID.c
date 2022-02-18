#include "PID.h"
#include "delay.h"
#include "MotorController.h"
#include "amt1450_uart.h"


void s_PIDInit(PID *s_PID)
{
	s_PID->error_acc = 0;
	s_PID->lastError = 0;
	s_PID->prevError = 0;

	s_PID->kp = 3.8; 
	s_PID->ki = 0;
	s_PID->kd = 1.4; 
	s_PID->target = 76;
	
	s_PID->filter=0.4;
	s_PID->lastFilter=0;
	
}

int32_t Straight_PID(PID *s_PID, uint8_t position)
{
	int iError, output = 0;
	iError = s_PID->target - position; //误差值计算
	s_PID->error_acc += iError;		   //积分

	output = s_PID->kp * iError + s_PID->ki * s_PID->error_acc * 0.5f + s_PID->kd * iError - s_PID->lastError;	
	output = s_PID->filter * output + (1 - s_PID->filter)* s_PID->lastFilter;
	
	s_PID->lastFilter = output;//滤波值存储
	s_PID->lastError = iError; // error值存储
	return (output);
}

void Turn_I(int nSpeed, int d_speed, int turn_angle)
{	
	if(turn_angle<0){
		d_speed= - d_speed;
		turn_angle = -turn_angle;
	}
	
	MotorController_SetSpeed(1, -nSpeed+d_speed);
	MotorController_SetSpeed(2, nSpeed+d_speed);
	
	float car_D=240+40;double now_angle=0;//+40修正一下，这是误差
	while(now_angle<turn_angle)
	{
		Delay_10us(10);
		now_angle=now_angle+(abs(d_speed)*2*0.0001)/(2*3.14159*car_D)*360;
		
		//开始强制修正
		uint8_t begin, jump, count[6]; // 最大6个跳变，即3条线
		uint8_t position;
		get_AMT1450Data_UART(&begin, &jump, count);
		
		if (jump == 2)
			position = 0.5f * (count[0] + count[1]); // position=两次跳变的中间位置
		
		
		if(position>72 &&position<80 && now_angle>turn_angle/3.0*2){
			break;
		}
	}
	
	MotorController_SetSpeed(1,0);
	MotorController_SetSpeed(2, 0);
	Delay_ms(100);
	
}
