#include "stm32f4xx.h"

typedef struct PID
{
	float kp;
	float ki;
	float kd;

	float error_acc;
	float lastError;
	float prevError;

	float target;
	
	float filter;
	float lastFilter;
} PID;

void s_PIDInit(PID *s_PID);

int32_t Straight_PID(PID *s_PID, uint8_t position);					//输入参数：pid结构，当前位置 返回：直线pid控制值
void Turn_I(int nSpeed,int d_speed, int turn_angle); //输入参数：基准前进速度，转速差，旋转角度  返回：旋转pid控制值

extern int car_direction;//小车方向
extern int car_position[2];//小车位置

void Car_Direction_change(int add);//小车方向更改
void Car_Position_add(void);//小车位置+1；
void Drive_Route(int nSpeed,int tox,int toy);
