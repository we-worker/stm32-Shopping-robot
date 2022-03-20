#include "PID.h"
#include "delay.h"
#include "MotorController.h"
#include "amt1450_uart.h"
#include "MotorDriver.h"
#include "AStarRoute.h"
#include "stdlib.h"

//小车默认方向和位置。
int car_direction = 3;
int car_position[2] = {9, 8};
#define car_D 320 //小车直径定义

//循迹pid的参数初始化
void s_PIDInit(PID *s_PID)
{
	s_PID->error_acc = 0;
	s_PID->lastError = 0;
	s_PID->prevError = 0;

	s_PID->kp = 8;//old:3.8 0 1.4
	s_PID->ki = 0;
	s_PID->kd = 1.4;
	s_PID->target = 76;

	s_PID->filter = 0.5;
	s_PID->lastFilter = 0;
}

//转向pid的初始化参数
void turn_PIDInit(PID *s_PID)
{
	s_PID->error_acc = 0;
	s_PID->lastError = 0;
	s_PID->prevError = 0;

	s_PID->kp = 4.5;
	s_PID->ki = 0;
	s_PID->kd = 3;

	s_PID->filter = 0.3;
	s_PID->lastFilter = 0;
}
//循迹的pid
int32_t Follow_PID(PID *s_PID, uint8_t position)
{
	int iError, output = 0;
	iError = s_PID->target - position; //误差值计算
	s_PID->error_acc += iError;		   //积分

	output = s_PID->kp * iError + s_PID->ki * s_PID->error_acc * 0.5f + s_PID->kd * iError - s_PID->lastError;
	output = s_PID->filter * output + (1 - s_PID->filter) * s_PID->lastFilter; //滤波操作

	s_PID->lastFilter = output; //滤波值存储
	s_PID->lastError = iError;	// error值存储

	return (output);
}

//直行的伪pid，根据目标距离自己的格子数，判定应该出多少速度，距离远就快一点，距离近就慢一点。
//但是由于还是兼顾循迹pid，800-900的速度基本就是极限了
int32_t Straight_PID(int nowPos, int targetPos)
{
	extern float Motor_speed1;
	extern float Motor_speed2;

	float dspeed = fabs(Motor_speed1 - Motor_speed2) / 2; //获得双轮转速的实际值的平均值

	int kp = 700;
	int out = (targetPos - nowPos) * kp;
	if (out >= 1000)
		out = 1000;
	return (out + dspeed) / 2; //目标速度与自身速度加和平均是为了速度过度更平滑
}

//转向的pid
int32_t Turn_PID(PID *t_PID, int now_angle, int target_angle)
{
	int iError, output = 0;
	iError = target_angle - now_angle; //误差值计算

	output = t_PID->kp * iError + t_PID->kd * iError - t_PID->lastError;

	t_PID->lastFilter = output; //滤波值存储
	t_PID->lastError = iError;	// error值存储
	return (output);
}

//通过速度pid来实现转向，输入目标方向即可，实现转速先快后慢。
void TurnBY_PID(int turn_angle)
{
	PID t_PID;
	turn_PIDInit(&t_PID);

	extern float Motor_speed1;
	extern float Motor_speed2;

	double now_angle = 0; //当前角度

	int flag_left = 1;	//是否为左转标志
	if (turn_angle < 0) //根据目标角度是向右还是向左修改，以便后续
	{
		flag_left = 0;
		turn_angle = -turn_angle;
	}

	while (now_angle < turn_angle)
	{
		//根据dt，积分积出但前的角度，原理不用太明白。
		Delay_10us(10);
		float dspeed = fabs(Motor_speed1 + Motor_speed2);
		now_angle = now_angle + (dspeed * 0.0001f) / (2 * 3.14159f * car_D) * 360;

		//开始强制修正,旋转角度在预期的2/3以上时，如果检测到线在中心位置，直接推出旋转。
		//标准的获取前方探头的数据
		uint8_t begin, jump, count[6];
		uint8_t position;
		get_AMT1450Data_UART(&begin, &jump, count);

		if (jump == 2)
			position = 0.5f * (count[0] + count[1]); // position=两次跳变的中间位置，即是线的位置

		if (position > 60 && position < 85 && now_angle > turn_angle / 3.0 * 2) //如果线在中心位置并且旋转角度在预期的2/3以上时，直接退出
		{
			break;
		}

		//所以如果没有转到线上，就一直转
		if (now_angle > turn_angle - 3 && jump <= 1)
		{
			turn_angle += 5;
		}

		int out = (Turn_PID(&t_PID, now_angle, turn_angle) + dspeed / 2) / 2; //更加平滑使用转向pid得到数值
		//根据是否为左转变化一下，同时需要固定30速度，不然后面转的太慢了
		if (flag_left == 0)
			out = -out - 70;
		else
			out = out + 70;
		//电机附速度值
		MotorController_SetSpeed(1, out);
		MotorController_SetSpeed(2, out);
	}
	//小挺顿一下
	//MotorController_SetSpeed(1, 0);
	//MotorController_SetSpeed(2, 0);
	//Delay_ms(50);
}

//转向，输入参数：基准前进速度，转速差，旋转角度  自动转好预设角度，原理和上面pid转向大同小异
void Turn_I(int nSpeed, int d_speed, int turn_angle)
{

	if (turn_angle < 0)
	{
		d_speed = -d_speed;
		turn_angle = -turn_angle;
	}

	MotorController_SetSpeed(1, nSpeed + d_speed);
	MotorController_SetSpeed(2, -nSpeed + d_speed);
	extern float Motor_speed1;
	extern float Motor_speed2;

	double now_angle = 0;
	while (now_angle < turn_angle)
	{
		Delay_10us(10);
		float dspeed = Motor_speed1 + Motor_speed2;
		if (dspeed < 0)
			dspeed = -dspeed;
		now_angle = now_angle + (dspeed * 0.0001f) / (2 * 3.14159f * car_D) * 360;

		//开始强制修正,旋转角度在预期的2/3以上时，如果检测到线在中心位置，直接推出旋转。
		uint8_t begin, jump, count[6]; // 最大6个跳变，即3条线
		uint8_t position;
		get_AMT1450Data_UART(&begin, &jump, count);

		if (jump == 2)
			position = 0.5f * (count[0] + count[1]); // position=两次跳变的中间位置

		if (position > 50 && position < 100 && now_angle > turn_angle / 3.0 * 2)
		{
			break;
		}

		//所以如果没有转到线上，就一直转
		if (now_angle > turn_angle - 3 && jump <= 1)
		{
			turn_angle += 5;
		}
	}

	MotorController_SetSpeed(1, 0);
	MotorController_SetSpeed(2, 0);
	Delay_ms(50);
}

//小车当前方向变换+1或-1，小车方向逆时针以此为1234循环
void Car_Direction_change(int add)
{
	while (add != 0)
	{
		if (add > 0)
		{
			car_direction += 1;
			add--;
		}
		else if (add < 0)
		{
			car_direction -= 1;
			add++;
		}
		if (car_direction > 4)
			car_direction = 1;
		if (car_direction < 1)
			car_direction = 4;
	}
};

//小车位置坐标+几，，过几个路口就是加几；，会根据小车目前的方向自动加好小车现在的坐标。
void Car_Position_add(int add)
{
	while (add--)
	{

		switch (car_direction)
		{
		case 1:
			car_position[0] += 1;
			break;
		case 2:
			car_position[1] += 1;
			break;
		case 3:
			car_position[0] -= 1;
			break;
		case 4:
			car_position[1] -= 1;
			break;
		default:
			break;
		}
	}
};

//单纯的走一个格子后退出
void Straight_go(int nSpeed) //直走一格后退出
{
	PID s_PID;
	s_PIDInit(&s_PID);
	int crossing_flag = 0;

	while (1)
	{
		uint8_t begin, jump, count[6]; // 最大6个跳变，即3条线
		uint8_t position;
		get_AMT1450Data_UART(&begin, &jump, count); //讲数据存储在三个变量中
		if (jump == 2)
		{
			crossing_flag = 1;
			position = 0.5f * (count[0] + count[1]); // position=两次跳变的中间位置
		}
		if (jump == 0 && begin == 0 && crossing_flag == 1)
		{
			Car_Position_add(1);
			crossing_flag = 0;
			printf("dir:%d  ; pos:%d,%d\r\n", car_direction, car_position[0], car_position[1]);
			break;
		}

		int32_t spid_out = Follow_PID(&s_PID, position);
		MotorController_SetSpeed(1, -nSpeed + spid_out);
		MotorController_SetSpeed(2, nSpeed + spid_out);
	}
}

void Straight_go_mm(int nSpeed, int distance) //直走多少毫米，输入速度和目标距离，自动完成走多少毫米。
{

	MotorController_SetSpeed(1, nSpeed);
	MotorController_SetSpeed(2, -nSpeed);
	extern float Motor_speed1;
	extern float Motor_speed2;

	double dis = 0;

	PID s_PID;
	s_PIDInit(&s_PID);
	int crossing_flag = 0;

	while (dis < distance)
	{
		Delay_10us(10);
		dis = dis + (fabs(Motor_speed1 - Motor_speed2) / 2.0f) * 0.0001f; //对线速度积分，就是路程长度。
		
		
		uint8_t begin, jump, count[6]; // 最大6个跳变，即3条线
		uint8_t position;
		get_AMT1450Data_UART(&begin, &jump, count); //讲数据存储在三个变量中
		if (jump == 2)
		{
			crossing_flag = 1;
			position = 0.5f * (count[0] + count[1]); // position=两次跳变的中间位置
		}
		if (jump == 0 && begin == 0 && crossing_flag == 1)
		{
			Car_Position_add(1);
			crossing_flag = 0;
			printf("dir:%d  ; pos:%d,%d\r\n", car_direction, car_position[0], car_position[1]);
			break;
		}

		int32_t spid_out = Follow_PID(&s_PID, position);
		MotorController_SetSpeed(1, nSpeed + spid_out);
		MotorController_SetSpeed(2, -nSpeed + spid_out);
		
	}
}

//与小车自身的方向编码想关，转到东南西北。不用看，还要改的。
void Turn_to_dir(int to_dir) //转到目标角度
{
	if (abs(car_direction - to_dir) == 2)
	{
		Turn_I(0, 500, 180);
		Car_Direction_change(2);
	}
	else
	{
		if (car_direction == 1 || car_direction == 4)
		{
			if (car_direction == 1)
			{
				if (car_direction + 3 == to_dir)
				{
					Turn_I(0, 500, -90);
					Car_Direction_change(-1);
				}
				else
				{
					Turn_I(0, 500, 90);
					Car_Direction_change(1);
				}
			}
			else
			{
				if (car_direction - 3 == to_dir)
				{
					Turn_I(0, 500, 90);
					Car_Direction_change(1);
				}
				else
				{
					Turn_I(0, 500, -90);
					Car_Direction_change(-1);
				}
			}
		}
		else
		{
			if (car_direction > to_dir)
			{
				Turn_I(0, 500, -90);
				Car_Direction_change(-1);
			}
			else
			{
				Turn_I(0, 500, 90);
				Car_Direction_change(1);
			}
		}
	}
}

// Drive_Route,自动生成路径，并完成行走，不用看，还要改的。
void Drive_Route(int nSpeed, int tox, int toy)
{
	int way[50] = {0};

	//FindPath(way, car_position[0], car_position[1], tox, toy);

	for (int i = 1; i < 50 && way[i] != -1; i++)
	{
		printf("%d\n", way[i]);
		if (car_direction == way[i])
		{
			Straight_go(nSpeed);
		}
		else
		{
			Turn_to_dir(way[i]);
			i = i - 1;
		}
	}

	MotorController_SetSpeed(1, 0);
	MotorController_SetSpeed(2, 0);
	Delay_ms(300);
};
