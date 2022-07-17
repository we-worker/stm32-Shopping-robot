#include "PID.h"
#include "delay.h"
#include "MotorController.h"
#include "amt1450_uart.h"
#include "MotorDriver.h"
#include "AStarRoute.h"
#include "stdlib.h"
#include "tcrt5000.h"
#include "NanoCommunication.h"
#include "ArmSolution.h"

int crossing_flag; //用于十字路口检测的临时变量
int map_count=1;	   //地图十字路口计数//1
int map_index;	   //用于地图内部下一个操作的索引
//
//	int map_count=36;
// int map_index=30;

//小车默认方向和位置。
int car_direction = 3;
int car_position[2] = {9, 8};

//检测中间白线的位置相关参数
uint8_t begin, jump, count[6];
uint8_t line_position;
uint8_t force_move=3;//同一个位置连续抓取3次就强制移动
#define car_D 320 //小车直径定义

//========================================以下为pid参数初始化函数===================
//循迹pid的参数初始化
void s_PIDInit(PID *s_PID)
{
	s_PID->error_acc = 0;
	s_PID->lastError = 0;
	s_PID->prevError = 0;

	s_PID->kp = 3.8; // old:3.8 0 1.4
	s_PID->ki = 0;
	s_PID->kd = 1.2;
	s_PID->target = 76;

	s_PID->filter = 0.2;
	s_PID->lastFilter = 0;
}

//转向pid的初始化参数
void turn_PIDInit(PID *s_PID)
{
	s_PID->error_acc = 0;
	s_PID->lastError = 0;
	s_PID->prevError = 0;

	s_PID->kp = 2;
	s_PID->ki = 0;
	s_PID->kd = 1.5;

	s_PID->filter = 0.3;
	s_PID->lastFilter = 0;
}

//========================================以下为直线行驶的函数======================
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

	int kp = 400;
	int out = (targetPos - nowPos) * kp;
	if (out >= 800)
		out = 800;
	return (out + dspeed) / 2; //目标速度与自身速度加和平均是为了速度过度更平滑
}

//单纯的走一个格子后退出
void Straight_go(int nSpeed) //直走一格后退出
{
	PID s_PID;
	s_PIDInit(&s_PID);
	int crossing_flag = 0;

	while (1)
	{
		get_AMT1450Data_UART(&begin, &jump, count); //讲数据存储在三个变量中

		if (jump == 2)
		{
			crossing_flag = 1;
			line_position = 0.5f * (count[0] + count[1]); // position=两次跳变的中间位置
		}
		if (jump == 0 && begin == 0 && crossing_flag == 1)
		{
			Car_Position_add(1);
			crossing_flag = 0;
			printf("dir:%d  ; pos:%d,%d\r\n", car_direction, car_position[0], car_position[1]);
			break;
		}

		int32_t spid_out = Follow_PID(&s_PID, line_position);
		MotorController_SetSpeed(1, -nSpeed + spid_out);
		MotorController_SetSpeed(2, nSpeed + spid_out);
	}
}

//直走多少毫米，输入速度和目标距离，自动完成走多少毫米。
void Straight_go_mm(int nSpeed, int distance)
{

	MotorController_SetSpeed(2, nSpeed);
	MotorController_SetSpeed(1, -nSpeed);
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

		get_AMT1450Data_UART(&begin, &jump, count); //讲数据存储在三个变量中
		if (jump == 2)
		{
			crossing_flag = 1;
			line_position = 0.5f * (count[0] + count[1]); // position=两次跳变的中间位置
		}
		if (jump == 0 && begin == 0 && crossing_flag == 1)
		{
			Car_Position_add(1);
			crossing_flag = 0;
			printf("dir:%d  ; pos:%d,%d\r\n", car_direction, car_position[0], car_position[1]);
			break;
		}

		int32_t spid_out = Follow_PID(&s_PID, line_position);
		MotorController_SetSpeed(2, nSpeed + spid_out);
		MotorController_SetSpeed(1, -nSpeed + spid_out);
	}
}

//单纯的后退一个格子后退出
void Straight_back(int nSpeed)
{
	PID s_PID;
	s_PIDInit(&s_PID);
	int crossing_flag = 0;

	while (1)
	{
		get_AMT1450Data_UART(&begin, &jump, count); //讲数据存储在三个变量中

		if (jump == 0 && begin == 0 && crossing_flag == 1)
		{
			Car_Position_add(1);
			crossing_flag = 0;
			printf("dir:%d  ; pos:%d,%d\r\n", car_direction, car_position[0], car_position[1]);
			break;
		}
		printf("jump:%d\r\n", jump);

		int spid_out = 0;
		if (TCRT5000_0 == 1)
			spid_out = -50;
		if (TCRT5000_2 == 1)
			spid_out = 50;
		crossing_flag = 1;
		// int32_t spid_out = 0;
		MotorController_SetSpeed(2, -nSpeed + spid_out);
		MotorController_SetSpeed(1, nSpeed + spid_out);
	}
}

void Straight_back_mm(int nSpeed, int distance) //直走一定距离后退出
{
	extern float Motor_speed1;
	extern float Motor_speed2;
	double dis = 0;

	while (dis < distance)
	{
		Delay_10us(10);
		dis = dis + (fabs(Motor_speed1 - Motor_speed2) / 2.0f) * 0.0001f; //对线速度积分，就是路程长度。

		int spid_out = 0;
		if (TCRT5000_0 == 1)
			spid_out = 50;
		if (TCRT5000_2 == 1)
			spid_out = -50;
		crossing_flag = 1;
		// int32_t spid_out = 0;
		MotorController_SetSpeed(2, -nSpeed + spid_out);
		MotorController_SetSpeed(1, nSpeed + spid_out);
	}
	Car_Position_add(1);
}

//========================================以下为各种转向的函数=======================

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
	SetServoAngle(8,120);//爪子先松开//为了能够让购物车过弯
	while (now_angle < turn_angle)
	{
		//根据dt，积分积出但前的角度，原理不用太明白。
		Delay_10us(10);
		float dspeed = fabs(Motor_speed1 + Motor_speed2);
		now_angle = now_angle + (dspeed * 0.0001f) / (2 * 3.14159f * car_D) * 360;

		//开始强制修正,旋转角度在预期的2/3以上时，如果检测到线在中心位置，直接推出旋转。
		//标准的获取前方探头的数据
		get_AMT1450Data_UART(&begin, &jump, count);

		if (jump == 2)
			line_position = 0.5f * (count[0] + count[1]); // line_position=两次跳变的中间位置，即是线的位置

		if (line_position > 70 && line_position < 80 && now_angle > turn_angle / 3.0 * 2) //如果线在中心位置并且旋转角度在预期的2/3以上时，直接退出
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
			out = -out - 50;
		else
			out = out + 50;
		//电机附速度值
		MotorController_SetSpeed(2, out);
		MotorController_SetSpeed(1, out);
	}
	SetServoAngle(8,140);//爪子再抓紧
	//小挺顿一下
	// MotorController_SetSpeed(1, 0);
	// MotorController_SetSpeed(2, 0);
	// Delay_ms(50);
}
//通过速度pid来实现转向，输入目标方向即可，实现转速先快后慢。不是强制转到线上
void TurnBY_PID_notforce_write(int turn_angle)
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
	SetServoAngle(8,120);//爪子先松开//为了能够让购物车过弯
	while (now_angle < turn_angle)
	{
		//根据dt，积分积出但前的角度，原理不用太明白。
		Delay_10us(10);
		float dspeed = fabs(Motor_speed1 + Motor_speed2);
		now_angle = now_angle + (dspeed * 0.0001f) / (2 * 3.14159f * car_D) * 360;

		//开始强制修正,旋转角度在预期的2/3以上时，如果检测到线在中心位置，直接推出旋转。
		//标准的获取前方探头的数据
		get_AMT1450Data_UART(&begin, &jump, count);

		if (jump == 2)
			line_position = 0.5f * (count[0] + count[1]); // line_position=两次跳变的中间位置，即是线的位置

		if (line_position > 70 && line_position < 80 && now_angle > turn_angle / 3.0 * 2) //如果线在中心位置并且旋转角度在预期的2/3以上时，直接退出
		{
			break;
		}


		int out = (Turn_PID(&t_PID, now_angle, turn_angle) + dspeed / 2) / 2; //更加平滑使用转向pid得到数值
		//根据是否为左转变化一下，同时需要固定30速度，不然后面转的太慢了
		if (flag_left == 0)
			out = -out - 50;
		else
			out = out + 50;
		//电机附速度值
		MotorController_SetSpeed(2, out);
		MotorController_SetSpeed(1, out);
	}
	SetServoAngle(8,140);//爪子再抓紧
	//小挺顿一下
	// MotorController_SetSpeed(1, 0);
	// MotorController_SetSpeed(2, 0);
	// Delay_ms(50);
}

//转向，输入参数：基准前进速度，转速差，旋转角度  自动转好预设角度，原理和上面pid转向大同小异
void Turn_I(int nSpeed, int d_speed, int turn_angle)
{

	if (turn_angle < 0)
	{
		d_speed = -d_speed;
		turn_angle = -turn_angle;
	}

	MotorController_SetSpeed(2, nSpeed + d_speed);
	MotorController_SetSpeed(1, -nSpeed + d_speed);
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
		get_AMT1450Data_UART(&begin, &jump, count);

		if (jump == 2)
			line_position = 0.5f * (count[0] + count[1]); // line_position=两次跳变的中间位置

		if (line_position > 50 && line_position < 100 && now_angle > turn_angle / 3.0 * 2)
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

//与小车自身的方向编码想关，转到东南西北。
void Turn_to_dir(int to_dir) //转到目标角度
{

	if (abs(to_dir - car_direction) == 2)
	{
		TurnBY_PID(180);
		Car_Direction_change(2);
	}
	else
	{

		if (to_dir - car_direction == 1 || to_dir - car_direction == -3)
		{
			TurnBY_PID(90);
			Car_Direction_change(1);
		}
		else
		{
			TurnBY_PID(-90);
			Car_Direction_change(-1);
		}
	}
}

//==============================================下面是小车坐标位置的相关操作的函数============================

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

//==============================================下面是地图相关操作的函数============================

// Drive_Route,自动生成路径，并完成行走，不用看，还要改的。
void Drive_Route(int nSpeed, int tox, int toy)
{
	int way[50] = {0};

	// FindPath(way, car_position[0], car_position[1], tox, toy);

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

//检测是否经过红绿灯
void Crossing_Detection()
{
	// AMT1450循迹模块的使用，见https://www.luheqiu.com/deane/begin-smart_tracking_car/

	get_AMT1450Data_UART(&begin, &jump, count); //讲数据存储在三个变量中
	if (jump == 2){
		line_position = 0.5f * (count[0] + count[1]); // position=两次跳变的中间位置，即线的位置
	}
		

	//如果颜色没有跳变，且最左端为白色，且没进入路口，则
	if (jump == 0 && begin == 0 && crossing_flag == 1)
	{
		map_count++;		 //地图计数加1
		Car_Position_add(1); //小车位置加1
		crossing_flag = 0;	 //标志进入路口
		printf("carPos:%d\n", map_count);
		// printf("carspeed1%.3f;  carspeed2%.3f\n",Motor_speed1,Motor_speed2);
	}
	//如果有线出现，那么代表驶出了路口，标志=1
	if (jump == 2)
	{
		crossing_flag = 1;
	}
	if(jump==0&&begin==0)
		line_position=77;//如果在白线，就直走，不要乱调整了
	
}


//地图行为，根据目前是在第几个路口，执行相关转向操作
void Map_Action()
{
	extern uint8_t car_flag;
	int map[][2] = {{2, 2},{10, 3},{1,4},{6,7},{6,8},{6,9},{6,10},{6,11},{6,12},{8,12},{6,15},{6,16},{6,17},{6,18},{6,19},{6,20},{8,20},
	{6,23},{6,24},{6,25},{6,26},{6,27},{6,28},{8,28},{6,31},{6,32},{6,33},{6,34},{6,35},{6,36},{9,36}};
	//int map[][2]={{9,2}};
	if (map_count == map[map_index][1])
	{
		switch (map[map_index][0])
		{
		case 1://原地左转
			Straight_go_mm(200, 120); //走过车身的一半长
			TurnBY_PID(90);			  // Turn_I(0,300,90);
			Car_Direction_change(1);  //小车方向转变，1为向左。
			break;
		case 2://原地右转
			Straight_go_mm(200, 120); //走过车身的一半长
			TurnBY_PID(-90);
			Car_Direction_change(-1);
			break;
		case 3://原地左转180
			TurnBY_PID(180);
			Car_Direction_change(2);
			break;
		case 4://原地右转180
			TurnBY_PID(-180);
			Car_Direction_change(2);
			break;
		case 5://直走一格
			Straight_go(200);
			Straight_go_mm(200, 120); //走过车身的一半长
			break;	
		case 6://等待上位机信号
			Straight_go_mm(200, 120); //走过车身的一半长
			MotorController_SetSpeed(1, 0);
			MotorController_SetSpeed(2, 0);
			Delay_ms(2000);
			car_flag=Car_Waiting;
			printf("wait_nano\n");
			break;	
		case 7://倒退一格
			Straight_back_mm(200,480);
			break;	
		case 8://转角处后退+右转
			Straight_back_mm(200,420);

			TurnBY_PID(-90);
			Car_Direction_change(-1);

			break;	
		case 9://放好购物车的倒退+放爪+向前一格
			/*
			TurnBY_PID(-90);
			Straight_go(200);
			Straight_go_mm(200, 120);//走过车身的一半长
			TurnBY_PID(90);
			Straight_go_mm(200, 120);//走过车身的一半长
			TurnBY_PID_notforce_write(-90);
			Straight_go_mm(0, 0);//停止
			Delay_ms(1000);
			car_flag=Car_Stop;
			*/
				TurnBY_PID(-90);
				Straight_go_mm(200, 120);//走过车身的一半长
				MotorController_SetSpeed(1, -200);
				MotorController_SetSpeed(2, 200);
				Delay_ms(500);
				TurnBY_PID_notforce_write(90);
				MotorController_SetSpeed(1, -200);
				MotorController_SetSpeed(2, 200);
				Delay_ms(800);
				TurnBY_PID_notforce_write(-90);
				Straight_go_mm(0, 0);//停止
				Delay_ms(1000);
				car_flag=Car_Stop;
		
			break;	
		case 10://左转+延时1s+倒退1格
			Straight_go_mm(200, 120); //走过车身的一半长
			TurnBY_PID(90);			  // Turn_I(0,300,90);
			Car_Direction_change(1);  //小车方向转变，1为向左。

			SetServoAngle(8,90);
			Straight_back_mm(200,230);
			SetServoAngle(8,140);
			Straight_back_mm(50,40);

			Straight_go(220);
			//Straight_go_mm(200, 120); //走过车身的一半长
		    break;
		default:
			break;
		}
		map_index++;
	}
}
