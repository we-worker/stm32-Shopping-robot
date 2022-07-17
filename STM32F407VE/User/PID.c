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

int crossing_flag; //����ʮ��·�ڼ�����ʱ����
int map_count=1;	   //��ͼʮ��·�ڼ���//1
int map_index;	   //���ڵ�ͼ�ڲ���һ������������
//
//	int map_count=36;
// int map_index=30;

//С��Ĭ�Ϸ����λ�á�
int car_direction = 3;
int car_position[2] = {9, 8};

//����м���ߵ�λ����ز���
uint8_t begin, jump, count[6];
uint8_t line_position;
uint8_t force_move=3;//ͬһ��λ������ץȡ3�ξ�ǿ���ƶ�
#define car_D 320 //С��ֱ������

//========================================����Ϊpid������ʼ������===================
//ѭ��pid�Ĳ�����ʼ��
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

//ת��pid�ĳ�ʼ������
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

//========================================����Ϊֱ����ʻ�ĺ���======================
//ѭ����pid
int32_t Follow_PID(PID *s_PID, uint8_t position)
{
	int iError, output = 0;
	iError = s_PID->target - position; //���ֵ����
	s_PID->error_acc += iError;		   //����

	output = s_PID->kp * iError + s_PID->ki * s_PID->error_acc * 0.5f + s_PID->kd * iError - s_PID->lastError;
	output = s_PID->filter * output + (1 - s_PID->filter) * s_PID->lastFilter; //�˲�����

	s_PID->lastFilter = output; //�˲�ֵ�洢
	s_PID->lastError = iError;	// errorֵ�洢

	return (output);
}

//ֱ�е�αpid������Ŀ������Լ��ĸ��������ж�Ӧ�ó������ٶȣ�����Զ�Ϳ�һ�㣬���������һ�㡣
//�������ڻ��Ǽ��ѭ��pid��800-900���ٶȻ������Ǽ�����
int32_t Straight_PID(int nowPos, int targetPos)
{
	extern float Motor_speed1;
	extern float Motor_speed2;

	float dspeed = fabs(Motor_speed1 - Motor_speed2) / 2; //���˫��ת�ٵ�ʵ��ֵ��ƽ��ֵ

	int kp = 400;
	int out = (targetPos - nowPos) * kp;
	if (out >= 800)
		out = 800;
	return (out + dspeed) / 2; //Ŀ���ٶ��������ٶȼӺ�ƽ����Ϊ���ٶȹ��ȸ�ƽ��
}

//��������һ�����Ӻ��˳�
void Straight_go(int nSpeed) //ֱ��һ����˳�
{
	PID s_PID;
	s_PIDInit(&s_PID);
	int crossing_flag = 0;

	while (1)
	{
		get_AMT1450Data_UART(&begin, &jump, count); //�����ݴ洢������������

		if (jump == 2)
		{
			crossing_flag = 1;
			line_position = 0.5f * (count[0] + count[1]); // position=����������м�λ��
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

//ֱ�߶��ٺ��ף������ٶȺ�Ŀ����룬�Զ�����߶��ٺ��ס�
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
		dis = dis + (fabs(Motor_speed1 - Motor_speed2) / 2.0f) * 0.0001f; //�����ٶȻ��֣�����·�̳��ȡ�

		get_AMT1450Data_UART(&begin, &jump, count); //�����ݴ洢������������
		if (jump == 2)
		{
			crossing_flag = 1;
			line_position = 0.5f * (count[0] + count[1]); // position=����������м�λ��
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

//�����ĺ���һ�����Ӻ��˳�
void Straight_back(int nSpeed)
{
	PID s_PID;
	s_PIDInit(&s_PID);
	int crossing_flag = 0;

	while (1)
	{
		get_AMT1450Data_UART(&begin, &jump, count); //�����ݴ洢������������

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

void Straight_back_mm(int nSpeed, int distance) //ֱ��һ��������˳�
{
	extern float Motor_speed1;
	extern float Motor_speed2;
	double dis = 0;

	while (dis < distance)
	{
		Delay_10us(10);
		dis = dis + (fabs(Motor_speed1 - Motor_speed2) / 2.0f) * 0.0001f; //�����ٶȻ��֣�����·�̳��ȡ�

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

//========================================����Ϊ����ת��ĺ���=======================

//ת���pid
int32_t Turn_PID(PID *t_PID, int now_angle, int target_angle)
{
	int iError, output = 0;
	iError = target_angle - now_angle; //���ֵ����

	output = t_PID->kp * iError + t_PID->kd * iError - t_PID->lastError;

	t_PID->lastFilter = output; //�˲�ֵ�洢
	t_PID->lastError = iError;	// errorֵ�洢
	return (output);
}

//ͨ���ٶ�pid��ʵ��ת������Ŀ�귽�򼴿ɣ�ʵ��ת���ȿ������
void TurnBY_PID(int turn_angle)
{
	PID t_PID;
	turn_PIDInit(&t_PID);

	extern float Motor_speed1;
	extern float Motor_speed2;

	double now_angle = 0; //��ǰ�Ƕ�

	int flag_left = 1;	//�Ƿ�Ϊ��ת��־
	if (turn_angle < 0) //����Ŀ��Ƕ������һ��������޸ģ��Ա����
	{
		flag_left = 0;
		turn_angle = -turn_angle;
	}
	SetServoAngle(8,120);//צ�����ɿ�//Ϊ���ܹ��ù��ﳵ����
	while (now_angle < turn_angle)
	{
		//����dt�����ֻ�����ǰ�ĽǶȣ�ԭ����̫���ס�
		Delay_10us(10);
		float dspeed = fabs(Motor_speed1 + Motor_speed2);
		now_angle = now_angle + (dspeed * 0.0001f) / (2 * 3.14159f * car_D) * 360;

		//��ʼǿ������,��ת�Ƕ���Ԥ�ڵ�2/3����ʱ�������⵽��������λ�ã�ֱ���Ƴ���ת��
		//��׼�Ļ�ȡǰ��̽ͷ������
		get_AMT1450Data_UART(&begin, &jump, count);

		if (jump == 2)
			line_position = 0.5f * (count[0] + count[1]); // line_position=����������м�λ�ã������ߵ�λ��

		if (line_position > 70 && line_position < 80 && now_angle > turn_angle / 3.0 * 2) //�����������λ�ò�����ת�Ƕ���Ԥ�ڵ�2/3����ʱ��ֱ���˳�
		{
			break;
		}

		//�������û��ת�����ϣ���һֱת
		if (now_angle > turn_angle - 3 && jump <= 1)
		{
			turn_angle += 5;
		}

		int out = (Turn_PID(&t_PID, now_angle, turn_angle) + dspeed / 2) / 2; //����ƽ��ʹ��ת��pid�õ���ֵ
		//�����Ƿ�Ϊ��ת�仯һ�£�ͬʱ��Ҫ�̶�30�ٶȣ���Ȼ����ת��̫����
		if (flag_left == 0)
			out = -out - 50;
		else
			out = out + 50;
		//������ٶ�ֵ
		MotorController_SetSpeed(2, out);
		MotorController_SetSpeed(1, out);
	}
	SetServoAngle(8,140);//צ����ץ��
	//Сͦ��һ��
	// MotorController_SetSpeed(1, 0);
	// MotorController_SetSpeed(2, 0);
	// Delay_ms(50);
}
//ͨ���ٶ�pid��ʵ��ת������Ŀ�귽�򼴿ɣ�ʵ��ת���ȿ����������ǿ��ת������
void TurnBY_PID_notforce_write(int turn_angle)
{
	PID t_PID;
	turn_PIDInit(&t_PID);

	extern float Motor_speed1;
	extern float Motor_speed2;

	double now_angle = 0; //��ǰ�Ƕ�

	int flag_left = 1;	//�Ƿ�Ϊ��ת��־
	if (turn_angle < 0) //����Ŀ��Ƕ������һ��������޸ģ��Ա����
	{
		flag_left = 0;
		turn_angle = -turn_angle;
	}
	SetServoAngle(8,120);//צ�����ɿ�//Ϊ���ܹ��ù��ﳵ����
	while (now_angle < turn_angle)
	{
		//����dt�����ֻ�����ǰ�ĽǶȣ�ԭ����̫���ס�
		Delay_10us(10);
		float dspeed = fabs(Motor_speed1 + Motor_speed2);
		now_angle = now_angle + (dspeed * 0.0001f) / (2 * 3.14159f * car_D) * 360;

		//��ʼǿ������,��ת�Ƕ���Ԥ�ڵ�2/3����ʱ�������⵽��������λ�ã�ֱ���Ƴ���ת��
		//��׼�Ļ�ȡǰ��̽ͷ������
		get_AMT1450Data_UART(&begin, &jump, count);

		if (jump == 2)
			line_position = 0.5f * (count[0] + count[1]); // line_position=����������м�λ�ã������ߵ�λ��

		if (line_position > 70 && line_position < 80 && now_angle > turn_angle / 3.0 * 2) //�����������λ�ò�����ת�Ƕ���Ԥ�ڵ�2/3����ʱ��ֱ���˳�
		{
			break;
		}


		int out = (Turn_PID(&t_PID, now_angle, turn_angle) + dspeed / 2) / 2; //����ƽ��ʹ��ת��pid�õ���ֵ
		//�����Ƿ�Ϊ��ת�仯һ�£�ͬʱ��Ҫ�̶�30�ٶȣ���Ȼ����ת��̫����
		if (flag_left == 0)
			out = -out - 50;
		else
			out = out + 50;
		//������ٶ�ֵ
		MotorController_SetSpeed(2, out);
		MotorController_SetSpeed(1, out);
	}
	SetServoAngle(8,140);//צ����ץ��
	//Сͦ��һ��
	// MotorController_SetSpeed(1, 0);
	// MotorController_SetSpeed(2, 0);
	// Delay_ms(50);
}

//ת�������������׼ǰ���ٶȣ�ת�ٲ��ת�Ƕ�  �Զ�ת��Ԥ��Ƕȣ�ԭ�������pidת���ͬС��
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

		//��ʼǿ������,��ת�Ƕ���Ԥ�ڵ�2/3����ʱ�������⵽��������λ�ã�ֱ���Ƴ���ת��
		get_AMT1450Data_UART(&begin, &jump, count);

		if (jump == 2)
			line_position = 0.5f * (count[0] + count[1]); // line_position=����������м�λ��

		if (line_position > 50 && line_position < 100 && now_angle > turn_angle / 3.0 * 2)
		{
			break;
		}

		//�������û��ת�����ϣ���һֱת
		if (now_angle > turn_angle - 3 && jump <= 1)
		{
			turn_angle += 5;
		}
	}

	MotorController_SetSpeed(1, 0);
	MotorController_SetSpeed(2, 0);
	Delay_ms(50);
}

//��С������ķ��������أ�ת������������
void Turn_to_dir(int to_dir) //ת��Ŀ��Ƕ�
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

//==============================================������С������λ�õ���ز����ĺ���============================

//С����ǰ����任+1��-1��С��������ʱ���Դ�Ϊ1234ѭ��
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

//С��λ������+������������·�ھ��ǼӼ����������С��Ŀǰ�ķ����Զ��Ӻ�С�����ڵ����ꡣ
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

//==============================================�����ǵ�ͼ��ز����ĺ���============================

// Drive_Route,�Զ�����·������������ߣ����ÿ�����Ҫ�ĵġ�
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

//����Ƿ񾭹����̵�
void Crossing_Detection()
{
	// AMT1450ѭ��ģ���ʹ�ã���https://www.luheqiu.com/deane/begin-smart_tracking_car/

	get_AMT1450Data_UART(&begin, &jump, count); //�����ݴ洢������������
	if (jump == 2){
		line_position = 0.5f * (count[0] + count[1]); // position=����������м�λ�ã����ߵ�λ��
	}
		

	//�����ɫû�����䣬�������Ϊ��ɫ����û����·�ڣ���
	if (jump == 0 && begin == 0 && crossing_flag == 1)
	{
		map_count++;		 //��ͼ������1
		Car_Position_add(1); //С��λ�ü�1
		crossing_flag = 0;	 //��־����·��
		printf("carPos:%d\n", map_count);
		// printf("carspeed1%.3f;  carspeed2%.3f\n",Motor_speed1,Motor_speed2);
	}
	//������߳��֣���ô����ʻ����·�ڣ���־=1
	if (jump == 2)
	{
		crossing_flag = 1;
	}
	if(jump==0&&begin==0)
		line_position=77;//����ڰ��ߣ���ֱ�ߣ���Ҫ�ҵ�����
	
}


//��ͼ��Ϊ������Ŀǰ���ڵڼ���·�ڣ�ִ�����ת�����
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
		case 1://ԭ����ת
			Straight_go_mm(200, 120); //�߹������һ�볤
			TurnBY_PID(90);			  // Turn_I(0,300,90);
			Car_Direction_change(1);  //С������ת�䣬1Ϊ����
			break;
		case 2://ԭ����ת
			Straight_go_mm(200, 120); //�߹������һ�볤
			TurnBY_PID(-90);
			Car_Direction_change(-1);
			break;
		case 3://ԭ����ת180
			TurnBY_PID(180);
			Car_Direction_change(2);
			break;
		case 4://ԭ����ת180
			TurnBY_PID(-180);
			Car_Direction_change(2);
			break;
		case 5://ֱ��һ��
			Straight_go(200);
			Straight_go_mm(200, 120); //�߹������һ�볤
			break;	
		case 6://�ȴ���λ���ź�
			Straight_go_mm(200, 120); //�߹������һ�볤
			MotorController_SetSpeed(1, 0);
			MotorController_SetSpeed(2, 0);
			Delay_ms(2000);
			car_flag=Car_Waiting;
			printf("wait_nano\n");
			break;	
		case 7://����һ��
			Straight_back_mm(200,480);
			break;	
		case 8://ת�Ǵ�����+��ת
			Straight_back_mm(200,420);

			TurnBY_PID(-90);
			Car_Direction_change(-1);

			break;	
		case 9://�źù��ﳵ�ĵ���+��צ+��ǰһ��
			/*
			TurnBY_PID(-90);
			Straight_go(200);
			Straight_go_mm(200, 120);//�߹������һ�볤
			TurnBY_PID(90);
			Straight_go_mm(200, 120);//�߹������һ�볤
			TurnBY_PID_notforce_write(-90);
			Straight_go_mm(0, 0);//ֹͣ
			Delay_ms(1000);
			car_flag=Car_Stop;
			*/
				TurnBY_PID(-90);
				Straight_go_mm(200, 120);//�߹������һ�볤
				MotorController_SetSpeed(1, -200);
				MotorController_SetSpeed(2, 200);
				Delay_ms(500);
				TurnBY_PID_notforce_write(90);
				MotorController_SetSpeed(1, -200);
				MotorController_SetSpeed(2, 200);
				Delay_ms(800);
				TurnBY_PID_notforce_write(-90);
				Straight_go_mm(0, 0);//ֹͣ
				Delay_ms(1000);
				car_flag=Car_Stop;
		
			break;	
		case 10://��ת+��ʱ1s+����1��
			Straight_go_mm(200, 120); //�߹������һ�볤
			TurnBY_PID(90);			  // Turn_I(0,300,90);
			Car_Direction_change(1);  //С������ת�䣬1Ϊ����

			SetServoAngle(8,90);
			Straight_back_mm(200,230);
			SetServoAngle(8,140);
			Straight_back_mm(50,40);

			Straight_go(220);
			//Straight_go_mm(200, 120); //�߹������һ�볤
		    break;
		default:
			break;
		}
		map_index++;
	}
}
