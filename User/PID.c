#include "PID.h"
#include "delay.h"
#include "MotorController.h"
#include "amt1450_uart.h"
#include "MotorDriver.h"
#include "AStarRoute.h"
#include "stdlib.h"

int car_direction=3;
int car_position[2]={9,8};
void s_PIDInit(PID *s_PID)
{
	s_PID->error_acc = 0;
	s_PID->lastError = 0;
	s_PID->prevError = 0;

	s_PID->kp = 3.8; 
	s_PID->ki = 0;
	s_PID->kd = 1.4; 
	s_PID->target = 76;
	
	s_PID->filter=0.5;
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
	extern float Motor_speed1;
	extern float Motor_speed2;
	
	float car_D=230;double now_angle=0;
	while(now_angle<turn_angle)
	{
		Delay_10us(10);
		float dspeed=Motor_speed1+Motor_speed2;
		if(dspeed<0)
			dspeed=-dspeed;
		now_angle=now_angle+(dspeed*0.0001f)/(2*3.14159f*car_D)*360;
		
		//开始强制修正,旋转角度在预期的2/3以上时，如果检测到线在中心位置，直接推出旋转。
		uint8_t begin, jump, count[6]; // 最大6个跳变，即3条线
		uint8_t position;
		get_AMT1450Data_UART(&begin, &jump, count);
		
		if (jump == 2)
			position = 0.5f * (count[0] + count[1]); // position=两次跳变的中间位置
		
		
		if(position>68 &&position<80 && now_angle>turn_angle/3.0*2){
			break;
		}
		
		//由于速度的调整有延迟，积分出来比较小，所以如果没有转到线上，就一直转
		//if(now_angle>turn_angle-3&&jump<=1){
		//	turn_angle+=5;
		//}
	}
	
	MotorController_SetSpeed(1,0);
	MotorController_SetSpeed(2, 0);
	Delay_ms(50);
	
}

void Car_Direction_change(int add)
{
	while(add!=0){
		if(add>0){
			car_direction+=1;
			add--;
		}
		else if(add<0){
			car_direction-=1;
			add++;
		}
		if(car_direction>4)car_direction=1;
		if(car_direction<1)car_direction=4;
	}
};

void Car_Position_add()
{
	switch(car_direction){
		case 1:car_position[0]+=1;
				break;
			case 2:car_position[1]+=1;
				break;
			case 3:car_position[0]-=1;
				break;
			case 4:car_position[1]-=1;
				break;
			default:
				break;
	
	}

};

void Straight_go(int nSpeed)//直走一格后退出
{
		PID s_PID;
		s_PIDInit(&s_PID);
		int crossing_flag=0;
		
		while(1){
				uint8_t begin, jump, count[6]; // 最大6个跳变，即3条线
				uint8_t position;
				get_AMT1450Data_UART(&begin, &jump, count); //讲数据存储在三个变量中
				if (jump == 2){
						crossing_flag=1;
						position = 0.5f * (count[0] + count[1]); // position=两次跳变的中间位置
				}
				if(jump==0&&begin==0&&crossing_flag==1){
					Car_Position_add();
					crossing_flag=0;
					printf("dir:%d  ; pos:%d,%d\r\n",car_direction,car_position[0],car_position[1]);
					break;
				}
				
				int32_t spid_out = Straight_PID(&s_PID, position);
				MotorController_SetSpeed(1, -nSpeed + spid_out);
				MotorController_SetSpeed(2, nSpeed + spid_out);
		}
}

void Turn_to_dir(int to_dir) //转到目标角度
{
		if(abs(car_direction-to_dir)==2){
				Turn_I(0,500,180);
				Car_Direction_change(2);
		}else{
				if(car_direction==1||car_direction==4){
					if(car_direction==1){
						if(car_direction+3==to_dir){
							Turn_I(0,500,-90);  
							Car_Direction_change(-1);
						}else{
							Turn_I(0,500,90);  
						Car_Direction_change(1);
						}
					}else{
						if(car_direction-3==to_dir){
						Turn_I(0,500,90);  
						Car_Direction_change(1);
						}else{
						Turn_I(0,500,-90);  
						Car_Direction_change(-1);
						}
					}
				}else{
					if(car_direction>to_dir){
						Turn_I(0,500,-90);  
						Car_Direction_change(-1);
					}else{
						Turn_I(0,500,90); 
						Car_Direction_change(1);
					}
				}
		}
}


void Drive_Route(int nSpeed,int tox,int toy)
{
	int way[50]={0};

	FindPath(way,car_position[0],car_position[1],tox,toy);
	
	for(int i=1;i<50&&way[i]!=-1;i++){
		printf("%d\n",way[i]);
		if(car_direction==way[i]){
			Straight_go(nSpeed);
		}
		else{
			Turn_to_dir(way[i]);
			i=i-1;
		}
	}

	MotorController_SetSpeed(1, 0);
	MotorController_SetSpeed(2, 0);
	Delay_ms(300);
	
};
