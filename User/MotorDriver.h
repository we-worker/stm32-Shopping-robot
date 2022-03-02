#ifndef __MOTORDRIVER_H
#define __MOTORDRIVER_H
#include "stm32f4xx.h"

extern float Motor_speed1;//ʵʱת��
extern float Motor_speed2;//ʵʱת��

/*���PWM��������
ʹ��TIM1��·PWMͨ�������ĸ������PA11��PA10��PA9��PA8
���A��IN1-PD14��IN2-PA11��TIM1-4����EF-PD15
���B��IN1-PD10��IN2-PA10��TIM1-3����EF-PD11
���C��IN1-PC8��IN2-PA9��TIM1-2����EF-PC9
���D��IN1-PD8��IN2-PA8��TIM1-1����EF-PD9
*/
#define PWM_DUTY_LIMIT 10000  //PWMռ�ձȷ�Χ0~10000
void MotorDriver_Init(uint8_t nMotorCount); //��ʼ�����������nMotorCount=1����ʼ�����A��nMotor=2����ʼ�����A��B���Դ����ƣ���1��4
																			//��ʼ����Ĭ�ϵ����ֹͣ�ģ���Ҫ��MotorDriver_Start�������																				
void MotorDriver_SetPWMDuty(uint8_t nMotor, uint16_t nDuty); //����PWMռ�ձ�
void MotorDriver_Start(uint8_t nMotor, uint16_t nDuty); //���������nMotor�������ţ�nDuty����ʼת�٣�PWM_DUTY_LIMIT/2����ת��0 
void MotorDriver_Stop(uint8_t nMotor, uint16_t nDuty); //ֹͣ�����nMotor�������ţ�nDuty��ֹͣ���ٶȣ�0�൱�ڼ�ɲ������ֵԽ��ͣ��Խ������󲻳���PWM_DUTY_LIMIT 
																											 //ÿ��ֹͣ����Ҫ��MotorDriver_Start�����������
uint8_t MotorDriver_GetMotorState(uint8_t nMotor); //��ȡ���nMotor��״̬��0-����״̬��1-ֹͣ״̬
																											 
/*����������
ʹ����TIM4��TIM8��TIM3��TIM2
������A��A-PB4(TIM3-1)��B-PB5��TIM3-2��
������B��A-PA15(TIM2-1)��B-PB3��TIM2-2��
������C��A-PC6(TIM8-1)��B-PC7��TIM8-2��
������D��A-PD12(TIM4-1)��B-PD13��TIM4-2��
*/
#define ENC_TIM_ARR 60000
void Encoder_Init(uint8_t nEncoderCount); //��ʼ����������nEncoderCount=1����ʼ��������A��=2����ʼ��������A��B���Դ����ƣ���1��4
uint16_t Encoder_GetCNT(uint8_t nEncoder); //���ر������ļ���ֵ��nEncoder=1���ر�����A���Դ�����
int32_t Encoder_GetEncCount(uint8_t nEncoder);//���ر������ۼƼ���ֵ��32λ�з���ֵ����Ϊ��ת����Ϊ��ת�������20�ڣ���ʱ������ע�������
#endif