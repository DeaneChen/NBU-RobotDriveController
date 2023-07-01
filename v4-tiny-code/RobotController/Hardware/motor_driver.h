#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H

#include "main.h"


#include "stm32f4xx.h"
#include "stm32f4xx_hal_tim.h"

/*���PWM��������
ʹ��TIM1��·PWMͨ�������ĸ������PA11��PA10��PA9��PA8
���1��IN1-PD3��IN2-PA11��TIM1-4����
���2��IN1-PDE11��IN2-PA8��TIM1-1����
���3��IN1-PD14��IN2-PA10��TIM1-3����
���4��IN1-PE15��IN2-PA9��TIM1-2����
*/

extern int16_t MotorDirver_Tim4_Update_Count;
extern int16_t MotorDirver_Tim5_Update_Count;
extern int16_t MotorDirver_Tim3_Update_Count;
extern int16_t MotorDirver_Tim2_Update_Count;

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
������1��A-PA15(TIM2-1)��B-PB3��TIM2-2��
������2��A-PD12(TIM4-1)��B-PD13��TIM4-2��
������3��A-PB4(TIM3-1)��B-PB5��TIM3-2��
������4��A-PA0(TIM5-1)��B-PA1��TIM5-2��
*/
#define ENC_TIM_ARR 60000
void Encoder_Init(uint8_t nEncoderCount); //��ʼ����������nEncoderCount=1����ʼ��������A��=2����ʼ��������A��B���Դ����ƣ���1��4
uint16_t Encoder_GetCNT(uint8_t nEncoder); //���ر������ļ���ֵ��nEncoder=1���ر�����A���Դ�����
int32_t Encoder_GetEncCount(uint8_t nEncoder);//���ر������ۼƼ���ֵ��32λ�з���ֵ����Ϊ��ת����Ϊ��ת�������20�ڣ���ʱ������ע�������



#endif

