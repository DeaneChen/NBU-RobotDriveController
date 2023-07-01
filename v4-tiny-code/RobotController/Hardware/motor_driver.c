#include "tim.h"
#include "gpio.h"
#include "motor_driver.h"
#include "delay.h"



//��֪���ò���OFF����
//			HAL_GPIO_WritePin(M1_OFF_GPIO_Port,M1_OFF_Pin,GPIO_PIN_RESET);

int16_t MotorDirver_Tim4_Update_Count=0;
int16_t MotorDirver_Tim5_Update_Count=0;
int16_t MotorDirver_Tim3_Update_Count=0;
int16_t MotorDirver_Tim2_Update_Count=0;

#define LedIO_Reset HAL_GPIO_WritePin(FnLEDn_GPIO_Port, FnLEDn_Pin, GPIO_PIN_RESET)
#define LedIO_Set HAL_GPIO_WritePin(FnLEDn_GPIO_Port, FnLEDn_Pin, GPIO_PIN_SET)
//�����ʼ����ע��case����û��break�����Ի��������������

//Drv8243 P23ҳ
//�����ʼ������
void MotorDriver_Init(uint8_t nMotorCount)
{
	// ����������Ƿ���Ч
	if(nMotorCount<1||nMotorCount>4) return;
	
	// forѭ�����λ��Ѻ�����ָ�����������
	for (uint8_t motor = 1; motor <= nMotorCount; motor++)
	{
			GPIO_TypeDef* nSLEEP_Port;
			uint16_t nSLEEP_Pin;
			GPIO_TypeDef* OFF_Port;
			uint16_t OFF_Pin;
			uint32_t channel;
			
			// ���ݵ�����ѡ����Ӧ�����ź�pwm���ͨ��
			switch (motor)
			{
					case 1:
							nSLEEP_Port = M1_nSLEEP_GPIO_Port;
							nSLEEP_Pin = M1_nSLEEP_Pin;
							OFF_Port = M1_OFF_GPIO_Port;
							OFF_Pin = M1_OFF_Pin;
							channel = TIM_CHANNEL_4;
							break;
					case 2:
							nSLEEP_Port = M2_nSLEEP_GPIO_Port;
							nSLEEP_Pin = M2_nSLEEP_Pin;
							OFF_Port = M2_OFF_GPIO_Port;
							OFF_Pin = M2_OFF_Pin;
							channel = TIM_CHANNEL_1;
							break;
					case 3:
							nSLEEP_Port = M3_nSLEEP_GPIO_Port;
							nSLEEP_Pin = M3_nSLEEP_Pin;
							OFF_Port = M3_OFF_GPIO_Port;
							OFF_Pin = M3_OFF_Pin;
							channel = TIM_CHANNEL_3;
							break;
					case 4:
							nSLEEP_Port = M4_nSLEEP_GPIO_Port;
							nSLEEP_Pin = M4_nSLEEP_Pin;
							OFF_Port = M4_OFF_GPIO_Port;
							OFF_Pin = M4_OFF_Pin;
							channel = TIM_CHANNEL_2;
							break;
			}
			
			// ���ѵ�����ȴ��������״̬
			HAL_GPIO_WritePin(nSLEEP_Port, nSLEEP_Pin, GPIO_PIN_SET);
			HAL_Delay(1);																								//�ȴ�������Ӧ
			
			// ȷ�ϵ���ѻ���
			HAL_GPIO_WritePin(nSLEEP_Port, nSLEEP_Pin, GPIO_PIN_RESET);
			delay_us(31);																								//�ȴ�������Ӧ
			//while(HAL_GPIO_ReadPin(M4_nFAULT_GPIO_Port,M4_nFAULT_Pin)!=1){}//����while����delay��д�����ǵ��Ŀ�����
			HAL_GPIO_WritePin(nSLEEP_Port, nSLEEP_Pin, GPIO_PIN_SET);		//nsleep����
			
			// ����PWM����͹ر�OFF����
			HAL_TIM_PWM_Start(&htim1, channel);
			HAL_GPIO_WritePin(OFF_Port, OFF_Pin, GPIO_PIN_RESET);
			
			// ֹͣ���
			MotorDriver_Stop(motor, 5000);
		}
}

void MotorDriver_Start(uint8_t nMotor, uint16_t nDuty)
{
	uint16_t nDutySet;
	if(nDuty>PWM_DUTY_LIMIT) nDutySet = PWM_DUTY_LIMIT;
	else nDutySet = nDuty;
	switch (nMotor)
	{
		case 4:
			TIM1->CCR2 = nDutySet;
			HAL_Delay(1);
			HAL_GPIO_WritePin(M4_OFF_GPIO_Port,M4_OFF_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(M4_IN1_GPIO_Port,M4_IN1_Pin,GPIO_PIN_SET);
			break;
		case 3:
			TIM1->CCR3 = nDutySet;
			HAL_Delay(1);		
			HAL_GPIO_WritePin(M3_OFF_GPIO_Port,M3_OFF_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(M3_IN1_GPIO_Port,M3_IN1_Pin,GPIO_PIN_SET);
			break;
		case 2:
			TIM1->CCR1 = nDutySet;
			HAL_Delay(1);		
			HAL_GPIO_WritePin(M2_OFF_GPIO_Port,M2_OFF_Pin,GPIO_PIN_RESET);	
			HAL_GPIO_WritePin(M2_IN1_GPIO_Port,M2_IN1_Pin,GPIO_PIN_SET);
			break;
		case 1:
			TIM1->CCR4 = nDutySet;
			HAL_Delay(1);		
			HAL_GPIO_WritePin(M1_OFF_GPIO_Port,M1_OFF_Pin,GPIO_PIN_RESET);	
			HAL_GPIO_WritePin(M1_IN1_GPIO_Port,M1_IN1_Pin,GPIO_PIN_SET);
			break;
		default:
			;
	}	
}


void MotorDriver_SetPWMDuty(uint8_t nMotor, uint16_t nDuty)
{
	uint16_t nDutySet;
	if(nDuty>PWM_DUTY_LIMIT) nDutySet = PWM_DUTY_LIMIT;
	else nDutySet = nDuty;
	switch (nMotor)
	{
		case 1:
			TIM1->CCR4 = nDutySet;
			break;
		case 2:
			TIM1->CCR1 = nDutySet;
			break;
		case 3:
			TIM1->CCR3 = nDutySet;
			break;
		case 4:
			TIM1->CCR2 = nDutySet;
			break;
		default:
			;
	}
}


void MotorDriver_Stop(uint8_t nMotor, uint16_t nDuty)
{
	uint16_t nDutySet;
	if(nDuty>PWM_DUTY_LIMIT) nDutySet = PWM_DUTY_LIMIT;
	else nDutySet = nDuty;
	switch (nMotor)
	{
		case 1:
			HAL_GPIO_WritePin(M1_IN1_GPIO_Port,M1_IN1_Pin,GPIO_PIN_RESET);
			TIM1->CCR4 = nDutySet;
			break;
		case 2:
			HAL_GPIO_WritePin(M2_IN1_GPIO_Port,M2_IN1_Pin,GPIO_PIN_RESET);
			TIM1->CCR1 = nDutySet;
			break;
		case 3:
			HAL_GPIO_WritePin(M3_IN1_GPIO_Port,M3_IN1_Pin,GPIO_PIN_RESET);
			TIM1->CCR3 = nDutySet;
			break;
		case 4:
			HAL_GPIO_WritePin(M4_IN1_GPIO_Port,M4_IN1_Pin,GPIO_PIN_RESET);
			TIM1->CCR2 = nDutySet;
			break;
		default:
			;
	}	
}
void MotorDriver_Off(uint8_t nMotor)
{
	switch (nMotor)
	{
		case 1:
			HAL_GPIO_WritePin(M1_OFF_GPIO_Port,M1_OFF_Pin,GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(M2_OFF_GPIO_Port,M2_OFF_Pin,GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(M3_OFF_GPIO_Port,M3_OFF_Pin,GPIO_PIN_SET);
			break;
		case 4:
			HAL_GPIO_WritePin(M4_OFF_GPIO_Port,M4_OFF_Pin,GPIO_PIN_SET);
			break;
		default:
			;
	}	
}


//�����û����
uint8_t MotorDriver_GetMotorState(uint8_t nMotor)
{
	switch (nMotor)
	{
		case 1:
			return HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_14);
		case 2:
			return HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10);
		case 3:
			return HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8);
		case 4:
			return HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8);
		default:
			return 1;
	}	
}

void Encoder_Init(uint8_t nEncoderCount)
{
	if(nEncoderCount<1||nEncoderCount>4) return;
	switch(nEncoderCount)
	{
		case 4:
			__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
			HAL_TIM_Base_Start_IT(&htim5);
			TIM5->CNT=0;
			HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
		case 3:
			__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
		  	HAL_TIM_Base_Start_IT(&htim3);
			TIM3->CNT=0;
			HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
		case 2:
			__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
		  	HAL_TIM_Base_Start_IT(&htim4);
			TIM4->CNT=0;
			HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
		case 1:
			__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
		  	HAL_TIM_Base_Start_IT(&htim2);
			TIM2->CNT=0;
			HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
		default:
			;
	}
	 
}


uint16_t Encoder_GetCNT(uint8_t nEncoder)
{
	switch (nEncoder)
	{
		case 1:
			return TIM2->CNT;
		case 2:
			return TIM4->CNT;
		case 3:
			return TIM3->CNT;
		case 4:
			return TIM5->CNT;
		default:
			return 0;
	}
}
int32_t Encoder_GetEncCount(uint8_t nEncoder)
{
	switch (nEncoder)
	{
		case 1:
			return (int32_t)ENC_TIM_ARR*MotorDirver_Tim2_Update_Count+TIM2->CNT;
		case 2:
			return (int32_t)ENC_TIM_ARR*MotorDirver_Tim4_Update_Count+TIM4->CNT;
		case 3:
			return (int32_t)ENC_TIM_ARR*MotorDirver_Tim3_Update_Count+TIM3->CNT;
		case 4:
			return (int32_t)ENC_TIM_ARR*MotorDirver_Tim5_Update_Count+TIM5->CNT;
		default:
			return 0;
	}
}


