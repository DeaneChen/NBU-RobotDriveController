#include "stm32f4xx.h"
#include "ArmSolution.h"
#include "delay.h"
#include "math.h"
#include "stdlib.h"

#define PWM_DUTY_LIMIT 10000 // PWMռ�ձ�����Ϊ10000,����20ms    250-1250 ���� 0-180��

// ��е�۲�����ʼ�����Լ����Ժ궨���е�۵ı۳�
#define pi 3.1415926f

// ÿ����������ƶ���Ŀ��pwm
uint16_t targetPwm[8] = {250};

/*���������ʼ��(��е��) ����������õĶ������ˣ�������ɾ��һЩ*/
void ArmDriver_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;		   // GPIO�ṹ�嶨��
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; // TIM�����ṹ�嶨��
	TIM_OCInitTypeDef TIM_OCInitStructure;		   // TIMͨ���ṹ�嶨��

	/* GPIOE clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); // ʹ��GPIOEʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // ʹ��GPIOAʱ��

	/* GPIOE Configuration: TIM9 CH2 (PE6)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; // ����GPIOE��5��6�ܽ�
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		   // ����GPIOEΪAFģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	   // ����GPIO����
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		   // �����������Ϊ�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		   // ����GPIOΪ����ģʽ
	GPIO_Init(GPIOE, &GPIO_InitStructure);				   // ��ʼ��GPIOE

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7; // ����GPIOA��0��1��2��3��6��7�ܽ�
	GPIO_Init(GPIOA, &GPIO_InitStructure);																	   // ��ʼ��GPIOA

	/*	����Gpio���� ����Ҫ�����Ŷ���Ϊ��ʱ��ģʽ������PWM����*/
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM13);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM14);

	/* TIM clock enable ��ʱ������ */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

	/* Time base configuration TIM�������� */
	TIM_TimeBaseStructure.TIM_Period = PWM_DUTY_LIMIT - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 336 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 336 / 2 - 1;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

	// ����PWM���
	/* PWM Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = PWM_DUTY_LIMIT / 2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	// ����Timͨ��
	TIM_OC1Init(TIM9, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM9, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);

	TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);

	TIM_OC1Init(TIM13, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM13, TIM_OCPreload_Enable);
	TIM_OC1Init(TIM14, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);

	// ʹ�ܶ�ʱ��
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	TIM_ARRPreloadConfig(TIM9, ENABLE);
	TIM_ARRPreloadConfig(TIM13, ENABLE);
	TIM_ARRPreloadConfig(TIM14, ENABLE);

	/* TIM9 enable counter ���ü��� */
	TIM_Cmd(TIM9, ENABLE);
	TIM_Cmd(TIM5, ENABLE);
	TIM_Cmd(TIM13, ENABLE);
	TIM_Cmd(TIM14, ENABLE);

	// ��ʼ��һ�¸��������Ĭ��λ��
	TIM9->CCR1 = 500;
	TIM9->CCR2 = 0;
	TIM5->CCR1 = 500;
	TIM5->CCR2 = 750;
	TIM5->CCR3 = 750;
	TIM5->CCR4 = 750;
	TIM13->CCR1 = 500;
	TIM14->CCR1 = 500;

	// ���������ö�ʱ��7��ʵ�ֻ�е�ۻ����ƶ�
	RCC_ClocksTypeDef RCC_Clocks;		 // RCCʱ�ӽṹ��
	NVIC_InitTypeDef NVIC_InitStructure; // NVIC�ж������ṹ��

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	RCC_GetClocksFreq(&RCC_Clocks); // ��ȡϵͳʱ��
	// Ԥ��Ƶֵ�ļ��㷽����ϵͳʱ��Ƶ��/TIM6����ʱ��Ƶ�� - 1
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(RCC_Clocks.SYSCLK_Frequency / 20000) - 1; // ����Ƶ��10KHz
	// ʹTIM6���Ƶ�ʵļ��㷽����TIM6����ʱ��Ƶ��/��ARR+1���������ARR����TIM_Period��ֵ�����9�����TIM6����ʱ��Ƶ��Ϊ10K�����������Ϊ1ms
	TIM_TimeBaseStructure.TIM_Period = (uint16_t)30 * 8 - 1; // Ҳ����50msһ��
	// TIM_TimeBaseStructure.TIM_Period = (uint16_t)30 * 10 - 1; //Ҳ����50msһ��
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

	/* Enable the TIM6 Update Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);	   // ����±�־λ
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE); // ������ж�
	TIM_Cmd(TIM7, ENABLE);					   // ʹ�ܼ���
}

/**
 * @description: ���ö���Ƕ�
 * @param {int} nServo���Ŷ��,��0-7
 * @param {float} angle�Ƕ�(0-180)
 * @return {*}
 */
void SetServoAngle(int nServo, float angle)
{
	if (angle < 0) // �Ƕ�����
		return;
	int pwm = angle * 1.0f / 90 / 20 * PWM_DUTY_LIMIT + 250; // �������Ӧ��pwm��

	// pwm���ƣ���ֹ��������
	if (pwm > 1250)
		pwm = 1250;
	if (pwm < 250)
		pwm = 250;

	targetPwm[nServo] = pwm;
}

/* �жϵ����Ƿ���� ��ÿ������Ƕ��Ƿ���Ŀ��һ�� 
	��Ҫ�����Լ�������ȥɾȥһЩû�õ��Ķ��pwm�ж�
*/
uint8_t ServoTunnerOK(void)	
{
	if(targetPwm[0] == TIM9->CCR1&& \
	   targetPwm[1] == TIM9->CCR2&& \
	   targetPwm[2] == TIM5->CCR1&& \
	   targetPwm[3] == TIM5->CCR2&& \
	   targetPwm[4] == TIM5->CCR3&& \
	   targetPwm[5] == TIM5->CCR4&& \
	   targetPwm[6] == TIM13->CCR1&& \
	   targetPwm[7] == TIM14->CCR1) 
		return 1;
	else
		return 0;
}

/*��������ƶ����������ÿ�����Ҫ����������ı�*/
void slowPwm(uint8_t nServo)
{
	volatile uint32_t *currentPwm;
	switch (nServo)
	{
	case 0:
		currentPwm = &(TIM9->CCR1);
		break;
	case 1:
		currentPwm = &TIM9->CCR2;
		break;
	case 2:
		currentPwm = &TIM5->CCR1;
		break;
	case 3:
		currentPwm = &TIM5->CCR2;
		break;
	case 4:
		currentPwm = &TIM5->CCR3;
		break;
	case 5:
		currentPwm = &TIM5->CCR4;
		break;
	case 6:
		currentPwm = &TIM13->CCR1;
		break;
	case 7:
		currentPwm = &TIM14->CCR1;
		break;
	default:
		return;
	}

	int8_t flag = 1;
	if (*currentPwm > targetPwm[nServo])
		flag = -1; // �����ǰpwmֵ����Ŀ��pwmֵ����flag����Ϊ-1
	if (*currentPwm == targetPwm[nServo])
		flag = 0; // �����ǰpwmֵ����Ŀ��pwmֵ����flag����Ϊ0
	if (abs((*currentPwm) - targetPwm[nServo]) > 10)
		flag = 10 * flag; // �����ǰpwmֵС��Ŀ��pwmֵ������֮��Ĳ�ֵ����10��flag��Ҫ����10
	// ����ǰpwmֵ����flag��ʵ�ֻ����ƶ���Ч��
	*currentPwm = (*currentPwm) + flag;
}

/*����Ϊ��е�ۻ����ƶ�ʹ�õ�tim7�ж�*/
void TIM7_IRQHandler(void)
{
	// �Ƿ��и����ж�
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		// ����жϱ�־
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		// �����ж�
		if (ServoTunnerOK() == 0) // δ�������
		{
			for (uint8_t i = 0; i < 8; i++) // �������ж��
				slowPwm(i);
		}
	}
}


/**
 * @description: ArmSolution ��е��λ�ý��㣬��Ҫ�Լ��������x��y���꣬��е���Զ��쵽�Ǹ�λ��
 * @return {*}�Զ��������ж���ĽǶ�
 */
void ArmSolution(double x, double y)
{
}

/**
 * @description: ��е��ץȡ��������Ŀ���λ�ã���е��ץȡ
 * @param {*}�Զ�����Ŀ��λ�ý�������ץȡ
 * @return {*}
 */
void Arm_Grab()
{
}
