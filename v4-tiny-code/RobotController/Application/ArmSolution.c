#include "stm32f4xx.h"
#include "ArmSolution.h"
#include "delay.h"
#include "math.h"
#include "stdlib.h"

#define PWM_DUTY_LIMIT 10000 // PWM占空比周期为10000,代表20ms    250-1250 代表 0-180度

// 机械臂参数初始化，自己可以宏定义机械臂的臂长
#define pi 3.1415926f

// 每个舵机缓慢移动的目标pwm
uint16_t targetPwm[8] = {250};

/*各个舵机初始化(机械臂) 我这里把能用的都用上了，可酌情删减一些*/
void ArmDriver_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;		   // GPIO结构体定义
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; // TIM基础结构体定义
	TIM_OCInitTypeDef TIM_OCInitStructure;		   // TIM通道结构体定义

	/* GPIOE clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); // 使能GPIOE时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 使能GPIOA时钟

	/* GPIOE Configuration: TIM9 CH2 (PE6)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; // 配置GPIOE的5、6管脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		   // 配置GPIOE为AF模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	   // 配置GPIO速率
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		   // 配置输出类型为推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		   // 配置GPIO为上拉模式
	GPIO_Init(GPIOE, &GPIO_InitStructure);				   // 初始化GPIOE

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7; // 配置GPIOA的0、1、2、3、6、7管脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);																	   // 初始化GPIOA

	/*	配置Gpio复用 都需要把引脚定义为定时器模式，方面PWM控制*/
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM13);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM14);

	/* TIM clock enable 定时器启用 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

	/* Time base configuration TIM基础设置 */
	TIM_TimeBaseStructure.TIM_Period = PWM_DUTY_LIMIT - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 336 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 336 / 2 - 1;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

	// 配置PWM输出
	/* PWM Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = PWM_DUTY_LIMIT / 2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	// 配置Tim通道
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

	// 使能定时器
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	TIM_ARRPreloadConfig(TIM9, ENABLE);
	TIM_ARRPreloadConfig(TIM13, ENABLE);
	TIM_ARRPreloadConfig(TIM14, ENABLE);

	/* TIM9 enable counter 启用计数 */
	TIM_Cmd(TIM9, ENABLE);
	TIM_Cmd(TIM5, ENABLE);
	TIM_Cmd(TIM13, ENABLE);
	TIM_Cmd(TIM14, ENABLE);

	// 初始化一下各个电机的默认位置
	TIM9->CCR1 = 500;
	TIM9->CCR2 = 0;
	TIM5->CCR1 = 500;
	TIM5->CCR2 = 750;
	TIM5->CCR3 = 750;
	TIM5->CCR4 = 750;
	TIM13->CCR1 = 500;
	TIM14->CCR1 = 500;

	// 接下来配置定时器7，实现机械臂缓慢移动
	RCC_ClocksTypeDef RCC_Clocks;		 // RCC时钟结构体
	NVIC_InitTypeDef NVIC_InitStructure; // NVIC中断向量结构体

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	RCC_GetClocksFreq(&RCC_Clocks); // 获取系统时钟
	// 预分频值的计算方法：系统时钟频率/TIM6计数时钟频率 - 1
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(RCC_Clocks.SYSCLK_Frequency / 20000) - 1; // 计数频率10KHz
	// 使TIM6溢出频率的计算方法：TIM6计数时钟频率/（ARR+1），这里的ARR就是TIM_Period的值，设成9，如果TIM6计数时钟频率为10K，则溢出周期为1ms
	TIM_TimeBaseStructure.TIM_Period = (uint16_t)30 * 8 - 1; // 也就是50ms一次
	// TIM_TimeBaseStructure.TIM_Period = (uint16_t)30 * 10 - 1; //也就是50ms一次
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

	/* Enable the TIM6 Update Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);	   // 清更新标志位
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE); // 打开溢出中断
	TIM_Cmd(TIM7, ENABLE);					   // 使能计数
}

/**
 * @description: 设置舵机角度
 * @param {int} nServo几号舵机,从0-7
 * @param {float} angle角度(0-180)
 * @return {*}
 */
void SetServoAngle(int nServo, float angle)
{
	if (angle < 0) // 角度限制
		return;
	int pwm = angle * 1.0f / 90 / 20 * PWM_DUTY_LIMIT + 250; // 解算出对应的pwm波

	// pwm限制，防止动作错误
	if (pwm > 1250)
		pwm = 1250;
	if (pwm < 250)
		pwm = 250;

	targetPwm[nServo] = pwm;
}

/* 判断调整是否结束 即每个舵机角度是否都与目标一致 
	需要按照自己的需求去删去一些没用到的舵机pwm判断
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

/*舵机缓慢移动函数，不用看核心要义就是慢慢改变*/
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
		flag = -1; // 如果当前pwm值大于目标pwm值，将flag设置为-1
	if (*currentPwm == targetPwm[nServo])
		flag = 0; // 如果当前pwm值等于目标pwm值，将flag设置为0
	if (abs((*currentPwm) - targetPwm[nServo]) > 10)
		flag = 10 * flag; // 如果当前pwm值小于目标pwm值，它们之间的差值大于10，flag需要乘以10
	// 将当前pwm值加上flag，实现缓慢移动的效果
	*currentPwm = (*currentPwm) + flag;
}

/*以下为机械臂缓慢移动使用的tim7中断*/
void TIM7_IRQHandler(void)
{
	// 是否有更新中断
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		// 清除中断标志
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
		// 处理中断
		if (ServoTunnerOK() == 0) // 未调节完成
		{
			for (uint8_t i = 0; i < 8; i++) // 遍历所有舵机
				slowPwm(i);
		}
	}
}


/**
 * @description: ArmSolution 机械臂位置解算，需要自己完成输入x，y坐标，机械臂自动伸到那个位置
 * @return {*}自动设置所有舵机的角度
 */
void ArmSolution(double x, double y)
{
}

/**
 * @description: 机械臂抓取程序，输入目标的位置，机械臂抓取
 * @param {*}自动根据目标位置进行整个抓取
 * @return {*}
 */
void Arm_Grab()
{
}
