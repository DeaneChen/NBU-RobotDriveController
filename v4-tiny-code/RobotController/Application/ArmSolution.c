
#include "ArmSolution.h"
#include "delay.h"
#include "math.h"
#include "tim.h"
#include "stdlib.h"

#define PWM_DUTY_LIMIT 10000 // PWM占空比周期为10000,代表20ms    250-1250 代表 0-180度

// 每个舵机缓慢移动的目标pwm
uint16_t targetPwm[8] = {250};


void ArmDriver_Init(void){
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);

	//直接给pwm赋初值  初始设定为初始机械臂状态  每次上电前应该把机械臂归位
	//下面为使用举例
	// Servo_init(0,177);
	// Servo_init(1,162);
	// Servo_init(2,68);
	// Servo_init(3,60);
}
void Servo_init(uint8_t nServo,int angle)
{
	volatile uint32_t *currentPwm;//指针指向不同的pwm控制地址，方便统一修改。
	switch (nServo)
	{
	case 0:
		currentPwm = &htim8.Instance->CCR3;
		break;
	case 1:
		currentPwm = &htim8.Instance->CCR4;
		break;
	case 2:
		currentPwm = &htim12.Instance->CCR1;
		break;
	case 3:
		currentPwm = &htim12.Instance->CCR2;
		break;
	case 4:
		currentPwm = &htim9.Instance->CCR1;
		break;
	case 5:
		currentPwm = &htim9.Instance->CCR2;
		break;
	case 6:
		currentPwm = &htim13.Instance->CCR1;
		break;
	case 7:
		currentPwm = &htim14.Instance->CCR1;
		break;
	default:
		return;
}
	if (angle < 0) // 角度限制
		return;
	int pwm = angle * 1.0f / 90 / 20 * PWM_DUTY_LIMIT + 250; // 解算出对应的pwm波

	// pwm限制，防止动作错误
	if (pwm > 1250)
		pwm = 1250;
	if (pwm < 250)
		pwm = 250;

	*currentPwm = pwm;	//设置定时器CCR中PWM值
	targetPwm[nServo]=pwm;	//设置目标PWM值
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
	if(targetPwm[0] == htim8.Instance->CCR3&& \
	   targetPwm[1] == htim8.Instance->CCR4&& \
	   targetPwm[2] == htim12.Instance->CCR1&& \
	   targetPwm[3] == htim12.Instance->CCR2&& \
	   targetPwm[4] == htim9.Instance->CCR1&& \
	   targetPwm[5] == htim9.Instance->CCR2&& \
	   targetPwm[6] == htim13.Instance->CCR1&& \
	   targetPwm[7] == htim14.Instance->CCR1) 
		return 1;
	else
		return 0;
}

/*舵机缓慢移动函数，不用看核心要义就是慢慢改变*/
void slowPwm(uint8_t nServo)
{
	volatile uint32_t *currentPwm;//指针指向不同的pwm控制地址，方便统一修改。
	switch (nServo)
	{
	case 0:
		currentPwm = &htim8.Instance->CCR3;
		break;
	case 1:
		currentPwm = &htim8.Instance->CCR4;
		break;
	case 2:
		currentPwm = &htim12.Instance->CCR1;
		break;
	case 3:
		currentPwm = &htim12.Instance->CCR2;
		break;
	case 4:
		currentPwm = &htim9.Instance->CCR1;
		break;
	case 5:
		currentPwm = &htim9.Instance->CCR2;
		break;
	case 6:
		currentPwm = &htim13.Instance->CCR1;
		break;
	case 7:
		currentPwm = &htim14.Instance->CCR1;
		break;
	default:
		return;
	}

	int8_t flag = 2;
	if (*currentPwm > targetPwm[nServo])
		flag = -2; // 如果当前pwm值大于目标pwm值，将flag设置为-1
	if (*currentPwm == targetPwm[nServo])
		flag = 0; // 如果当前pwm值等于目标pwm值，将flag设置为0
	if (*currentPwm > targetPwm[nServo] ? (*currentPwm - targetPwm[nServo] > 10) : (targetPwm[nServo] - *currentPwm > 10)) {
    	// 处理差值大于10的情况,三目运算符 不能用abs，因为是无符号类似的变量
		flag = 10 * flag; // 如果当前pwm值小于目标pwm值，它们之间的差值大于10，flag需要乘以10
	}
	// 将当前pwm值加上flag，实现缓慢移动的效果
	*currentPwm = (*currentPwm) + flag;
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
