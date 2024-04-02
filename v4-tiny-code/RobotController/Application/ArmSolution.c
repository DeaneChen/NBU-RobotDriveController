
#include "ArmSolution.h"
#include "delay.h"
#include "math.h"
#include "tim.h"
#include "stdlib.h"

#define PWM_DUTY_LIMIT 10000 // PWMռ�ձ�����Ϊ10000,����20ms    250-1250 ���� 0-180��

// ÿ����������ƶ���Ŀ��pwm
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

	//ֱ�Ӹ�pwm����ֵ  ��ʼ�趨Ϊ��ʼ��е��״̬  ÿ���ϵ�ǰӦ�ðѻ�е�۹�λ
	//����Ϊʹ�þ���
	// Servo_init(0,177);
	// Servo_init(1,162);
	// Servo_init(2,68);
	// Servo_init(3,60);
}
void Servo_init(uint8_t nServo,int angle)
{
	volatile uint32_t *currentPwm;//ָ��ָ��ͬ��pwm���Ƶ�ַ������ͳһ�޸ġ�
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
	if (angle < 0) // �Ƕ�����
		return;
	int pwm = angle * 1.0f / 90 / 20 * PWM_DUTY_LIMIT + 250; // �������Ӧ��pwm��

	// pwm���ƣ���ֹ��������
	if (pwm > 1250)
		pwm = 1250;
	if (pwm < 250)
		pwm = 250;

	*currentPwm = pwm;	//���ö�ʱ��CCR��PWMֵ
	targetPwm[nServo]=pwm;	//����Ŀ��PWMֵ
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

/*��������ƶ����������ÿ�����Ҫ����������ı�*/
void slowPwm(uint8_t nServo)
{
	volatile uint32_t *currentPwm;//ָ��ָ��ͬ��pwm���Ƶ�ַ������ͳһ�޸ġ�
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
		flag = -2; // �����ǰpwmֵ����Ŀ��pwmֵ����flag����Ϊ-1
	if (*currentPwm == targetPwm[nServo])
		flag = 0; // �����ǰpwmֵ����Ŀ��pwmֵ����flag����Ϊ0
	if (*currentPwm > targetPwm[nServo] ? (*currentPwm - targetPwm[nServo] > 10) : (targetPwm[nServo] - *currentPwm > 10)) {
    	// �����ֵ����10�����,��Ŀ����� ������abs����Ϊ���޷������Ƶı���
		flag = 10 * flag; // �����ǰpwmֵС��Ŀ��pwmֵ������֮��Ĳ�ֵ����10��flag��Ҫ����10
	}
	// ����ǰpwmֵ����flag��ʵ�ֻ����ƶ���Ч��
	*currentPwm = (*currentPwm) + flag;
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
