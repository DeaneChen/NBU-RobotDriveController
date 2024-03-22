#include "motor_controller.h"
#include "delay.h"
#include "gpio.h"
#include "motor_driver.h"
#include "tim.h"

typedef struct {
    int16_t SpeedCur;  // ��ǰ�ٶ�
    int16_t SpeedSet;  // Ŀ���ٶ�
    int16_t SpeedPWM;  // ���PWMռ�ձ�
    float SpeedErr1;   // ��һ���ٶ����
    float SpeedErr2;   // ���ϴ��ٶ����
    int32_t EncCnt;    // ����������
} Motor;

typedef struct {
    float KP;                   // ����ϵ��
    float KI;                   // ����ϵ��
    float KD;                   // ΢��ϵ��
    int16_t Acc;                // ���ٶ�
    float WheelDiameter;        // ����ֱ��
    int32_t EncoderResolution;  // �������ֱ��� һȦ��������
    int16_t MotorEnabledCount;  // ���õĵ������
    Motor Motors[4];            // �������
} MotorController;

MotorController controller;  // ����������

// MotorController_Init() ��ʼ������
void MotorController_Init(void)
{
    __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);  // �����ʱ���жϱ�־
    HAL_TIM_Base_Start_IT(&htim6);              // ������ʱ����ʹ���ж�

    MotorController_Enable(DISABLE);  // ���õ��������

    // ��ʼ��PID����
    controller.KP = MOTOR_CONTROLLER_KP;  // ����ϵ��
    controller.KI = MOTOR_CONTROLLER_KI;  // ����ϵ��
    controller.KD = MOTOR_CONTROLLER_KD;  // ΢��ϵ��

    controller.Acc = MOTOR_CONTROLLER_ACC_LIMIT;                         // Ĭ�ϼ��ٶ�
    controller.WheelDiameter = MOTOR_WHEEL_DIAMETER;                     // ����ֱ��
    controller.EncoderResolution = MOTOR_CONTROLLER_ENCODER_RESOLUTION;  // �������ֱ���
    controller.MotorEnabledCount = MOTOR_COUNT;                          // ���õĵ������
}

// MotorController_SetAcceleration() �������ӵļ��ٶ�ֵ����λmm/s/s����Ϊ0�൱����Сֵ1��
void MotorController_SetAcceleration(uint16_t nAcc) {
    controller.Acc = nAcc * MOTOR_CONTROLLER_PERIOD / 1000 + 1;
}

// MotorController_SetPIDParam() ����PID����
void MotorController_SetPIDParam(float Kp, float Ki, float Kd) {
    controller.KP = Kp;  // ����ϵ��
    controller.KI = Ki;  // ����ϵ��
    controller.KD = Kd;  // ΢��ϵ��
}

void MotorController_Enable(FunctionalState NewState) {
    assert_param(IS_FUNCTIONAL_STATE(NewState));

	// ��ʼ������ֵ��pwm���á�
	// ��һ�����⣬����������ӵ�ʱ�򣬵������һ���ٶȣ����´�ʱ���õĵ����ʼ��������Ϣ�����ᵼ�³�ʼʱ��һ�¡�
	// �ӳ�50������Եȴ����ֹͣ�˶�
	HAL_Delay(50);
	for (int8_t i = 0; i < 4; i++)
	{
		controller.Motors[i] = (Motor){0}; // �����ٶȵ�����ǰ�������м���������㡣
		controller.Motors[i].EncCnt = Encoder_GetEncCount(i + 1);        //��ȡ��ǰ������������ݣ�����ͻ��
		controller.Motors[i].SpeedPWM = MOTOR_DRIVER_PWM_DUTY_LIMIT / 2; //����Ĭ��pwmֵ����ʱΪһ�룬����ֹͣ��
	}

    if (NewState != DISABLE) {
        HAL_TIM_Base_Start_IT(&htim6);
    } else {
        HAL_TIM_Base_Stop_IT(&htim6);
    }
}

// ��������ת�٣�nMotor�����ţ�nSpeed�������ٶȣ���λ��mm/s
void MotorController_SetSpeed(uint8_t nMotor, int16_t nSpeed) {
    if (nMotor > 4)
        return;
    controller.Motors[nMotor - 1].SpeedSet = nSpeed;
}

void MotorController_SpeedTunner(void) {
    // ѭ���趨�ĵ������
    for (int i = 0; i < controller.MotorEnabledCount; i++) {
        Motor* motor = &controller.Motors[i];
        int16_t nSpeedExpect;

        // �����ǰ�ٶ�С���趨�ٶ�
        if (motor->SpeedCur < motor->SpeedSet) {
            // Ԥ���ٶ�����һ�����ٶ�ֵ
            nSpeedExpect = motor->SpeedCur + controller.Acc;
            // ���Ԥ���ٶȳ����趨�ٶȣ���Ԥ���ٶ���Ϊ�趨�ٶ�
            if (nSpeedExpect > motor->SpeedSet)
                nSpeedExpect = motor->SpeedSet;
        } else if (motor->SpeedCur > motor->SpeedSet) {  // Ԥ���ٶȼ���һ�����ٶ�ֵ
            nSpeedExpect = motor->SpeedCur - controller.Acc;
            // ���Ԥ���ٶȵ����趨�ٶȣ���Ԥ���ٶ���Ϊ�趨�ٶ�
            if (nSpeedExpect < motor->SpeedSet)
                nSpeedExpect = motor->SpeedSet;
        } else {
            // ��ǰ�ٶȵ����趨�ٶȣ���Ԥ���ٶ�Ҳ��Ϊ�趨�ٶ�
            nSpeedExpect = motor->SpeedSet;
        }
        // ���µ�ǰ�ٶ�ΪԤ���ٶ�
        motor->SpeedCur = nSpeedExpect;

		int32_t nCnt = Encoder_GetEncCount(i + 1);		// ��ȡ����������ֵ
		// ���ݱ���������ֵ���㵱ǰ�ٶ�
		float fSpeedCur = 3.14f * (nCnt - motor->EncCnt) * controller.WheelDiameter * 1000 / (controller.EncoderResolution * 4 * MOTOR_CONTROLLER_PERIOD);

        float fError = nSpeedExpect - fSpeedCur;  // �����ٶ����
        // �����ٶ�������PWM����
        int16_t pwmDelta = controller.KP * (fError - motor->SpeedErr1) + controller.KI * (fError + motor->SpeedErr1) / 2 +
                           controller.KD * (fError - 2 * motor->SpeedErr1 + motor->SpeedErr2);
        // ����PWM��������PWM�趨ֵ
        int16_t pwmSet = motor->SpeedPWM + pwmDelta;

        // ���PWM�趨ֵ�������ƣ�����Ϊ����ֵ
        if (pwmSet > MOTOR_PWM_DUTY_LIMIT) {
            pwmSet = MOTOR_PWM_DUTY_LIMIT;
        }
        // ���PWM�趨ֵС��0������Ϊ0
        else if (pwmSet < 0) {
            pwmSet = 0;
        }

        // ���µ����PWM�趨ֵ���ٶ����ͱ���������ֵ
        motor->SpeedPWM = pwmSet;
        motor->SpeedErr2 = motor->SpeedErr1;
        motor->SpeedErr1 = fError;
        motor->EncCnt = nCnt;
    }

    // �������е����������õ���PWM�趨ֵ���ø����������
    // pwm������������÷��룬��Ҫ��Ϊ���������е��ͬʱ������
    for (int i = 0; i < controller.MotorEnabledCount; i++) {
        Motor* motor = &controller.Motors[i];
        MotorDriver_SetPWMDuty(i + 1, motor->SpeedPWM);
    }
}
