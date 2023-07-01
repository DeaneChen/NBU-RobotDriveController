#include "motor_controller.h"
#include "motor_driver.h"
#include "tim.h"
#include "gpio.h"
#include "delay.h"

__IO uint16_t MotorController_EncoderResolution= 390;
__IO uint8_t MotorController_WheelDiameter = 64;
__IO uint16_t MotorController_Acc=0;
__IO int16_t MotorController_MotorA_SpeedSet,MotorController_MotorB_SpeedSet,MotorController_MotorC_SpeedSet,MotorController_MotorD_SpeedSet;
__IO int16_t MotorController_MotorA_SpeedCur,MotorController_MotorB_SpeedCur,MotorController_MotorC_SpeedCur,MotorController_MotorD_SpeedCur;
__IO uint16_t MotorController_MotorA_SpeedPWM,MotorController_MotorB_SpeedPWM,MotorController_MotorC_SpeedPWM,MotorController_MotorD_SpeedPWM;
__IO int32_t MotorController_MotorA_EncCnt,MotorController_MotorB_EncCnt,MotorController_MotorC_EncCnt,MotorController_MotorD_EncCnt;
__IO float MotorController_MotorA_SpeedErr1,MotorController_MotorB_SpeedErr1,MotorController_MotorC_SpeedErr1,MotorController_MotorD_SpeedErr1;
__IO float MotorController_MotorA_SpeedErr2,MotorController_MotorB_SpeedErr2,MotorController_MotorC_SpeedErr2,MotorController_MotorD_SpeedErr2;
__IO uint8_t MotorController_MotorEnabledCount;  //需要调节的电机数量
__IO float MotorController_KP, MotorController_KI, MotorController_KD;  //PID参数

//MotorController_Init() 初始化函数
//nEncoderResolution编码器分辨率，轮子一圈的脉冲数；nWheelDiameter轮子的直径，单位：mm
//nMotorCount电机数量，如果为2，则开启A，B电机；如果为4，则开启A、B、C、D四个电机
void MotorController_Init(uint16_t nEncoderResolution, uint8_t nWheelDiameter,uint8_t nMotorCount)
{
	__HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim6);
			
  	MotorController_Enable(DISABLE);

	MotorController_EncoderResolution = nEncoderResolution;
	MotorController_WheelDiameter = nWheelDiameter;
	MotorController_Acc = 0;
	
	MotorController_MotorEnabledCount = nMotorCount;
	
	//初始化PID参数
	MotorController_KP = MOTOR_CONTROLLER_KP;
	MotorController_KI = MOTOR_CONTROLLER_KI;
	MotorController_KD = MOTOR_CONTROLLER_KD;
}

void MotorController_SetAcceleration(uint16_t nAcc) //设置轮子的加速度值，单位mm/s/s，设为0相当于最小值1。
{
	MotorController_Acc = nAcc * MOTOR_CONTROLLER_PERIOD / 1000 + 1;
}
void MotorController_SetSpeed(uint8_t nMotor, int16_t nSpeed) //设置轮子转速，nMotor电机编号，nSpeed轮子线速度，单位：mm/s
{
	switch(nMotor)
	{
		case 1:
			MotorController_MotorA_SpeedSet = nSpeed;
			if(MotorDriver_GetMotorState(1) == 1)  //如果电机处于停止状态
			{
				MotorController_MotorA_SpeedPWM = PWM_DUTY_LIMIT/2;
				MotorDriver_Start(1,MotorController_MotorD_SpeedPWM);
			}
			break;
		case 2:
			MotorController_MotorB_SpeedSet = nSpeed;
			if(MotorDriver_GetMotorState(2) == 1)  //如果电机处于停止状态
			{
				MotorController_MotorB_SpeedPWM = PWM_DUTY_LIMIT/2;
				MotorDriver_Start(2,MotorController_MotorD_SpeedPWM);
			}		
			break;
		case 3:
			MotorController_MotorC_SpeedSet = nSpeed;
			if(MotorDriver_GetMotorState(3) == 1)  //如果电机处于停止状态
			{
				MotorController_MotorC_SpeedPWM = PWM_DUTY_LIMIT/2;
				MotorDriver_Start(3,MotorController_MotorD_SpeedPWM);
			}		
			break;
		case 4:
			MotorController_MotorD_SpeedSet = nSpeed;
			if(MotorDriver_GetMotorState(4) == 1)  //如果电机处于停止状态
			{
				MotorController_MotorD_SpeedPWM = PWM_DUTY_LIMIT/2;
				MotorDriver_Start(4,MotorController_MotorD_SpeedPWM);
			}		
			break;
		default:
			;
	}
}

void MotorController_Enable(FunctionalState NewState)
{
  	assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	//启用速度调节器前把所有中间变量都清零。
	MotorController_MotorA_EncCnt = Encoder_GetEncCount(1);
	MotorController_MotorA_SpeedErr2 = 0;
	MotorController_MotorA_SpeedErr1 = 0;
	
	MotorController_MotorB_EncCnt = Encoder_GetEncCount(2);
	MotorController_MotorB_SpeedErr2 = 0;
	MotorController_MotorB_SpeedErr1 = 0;

	MotorController_MotorC_EncCnt = Encoder_GetEncCount(3);
	MotorController_MotorC_SpeedErr2 = 0;
	MotorController_MotorC_SpeedErr1 = 0;
	
	MotorController_MotorD_EncCnt = Encoder_GetEncCount(4);
	MotorController_MotorD_SpeedErr2 = 0;
	MotorController_MotorD_SpeedErr1 = 0;
	
	MotorController_MotorA_SpeedSet = 0;
	MotorController_MotorB_SpeedSet = 0;
	MotorController_MotorC_SpeedSet = 0;
	MotorController_MotorD_SpeedSet = 0;

	MotorController_MotorA_SpeedCur = 0;
	MotorController_MotorB_SpeedCur = 0;
	MotorController_MotorC_SpeedCur = 0;
	MotorController_MotorD_SpeedCur = 0;
	MotorController_MotorA_SpeedPWM = PWM_DUTY_LIMIT/2;
	MotorController_MotorB_SpeedPWM = PWM_DUTY_LIMIT/2;
	MotorController_MotorC_SpeedPWM = PWM_DUTY_LIMIT/2;
	MotorController_MotorD_SpeedPWM = PWM_DUTY_LIMIT/2;	
  
	if (NewState != DISABLE)
	{
		HAL_TIM_Base_Start_IT(&htim6);
	}
	else
	{
		HAL_TIM_Base_Stop_IT(&htim6);
	}
}

void MotorController_SpeedTunner(void)
{
	int16_t nSpeedExpect;
	static int16_t pwmDelta = 0;
	int16_t pwmSet;
	float fError;
	int32_t nCnt;
	float fSpeedCur;
	switch(MotorController_MotorEnabledCount)
	{
		case 4:
			//D
			if(MotorController_MotorD_SpeedCur < MotorController_MotorD_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorD_SpeedCur + MotorController_Acc;
				if(nSpeedExpect > MotorController_MotorD_SpeedSet) nSpeedExpect = MotorController_MotorD_SpeedSet;
			}
			else if(MotorController_MotorD_SpeedCur > MotorController_MotorD_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorD_SpeedCur - MotorController_Acc;
				if(nSpeedExpect < MotorController_MotorD_SpeedSet) nSpeedExpect = MotorController_MotorD_SpeedSet;
			}
			else
			{
				nSpeedExpect = MotorController_MotorD_SpeedSet;		
			}
			MotorController_MotorD_SpeedCur = nSpeedExpect;	
			
			nCnt = Encoder_GetEncCount(4);
			fSpeedCur = 3.14*(nCnt - MotorController_MotorD_EncCnt)* MotorController_WheelDiameter * 1000 / (MotorController_EncoderResolution*4*MOTOR_CONTROLLER_PERIOD);
			
			fError = nSpeedExpect - fSpeedCur;
			
			pwmDelta = MotorController_KP * (fError - MotorController_MotorD_SpeedErr1) 
									+ MotorController_KI * (fError + MotorController_MotorD_SpeedErr1)/2
									+ MotorController_KD * (fError-2*MotorController_MotorD_SpeedErr1+MotorController_MotorD_SpeedErr2);
			pwmSet = MotorController_MotorD_SpeedPWM + pwmDelta;

			if(pwmSet>PWM_DUTY_LIMIT) pwmSet = PWM_DUTY_LIMIT;
			else if(pwmSet<0) pwmSet = 0;
			MotorController_MotorD_SpeedPWM = pwmSet;					
			MotorController_MotorD_SpeedErr2 = MotorController_MotorD_SpeedErr1;
			MotorController_MotorD_SpeedErr1 = fError;
			MotorController_MotorD_EncCnt = nCnt;

			//C
			if(MotorController_MotorC_SpeedCur < MotorController_MotorC_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorC_SpeedCur + MotorController_Acc;
				if(nSpeedExpect > MotorController_MotorC_SpeedSet) nSpeedExpect = MotorController_MotorC_SpeedSet;
			}
			else if(MotorController_MotorC_SpeedCur > MotorController_MotorC_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorC_SpeedCur - MotorController_Acc;
				if(nSpeedExpect < MotorController_MotorC_SpeedSet) nSpeedExpect = MotorController_MotorC_SpeedSet;
			}
			else
			{
				nSpeedExpect = MotorController_MotorC_SpeedSet;		
			}
			MotorController_MotorC_SpeedCur = nSpeedExpect;	
			
			nCnt = Encoder_GetEncCount(3);
			fSpeedCur = 3.14*(nCnt - MotorController_MotorC_EncCnt)* MotorController_WheelDiameter * 1000 / (MotorController_EncoderResolution*4*MOTOR_CONTROLLER_PERIOD);
			fError = nSpeedExpect - fSpeedCur;
			
			pwmDelta = MotorController_KP * (fError - MotorController_MotorC_SpeedErr1) 
									+ MotorController_KI * (fError + MotorController_MotorC_SpeedErr1)/2
									+ MotorController_KD * (fError-2*MotorController_MotorC_SpeedErr1+MotorController_MotorC_SpeedErr2);

			pwmSet = MotorController_MotorC_SpeedPWM + pwmDelta;

			if(pwmSet>PWM_DUTY_LIMIT) pwmSet = PWM_DUTY_LIMIT;
			else if(pwmSet<0) pwmSet = 0;
			MotorController_MotorC_SpeedPWM = pwmSet;					
			MotorController_MotorC_SpeedErr2 = MotorController_MotorC_SpeedErr1;
			MotorController_MotorC_SpeedErr1 = fError;
			MotorController_MotorC_EncCnt = nCnt;
			//A
			if(MotorController_MotorA_SpeedCur < MotorController_MotorA_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorA_SpeedCur + MotorController_Acc;
				if(nSpeedExpect > MotorController_MotorA_SpeedSet) nSpeedExpect = MotorController_MotorA_SpeedSet;
			}
			else if(MotorController_MotorA_SpeedCur > MotorController_MotorA_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorA_SpeedCur - MotorController_Acc;
				if(nSpeedExpect < MotorController_MotorA_SpeedSet) nSpeedExpect = MotorController_MotorA_SpeedSet;
			}
			else
			{
				nSpeedExpect = MotorController_MotorA_SpeedSet;		
			}
			MotorController_MotorA_SpeedCur = nSpeedExpect;	
//			previousSpeed = currentSpeed;
//			currentSpeed = MotorController_MotorA_SpeedCur;
//			totalDistance = totalDistance+(previousSpeed+currentSpeed)*0.50/100;
			
			nCnt = Encoder_GetEncCount(1);
			fSpeedCur = 3.14*(nCnt - MotorController_MotorA_EncCnt)* MotorController_WheelDiameter * 1000 / (MotorController_EncoderResolution*4*MOTOR_CONTROLLER_PERIOD);
			fError = nSpeedExpect - fSpeedCur;
			
			pwmDelta = MotorController_KP * (fError - MotorController_MotorA_SpeedErr1) 
									+ MotorController_KI * (fError + MotorController_MotorA_SpeedErr1)/2
									+ MotorController_KD * (fError-2*MotorController_MotorA_SpeedErr1+MotorController_MotorA_SpeedErr2);
			pwmSet = MotorController_MotorA_SpeedPWM + pwmDelta;

			if(pwmSet>PWM_DUTY_LIMIT) pwmSet = PWM_DUTY_LIMIT;
			else if(pwmSet<0) pwmSet = 0;
			MotorController_MotorA_SpeedPWM = pwmSet;			
			MotorController_MotorA_SpeedErr2 = MotorController_MotorA_SpeedErr1;
			MotorController_MotorA_SpeedErr1 = fError;
			MotorController_MotorA_EncCnt = nCnt;			
			//B
			if(MotorController_MotorB_SpeedCur < MotorController_MotorB_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorB_SpeedCur + MotorController_Acc;
				if(nSpeedExpect > MotorController_MotorB_SpeedSet) 
					nSpeedExpect = MotorController_MotorB_SpeedSet;
			}
			else if(MotorController_MotorB_SpeedCur > MotorController_MotorB_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorB_SpeedCur - MotorController_Acc;
				if(nSpeedExpect < MotorController_MotorB_SpeedSet) 
					nSpeedExpect = MotorController_MotorB_SpeedSet;
			}
			else
			{
				nSpeedExpect = MotorController_MotorB_SpeedSet;		
			}
			MotorController_MotorB_SpeedCur = nSpeedExpect;			
			nCnt = Encoder_GetEncCount(2);

			fSpeedCur = 3.14*(nCnt - MotorController_MotorB_EncCnt)* MotorController_WheelDiameter * 1000 / (MotorController_EncoderResolution*4*MOTOR_CONTROLLER_PERIOD);
			
			fError = nSpeedExpect - fSpeedCur;

			pwmDelta = MotorController_KP * (fError - MotorController_MotorB_SpeedErr1) 
									+ MotorController_KI * (fError + MotorController_MotorB_SpeedErr1)/2
									+ MotorController_KD * (fError-2*MotorController_MotorB_SpeedErr1+MotorController_MotorB_SpeedErr2);
			pwmSet = MotorController_MotorB_SpeedPWM + pwmDelta;

			if(pwmSet>PWM_DUTY_LIMIT) pwmSet = PWM_DUTY_LIMIT;
			else if(pwmSet<0) pwmSet = 0;
			MotorController_MotorB_SpeedPWM = pwmSet;			

			MotorController_MotorB_SpeedErr2 = MotorController_MotorB_SpeedErr1;
			MotorController_MotorB_SpeedErr1 = fError;
			MotorController_MotorB_EncCnt = nCnt;
			
			MotorDriver_SetPWMDuty(4,MotorController_MotorD_SpeedPWM);
			MotorDriver_SetPWMDuty(3,MotorController_MotorC_SpeedPWM);
			MotorDriver_SetPWMDuty(1,MotorController_MotorA_SpeedPWM);			
			MotorDriver_SetPWMDuty(2,MotorController_MotorB_SpeedPWM);

		break;
		case 3:
			if(MotorController_MotorC_SpeedCur < MotorController_MotorC_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorC_SpeedCur + MotorController_Acc;
				if(nSpeedExpect > MotorController_MotorC_SpeedSet) nSpeedExpect = MotorController_MotorC_SpeedSet;
			}
			else if(MotorController_MotorC_SpeedCur > MotorController_MotorC_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorC_SpeedCur - MotorController_Acc;
				if(nSpeedExpect < MotorController_MotorC_SpeedSet) nSpeedExpect = MotorController_MotorC_SpeedSet;
			}
			else
			{
				nSpeedExpect = MotorController_MotorC_SpeedSet;		
			}
			MotorController_MotorC_SpeedCur = nSpeedExpect;	
			
			nCnt = Encoder_GetEncCount(3);
			fSpeedCur = 3.14*(nCnt - MotorController_MotorC_EncCnt)* MotorController_WheelDiameter * 1000 / (MotorController_EncoderResolution*4*MOTOR_CONTROLLER_PERIOD);
			fError = nSpeedExpect - fSpeedCur;
			
			pwmDelta = MotorController_KP * (fError - MotorController_MotorC_SpeedErr1) 
									+ MotorController_KI * (fError + MotorController_MotorC_SpeedErr1)/2
									+ MotorController_KD * (fError-2*MotorController_MotorC_SpeedErr1+MotorController_MotorC_SpeedErr2);

			pwmSet = MotorController_MotorC_SpeedPWM + pwmDelta;

			if(pwmSet>PWM_DUTY_LIMIT) pwmSet = PWM_DUTY_LIMIT;
			else if(pwmSet<0) pwmSet = 0;
			MotorController_MotorC_SpeedPWM = pwmSet;					
			MotorDriver_SetPWMDuty(3,MotorController_MotorC_SpeedPWM);
			MotorController_MotorC_SpeedErr2 = MotorController_MotorC_SpeedErr1;
			MotorController_MotorC_SpeedErr1 = fError;
			MotorController_MotorC_EncCnt = nCnt;
		case 2:
			if(MotorController_MotorB_SpeedCur < MotorController_MotorB_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorB_SpeedCur + MotorController_Acc;
				if(nSpeedExpect > MotorController_MotorB_SpeedSet) 
					nSpeedExpect = MotorController_MotorB_SpeedSet;
			}
			else if(MotorController_MotorB_SpeedCur > MotorController_MotorB_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorB_SpeedCur - MotorController_Acc;
				if(nSpeedExpect < MotorController_MotorB_SpeedSet) 
					nSpeedExpect = MotorController_MotorB_SpeedSet;
			}
			else
			{
				nSpeedExpect = MotorController_MotorB_SpeedSet;		
			}
			MotorController_MotorB_SpeedCur = nSpeedExpect;			
			nCnt = Encoder_GetEncCount(2);

			fSpeedCur = 3.14*(nCnt - MotorController_MotorB_EncCnt)* MotorController_WheelDiameter * 1000 / (MotorController_EncoderResolution*4*MOTOR_CONTROLLER_PERIOD);
			
			fError = nSpeedExpect - fSpeedCur;

			pwmDelta = MotorController_KP * (fError - MotorController_MotorB_SpeedErr1) 
									+ MotorController_KI * (fError + MotorController_MotorB_SpeedErr1)/2
									+ MotorController_KD * (fError-2*MotorController_MotorB_SpeedErr1+MotorController_MotorB_SpeedErr2);
			pwmSet = MotorController_MotorB_SpeedPWM + pwmDelta;

			if(pwmSet>PWM_DUTY_LIMIT) pwmSet = PWM_DUTY_LIMIT;
			else if(pwmSet<0) pwmSet = 0;
			MotorController_MotorB_SpeedPWM = pwmSet;			
			MotorDriver_SetPWMDuty(2,MotorController_MotorB_SpeedPWM);

			MotorController_MotorB_SpeedErr2 = MotorController_MotorB_SpeedErr1;
			MotorController_MotorB_SpeedErr1 = fError;
			MotorController_MotorB_EncCnt = nCnt;
		case 1:
			if(MotorController_MotorA_SpeedCur < MotorController_MotorA_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorA_SpeedCur + MotorController_Acc;
				if(nSpeedExpect > MotorController_MotorA_SpeedSet) nSpeedExpect = MotorController_MotorA_SpeedSet;
			}
			else if(MotorController_MotorA_SpeedCur > MotorController_MotorA_SpeedSet)
			{
				nSpeedExpect = MotorController_MotorA_SpeedCur - MotorController_Acc;
				if(nSpeedExpect < MotorController_MotorA_SpeedSet) nSpeedExpect = MotorController_MotorA_SpeedSet;
			}
			else
			{
				nSpeedExpect = MotorController_MotorA_SpeedSet;		
			}
			MotorController_MotorA_SpeedCur = nSpeedExpect;	
//			previousSpeed = currentSpeed;
//			currentSpeed = MotorController_MotorA_SpeedCur;
//			totalDistance = totalDistance+(previousSpeed+currentSpeed)*0.50/100;
			
			nCnt = Encoder_GetEncCount(1);
			fSpeedCur = 3.14*(nCnt - MotorController_MotorA_EncCnt)* MotorController_WheelDiameter * 1000 / (MotorController_EncoderResolution*4*MOTOR_CONTROLLER_PERIOD);
			fError = nSpeedExpect - fSpeedCur;
			
			pwmDelta = MotorController_KP * (fError - MotorController_MotorA_SpeedErr1) 
									+ MotorController_KI * (fError + MotorController_MotorA_SpeedErr1)/2
									+ MotorController_KD * (fError-2*MotorController_MotorA_SpeedErr1+MotorController_MotorA_SpeedErr2);
			pwmSet = MotorController_MotorA_SpeedPWM + pwmDelta;

			if(pwmSet>PWM_DUTY_LIMIT) pwmSet = PWM_DUTY_LIMIT;
			else if(pwmSet<0) pwmSet = 0;
			MotorController_MotorA_SpeedPWM = pwmSet;			
			MotorDriver_SetPWMDuty(1,MotorController_MotorA_SpeedPWM);
			MotorController_MotorA_SpeedErr2 = MotorController_MotorA_SpeedErr1;
			MotorController_MotorA_SpeedErr1 = fError;
			MotorController_MotorA_EncCnt = nCnt;			
		default:
			;
	}
}


void MotorController_SetPIDParam(float Kp,float Ki,float Kd)
{
	MotorController_KP = Kp;
	MotorController_KI = Ki;
	MotorController_KD = Kd;	
}
