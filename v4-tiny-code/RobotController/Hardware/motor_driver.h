#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H

#include "main.h"


#include "stm32f4xx.h"
#include "stm32f4xx_hal_tim.h"

/*电机PWM驱动部分
使用TIM1四路PWM通道控制四个电机，PA11，PA10，PA9，PA8
电机1：IN1-PD3，IN2-PA11（TIM1-4），
电机2：IN1-PDE11，IN2-PA8（TIM1-1），
电机3：IN1-PD14，IN2-PA10（TIM1-3），
电机4：IN1-PE15，IN2-PA9（TIM1-2），
*/

extern int16_t MotorDirver_Tim4_Update_Count;
extern int16_t MotorDirver_Tim5_Update_Count;
extern int16_t MotorDirver_Tim3_Update_Count;
extern int16_t MotorDirver_Tim2_Update_Count;

#define PWM_DUTY_LIMIT 10000  //PWM占空比范围0~10000
void MotorDriver_Init(uint8_t nMotorCount); //初始化电机驱动，nMotorCount=1，初始化电机A，nMotor=2，初始化电机A和B，以此类推，从1到4
																			//初始化后，默认电机是停止的，需要用MotorDriver_Start启动电机																				
void MotorDriver_SetPWMDuty(uint8_t nMotor, uint16_t nDuty); //设置PWM占空比
void MotorDriver_Start(uint8_t nMotor, uint16_t nDuty); //启动电机，nMotor：电机序号，nDuty：初始转速，PWM_DUTY_LIMIT/2等于转速0 
void MotorDriver_Stop(uint8_t nMotor, uint16_t nDuty); //停止电机，nMotor：电机序号，nDuty：停止的速度，0相当于急刹车，数值越大停的越慢，最大不超过PWM_DUTY_LIMIT 
																											 //每次停止后，需要用MotorDriver_Start重新启动电机
uint8_t MotorDriver_GetMotorState(uint8_t nMotor); //获取电机nMotor的状态，0-运行状态，1-停止状态
																											 
/*编码器部分
使用了TIM4，TIM8，TIM3，TIM2
编码器1，A-PA15(TIM2-1)，B-PB3（TIM2-2）
编码器2，A-PD12(TIM4-1)，B-PD13（TIM4-2）
编码器3，A-PB4(TIM3-1)，B-PB5（TIM3-2）
编码器4，A-PA0(TIM5-1)，B-PA1（TIM5-2）
*/
#define ENC_TIM_ARR 60000
void Encoder_Init(uint8_t nEncoderCount); //初始化编码器，nEncoderCount=1，初始化编码器A，=2，初始化编码器A和B，以此类推，从1到4
uint16_t Encoder_GetCNT(uint8_t nEncoder); //返回编码器的计数值，nEncoder=1返回编码器A，以此类推
int32_t Encoder_GetEncCount(uint8_t nEncoder);//返回编码器累计计数值，32位有符号值，正为正转，负为反转。最大大概20亿，长时间运行注意溢出。



#endif

