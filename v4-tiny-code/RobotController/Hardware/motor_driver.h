#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H

#include "main.h"


#include "stm32f4xx.h"
#include "stm32f4xx_hal_tim.h"

/**
 * 电机PWM驱动部分
 * 使用TIM1四路PWM通道控制四个电机，PA11，PA10，PA9，PA8
 * 电机1：IN1-PD3，IN2-PA11（TIM1-4），
 * 电机2：IN1-PE11，IN2-PA8（TIM1-1），
 * 电机3：IN1-PD14，IN2-PA10（TIM1-3），
 * 电机4：IN1-PE15，IN2-PA9（TIM1-2），
 */
extern int16_t MotorDirver_Tim4_Update_Count;
extern int16_t MotorDirver_Tim5_Update_Count;
extern int16_t MotorDirver_Tim3_Update_Count;
extern int16_t MotorDirver_Tim2_Update_Count;

/* PWM占空比最大值，0~PWM_DUTY_LIMIT 对应 0~100% */
#define PWM_DUTY_LIMIT MOTOR_PWM_DUTY_LIMIT  

/**
 * @brief  电机驱动初始化函数
 * @param  nMotorCount 初始化的电机数量
 * @note   基于 DRV8243HW 芯片的电机驱动初始化函数
 * @details  nMotorCount=1，初始化电机1，nMotor=2，初始化电机1和2，以此类推，从1到4
 *           初始化后，默认电机是停止的，需要用MotorDriver_Start启动电机
 */
void MotorDriver_Init(uint8_t nMotorCount); 

/**
 * @brief  设置电机驱动的PWM占空比
 * @param  nMotor 电机编号
 * @param  nDuty  PWM占空比，0 ~ PWM_DUTY_LIMIT 对应 0 ~ 100%
 * @details  该函数通过设置PWM的占空比，控制电机驱动H桥的开关周期，从而实现电机的降压控制。
 *           占空比 0 ~ 100%  对应电压 0 ~ VCC。
 */
void MotorDriver_SetPWMDuty(uint8_t nMotor, uint16_t nDuty); 

/**
 * @brief  电机启动函数
 * @param  nMotor 电机编号
 * @param  nDuty  PWM占空比，0 ~ PWM_DUTY_LIMIT 对应 0 ~ 100%，占空比 50% 时 电机转速为 0 
 */
void MotorDriver_Start(uint8_t nMotor, uint16_t nDuty);


/**
 * @brief  电机停止函数
 * @param  nMotor 电机编号
 * @param  nDuty  PWM占空比，0 ~ PWM_DUTY_LIMIT 对应 0 ~ 100%，占空比 50% 时 电机转速为 0 
 * @details  每次停止后，需要用MotorDriver_Start重新启动电机
 */
void MotorDriver_Stop(uint8_t nMotor, uint16_t nDuty);  

/**
 * @brief  关闭电机驱动
 * @param  nMotor
 * @note   该函数将关闭电机驱动的H桥输出，此时电机正负极均为高阻态，此使电机可以自由旋转。
 *         只有关闭电机驱动时，才可以进行电机侧的 开路/短路 诊断
 * @details  每次关闭后，需要用MotorDriver_ON重新启用驱动
 *           该函数需要与Stop函数进行区分，Stop函数为电机停止，且电机处于制动状态，不能自由旋转。
 */																											 
void MotorDriver_OFF(uint8_t nMotor);

/**
 * @brief   启用电机驱动
 * @param   nMotor
 * @details 该函数将启用电机驱动的H桥输出
 */
void MotorDriver_ON(uint8_t nMotor);


uint8_t MotorDriver_GetMotorState(uint8_t nMotor); //获取电机nMotor的状态，0-运行状态，1-停止状态
																											 
/**
 * 编码器部分
 * 使用了TIM2，TIM4，TIM3，TIM5
 * 编码器1，A-PA15(TIM2-1)，B-PB3（TIM2-2）
 * 编码器2，A-PD12(TIM4-1)，B-PD13（TIM4-2）
 * 编码器3，A-PB4(TIM3-1)，B-PB5（TIM3-2）
 * 编码器4，A-PA0(TIM5-1)，B-PA1（TIM5-2）
 */

/* 编码器范围 */
#define ENC_TIM_ARR MOTOR_TIM_ENCODER_ARR


/**
 * @brief  编码器初始化函数
 * @param  nEncoderCount 初始化的编码器数量，通常与使能的电机数量相同。
 * @details  基于 AB 相反馈 的编码器初始化
 *           nEncoderCount=1，初始化编码器1，=2，初始化编码器1和2，以此类推，从1到4
 */
void Encoder_Init(uint8_t nEncoderCount); 


uint16_t Encoder_GetCNT(uint8_t nEncoder); //返回编码器的计数值，nEncoder=1返回编码器A，以此类推
int32_t Encoder_GetEncCount(uint8_t nEncoder);//返回编码器累计计数值，32位有符号值，正为正转，负为反转。最大大概20亿，长时间运行注意溢出。



#endif

