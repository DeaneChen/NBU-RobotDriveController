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
extern __IO int16_t MotorDirver_Tim4_Update_Count;
extern __IO int16_t MotorDirver_Tim5_Update_Count;
extern __IO int16_t MotorDirver_Tim3_Update_Count;
extern __IO int16_t MotorDirver_Tim2_Update_Count;


/* !!! 通常情况下，在config.h中修改即可 !!! */
#ifndef MOTOR_PWM_DUTY_LIMIT  /* 如果 MOTOR_PWM_DUTY_LIMIT 未在外部配置文件定义才生效 */
    #define MOTOR_PWM_DUTY_LIMIT 10000  
#endif
#ifndef MOTOR_COUNT           /* 如果 MOTOR_COUNT 未在外部配置文件定义才生效 */
    #define MOTOR_COUNT       4
#endif

/* PWM占空比最大值，0 ~ MOTOR_DRIVER_PWM_DUTY_LIMIT 对应 0~100% */
#define MOTOR_DRIVER_PWM_DUTY_LIMIT MOTOR_PWM_DUTY_LIMIT

/**
 * @brief  电机驱动初始化函数
 * @note   该函数将根据 MOTOR_COUNT 确定初始化的电机数量
 *         该函数为基于 DRV8243HW 芯片的电机驱动初始化函数
 * @details  nMotorCount=1，初始化电机1，nMotor=2，初始化电机1和2，以此类推，从1到4
 *           初始化后，默认电机是停止的，需要用MotorDriver_Start启动电机
 */
extern void MotorDriver_Init(void);

/**
 * @brief  设置电机驱动的PWM占空比
 * @param  nMotor 电机编号，可选值1-4
 * @param  nDuty  PWM占空比，0 ~ MOTOR_DRIVER_PWM_DUTY_LIMIT 对应 0 ~ 100%
 * @details  该函数通过设置PWM的占空比，控制电机驱动H桥的开关周期，从而实现电机的降压控制。
 *           占空比 0 ~ 100%  对应电压 -VCC ~ +VCC。
 */
extern void MotorDriver_SetPWMDuty(uint8_t nMotor, uint16_t nDuty);

/**
 * @brief  电机启动函数
 * @param  nMotor 电机编号，可选值1-4
 * @param  nDuty  PWM占空比，0 ~ MOTOR_DRIVER_PWM_DUTY_LIMIT 对应 0 ~ 100%，占空比 50% 时 电机转速为 0
 */
extern void MotorDriver_Start(uint8_t nMotor, uint16_t nDuty);

/**
 * @brief  电机停止函数
 * @param  nMotor 电机编号，可选值1-4
 * @param  nDuty  PWM占空比，0 ~ MOTOR_DRIVER_PWM_DUTY_LIMIT 对应 0 ~ 100%，占空比 50% 时 电机转速为 0
 * @details  每次停止后，需要用MotorDriver_Start重新启动电机
 */
extern void MotorDriver_Stop(uint8_t nMotor, uint16_t nDuty);

/**
 * @brief  关闭电机驱动
 * @param  nMotor 电机编号，可选值1-4
 * @note   该函数将关闭电机驱动的H桥输出，此时电机正负极均为高阻态，此使电机可以自由旋转。
 *         只有关闭电机驱动时，才可以进行电机侧的 开路/短路 诊断
 * @details  每次关闭后，需要用MotorDriver_ON重新启用驱动
 *           该函数需要与Stop函数进行区分，Stop函数为电机停止，且电机处于制动状态，不能自由旋转。
 */
extern void MotorDriver_OFF(uint8_t nMotor);

/**
 * @brief   启用电机驱动
 * @param   nMotor 电机编号，可选值1-4
 * @details 该函数将启用电机驱动的H桥输出
 */
extern void MotorDriver_ON(uint8_t nMotor);

/**
 * @brief  获取电机运行状态
 * @param  nMotor 电机编号，可选值1-4
 * @note   该函数可以用于确认电机是处于Start还是Stop
 * @retval 1 电机已经Start
 * @retval 0 电机已经Stop
 */
extern uint8_t MotorDriver_GetMotorRunState(uint8_t nMotor);

/**
 * @brief  获取驱动工作状态
 * @param  nMotor 电机编号，可选值1-4
 * @note   该函数可以用于确认驱动是处于ON还是OFF
 * @retval 1 驱动已经ON
 * @retval 0 驱动已经OFF
 */
extern uint8_t MotorDriver_GetDriveWorkState(uint8_t nMotor);

/**
 * @brief  获取电机的负载电流
 * @param[out]  motor_currents 存储电机负载电流的数组
 * @note   该函数为阻塞查询函数，消耗的时间一般可以忽略不计（理想情况下ADC查询开销10~几十us），但在最坏的情况下可能发生ADC超时（极少）。
 *         传入的 motor_currents 需要确保 空间大于 4 从而避免溢出问题
 * @retval 1 获取成功
 * @retval 0 获取失败，一般为ADC超时，此时必须进行DEBUG排查。
 */
extern uint8_t MotorDriver_GetCurrent(uint32_t* motor_currents);

/**
 * @brief  获取驱动的全桥负载故障状态
 * @param  nMotor 电机编号，可选值1-4
 * @note   该函数必须在驱动关闭的状态下调用，该函数可以用于判断负载故障情况，例如负载短路，负载开路，负载短路到VCC，短路到GND等
 *         该函数在默认配置下处于不可用状态，若要启用需要根据 DRV8243HW 芯片的电机驱动调整电路板上的配置电阻。
 * @retval 0 负载正常
 * @retval 1 负载开路
 * @retval 2 负载GND短路
 * @retval 3 负载VCC短路
 * @retval 4 其他未知异常
 * @retval 8 驱动未关闭，不允许诊断
 */
uint8_t MotorDriver_GetLoadErrorState(uint8_t nMotor);

/**
 * 编码器部分
 * 使用了TIM2，TIM4，TIM3，TIM5
 * 编码器1，A-PA15(TIM2-1)，B-PB3（TIM2-2）
 * 编码器2，A-PD12(TIM4-1)，B-PD13（TIM4-2）
 * 编码器3，A-PB4(TIM3-1)，B-PB5（TIM3-2）
 * 编码器4，A-PA0(TIM5-1)，B-PA1（TIM5-2）
 */

#ifndef MOTOR_TIM_ENCODER_ARR  /* 如果 MOTOR_TIM_ENCODER_ARR 未在外部配置文件定义 */
    #define MOTOR_TIM_ENCODER_ARR 60000
#endif
#ifndef ENCODER_COUNT          /* 如果 ENCODER_COUNT 未在外部配置文件定义才生效 */
    #define ENCODER_COUNT       MOTOR_COUNT
#endif

/* 编码器范围 */
#define ENC_TIM_ARR MOTOR_TIM_ENCODER_ARR


/**
 * @brief  编码器初始化函数
 * @note   该函数将根据 ENCODER_COUNT 宏定义确定初始化的编码器数量
 * @details  基于 AB 相反馈 的编码器初始化
 *           nEncoderCount=1，初始化编码器1，=2，初始化编码器1和2，以此类推，从1到4
 */
extern void Encoder_Init(void); 

/**
 * @brief  获取编码器的当前计数值
 * @param  nEncoder 编码器编号，nEncoder=1返回编码器1，以此类推。
 * @retval 编码器的当前计数值
 */
extern uint16_t Encoder_GetCNT(uint8_t nEncoder); 

/**
 * @brief  获取编码器累计计数值
 * @param  nEncoder 编码器编号，nEncoder=1返回编码器1，以此类推。
 * @return 编码器的累计计数值 32位带符号整形
 * @note   返回值为32位带符号整形，注意长时间运行的溢出。一般情况下，数小时没有问题。
 *         通常可以间隔一定时间进行两次调用，将两次的返回值作差运算得到增量结果，最后通过增量结果计算电机速度。
 *         
 */
extern int32_t Encoder_GetEncCount(uint8_t nEncoder);


/* !!! 通常情况下，在config.h中修改即可 !!! */
/* 相关参数定义 - 若未在外部定义则生效 */
#ifndef MOTOR_CURRENT_DETECTION_R_SAMPLE_REC
    #define MOTOR_CURRENT_DETECTION_R_SAMPLE_REC   (0.001f)
#endif
#ifndef MOTOR_CURRENT_DETECTION_R_SAMPLE
    #define MOTOR_CURRENT_DETECTION_R_SAMPLE       (1000f)
#endif 
#ifndef IS_ENABLE_MOTOR_CURRENT_FULL_DETECTION
    #define IS_ENABLE_MOTOR_CURRENT_FULL_DETECTION (1)
#endif
#ifndef IS_ENABLE_MOTOR_CURRENT_DETECTION
    #define IS_ENABLE_MOTOR_CURRENT_DETECTION      (1)
#endif

#endif

