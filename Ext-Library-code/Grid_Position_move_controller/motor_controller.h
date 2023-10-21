/*
 * @Author       : LuHeQiu
 * @Date         : 2022-04-04 15:28:39
 * @LastEditTime : 2022-07-07 18:59:01
 * @LastEditors  : LuHeQiu
 * @Description  : 
 * @FilePath     : \CarBaseControl\USER\motor_controller.h
 * @HomePage     : https://www.luheqiu.com
 */
#ifndef __MOTOR_CONTROLLER_H
#define __MOTOR_CONTROLLER_H

#include "pid_controller.h"

#define ENCODER_RESOLUTION       (13500)  /* 编码器分辨率 */

/**
 * 电机速度环相关定义
 */
#define MOTOR_SPEED_LOOP_CONTROLLER_PERIOD  (5)      /* 电机速度环控制周期  单位：ms                */
#define MOTORCONTROLLER_ACC                 (400)    /* 电机加速度          单位：mm/s/s   6000     */
#define MOTORCONTROLLER_MIN_SEPPED_DEFAULT  ( 80)    /* 电机默认速度最小值  单位：mm/s              */
#define MOTORCONTROLLER_MAX_SEPPED_DEFAULT  (400)    /* 电机默认速度最大值  单位：mm/s              */

#define MOTOR_SPEED_LOOP_CONTROLLER_KP (6.0f)        /* PID控制器比例系数  0.80  6.0 */
#define MOTOR_SPEED_LOOP_CONTROLLER_KI (1.2f)        /* PID控制器积分系数  0.25  1.2 */
#define MOTOR_SPEED_LOOP_CONTROLLER_KD (0.8f)        /* PID控制器微分系数  0.10  0.8 */

#define MOTOR_SPEED_LOOP_CONTROLLER_KP_LOW   (1.00f)        /* PID控制器比例系数 */
#define MOTOR_SPEED_LOOP_CONTROLLER_KI_LOW   (0.80f)        /* PID控制器积分系数 */
#define MOTOR_SPEED_LOOP_CONTROLLER_KD_LOW   (0.05f)        /* PID控制器微分系数 */

#define CAR_MAX_ROTATION_RADIUS  (32500)

/**
 *  电机位置环相关定义
 */
// #define MOTOR_POSITION_LOOP_CONTROLLER_PERIOD  (10)  /* 电机位置环控制周期 单位：ms */
// #define MOTOR_POSITION_LOOP_CONTROLLER_KP    (4.5f)  /* PID控制器比例系数 */  
// #define MOTOR_POSITION_LOOP_CONTROLLER_KI    (0.0f)  /* PID控制器积分系数 */
// #define MOTOR_POSITION_LOOP_CONTROLLER_KD    (0.6f)  /* PID控制器微分系数 */

// #define POSITION_ERROR_THRESHOLD               (5)   /* 位置误差阈值 */

typedef struct{
    
    int16_t curSpeed;       /* 当前实时速度 */
    int16_t setSpeed;       /* 设定速度 */
    int16_t expSpeed;       /* 瞬时期望速度 */

    int16_t accelerate;     /* 最大加速度 */

    INCPIDController speedLoopPID;    /* 速度环PID控制器 */

    /* 当前实时位置 单位:um */
    int32_t curPosition;    

    /* 位置参考平面 单位:um */
    int32_t motorPostionRef; 
    //int32_t setPosition;     /* 设定位置 */
    //int32_t expPosition;     /* 瞬时期望位置 */

    //POSPIDController positionLoopPID;  /* 位置环PID控制器 */

}MotorController;



/**
 * @brief:  电机控制器初始化函数
 *          使用电机控制器前必须调用，避免出现不可预料的结果
 */
void MotorController_Init(void);

/**
 * @brief  电机速度设置函数
 * @param  nMotor 电机编号
 * @param  nSpeed 线速度，单位：mm/s
 */
void SetMotorSpeed(uint8_t nMotor, int16_t nSpeed);

/**
 * @brief  获取电机速度
 *         该速度从电机控制器中直接读取，若电机控制器速度闭环任务没有定时执行刷新速度缓存值则该函数读取的速度可能不实时。
 * @param  nMotor 电机编号 可以取0或1
 * @retval 电机线速度，单位：mm/s
 */
int16_t GetMotorSpeed(uint8_t nMotor);

/**
 * @brief  设置电机位置
 *         该函数并不用于电机的位置控制，而是作为重置基准值使用，便于上层规划控制。
 * @param  nMotor   电机编号 可以取0或1
 * @param  nPostion 位置，单位：um
 * @retval None
 */
void SetMotorPosition(uint8_t nMotor, int32_t nPostion);

/**
 * @brief  获取电机位置
 *         该位置从电机控制器中直接读取，若电机控制器闭环任务没有定时执行刷新位置缓存值则该函数读取的位置可能不实时。
 * @param  nMotor 电机编号 可以取0或1
 * @retval 电机位置，单位：um
 */
int32_t GetMotorPosition(uint8_t nMotor);


static void UpdateMotorCurSpeed(void);

static void UpdateMotorCurPostion(void);

/**
 * @brief:  电机速度环控制任务
 *          应在调度系统中以 MOTOR_SPEED_LOOP_CONTROLLER_PERIOD 时间为周期严格定时调用
 *          调用时间的误差会影响电机的控制性能以及速度计算的准确度
 */
extern uint8_t MotorSpeedLoopControlTask(uint8_t info);

/**
 * @brief:  电机位置环控制任务
 *          应在调度系统中以 MOTOR_POSITION_LOOP_CONTROLLER_PERIOD 时间为周期严格定时调用
 *          调用时间的误差会影响电机的控制性能
 */
// extern uint8_t MotorPositionLoopControlTask(uint8_t info);

/**
 * @brief  获取购物机器人实时线速度 
 * @retval int16_t speed 线速度  单位：mm/s
 */
int16_t GetCarSpeed(void);

/**
 * @brief  获取购物机器人速度设定值 
 * @retval int16_t speed 线速度  单位：mm/s
 */
int16_t GetCarSetSpeed(void);


int16_t GetCarExpSpeed(void);

/**
 * @brief  获取购物机器人实时角速度
 * @retval int16_t speed 角速度  单位：mm/s
 */
int16_t GetCarRotationSpeed(void);

/**
 * @brief  获取购物机器人角速度设定值 
 * @retval int16_t speed 角速度  单位：mm/s
 */
int16_t GetCarSetRotationSpeed(void);

/**
 * @brief  设定购物机器人线速度
 * @param  speed 设定的速度值 单位：mm/s
 */
void SetCarSpeed(int16_t speed);

/**
 * @brief  设定购物机器人角速度
 * @param  speed 设定的速度值 单位：mm/s
 */
void SetCarRotationSpeed(int16_t iRotationSpeed);


static void SpeedModelSolution(void);
extern void SendMotorData(void);

#endif
