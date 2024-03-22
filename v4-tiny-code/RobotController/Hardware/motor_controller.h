#ifndef __MOTOR_CONTROLLER_H
#define __MOTOR_CONTROLLER_H

#include "stm32f4xx.h"


void MotorController_Init(void); //初始化函数
void MotorController_SetAcceleration(uint16_t nAcc); //设置轮子的加速度值，单位mm/s/s，设为0相当于最小值1。
void MotorController_SetSpeed(uint8_t nMotor, int16_t nSpeed); //设置轮子转速，nMotor电机编号，nSpeed轮子线速度，单位：mm/s
void MotorController_Enable(FunctionalState NewState); //nEnable=1启用控制器，=0 停止控制器
void MotorController_SpeedTunner(void); //调速函数，由定时器按照控制周期定时调用。
void MotorController_SetPIDParam(float Kp,float Ki,float Kd); //设置PID参数

#endif
