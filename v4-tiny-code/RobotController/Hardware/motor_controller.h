#ifndef __MOTOR_CONTROLLER_H
#define __MOTOR_CONTROLLER_H

#include "stm32f4xx.h"
//使用了定时器6
#define MOTOR_CONTROLLER_PERIOD 10  //控制周期，单位：ms
//PID参数初始值，运行中可通过调用函数改变

#define MOTOR_CONTROLLER_KP 20
#define MOTOR_CONTROLLER_KI 4
#define MOTOR_CONTROLLER_KD 0.8

//MotorController_Init() 初始化函数
//nEncoderResolution编码器分辨率，轮子一圈的脉冲数；nWheelDiameter轮子的直径，单位：mm
//nMotorCount电机数量，如果为2，则开启A，B电机；如果为4，则开启A、B、C、D四个电机
void MotorController_Init(uint16_t nEncoderResolution, uint8_t nWheelDiameter,uint8_t nMotorCount); 
void MotorController_SetAcceleration(uint16_t nAcc); //设置轮子的加速度值，单位mm/s/s，设为0相当于最小值1。
void MotorController_SetSpeed(uint8_t nMotor, int16_t nSpeed); //设置轮子转速，nMotor电机编号，nSpeed轮子线速度，单位：mm/s
void MotorController_Enable(FunctionalState NewState); //nEnable=1启用控制器，=0 停止控制器
void MotorController_SpeedTunner(void); //调速函数，由定时器按照控制周期定时调用。
void MotorController_SetPIDParam(float Kp,float Ki,float Kd); //设置PID参数

#endif
