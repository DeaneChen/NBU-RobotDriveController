/*
 * @Author       : LuHeQiu
 * @Date         : 2023-10-21 15:32:17
 * @LastEditTime : 2023-10-21 15:32:38
 * @LastEditors  : LuHeQiu
 * @Description  : 
 * @FilePath     : /NBU-RobotDriveController/Ext-Library-code/Grid_Position_move_controller/pid_controller.h
 * @HomePage     : https://www.luheqiu.com
 */
#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#include "config.h"

#define Abs(value)                 (((value)>=0)?(value):(0-(value)))
#define Constrain(input,low,high)  ((input)<(low)?(low):((input)>(high)?(high):(input))) 
#define Round(value)               ((int)(value + 0.5f))

#define PI    (3.14159265f)

/* PID Part */
typedef struct{
    float kp,ki,kd;         //PID参数
}PIDParam;

/* 增量式PID控制器结构体 */
typedef struct{

    PIDParam pidParam;      /* PID参数 */
    
    float lastError;        /* 上次误差 */
    float prevError;        /* 前次误差 */
    float output;           /* 输出 */
    float outMINLimit;      /* 输出最大值限制 */
    float outMAXLimit;      /* 输出最小值限制 */
    
}INCPIDController;          

/* 位置式PID控制器结构体 */
typedef struct{

    /* PID参数 */
    PIDParam pidParam;

    /* 上次误差 */  
    float lastError;      
    /* 积分项 */
    float iTerm;
    /* 积分幅限 */          
	float integrationLimit; 
    
    /* 一阶低通滤波系数(0,1] */
	float FilterPercent;  

    /* 输出 */  
    float output;
    /* 输出最大值限制 */      
    float outMINLimit;
    /* 输出最小值限制 */   
    float outMAXLimit;      
    
}POSPIDController;  

/**
 * @brief  增量式PID控制器计算更新
 * @param  PID    PID控制器
 * @param  target 目标设定值
 * @param  input  当前输入值
 * @note   每调用一次该函数，即表示进行一次PID控制运算，其输出在PID控制器的结构体内
 * @return PID控制器计算输出值
 */
float INCPID_Update(INCPIDController *PID,float target,float input);
/**
 * @brief  位置式PID控制器计算更新
 * @param  PID    PID控制器
 * @param  target 目标设定值
 * @param  input  当前输入值
 * @param  dt     控制周期
 * @note   每调用一次该函数，即表示进行一次PID控制运算，其输出在PID控制器的结构体内
 * @return PID控制器计算输出值
 */
float POSPID_Update(POSPIDController *PID,float target,float input,float dt);

#endif
