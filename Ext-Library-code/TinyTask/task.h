/*
 * @Author       : LuHeQiu
 * @Date         : 2021-04-01 18:51:38
 * @LastEditTime : 2023-04-26 16:49:57
 * @LastEditors  : LuHeQiu
 * @Description  : 这是一个基于时间片轮询的任务调度器，可以提供精准的任务执行周期
 *                 适用场合：在多数实时控制系统中，可能需要定时地执行某些任务，当任务较多时，一般会采用配置一个定时器中断来
 *                 提供一个基准，在该定时器中利用计数的方式，在需要的时间点调用指定的任务，而该系统可以自动的完成这一过程，
 *                 需要使用的资源：    定时器 x 1 用于提供任务调度中断 
 *                                 滴答计时器 x 1 用于提供标准时间
 *                 功能：1、提供了 创建、销毁、休眠、唤醒任务函数。
 *                       2、在时间片不足，也就是CPU过载时调用TASK_CPU_OVERLOAD_DEBUG()报错，此时已非实时响应，属于异常情况
 *                 不适用场合：对长耗时任务、死循环while任务等单次执行就超过任务调度周期的任务不能被加入此调度器，
 *                             对于这类建议采用前后台设计或直接使用成型的实时操作系统
 *                 不具备的功能：该系统只提供定时调度和自适应耗时匹配，未作任何信号量、消息传递、内存管理等设计。
 *                 移植步骤：1、在task.h中设置好默认参数，特别时两个需要的函数接口
 *                           2、根据环境调整task.c中的InitTaskSystem()任务初始化函数，主要是配置定时器及其中断以提供标准时基，
 *                              中断时间为时间片长度，例如本例默认任务调度周期1000us，时间片数量50，因此时间片长度为20us，则中断为20us。
 *                           3、根据环境调整task.c中的TIM6_DAC_IRQHandler()中断服务函数。
 * @FilePath     : \CarBaseControl\SYSTEM\Task\task.h
 * @HomePage     : https://www.luheqiu.com
 */

#ifndef __TASK_H
#define __TASK_H

#include "main.h"


/* 设置默认参数 ----------------------------------------------------*/

#define MAX_TASK_NUM (10)             /* 任务栈最大任务数量                    */
#define MAX_TICK_NUM (20)             /* 时间片最大数目，                      */
#define TASKSYSTEM_PERIOD (10)        /* 任务调度周期，单位ms，建议为整数      */
                                      /* 一般建议任务调度周期是时间片的整数倍，例如50 * 20us = 1000us */

/* 定义系统变量 -------------------------------------------*/

typedef enum{
    sleep,     /* 休眠态 */
    ready,     /* 就绪态 */
    running,   /* 运行态 */
    waiting,   /* 等待态 */
    release    /* 释放态 */
}TaskState;

typedef enum{
    TASK_CALL_INIT = 0 ,   /* 任务初始化 */
    TASK_CALL_RUNNING      /* 任务运行 */
}TaskCallState;

typedef enum{
    TASK_TO_WATE = 0 ,   /* 任务等待 */
    TASK_TO_SUCCEED,
    TASK_TO_SLEEP,
    TASK_TO_DESTROY      /* 任务销毁 */
}TaskStateShift;

typedef struct{
    TaskStateShift (*taskFunction)(TaskCallState);  /* 任务函数指针 */
    uint8_t taskTicks;                              /* 任务占用的时间片 */
    uint16_t defaultPeriod;                         /* 任务默认周期 */
    TaskState taskstate;                            /* 任务状态 */
    uint16_t cTaskPerid;                            /* 当前任务周期 */      
}Task;

typedef struct{
    Task tasks[MAX_TASK_NUM+1];     /* 任务栈 */
    uint8_t maxTaskNum;             /* 最大任务数量 */
    uint8_t resTaskNum;             /* 剩余任务数量 */
    uint8_t maxTickNum;             /* 最大时间片数量 */
    uint8_t resTickNum;             /* 剩余时间片数量 */
}TaskStack;


/* 定义系统私有函数 ------------------------------------------*/

static uint8_t RunningTask( uint8_t taskID,uint8_t* taskTicks);
 
/* 定义任务系统接口 ------------------------------------------*/

void TaskScheduleLoop(void);

/**
 * @brief  初始化任务调度系统
 */
extern void InitTaskSystem(void);


/**
 * @brief  启动任务调度器
 */
extern void LaunchTaskSchedule(void);


/**
 * @brief  关闭任务调度器
 */
extern void CloseTaskSchedule(void);


/**
 * @brief  创建一个任务并加入到任务栈中，任务默认处于sleep休眠态不会被运行
 *         任务创建顺序决定了加入到任务栈的顺序，从某种意义上来说在一个周期内会先执行先创建的任务
 * @param  taskFunction 任务函数地址
 *                      需要周期性调度的任务的地址，一般对函数名取地址即可。
 * @param  ticks        预估需要的时间片数量
 *                      实际上系统会在每次的执行中测量消耗的时间并给任务分配更多时间片（如果有），因此参数不是特别重要
 *                      但还是建议预先进行恰当的分配
 * @param  period       任务执行周期 单位ms
 *                      例如，设定为100，表示任务会每隔100ms执行一次
 * @retval 创建结果代码  0 创建成功
 *                       1 任务数量已达最大值，无法创建
 *                       2 需要分配的任务片不足，无法创建
 *                       3 当前任务已经存在，不支持重入
 */
extern uint8_t CreateTask( TaskStateShift (*taskFunction)(TaskCallState),uint8_t ticks,uint16_t period);

/**
 * @brief  从任务栈中销毁一个任务，此时该任务被彻底删除，需要重新调用CreateTask创建
 * @param  taskFunction 提供待销毁的任务函数地址
 * @retval 结果代码 0 销毁成功
 *                  1 保留
 *                  2 保留
 *                  3 任务并不存在，无法销毁
 */
extern uint8_t DestroyTask(TaskStateShift (*taskFunction)(TaskCallState));


/**
 * @brief  唤醒一个sleep休眠态的任务，
 *         此时任务从sleep态切换到waiting阻塞态，并根据创建时预设的参数定时执行
 * @param  taskFunction 提供待唤醒的任务函数地址
 * @retval 结果代码 0 唤醒成功
 *                  1 该任务未休眠，无法唤醒
 *                  2 保留
 *                  3 任务不存在，请先创建
 */
extern uint8_t WakeUpTask(TaskStateShift (*taskFunction)(TaskCallState));


/**
 * @brief  休眠一个未处于休眠态的任务
 *         此时任务切换到sleep态，与DestroyTask不同的是，任务仍然存在任务栈中，也仍然占用预留的时间片
 *         且可以随时使用WakeUpTask唤醒
 * @param  taskFunction 提供待休眠的任务函数地址
 * @retval 结果代码  0 休眠成功
 *                   1 该任务已经休眠，无法重复休眠
 *                   2 保留
 *                   3 任务不存在，请先创建
 */
extern uint8_t SleepTask(TaskStateShift (*taskFunction)(TaskCallState));

#endif
