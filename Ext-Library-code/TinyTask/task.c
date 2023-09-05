/*
 * @Author       : LuHeQiu
 * @Date         : 2021-04-01 18:51:38
 * @LastEditTime : 2023-04-21 03:04:41
 * @LastEditors  : LuHeQiu
 * @Description  : 
 * @FilePath     : /UnionLink-Ctrl/System/task.c
 * @HomePage     : https://www.luheqiu.com
 */

/* Includes ------------------------------------------------------------------*/
#include "task.h"
#include "tim.h"

/* 接口配置 ------------------------------------------------------------------*/
// #include "delay.h"           /* 包含 GetsystemRunningTime()  全局时间获取函数 */
#include "function.h"           /* 包含 task_controller_error() 过载错误报错 */
#define TASK_GET_SYSTEMTIME()       (HAL_GetTick()*1000)           /* 全局时间获取函数，单位us */
#define TASK_CPU_OVERLOAD_DEBUG()   task_controller_error()        /* CPU过载错误报错          */

/* Private variables ---------------------------------------------------------*/
TaskStack taskStack;

/**
  * @brief  Initializes the TaskSystem according to the default parameters.
  * @param  None
  * @retval None
  */
void InitTaskSystem(){
    
    /* initialize the task stack */
    
    taskStack.maxTaskNum=MAX_TASK_NUM;
    taskStack.resTaskNum=MAX_TASK_NUM;
    taskStack.maxTickNum=MAX_TICK_NUM;
    taskStack.resTickNum=MAX_TICK_NUM;
    
    /* Config TIM14 */
    
    /* 已经在main函数初始化 */
   
}

/**
  * @brief  Create a new task and add it to the taskStack.
  * @param  taskFunction: Provides a handle to a task function through which the system can execute a task.
  * @param  ticks: Determine how much time the task will take.
  * @param  period: Determine the execution period of the task.(ms)
  * @retval None
  */
uint8_t CreateTask( TaskStateShift (*taskFunction)(TaskCallState),uint8_t ticks,uint16_t period){
    
    if(taskStack.resTaskNum<=0)return(1);               /* The maximum number of tasks has been exceeded */
    if(taskStack.resTickNum<=0)return(2);               /* The maximum number of ticks has been exceeded */
    
    for(uint8_t i=0;i<taskStack.maxTaskNum-taskStack.resTaskNum;i++){  
        if(taskStack.tasks[i].taskFunction==taskFunction){
            return(3);                                  /* This task is already in the taskStack         */
        }
    }

    taskStack.tasks[taskStack.maxTaskNum - taskStack.resTaskNum].taskFunction = taskFunction;
    taskStack.tasks[taskStack.maxTaskNum - taskStack.resTaskNum].taskTicks = ticks;
    taskStack.tasks[taskStack.maxTaskNum - taskStack.resTaskNum].defaultPeriod = period;

    taskStack.tasks[taskStack.maxTaskNum - taskStack.resTaskNum].taskstate = sleep;
    taskStack.tasks[taskStack.maxTaskNum - taskStack.resTaskNum].cTaskPerid = period / TASKSYSTEM_PERIOD;

    taskStack.resTaskNum -= 1;
    taskStack.resTickNum -= ticks;

    /* 以指令码0运行一次表示初始化 */
    (*taskFunction)(TASK_CALL_INIT);
    
    return(0);
}

/**
 * @brief  Wake up a sleeping task which is in the taskStack.
 * @param  taskFunction: Provides a handle to a task function through which the system can execute a task.
 * @retval the state  : 0 succeed
 *                      1 the task is not sleep , cannot to wake
 *                      2 the task is not in the taskStack , please Create first
 */
uint8_t WakeUpTask(TaskStateShift(*taskFunction)(TaskCallState)){
    
    for(uint8_t i=0;i<taskStack.maxTaskNum-taskStack.resTaskNum;i++){  
        if(taskStack.tasks[i].taskFunction==taskFunction){
            if(taskStack.tasks[i].taskstate == sleep){
                taskStack.tasks[i].taskstate = waiting;
                return 0;
            }else{
                return 1;
            }
        }
    }
    return 3;
}

/**
 * @brief  Sleep a task which is in the taskStack.
 * @param  taskFunction: Provides a handle to a task function through which the system can execute a task.
 * @retval the state  : 0 succeed
 *                      1 the task is already sleep 
 *                      2 the task is not in the taskStack , please Create first
 */
uint8_t SleepTask(TaskStateShift(*taskFunction)(TaskCallState)){
    
    for(uint8_t i=0;i<taskStack.maxTaskNum-taskStack.resTaskNum;i++){  
        if(taskStack.tasks[i].taskFunction==taskFunction){
            if(taskStack.tasks[i].taskstate == sleep){
                return 1;
            }else{
                taskStack.tasks[i].taskstate = sleep;
                return 0;
            }
        }
    }
    return 3;
}

/**
  * @brief  Launch a task which in the taskStack
  * @param  taskFunction: Provides a handle to a task function through which the system can execute a task.
  * @retval None
  */
uint8_t RunningTask( uint8_t taskID,uint8_t* taskTicks){
    
    if(taskID>=(taskStack.maxTaskNum-taskStack.resTaskNum))return(1);    /* The task is not on the stack */

    (*taskTicks)=taskStack.tasks[taskID].taskTicks;
    taskStack.tasks[taskID].taskstate=running;
    
    return(0);
}

/**
  * @brief  Stop a task which in the taskStack
  * @param  taskFunction: Provides a handle to a task function through which the system can execute a task.
  * @retval None
  */
uint8_t DestroyTask(uint8_t (*taskFunction)(uint8_t)){
    
    for(uint8_t i=0;i<taskStack.maxTaskNum-taskStack.resTaskNum;i++){  
        if(taskStack.tasks[i].taskFunction==taskFunction){
            taskStack.tasks[i].taskstate=release;
            return 0;
        }
    }
    return 3;
}

/**
  * @brief  Launch the task scheduling system
  * @param  None
  * @retval None
  */
void LaunchTaskSchedule(){
    HAL_TIM_Base_Start_IT(&htim14);
}

/**
  * @brief  Close the task scheduling system
  * @param  None
  * @retval None
  */
void CloseTaskSchedule(){
    HAL_TIM_Base_Stop_IT(&htim14);
}

/**
  * @brief  Task polling
  * @param  None
  * @retval None
  */
void TaskScheduleLoop(void){
    
    static uint8_t taskID=0;        /* 任务ID */
    static uint8_t taskticks=0;     /* 任务时间片数 */
    static uint8_t periodticks=0;   /* 当前时间片周期序数 */
    static uint8_t isFree=1;        /* 是否空闲，即没有任务执行 */
    
    uint32_t lastTime = TASK_GET_SYSTEMTIME();

 
    if(periodticks < taskStack.maxTickNum-taskStack.resTickNum && taskticks==0){

        /* 任务轮询结束，返回任务栈栈首 */
        if(taskID>=taskStack.maxTaskNum-taskStack.resTaskNum){
            taskID=0;
        }
        
        /* 判断当前任务状态 */
        switch(taskStack.tasks[taskID].taskstate){

            /* 若任务为休眠态 */
            case sleep:   // 0 1 2 3
            {
                /* 直接跳过该任务 */
                if(isFree){
                    taskticks = taskStack.tasks[taskID].taskTicks;
                }
                break;
            }
            
            /* 若任务为就绪态 */
            case ready:
            {
                /* 如果任务栈为空闲状态则切换当前任务为运行态，并分配时间片 */
                if(isFree){
                    RunningTask(taskID,&taskticks);
                    isFree=0;
                }
            }
                
            /* 若任务为运行态 */
            case running:
            {
                taskticks = taskStack.tasks[taskID].taskTicks;

                if( taskticks==0 ){
                    taskStack.tasks[taskID].taskstate=waiting;
                    break;
                }

                /* 以指令码 1 运行任务函数 并接收返回码 */
                uint8_t res = (*taskStack.tasks[taskID].taskFunction)(TASK_CALL_RUNNING);

                /* 返回码为 0 表示任务暂时完成，进入等待态 */
                if( res == 0 || res == 1 ){
                    taskStack.tasks[taskID].taskstate=waiting;
                    taskStack.tasks[taskID].cTaskPerid=taskStack.tasks[taskID].defaultPeriod/TASKSYSTEM_PERIOD;
                /* 返回码为 3 表示任务全部完成，进入释放态，准备释放当前任务 */
                }else if(res == 2){
                    taskStack.tasks[taskID].taskstate=sleep;
                } 
                if(res == 3){
                    taskStack.tasks[taskID].taskstate=release;
                }
                    
                break;
            }
            
            /* 若任务为等待态 */
            case waiting:
            {
                /* 跳过当前任务 */
                if(isFree){
                    taskticks = taskStack.tasks[taskID].taskTicks;
                }
                break;
            }
            
            /* 若任务为释放态 */
            case release:    //  0  x  2  3
            {
                uint8_t releasedTicks=taskStack.tasks[taskID].taskTicks;   /* 释放的时间片数量 */
                
                /* 任务前移 */
                /* PS虽说链表更优雅就是了 */
                for(uint8_t i=taskID;i<taskStack.maxTaskNum-taskStack.resTaskNum;i++){  
                    taskStack.tasks[i]=taskStack.tasks[i+1];               
                }
                
                taskStack.resTickNum+=releasedTicks;
                taskStack.resTaskNum+=1;
                isFree=1;
                break;
            }
            
        }
    }
    
    /* 真实时间片消耗测量 */
    uint32_t divTime = TASK_GET_SYSTEMTIME() - lastTime;
    uint8_t usedTicks = divTime / (TASKSYSTEM_PERIOD * 1000 / MAX_TICK_NUM) + 1;


    if(periodticks < taskStack.maxTickNum-taskStack.resTickNum){

        /* 如果有任务在运行，矫正运行时间 */
        if( !isFree ){

            if(taskticks > usedTicks){
                taskticks -= usedTicks;
            }else if(taskticks == usedTicks){
                taskticks -= usedTicks;
                taskID++;
            }
            else{
                /* 时间片不足，尝试给任务增加时间片 */
                if( usedTicks <= taskStack.resTickNum ){
                    taskStack.tasks[taskID].taskTicks = usedTicks;
                    taskStack.resTickNum -= (usedTicks - taskticks);
                }else{
                    /* 系统剩余时间片不足，报超载错误 */
                    TASK_CPU_OVERLOAD_DEBUG();
                }
                taskID++;
                taskticks = 0;  
            }
            
            isFree = 1;
            
        }else{
            /* 如果没有任务运行 */

            /* 非紧缩策略  上一任务分配的时间片未用完，CPU空转 */
            if(taskticks > 0){
                taskticks--;
            }
            
            if(taskticks==0){
                if(taskID < taskStack.maxTaskNum){
                    taskID++;
                }
            }
        }
    
    }
    
    periodticks += usedTicks;
    

    /* 任务时间片轮询结束，进行下周期轮询准备 */
    if(periodticks==MAX_TICK_NUM){

        for(uint8_t i=0;i<taskStack.maxTaskNum-taskStack.resTaskNum;i++){

            /* 如果任务为等待态 */
            if(taskStack.tasks[i].taskstate==waiting){
                
                /* 等待周期减一 */
                if(taskStack.tasks[i].cTaskPerid>0){
                    taskStack.tasks[i].cTaskPerid--;
                }
                
                /* 等待完成，变更为就绪态 */
                if(taskStack.tasks[i].cTaskPerid==0){
                    taskStack.tasks[i].taskstate=ready;
                }

            }
        }
        periodticks=0;
        
        taskID = 0;
        
    }else if(periodticks > MAX_TICK_NUM ){
        /* 系统剩余时间片不足，报超载错误 */
        TASK_CPU_OVERLOAD_DEBUG();

    }
    
}


