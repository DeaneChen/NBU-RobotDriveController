#include "task.h"


TaskStateShift task1(TaskCallState state){

    /* 如果任务初始化，第一次预调用 */
    if(state == TASK_CALL_INIT){
        /* 做一些初始化的事情 */
        /* fun1(); */
        /* fun2(); */

        return TASK_TO_WATE; /* 任务等待，等待下一次调用 */
    }

    /* 如果任务持续运行 */
    if(state == TASK_CALL_RUNNING){
        /* 做一些事情 */
        /* fun3(); */
        /* fun4(); */
    }

    /* 如果任务持续运行 */
    if(state == TASK_CALL_RUNNING){
        /* 满足了一些条件，任务可以休眠了，但不要销毁 */
        if(1){
            return TASK_TO_SLEEP;
        }
    }


    return TASK_TO_WATE; /* 任务等待，等待下一次调用 */
    
}

TaskStateShift task2(TaskCallState state){
    ;
}


void main(void) {
    InitTaskSystem();
    LaunchTaskSchedule();

    CreateTask(&task1, 2, 10);
    WakeUpTask(&task1);
    CreateTask(&task2, 2, 10);
    WakeUpTask(&task2);

    while (1){
        ;
    }
}