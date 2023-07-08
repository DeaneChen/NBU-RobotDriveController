/**
  ******************************************************************************
  * @file    backend_loop.c
  * @brief   后台循环程序，用于提供周期性的循环控制
  ******************************************************************************
  */

#include "backend_loop.h"
#include "vcc_sense.h"
#include "beep.h"

/**
 * @brief 多周期控制扩展函数 展开宏
 * @note  可能有一点邪教
 * 用法
 * 假定函数fun每隔20ms调用一次，则以下code1每40ms调用一次，code2每60ms调用一次。
 * 本质上就是通过宏定义，简化了计数过程，CYCLE_OK函数中会计数fun函数的调用次数，
 * 并每隔若干次执行一次code。
 * void fun(void){
 *    if(CYCLE_OK(40)){
 *       code1;
 *    }
 *    if(CYCLE_OK(60)){
 *       code2;
 *    }
 * }
 *
 */
#define CYCLE_OK(time) 1){                                        \
        cycle_ok = 0;                                             \
        do {                                                      \
            static uint32_t cycle_##time;                         \
            cycle_##time++;                                       \
            if (cycle_##time >= time / BACKEND_LOOP_CYCLE_TIME) { \
                cycle_ok = 1;                                     \
                cycle_##time = 0;                                 \
            }                                                     \
        } while (0);                                              \
    }if(cycle_ok


static uint8_t cycle_ok = 0;

/**
 * @brief  后台循环函数
 */
void Backend_Loop(void){
    
    /* 计算后台循环运行耗时 */
    static uint32_t start_time, end_time;
    start_time = HAL_GetTick();

    /* ----------------- 后台程序开始 ------------------ */

    /* 获取电池电压 */
    int16_t vbat = Get_BattryVoltage();
    /* 低电压报警 */
    if( vbat <= LOW_VBAT_ALARM_THRESHOLD ){
        BEEP_ON();
    }

    /* 每100ms执行一次 */
    if(CYCLE_OK(100)){
        ;
    }

    /* 每2000ms执行一次 */
    if(CYCLE_OK(2000)){
        ;
    }

    /* ----------------- 后台程序结束 ------------------ */

    end_time = HAL_GetTick();
    if(start_time - end_time >= BACKEND_LOOP_CYCLE_TIME ){
        /* 后台程序死循环异常 */
        /* 可以使用自定义异常函数替换while(1) */
        while(1);
    }

}



