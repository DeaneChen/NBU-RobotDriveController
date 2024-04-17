/**
  ******************************************************************************
  * @file    backend_loop.c
  * @brief   后台循环程序，用于提供周期性的循环控制
  ******************************************************************************
  */

#include "backend_loop.h"
#include "vcc_sense.h"
#include "beep.h"
#include "led.h"
#include "ArmSolution.h"

/**
 * @brief 多周期控制扩展函数 展开宏
 * @param time 必须为 BACKEND_LOOP_CYCLE_TIME 的整数倍，否则会向下取整。
 * @note  可能有一点邪教
 * 用法
 * 需要定义宏BACKEND_LOOP_CYCLE_TIME，该表示外部循环函数的周期
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
    uint16_t vbat = Get_BattryVoltage();

    /* 每50ms执行一次 */
    if(CYCLE_OK(50)){
        /*以下为机械臂缓慢移动*/
		if (ServoTunnerOK() == 0) // 未调节完成
		{
			for (uint8_t i = 0; i < 8; i++) // 遍历所有舵机
				slowPwm(i);
		}

    }
    
    
    if(CYCLE_OK(1000)){
        /* LED1闪烁 */
        FnLED1_SHIFT();

        /* 低电压报警 */
        if (vbat <= LOW_3S_VBAT_ALARM_THRESHOLD && vbat >= DEBUG_V_WITHOUT_BAT || vbat <= LOW_4S_VBAT_ALARM_THRESHOLD && vbat >= MAX_3S_VBAT){
            BEEP_SHIFT(); /* 蜂鸣器间歇性提示 */
        }else{
            BEEP_OFF();  /* 关闭蜂鸣器 */
        }
    }

    /* 每2000ms执行一次 */
    if(CYCLE_OK(2000)){
        ;
    }

    /* ----------------- 后台程序结束 ------------------ */

    end_time = HAL_GetTick();
    if(start_time - end_time >= BACKEND_LOOP_CYCLE_TIME ){
        /* 后台程序死循环异常 */
        /* 后台程序每BACKEND_LOOP_CYCLE_TIME时间调用一次，若后台单次耗时过长会导致程序永远卡在后台进程中 */
        /* 可以使用自定义异常函数替换while(1)从而便于检查 */
        //while(1);
    }

}



