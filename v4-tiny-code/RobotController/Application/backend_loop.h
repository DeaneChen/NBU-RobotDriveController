#ifndef __BACKEND_LOOP_H
#define __BACKEND_LOOP_H

#include "main.h"

// #ifndef BACKEND_LOOP_CYCLE_TIME
//     #define BACKEND_LOOP_CYCLE_TIME 20
// #endif

/**
 * @brief  后台循环函数
 * @note   可以将需要周期性调用的函数放在本函数内统一执行，注意避免使用长时间的阻塞性函数。
 *         需要注意的是，该周期性不一定严格正确，需要准确时间间隔的函数建议单独处理。
 *         本程序默认情况下，在 stm32f4xx_it.c 文件中被 TIM7 的 20ms中断 循环调用。
 */
void Backend_Loop(void);

#endif

