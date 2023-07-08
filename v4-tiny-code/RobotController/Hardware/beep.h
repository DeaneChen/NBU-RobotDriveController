#ifndef __BEEP_H
#define __BEEP_H

#include "main.h"

/* 函数声明 ---------------------------------------------------------------- */

/* 蜂鸣器 鸣叫 */
#define BEEP_ON()    HAL_GPIO_WritePin(BEEP_EN_GPIO_Port, BEEP_EN_Pin, GPIO_PIN_SET)
/* 蜂鸣器 闭嘴 :P */
#define BEEP_OFF()   HAL_GPIO_WritePin(BEEP_EN_GPIO_Port, BEEP_EN_Pin, GPIO_PIN_RESET)
/* 蜂鸣器 切换状态 */
#define BEEP_SHIFT() HAL_GPIO_TogglePin(BEEP_EN_GPIO_Port, BEEP_EN_Pin)
/* 检查蜂鸣器状态 */
#define IS_BEEP_ON() (HAL_GPIO_ReadPin(BEEP_EN_GPIO_Port,BEEP_EN_Pin)==GPIO_PIN_SET)




#endif
