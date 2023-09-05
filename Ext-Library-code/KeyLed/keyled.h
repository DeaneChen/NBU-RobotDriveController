/*
 * @Author       : 来自互联网，常见于各类开发板的库中
 * @note         : 主要目的是提供长按按键响应的一种实现思路
 */
#ifndef __KEYLED_H
#define __KEYLED_H	 
#include "stm32f4xx.h"	  
 
// KEY Part

#define KEY1_PIN 		HAL_GPIO_ReadPin(FnKEY1_GPIO_Port,FnKEY2_Pin)
#define KEY2_PIN 		HAL_GPIO_ReadPin(FnKEY1_GPIO_Port,FnKEY2_Pin)

#define KEY1 	1
#define KEY2    2

void Key_Init(void);	     //IO初始化
uint8_t Key_Scan(uint8_t);   //按键扫描函数	

// LED Part

#define FnLED1_ON()    HAL_GPIO_WritePin(FnLED1_GPIO_Port, FnLED1_Pin, GPIO_PIN_RESET)
#define FnLED1_OFF()   HAL_GPIO_WritePin(FnLED1_GPIO_Port, FnLED1_Pin, GPIO_PIN_SET)
#define FnLED1_SHIFT() HAL_GPIO_TogglePin(FnLED1_GPIO_Port,FnLED1_Pin)
#define FnLED2_ON()    HAL_GPIO_WritePin(FnLED2_GPIO_Port, FnLED2_Pin, GPIO_PIN_RESET)
#define FnLED2_OFF()   HAL_GPIO_WritePin(FnLED2_GPIO_Port, FnLED2_Pin, GPIO_PIN_SET)
#define FnLED2_SHIFT() HAL_GPIO_TogglePin(FnLED2_GPIO_Port,FnLED2_Pin)
#define FnLED3_ON()    HAL_GPIO_WritePin(FnLED3_GPIO_Port, FnLED3_Pin, GPIO_PIN_RESET)
#define FnLED3_OFF()   HAL_GPIO_WritePin(FnLED3_GPIO_Port, FnLED3_Pin, GPIO_PIN_SET)
#define FnLED3_SHIFT() HAL_GPIO_TogglePin(FnLED3_GPIO_Port,FnLED3_Pin)
     
void LED_Init(void);    //初始化


#endif
