/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.h
 * @brief   This file contains all the function prototypes for
 *          the usart.c file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */


//#define FRAME_BYTE_LENGTH 9  /* 串口通讯一帧数据的字节数（含帧头和帧尾），
//                                 譬如20个字节为一个完整的数据帧，第1个字节帧头，第2个字节代表命令类型，第3~6字节是命令参数，第7个字节为帧尾 */
//#define FRAME_START 0xA5     // 帧头
//#define FRAME_END 0x5A       // 帧尾

//typedef struct {
//    char RxBuffer[FRAME_BYTE_LENGTH];  // 接收缓冲区
//    uint8_t aRxBuffer;                 // 接收中断缓冲
//    uint8_t Rx_Cnt;                    // 接收缓冲计数
//    uint8_t USART_FrameFlag;           // 接收完整数据帧标志，1完整，0不完整
//} UartStruct;

// extern UartStruct uart3Data;  // usart3的数据结构体

extern uint8_t uart3_rx;

// // void USART2_Init(void);
// // void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...);
// // char *itoa(int value, char *string, int radix);
// // int fputc(int ch, FILE *f);

// void USART_GetChar(UartStruct* Uartn, uint8_t nChar);  // 串口接收到一个字节
// void USART_Process(void);



/* USER CODE END Private defines */

void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
