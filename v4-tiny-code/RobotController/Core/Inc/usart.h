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


#define FRAME_BYTE_LENGTH 9 //����ͨѶһ֡���ݵ��ֽ�������֡ͷ��֡β����Ʃ��20���ֽ�Ϊһ������������֡����1���ֽ�֡ͷ����2���ֽڴ����������ͣ���3~6�ֽ��������������7���ֽ�Ϊ֡β
#define FRAME_START 0xA5 //֡ͷ
#define FRAME_END 0x5A  //֡β

typedef struct
{
	char RxBuffer[FRAME_BYTE_LENGTH];   //���ջ�����
	uint8_t aRxBuffer;			//�����жϻ���
	uint8_t Rx_Cnt; 		//���ջ������
	uint8_t USART_FrameFlag;//������������֡��־��1������0������
} UartStruct;


extern UartStruct uart3Data;  //usart3�����ݽṹ��

extern uint8_t uart3_rx;
//void USART2_Init(void);
//void USART_OUT(USART_TypeDef* USARTx, uint8_t *Data,...);
//char *itoa(int value, char *string, int radix);
//int fputc(int ch, FILE *f);
void USART_GetChar(UartStruct *Uartn,uint8_t nChar); //���ڽ��յ�һ���ֽ�
//void USART_Process(void);

/* USER CODE END Private defines */

void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

