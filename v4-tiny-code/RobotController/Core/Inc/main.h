/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM_MOTOR_PWM_DUTY_LIMIT MOTOR_PWM_DUTY_LIMIT
#define TIM_MOTOR_TIM_ENCOER_ARR MOTOR_TIM_ENCODER_ARR
#define TIM_SERVO_PWM_DUTY_LIMIT SERVO_PWM_DUTY_LIMIT
#define SWITCH1_Pin GPIO_PIN_2
#define SWITCH1_GPIO_Port GPIOE
#define SWITCH2_Pin GPIO_PIN_3
#define SWITCH2_GPIO_Port GPIOE
#define SWITCH3_Pin GPIO_PIN_4
#define SWITCH3_GPIO_Port GPIOE
#define VBAT_SENSE_Pin GPIO_PIN_0
#define VBAT_SENSE_GPIO_Port GPIOC
#define M1_IPROPI_Pin GPIO_PIN_4
#define M1_IPROPI_GPIO_Port GPIOC
#define M3_IPROPI_Pin GPIO_PIN_5
#define M3_IPROPI_GPIO_Port GPIOC
#define M4_IPROPI_Pin GPIO_PIN_0
#define M4_IPROPI_GPIO_Port GPIOB
#define M2_IPROPI_Pin GPIO_PIN_1
#define M2_IPROPI_GPIO_Port GPIOB
#define FnKEY2_Pin GPIO_PIN_7
#define FnKEY2_GPIO_Port GPIOE
#define FnKEY1_Pin GPIO_PIN_8
#define FnKEY1_GPIO_Port GPIOE
#define FnLEDn_Pin GPIO_PIN_9
#define FnLEDn_GPIO_Port GPIOE
#define FnLED1_Pin GPIO_PIN_10
#define FnLED1_GPIO_Port GPIOE
#define M2_IN1_Pin GPIO_PIN_11
#define M2_IN1_GPIO_Port GPIOE
#define M2_OFF_Pin GPIO_PIN_12
#define M2_OFF_GPIO_Port GPIOE
#define M2_nFAULT_Pin GPIO_PIN_13
#define M2_nFAULT_GPIO_Port GPIOE
#define M2_nSLEEP_Pin GPIO_PIN_14
#define M2_nSLEEP_GPIO_Port GPIOE
#define M4_IN1_Pin GPIO_PIN_15
#define M4_IN1_GPIO_Port GPIOE
#define M4_OFF_Pin GPIO_PIN_12
#define M4_OFF_GPIO_Port GPIOB
#define M4_nFAULT_Pin GPIO_PIN_10
#define M4_nFAULT_GPIO_Port GPIOD
#define M4_nSLEEP_Pin GPIO_PIN_11
#define M4_nSLEEP_GPIO_Port GPIOD
#define M3_IN1_Pin GPIO_PIN_14
#define M3_IN1_GPIO_Port GPIOD
#define M3_OFF_Pin GPIO_PIN_15
#define M3_OFF_GPIO_Port GPIOD
#define M2_IN2_Pin GPIO_PIN_8
#define M2_IN2_GPIO_Port GPIOA
#define M4_IN2_Pin GPIO_PIN_9
#define M4_IN2_GPIO_Port GPIOA
#define M3_IN2_Pin GPIO_PIN_10
#define M3_IN2_GPIO_Port GPIOA
#define M1_IN2_Pin GPIO_PIN_11
#define M1_IN2_GPIO_Port GPIOA
#define M3_nFAULT_Pin GPIO_PIN_12
#define M3_nFAULT_GPIO_Port GPIOA
#define M3_nSLEEP_Pin GPIO_PIN_0
#define M3_nSLEEP_GPIO_Port GPIOD
#define M1_OFF_Pin GPIO_PIN_1
#define M1_OFF_GPIO_Port GPIOD
#define M1_IN1_Pin GPIO_PIN_3
#define M1_IN1_GPIO_Port GPIOD
#define M1_nFAULT_Pin GPIO_PIN_4
#define M1_nFAULT_GPIO_Port GPIOD
#define M1_nSLEEP_Pin GPIO_PIN_7
#define M1_nSLEEP_GPIO_Port GPIOD
#define BEEP_EN_Pin GPIO_PIN_1
#define BEEP_EN_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
