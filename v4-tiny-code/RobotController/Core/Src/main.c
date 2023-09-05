/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_driver.h"
#include "motor_controller.h"
#include "keys.h"
#include "led.h"
#include "mpu6500dmp.h"
#include "amt1450_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  MPU6500_DMP_Init(); // MPU6500初始化
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //===================电机启动================
  MotorDriver_Init();
  //一旦开启start后，就开始耗电了，一个电机大概120ma的电流,MotorDriver_OFF才能关闭耗电。
  // MotorDriver_Start(4, PWM_DUTY_LIMIT / 2);
  // MotorDriver_Start(3, PWM_DUTY_LIMIT / 2);
  MotorDriver_Start(2, PWM_DUTY_LIMIT / 2);
  MotorDriver_Start(1, PWM_DUTY_LIMIT / 2);

  Encoder_Init();
  //==================电机转速控制器启动===============
  MotorController_Init(330, 82, 2); // 初始化调速器，参数1：轮子转一圈输出的脉冲个数；参数2：轮子直径，单位mm；参数3：几个电机需要调速
  MotorController_SetAcceleration(800);  // 设置加速度值，单位：mm/秒*秒
  MotorController_Enable(ENABLE);
  int nSpeed = 0; // 转速变量

  //===================红外传感器启动===============
  AMT1450_UART_Cmd(ENABLE);

  //=================led测试=================
  FnLED_SetRGB(FnLED2, 33, 0, 0, 1);
  uint8_t led_val = 0; // RGB变换中Green色数值变化变量
  HAL_Delay(500);      // 延迟500ms后关闭led2
  FnLED_OFF(FnLED2);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    //
    // 不断循环执行的代码块
    //
    //============================按键实现转速改变=====================
    if (Key_Released(1) == 1)
    {
      nSpeed += 100;
      MotorController_SetSpeed(1, -nSpeed);
      MotorController_SetSpeed(2, nSpeed);
      // MotorController_SetSpeed(3,nSpeed);
      // MotorController_SetSpeed(4,nSpeed);
    }
    if (Key_Released(2) == 1)
    {
      nSpeed -= 100;
      MotorController_SetSpeed(1, -nSpeed);
      MotorController_SetSpeed(2, nSpeed);
      // MotorController_SetSpeed(3,nSpeed);
      // MotorController_SetSpeed(4,nSpeed);
    }

    //===============LED测试程序---绿色渐变实现=============
    FnLED_SetRGB(FnLED3, 0, led_val, 0, 1);
    led_val += 1;
    if (led_val > 66)
      led_val = 0;
    HAL_Delay(7);

    //===================MPU6500测试（加速度传感器）===============
    /* 第 10s 校准MPU6500 为零 */
    /* 读取mpu6500数据，俯仰角数据在函数内部可以看 */
    Get_MPU6500_DMP_Data();

    //==================红外传感器数据采集========================

    uint8_t begin, jump, count[6]; // 最大6个跳变，即3条线
    uint8_t position;
    get_AMT1450Data_UART(&begin, &jump, count);
    if (jump == 2)
      position = 0.5f * (count[0] + count[1]);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
