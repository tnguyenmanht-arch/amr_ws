/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* Handle ngoại vi khai báo trong main.c, dùng chung cho các module ứng dụng */
extern ADC_HandleTypeDef  hadc1;
extern I2C_HandleTypeDef  hi2c2;
extern TIM_HandleTypeDef  htim1;
extern TIM_HandleTypeDef  htim2;
extern TIM_HandleTypeDef  htim5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M3_IN_A_Pin GPIO_PIN_5
#define M3_IN_A_GPIO_Port GPIOE
#define M3_IN_B_Pin GPIO_PIN_6
#define M3_IN_B_GPIO_Port GPIOE
#define SERVO_TX_EN_Pin GPIO_PIN_7
#define SERVO_TX_EN_GPIO_Port GPIOE
#define SERVO_RX_EN_Pin GPIO_PIN_8
#define SERVO_RX_EN_GPIO_Port GPIOE
#define LED_Pin GPIO_PIN_10
#define LED_GPIO_Port GPIOE
#define MPU_INT_Pin GPIO_PIN_12
#define MPU_INT_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOA
#define EN_SW_Pin GPIO_PIN_3
#define EN_SW_GPIO_Port GPIOD
#define M4_IN_A_Pin GPIO_PIN_8
#define M4_IN_A_GPIO_Port GPIOB
#define M4_IN_B_Pin GPIO_PIN_9
#define M4_IN_B_GPIO_Port GPIOB
#define KEY2_Pin GPIO_PIN_0
#define KEY2_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_1
#define KEY1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
