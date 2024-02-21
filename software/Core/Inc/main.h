/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
#define C0_Pin GPIO_PIN_0
#define C0_GPIO_Port GPIOA
#define C1_Pin GPIO_PIN_1
#define C1_GPIO_Port GPIOA
#define C2_Pin GPIO_PIN_2
#define C2_GPIO_Port GPIOA
#define C3_Pin GPIO_PIN_3
#define C3_GPIO_Port GPIOA
#define HEART_ADC_Pin GPIO_PIN_4
#define HEART_ADC_GPIO_Port GPIOA
#define LEDG_Pin GPIO_PIN_5
#define LEDG_GPIO_Port GPIOA
#define C4_Pin GPIO_PIN_6
#define C4_GPIO_Port GPIOA
#define C5_Pin GPIO_PIN_7
#define C5_GPIO_Port GPIOA
#define C6_Pin GPIO_PIN_0
#define C6_GPIO_Port GPIOB
#define C7_Pin GPIO_PIN_1
#define C7_GPIO_Port GPIOB
#define A0_Pin GPIO_PIN_2
#define A0_GPIO_Port GPIOB
#define A8_Pin GPIO_PIN_10
#define A8_GPIO_Port GPIOB
#define A9_Pin GPIO_PIN_11
#define A9_GPIO_Port GPIOB
#define A10_Pin GPIO_PIN_12
#define A10_GPIO_Port GPIOB
#define BUTT1_Pin GPIO_PIN_13
#define BUTT1_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_14
#define LED1_GPIO_Port GPIOB
#define A1_Pin GPIO_PIN_3
#define A1_GPIO_Port GPIOB
#define A2_Pin GPIO_PIN_4
#define A2_GPIO_Port GPIOB
#define A3_Pin GPIO_PIN_5
#define A3_GPIO_Port GPIOB
#define A4_Pin GPIO_PIN_6
#define A4_GPIO_Port GPIOB
#define A5_Pin GPIO_PIN_7
#define A5_GPIO_Port GPIOB
#define A6_Pin GPIO_PIN_8
#define A6_GPIO_Port GPIOB
#define A7_Pin GPIO_PIN_9
#define A7_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
