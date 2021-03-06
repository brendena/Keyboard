/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define ROW_0_OUTPUT_Pin GPIO_PIN_0
#define ROW_0_OUTPUT_GPIO_Port GPIOH
#define ROW_1_OUTPUT_Pin GPIO_PIN_1
#define ROW_1_OUTPUT_GPIO_Port GPIOH
#define ROW_2_OUTPUT_Pin GPIO_PIN_0
#define ROW_2_OUTPUT_GPIO_Port GPIOC
#define ROW_3_OUTPUT_Pin GPIO_PIN_1
#define ROW_3_OUTPUT_GPIO_Port GPIOC
#define ROW_4_OUTPUT_Pin GPIO_PIN_2
#define ROW_4_OUTPUT_GPIO_Port GPIOC
#define COL_0_INPUT_Pin GPIO_PIN_3
#define COL_0_INPUT_GPIO_Port GPIOC
#define COL_1_INPUT_Pin GPIO_PIN_0
#define COL_1_INPUT_GPIO_Port GPIOA
#define COL_2_INPUT_Pin GPIO_PIN_1
#define COL_2_INPUT_GPIO_Port GPIOA
#define COL_3_INPUT_Pin GPIO_PIN_2
#define COL_3_INPUT_GPIO_Port GPIOA
#define ERROR_LED_Pin GPIO_PIN_4
#define ERROR_LED_GPIO_Port GPIOB
#define MODE_1_Pin GPIO_PIN_5
#define MODE_1_GPIO_Port GPIOB
#define MODE_2_Pin GPIO_PIN_6
#define MODE_2_GPIO_Port GPIOB
#define MODE_3_Pin GPIO_PIN_7
#define MODE_3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
