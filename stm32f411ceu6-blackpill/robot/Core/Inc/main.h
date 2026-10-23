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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define START_BUTTON_Pin GPIO_PIN_3
#define START_BUTTON_GPIO_Port GPIOA
#define START_BUTTON_EXTI_IRQn EXTI3_IRQn
#define DRIVE_PWM_Pin GPIO_PIN_7
#define DRIVE_PWM_GPIO_Port GPIOA
#define DRILL_PWM_Pin GPIO_PIN_0
#define DRILL_PWM_GPIO_Port GPIOB
#define DRIVE_DIR_Pin GPIO_PIN_1
#define DRIVE_DIR_GPIO_Port GPIOB
#define DRILL_DIR_Pin GPIO_PIN_10
#define DRILL_DIR_GPIO_Port GPIOB
#define EXT_MAX_Pin GPIO_PIN_12
#define EXT_MAX_GPIO_Port GPIOB
#define EXT_MAX_EXTI_IRQn EXTI15_10_IRQn
#define EXT_MIN_Pin GPIO_PIN_13
#define EXT_MIN_GPIO_Port GPIOB
#define EXT_MIN_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
